#ifndef FINGER_HPP
#define FINGER_HPP

#include <Arduino.h>

#include "ClawShared.hpp"
#include "Configuration.h"
#include "MotorDriver.hpp"

// One tendon finger: an N20 gear motor winding a cord on a drum. No encoder, so everything is
// decided from time and motor current. update() runs every control period and owns the motor.
//
//   close  -> Closing --(current > grip threshold for GRIP_CONFIRM_MS)--> Gripped (hold)
//                     --(CLOSE_TIMEOUT_MS)--> Fault(Timeout)
//   open   -> Opening --(OPEN_MODE signature or OPEN_TIME_MS)--> Open (coast)
//   move   -> Moving  --(duration)--> Braked
//   any driven state --(current > hard limit for HARD_LIMIT_CONFIRM_MS)--> Fault(OverCurrent)
//
// Faults brake the motor (no current flows while braked). Any new command clears the fault.
class Finger {
 public:
  struct Params {
    float gripThresholdmA;
    float hardLimitmA;
  };

  Finger(char name, MotorChannel& motor, const Params& params)
      : _name(name), _motor(motor), _params(params) {}

  void begin() { _motor.begin(); }

  char name() const { return _name; }
  claw::FingerState state() const { return _state; }
  claw::FingerFault fault() const { return _fault; }
  const Params& params() const { return _params; }

  // Without a working current sensor there is no stall protection: lock the motor out.
  void setSensorOk(bool ok, uint32_t now) {
    if (ok == _sensorOk) return;
    _sensorOk = ok;
    if (ALLOW_MOTOR_WITHOUT_SENSOR) return;
    if (!ok) coast(now, claw::FingerState::Disabled);
    else if (_state == claw::FingerState::Disabled) enter(claw::FingerState::Idle, now);
  }

  bool canDrive() const { return _sensorOk || ALLOW_MOTOR_WITHOUT_SENSOR; }

  // thresholdmA = 0 uses the configured grip threshold.
  void close(uint32_t now, float thresholdmA = 0, float effortPct = CLOSE_EFFORT_PCT) {
    if (!canDrive()) return;
    _fault = claw::FingerFault::None;
    _gripThresholdmA = thresholdmA > 0 ? thresholdmA : _params.gripThresholdmA;
    startDrive(+1, effortPct / 100.0f, now);
    enter(claw::FingerState::Closing, now);
  }

  void open(uint32_t now) {
    if (!canDrive()) return;
    _fault = claw::FingerFault::None;
    startDrive(-1, OPEN_EFFORT_PCT / 100.0f, now);
    enter(claw::FingerState::Opening, now);
  }

  // Manual timed move: dir +1 close / -1 open, duty 0..1.
  void move(int dir, float duty, uint32_t durationMs, uint32_t now) {
    if (!canDrive()) return;
    _fault = claw::FingerFault::None;
    _moveDurationMs = min<uint32_t>(durationMs, MAX_MANUAL_MOVE_MS);
    startDrive(dir, duty, now);
    enter(claw::FingerState::Moving, now);
  }

  void stop(uint32_t now) {
    brake(now, canDrive() ? claw::FingerState::Braked : claw::FingerState::Disabled);
  }

  void emergencyStop(uint32_t now) {
    brake(now, claw::FingerState::Fault);
    _fault = claw::FingerFault::EStop;
  }

  void clearFault(uint32_t now) {
    if (_state != claw::FingerState::Fault) return;
    _fault = claw::FingerFault::None;
    enter(claw::FingerState::Braked, now);
  }

  // currentmA: filtered magnitude of the motor current.
  void update(uint32_t now, float currentmA) {
    serviceDrive(now);
    if (!isDriven()) return;

    uint32_t inState = now - _stateSinceMs;
    bool pastInrush = inState >= INRUSH_BLANK_MS;

    // Hard limit: applies to every driven state once the start-up current has passed.
    if (pastInrush && held(currentmA > _params.hardLimitmA, _overSinceMs, now, HARD_LIMIT_CONFIRM_MS)) {
      shared::logEvent("%c OVERCURRENT %.0f mA (limit %.0f) - stopped", _name, currentmA, _params.hardLimitmA);
      fail(claw::FingerFault::OverCurrent, now);
      return;
    }

    switch (_state) {
      case claw::FingerState::Closing:
        if (pastInrush && held(currentmA > _gripThresholdmA, _condSinceMs, now, GRIP_CONFIRM_MS)) {
          shared::logEvent("%c gripped at %.0f mA after %lu ms", _name, currentmA, (unsigned long)inState);
          enter(claw::FingerState::Gripped, now);
          if (HOLD_EFFORT_PCT > 0) setDriveDuty(HOLD_EFFORT_PCT / 100.0f);
          else holdWithBrake();
        } else if (inState >= CLOSE_TIMEOUT_MS) {
          shared::logEvent("%c close TIMEOUT after %lu ms (%.0f mA, threshold %.0f)", _name,
                           (unsigned long)inState, currentmA, _gripThresholdmA);
          fail(claw::FingerFault::Timeout, now);
        }
        break;

      case claw::FingerState::Gripped:
        if (HOLD_MAX_MS > 0 && _driving && inState >= HOLD_MAX_MS) {
          holdWithBrake();
          shared::logEvent("%c held for %lu ms - switching to brake", _name, (unsigned long)inState);
        }
        break;

      case claw::FingerState::Opening: {
        bool signature = false;
        if (OPEN_MODE == OPEN_MODE_CURRENT_BELOW)
          signature = pastInrush && held(currentmA < OPEN_SLACK_BELOW_MA, _condSinceMs, now, OPEN_CONFIRM_MS);
        else if (OPEN_MODE == OPEN_MODE_CURRENT_ABOVE)
          signature = pastInrush && held(currentmA > OPEN_STOP_ABOVE_MA, _condSinceMs, now, OPEN_CONFIRM_MS);
        if (signature || inState >= OPEN_TIME_MS) {
          if (OPEN_MODE != OPEN_MODE_TIME)
            shared::logEvent("%c open: %s after %lu ms (%.0f mA)", _name,
                             signature ? "current signature" : "time limit", (unsigned long)inState, currentmA);
          coast(now, claw::FingerState::Open);
        }
        break;
      }

      case claw::FingerState::Moving:
        if (inState >= _moveDurationMs) stop(now);
        break;

      default:
        break;
    }
  }

 private:
  char _name;
  MotorChannel& _motor;
  Params _params;
  claw::FingerState _state = claw::FingerState::Idle;
  claw::FingerFault _fault = claw::FingerFault::None;
  uint32_t _stateSinceMs = 0;
  uint32_t _moveDurationMs = 0;
  float _gripThresholdmA = 0;
  bool _sensorOk = true;
  uint32_t _condSinceMs = 0;  // start of the current grip / open condition, 0 = not met
  uint32_t _overSinceMs = 0;  // start of the current over-limit condition, 0 = not met

  // Requested drive, applied by serviceDrive(): brake first if reversing, then ramp the duty.
  int _driveDir = 0;
  float _targetDuty = 0;
  uint32_t _brakeUntilMs = 0;
  uint32_t _rampStartMs = 0;
  bool _driving = false;

  bool isDriven() const {
    return _state == claw::FingerState::Closing || _state == claw::FingerState::Gripped ||
           _state == claw::FingerState::Opening || _state == claw::FingerState::Moving;
  }

  // True once `condition` has held continuously for `ms`.
  static bool held(bool condition, uint32_t& since, uint32_t now, uint32_t ms) {
    if (!condition) {
      since = 0;
      return false;
    }
    if (since == 0) since = now ? now : 1;
    return now - since >= ms;
  }

  void enter(claw::FingerState s, uint32_t now) {
    _state = s;
    _stateSinceMs = now;
    _condSinceMs = 0;
    _overSinceMs = 0;
  }

  void fail(claw::FingerFault f, uint32_t now) {
    brake(now, claw::FingerState::Fault);
    _fault = f;
  }

  void startDrive(int dir, float duty, uint32_t now) {
    dir = dir > 0 ? 1 : -1;
    bool reversing = _motor.mode() == MotorChannel::Mode::Drive && _motor.direction() != dir;
    _driveDir = dir;
    _targetDuty = constrain(duty, 0.0f, 1.0f);
    _driving = false;
    if (reversing) {
      _motor.brake();
      _brakeUntilMs = now + DIRECTION_CHANGE_BRAKE_MS;
    } else if (_motor.mode() == MotorChannel::Mode::Drive) {
      _driving = true;  // same direction: keep going, only the duty target changes
    } else {
      _brakeUntilMs = now;
    }
  }

  // Change the duty of a running drive without a new ramp (grip -> hold).
  void setDriveDuty(float duty) {
    _targetDuty = constrain(duty, 0.0f, 1.0f);
    if (_driving) _motor.setDuty(_targetDuty);
  }

  // Hold a grip with the H-bridge shorted: no supply current, the gearbox and brake hold.
  void holdWithBrake() {
    _motor.brake();
    _driveDir = 0;
    _driving = false;
  }

  void serviceDrive(uint32_t now) {
    if (_driveDir == 0) return;
    if (!_driving) {
      if ((int32_t)(now - _brakeUntilMs) < 0) return;
      _motor.drive(_driveDir, 0);
      _rampStartMs = now;
      _driving = true;
    }
    uint32_t t = now - _rampStartMs;
    if (t < MOTOR_RAMP_MS) _motor.setDuty(_targetDuty * t / MOTOR_RAMP_MS);
    else if (_motor.duty() != _targetDuty) _motor.setDuty(_targetDuty);
  }

  void brake(uint32_t now, claw::FingerState next) {
    _motor.brake();
    _driveDir = 0;
    _driving = false;
    enter(next, now);
  }

  void coast(uint32_t now, claw::FingerState next) {
    _motor.coast();
    _driveDir = 0;
    _driving = false;
    enter(next, now);
  }
};

#endif
