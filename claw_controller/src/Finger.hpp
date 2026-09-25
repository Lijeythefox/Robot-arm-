#ifndef FINGER_HPP
#define FINGER_HPP

#include <Arduino.h>

#include "ClawShared.hpp"
#include "Configuration.h"
#include "MotorDriver.hpp"

// One tendon finger: an N20 gear motor winding a cord on a drum. update() runs every control
// period and owns the motor output.
class Finger {
 public:
  Finger(char name, MotorChannel& motor) : _name(name), _motor(motor) {}

  void begin() { _motor.begin(); }

  char name() const { return _name; }
  claw::FingerState state() const { return _state; }

  // Manual timed move: dir +1 close / -1 open, duty 0..1.
  void move(int dir, float duty, uint32_t durationMs, uint32_t now) {
    _moveDurationMs = min<uint32_t>(durationMs, MAX_MANUAL_MOVE_MS);
    startDrive(dir, duty, now);
    enter(claw::FingerState::Moving, now);
  }

  void stop(uint32_t now) {
    _motor.brake();
    _driveDir = 0;
    _driving = false;
    enter(claw::FingerState::Braked, now);
  }

  void update(uint32_t now) {
    serviceDrive(now);
    switch (_state) {
      case claw::FingerState::Moving:
        if (now - _stateSinceMs >= _moveDurationMs) stop(now);
        break;
      default:
        break;
    }
  }

 protected:
  char _name;
  MotorChannel& _motor;
  claw::FingerState _state = claw::FingerState::Idle;
  uint32_t _stateSinceMs = 0;
  uint32_t _moveDurationMs = 0;

  // Requested drive, applied by serviceDrive(): brake first if reversing, then ramp the duty.
  int _driveDir = 0;
  float _targetDuty = 0;
  uint32_t _brakeUntilMs = 0;
  uint32_t _rampStartMs = 0;
  bool _driving = false;

  void enter(claw::FingerState s, uint32_t now) {
    _state = s;
    _stateSinceMs = now;
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

  // Change the duty of a running drive without a new ramp (e.g. grip -> hold).
  void setDriveDuty(float duty) {
    _targetDuty = constrain(duty, 0.0f, 1.0f);
    if (_driving) _motor.setDuty(_targetDuty);
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

  void coast(uint32_t now, claw::FingerState next) {
    _motor.coast();
    _driveDir = 0;
    _driving = false;
    enter(next, now);
  }
};

#endif
