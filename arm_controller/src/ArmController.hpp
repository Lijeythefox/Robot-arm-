#ifndef ARM_CONTROLLER_HPP
#define ARM_CONTROLLER_HPP

#include <Arduino.h>
#include <math.h>

#include "Axis.hpp"
#include "Configuration.h"
#include "Endstops.hpp"
#include "Homing.hpp"
#include "SharedState.hpp"

// Runs on core 1 inside the Arduino loop. Owns the axes, the driver enable line and the
// endstops. loopOnce() polls the steppers as fast as possible and runs the control tick
// (endstops, safety checks, commands, setpoints, homing) every CONTROL_TICK_MS.
class ArmController {
 public:
  void begin() {
    pinMode(HARDWARE_ENABLE, INPUT);  // GPIO34 is input-only and has no internal pull
    pinMode(ENABLE_ALL, OUTPUT);
    digitalWrite(ENABLE_ALL, LOW);    // drivers asleep until the first command
    _endstops.begin();
    Axis::beginAll();
    clearPendingTargets();
    _lastTickUs = micros();
    shared::logEvent("arm controller ready, drivers disabled");
  }

  void loopOnce() {
    Axis::runAll();
    uint32_t nowUs = micros();
    if (nowUs - _lastTickUs >= CONTROL_TICK_MS * 1000UL) {
      _lastTickUs = nowUs;
      tick(millis());
    }
  }

 private:
  struct Limits {
    float vel, acc, unhomedVel, unhomedAcc;
  };
  static constexpr Limits kLimits[arm::kNumAxes] = {
      {AXIS_1_VEL_MAX, AXIS_1_ACC_MAX, AXIS_1_UNHOMED_VEL_MAX, AXIS_1_UNHOMED_ACC_MAX},
      {AXIS_2_VEL_MAX, AXIS_2_ACC_MAX, AXIS_2_UNHOMED_VEL_MAX, AXIS_2_UNHOMED_ACC_MAX},
      {AXIS_3_VEL_MAX, AXIS_3_ACC_MAX, AXIS_3_UNHOMED_VEL_MAX, AXIS_3_UNHOMED_ACC_MAX},
      {AXIS_4_VEL_MAX, AXIS_4_ACC_MAX, AXIS_4_UNHOMED_VEL_MAX, AXIS_4_UNHOMED_ACC_MAX},
      {AXIS_5_VEL_MAX, AXIS_5_ACC_MAX, AXIS_5_UNHOMED_VEL_MAX, AXIS_5_UNHOMED_ACC_MAX},
      {AXIS_6_VEL_MAX, AXIS_6_ACC_MAX, AXIS_6_UNHOMED_VEL_MAX, AXIS_6_UNHOMED_ACC_MAX},
  };

  Endstops _endstops;
  uint8_t _homedMask = 0;
  Homing _homing{_endstops, _homedMask};
  arm::State _state = arm::State::Disabled;
  bool _driversEnabled = false;
  uint32_t _driversEnabledMs = 0;
  bool _enableNetHigh = false;
  bool _active = false;
  uint32_t _lastSetpointMs = 0;
  bool _commTimedOut = false;
  bool _netLowReported = false;
  bool _syncRequired = false;
  bool _homingFailed = false;
  uint8_t _homingReportAxis = 0;  // axis shown in the status (current or last failed)
  uint32_t _idleSinceMs = 0;
  uint8_t _endstopStopMask = 0;   // axes stopped by their endstop, until it releases
  uint8_t _appliedHomedMask = 0xFF;  // homed mask the speed limits were last set for
  uint8_t _lastCommandSeq = 0;
  arm::CommandResult _lastCommandResult = arm::CommandResult::None;
  float _pendingTarget[arm::kNumAxes];  // degrees, NAN = none; applied once drivers are awake
  uint32_t _lastTickUs = 0;

  Axis& axis(int number) { return *Axis::axisList[number - 1]; }
  bool homed(int number) const { return _homedMask & (1 << (number - 1)); }
  void clearPendingTargets() {
    for (int i = 0; i < arm::kNumAxes; i++) _pendingTarget[i] = NAN;
  }

  void tick(uint32_t now) {
    _endstops.update();
    _enableNetHigh = digitalRead(HARDWARE_ENABLE) == HIGH;

    handleCommands(now);
    handleSetpoints(now);
    checkEnableNet(now);
    enforceEndstops();
    runHoming(now);
    updateSpeedLimits();
    applyPendingTargets(now);
    manageDriverSleep(now);
    publishStatus(now);
  }

  // ---------------------------------------------------------------------------------------
  // Drivers
  // ---------------------------------------------------------------------------------------
  void enableDrivers(uint32_t now) {
    if (_driversEnabled) return;
    digitalWrite(ENABLE_ALL, HIGH);
    _driversEnabled = true;
    _driversEnabledMs = now;
    if (_state == arm::State::Disabled) _state = arm::State::Ready;
    _idleSinceMs = now;
  }

  // Sleeping drivers hold nothing, so the counted positions can no longer be trusted.
  void disableDrivers() {
    for (Axis* a : Axis::axisList) a->hardStop();
    digitalWrite(ENABLE_ALL, LOW);
    _driversEnabled = false;
    _state = arm::State::Disabled;
    clearPendingTargets();
    if (_homedMask) shared::logEvent("drivers asleep - homing invalidated");
    _homedMask = 0;
  }

  bool driversAwake(uint32_t now) const {
    return _driversEnabled && (now - _driversEnabledMs) >= DRIVER_WAKE_MS;
  }

  // Original behaviour: if the ENABLE net reads LOW while we drive it HIGH (E-stop open),
  // cancel all motion at the current position and ignore new targets.
  void checkEnableNet(uint32_t now) {
    if (!driversAwake(now) || _enableNetHigh) {
      _netLowReported = false;
      return;
    }
    for (Axis* a : Axis::axisList) a->hardStop();
    clearPendingTargets();
    if (_homing.active()) _homing.abort("ENABLE net LOW");
    if (!_netLowReported) {
      shared::logEvent("ENABLE net LOW (E-stop open?) - motion cancelled");
      _netLowReported = true;
    }
  }

  void manageDriverSleep(uint32_t now) {
    if (!_driversEnabled) return;
    bool busy = _state != arm::State::Ready;
    for (Axis* a : Axis::axisList) busy |= a->isRunning();
    for (int i = 0; i < arm::kNumAxes; i++) busy |= !isnan(_pendingTarget[i]);
    if (busy) {
      _idleSinceMs = now;
      return;
    }
    bool allowed = IDLE_SLEEP_MODE == IDLE_SLEEP_ALWAYS ||
                   (IDLE_SLEEP_MODE == IDLE_SLEEP_WHEN_UNHOMED && _homedMask == 0);
    if (allowed && now - _idleSinceMs >= IDLE_SLEEP_DELAY_MS) disableDrivers();
  }

  // Homed axes get the full limits, unhomed axes the conservative ones.
  void updateSpeedLimits() {
    if (_homedMask == _appliedHomedMask || _homing.active()) return;
    for (int n = 1; n <= arm::kNumAxes; n++) {
      const Limits& l = kLimits[n - 1];
      if (homed(n)) axis(n).setSpeedLimits(l.vel, l.acc);
      else axis(n).setSpeedLimits(l.unhomedVel, l.unhomedAcc);
    }
    _appliedHomedMask = _homedMask;
  }

  // ---------------------------------------------------------------------------------------
  // Targets
  // ---------------------------------------------------------------------------------------
  // Queue a move; it starts once the drivers are awake (waking them if needed).
  void requestTarget(int axisNumber, double degrees, uint32_t now) {
    _pendingTarget[axisNumber - 1] = degrees;
    enableDrivers(now);
  }

  void applyPendingTargets(uint32_t now) {
    if (_state != arm::State::Ready || !driversAwake(now) || !_enableNetHigh) return;
    for (int n = 1; n <= arm::kNumAxes; n++) {
      float target = _pendingTarget[n - 1];
      if (isnan(target)) continue;
      _pendingTarget[n - 1] = NAN;
      Axis& a = axis(n);
      // Never start a move towards an endstop that is already triggered.
      if (_endstops.triggered(n)) {
        double delta = target - a.getPosition();
        if (delta != 0 && ((delta > 0) ? 1 : -1) == _endstops.homeDir(n)) continue;
      }
      a.moveToPosition(target);  // clamps to the soft limits
    }
  }

  // An endstop reading HIGH while its axis moves towards it stops that axis immediately.
  // Homing relies on the same check; it only suppresses the fault report for its own axis.
  void enforceEndstops() {
    for (int n = 1; n <= arm::kNumAxes; n++) {
      uint8_t bit = 1 << (n - 1);
      if (!_endstops.triggered(n)) {
        _endstopStopMask &= ~bit;
        continue;
      }
      Axis& a = axis(n);
      if (a.directionOfTravel() != _endstops.homeDir(n)) continue;
      a.hardStop();
      _pendingTarget[n - 1] = NAN;
      if (_homing.active() && _homing.axis() == n) continue;
      if (!(_endstopStopMask & bit))
        shared::logEvent("A%d endstop triggered - axis stopped at %.2f deg", n, a.getPosition());
      _endstopStopMask |= bit;
    }
  }

  // ---------------------------------------------------------------------------------------
  // Homing
  // ---------------------------------------------------------------------------------------
  arm::CommandResult startHoming(const uint8_t* axes, int count, uint32_t now) {
    if (_state == arm::State::Homing) return arm::CommandResult::Rejected;
    for (Axis* a : Axis::axisList) a->stop();
    clearPendingTargets();
    enableDrivers(now);
    _homingFailed = false;
    _state = arm::State::Homing;
    _homingPendingStart = true;
    _homingRequestCount = min(count, 6);
    memcpy(_homingRequest, axes, _homingRequestCount);
    return arm::CommandResult::Accepted;
  }

  uint8_t _homingRequest[6] = {};
  int _homingRequestCount = 0;
  bool _homingPendingStart = false;

  void runHoming(uint32_t now) {
    if (_state != arm::State::Homing || !driversAwake(now)) return;
    if (_homingPendingStart) {
      // Wait for any running move to finish decelerating before homing takes over.
      for (Axis* a : Axis::axisList)
        if (a->isRunning()) return;
      _homingPendingStart = false;
      if (!_homing.start(_homingRequest, _homingRequestCount)) {
        finishHoming(true);
        return;
      }
    }
    if (!_homing.active()) {  // aborted from elsewhere (ENABLE net, stop)
      finishHoming(true);
      return;
    }
    _homingReportAxis = _homing.axis();
    if (_homing.tick()) finishHoming(_homing.failed());
  }

  void finishHoming(bool failed) {
    _state = arm::State::Ready;
    _homingFailed = failed;
    _appliedHomedMask = 0xFF;  // homing changed the homing axis' limits; restore all
    _syncRequired = true;
    for (Axis* a : Axis::axisList) a->stop();
  }

  // ---------------------------------------------------------------------------------------
  // Setpoints from the PC (legacy ROS2 stream or tools with kExtSetpointsValid)
  // ---------------------------------------------------------------------------------------
  void handleSetpoints(uint32_t now) {
    SetpointMessage msg;
    if (xQueueReceive(shared::setpointMailbox, &msg, 0) == pdTRUE) {
      _active = msg.activate;
      _lastSetpointMs = msg.receivedMs;
      if (_commTimedOut) {
        _commTimedOut = false;
        shared::logEvent("setpoint stream resumed");
      }
      if (_active && _state != arm::State::Homing) followSetpoints(msg, now);
    }

    if (COMM_TIMEOUT_MS > 0 && _active && _lastSetpointMs != 0 && !_commTimedOut &&
        now - _lastSetpointMs > COMM_TIMEOUT_MS) {
      _commTimedOut = true;
      if (_state == arm::State::Ready) {
        for (Axis* a : Axis::axisList) a->stop();
        clearPendingTargets();
      }
      shared::logEvent("no setpoints for %d ms - axes stopped", COMM_TIMEOUT_MS);
    }
  }

  void followSetpoints(const SetpointMessage& msg, uint32_t now) {
    if (_syncRequired) {
      for (int n = 1; n <= arm::kNumAxes; n++)
        if (fabs(axis(n).getPosition() - msg.milliDegrees[n - 1] * 1e-3) > SYNC_TOLERANCE_DEG) return;
      _syncRequired = false;
      shared::logEvent("PC setpoints in sync with the arm - following again");
    }
    for (int n = 1; n <= arm::kNumAxes; n++) {
      double target = msg.milliDegrees[n - 1] * 1e-3;
      // Same 0.1 deg dead band as the original: identical setpoints do not wake the drivers.
      if (fabs(axis(n).getPosition() - target) > 0.1) requestTarget(n, target, now);
    }
  }

  // ---------------------------------------------------------------------------------------
  // Commands (network extension and serial console)
  // ---------------------------------------------------------------------------------------
  void handleCommands(uint32_t now) {
    MotionCommand cmd;
    while (xQueueReceive(shared::commandQueue, &cmd, 0) == pdTRUE) {
      arm::CommandResult result = execute(cmd, now);
      if (result == arm::CommandResult::Rejected)
        shared::logEvent("command %d rejected in state %d", (int)cmd.type, (int)_state);
      if (cmd.reportResult) {
        _lastCommandSeq = cmd.seq;
        _lastCommandResult = result;
      }
    }
  }

  arm::CommandResult execute(const MotionCommand& cmd, uint32_t now) {
    bool axisValid = cmd.axis >= 1 && cmd.axis <= arm::kNumAxes;
    bool busy = _state == arm::State::Homing;
    switch (cmd.type) {
      case arm::Command::Stop:
        if (_homing.active()) _homing.abort("stop command");
        if (_state == arm::State::Homing) finishHoming(true);
        for (Axis* a : Axis::axisList) a->stop();
        clearPendingTargets();
        shared::logEvent("stop");
        return arm::CommandResult::Accepted;

      case arm::Command::HomeAll: {
        const uint8_t order[] = HOMING_ORDER;
        return startHoming(order, sizeof(order), now);
      }

      case arm::Command::HomeAxis:
        if (!axisValid) return arm::CommandResult::BadArgument;
        if (!_endstops.enabled(cmd.axis)) {
          shared::logEvent("A%d has no endstop enabled - use sethome", cmd.axis);
          return arm::CommandResult::Rejected;
        }
        return startHoming(&cmd.axis, 1, now);

      case arm::Command::SetHomeHere:
        if (busy) return arm::CommandResult::Rejected;
        if (cmd.axis == 0) {
          _homedMask = 0x3F;
          shared::logEvent("all axes declared homed at their current positions");
        } else if (axisValid) {
          axis(cmd.axis).setCurrentPosition(cmd.param[0] * 1e-3);
          _pendingTarget[cmd.axis - 1] = NAN;
          _homedMask |= 1 << (cmd.axis - 1);
          shared::logEvent("A%d declared homed at %.2f deg", cmd.axis, cmd.param[0] * 1e-3);
        } else {
          return arm::CommandResult::BadArgument;
        }
        _syncRequired = true;
        return arm::CommandResult::Accepted;

      case arm::Command::Jog: {
        if (!axisValid) return arm::CommandResult::BadArgument;
        if (busy) return arm::CommandResult::Rejected;
        double from = isnan(_pendingTarget[cmd.axis - 1]) ? axis(cmd.axis).getPosition()
                                                          : _pendingTarget[cmd.axis - 1];
        requestTarget(cmd.axis, from + cmd.param[0] * 1e-3, now);
        shared::logEvent("jog A%d by %.2f deg", cmd.axis, cmd.param[0] * 1e-3);
        return arm::CommandResult::Accepted;
      }

      case arm::Command::EnableDrivers:
        enableDrivers(now);
        shared::logEvent("drivers enabled");
        return arm::CommandResult::Accepted;

      case arm::Command::DisableDrivers:
        if (_homing.active()) _homing.abort("drivers disabled");
        disableDrivers();
        shared::logEvent("drivers disabled");
        return arm::CommandResult::Accepted;

      default:
        shared::logEvent("command %d not supported yet", (int)cmd.type);
        return arm::CommandResult::Rejected;
    }
  }

  // ---------------------------------------------------------------------------------------
  // Status
  // ---------------------------------------------------------------------------------------
  void publishStatus(uint32_t now) {
    ArmStatus s;
    s.state = _state;
    s.homedMask = _homedMask;
    s.endstopMask = _endstops.triggeredMask();
    s.endstopEnabledMask = _endstops.enabledMask();
    if (_endstopStopMask) s.faultFlags |= arm::kFaultEndstopHit;
    if (_commTimedOut) s.faultFlags |= arm::kFaultCommTimeout;
    if (_homingFailed) s.faultFlags |= arm::kFaultHomingFailed;
    if (_syncRequired) s.faultFlags |= arm::kFaultSyncWait;
    if (driversAwake(now) && !_enableNetHigh) s.faultFlags |= arm::kFaultEStopOpen;
    if (_state == arm::State::Homing || _homingFailed) {
      s.homingAxis = _homingReportAxis;
      s.homingPhase = (uint8_t)_homing.phase();
    }
    s.lastCommandSeq = _lastCommandSeq;
    s.lastCommandResult = _lastCommandResult;
    s.driversEnabled = _driversEnabled;
    s.enableNetHigh = _enableNetHigh;
    s.active = _active;
    for (int n = 1; n <= arm::kNumAxes; n++)
      s.positionMilliDeg[n - 1] = (int32_t)lround(axis(n).getPosition() * 1000.0);
    s.updatedMs = now;
    shared::writeStatus(s);
  }
};

#endif
