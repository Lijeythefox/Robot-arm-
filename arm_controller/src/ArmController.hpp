#ifndef ARM_CONTROLLER_HPP
#define ARM_CONTROLLER_HPP

#include <Arduino.h>
#include <math.h>

#include "Axis.hpp"
#include "Configuration.h"
#include "Endstops.hpp"
#include "SharedState.hpp"

// Runs on core 1 inside the Arduino loop. Owns the axes, the driver enable line and the
// endstops. loopOnce() polls the steppers as fast as possible and runs the control tick
// (endstops, safety checks, commands, setpoints) every CONTROL_TICK_MS.
class ArmController {
 public:
  void begin() {
    pinMode(HARDWARE_ENABLE, INPUT);  // GPIO34 is input-only and has no internal pull
    pinMode(ENABLE_ALL, OUTPUT);
    digitalWrite(ENABLE_ALL, LOW);    // drivers asleep until the first command
    _endstops.begin();
    Axis::beginAll();
    for (int i = 0; i < arm::kNumAxes; i++) _pendingTarget[i] = NAN;
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
  Endstops _endstops;
  arm::State _state = arm::State::Disabled;
  bool _driversEnabled = false;
  uint32_t _driversEnabledMs = 0;
  bool _enableNetHigh = false;
  bool _active = false;
  uint32_t _lastSetpointMs = 0;
  bool _commTimedOut = false;
  bool _netLowReported = false;
  uint32_t _idleSinceMs = 0;
  uint8_t _endstopStopMask = 0;  // axes stopped by their endstop, until it releases
  uint8_t _lastCommandSeq = 0;
  arm::CommandResult _lastCommandResult = arm::CommandResult::None;
  float _pendingTarget[arm::kNumAxes];  // degrees, NAN = none; applied once drivers are awake
  uint32_t _lastTickUs = 0;

  Axis& axis(int number) { return *Axis::axisList[number - 1]; }

  void tick(uint32_t now) {
    _endstops.update();
    _enableNetHigh = digitalRead(HARDWARE_ENABLE) == HIGH;

    handleCommands();
    handleSetpoints(now);
    checkEnableNet(now);
    enforceEndstops();
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
    _state = arm::State::Ready;
    _idleSinceMs = now;
  }

  void disableDrivers() {
    for (Axis* a : Axis::axisList) a->hardStop();
    digitalWrite(ENABLE_ALL, LOW);
    _driversEnabled = false;
    _state = arm::State::Disabled;
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
    for (int i = 0; i < arm::kNumAxes; i++) _pendingTarget[i] = NAN;
    if (!_netLowReported) {
      shared::logEvent("ENABLE net LOW (E-stop open?) - motion cancelled");
      _netLowReported = true;
    }
  }

  // Put the drivers to sleep once nothing has moved for a while, like the original firmware
  // did, so idle motors stay cool.
  void manageDriverSleep(uint32_t now) {
    if (!_driversEnabled) return;
    bool busy = false;
    for (Axis* a : Axis::axisList) busy |= a->isRunning();
    for (int i = 0; i < arm::kNumAxes; i++) busy |= !isnan(_pendingTarget[i]);
    if (busy) {
      _idleSinceMs = now;
      return;
    }
    if (IDLE_SLEEP_DRIVERS && now - _idleSinceMs >= IDLE_SLEEP_DELAY_MS) disableDrivers();
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
    if (!driversAwake(now) || !_enableNetHigh) return;
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
      a.moveToPosition(target);
    }
  }

  // An endstop reading HIGH while its axis moves towards it stops that axis immediately.
  void enforceEndstops() {
    for (int n = 1; n <= arm::kNumAxes; n++) {
      uint8_t bit = 1 << (n - 1);
      if (!_endstops.triggered(n)) {
        _endstopStopMask &= ~bit;
        continue;
      }
      Axis& a = axis(n);
      if (a.directionOfTravel() == _endstops.homeDir(n)) {
        a.hardStop();
        _pendingTarget[n - 1] = NAN;
        if (!(_endstopStopMask & bit))
          shared::logEvent("A%d endstop triggered - axis stopped at %.2f deg", n, a.getPosition());
        _endstopStopMask |= bit;
      }
    }
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
      if (_active) {
        for (int n = 1; n <= arm::kNumAxes; n++) {
          double target = msg.milliDegrees[n - 1] * 1e-3;
          // Same 0.1 deg dead band as the original: identical setpoints do not wake the drivers.
          if (fabs(axis(n).getPosition() - target) > 0.1) requestTarget(n, target, now);
        }
      }
    }

    if (COMM_TIMEOUT_MS > 0 && _active && _lastSetpointMs != 0 && !_commTimedOut &&
        now - _lastSetpointMs > COMM_TIMEOUT_MS) {
      _commTimedOut = true;
      for (Axis* a : Axis::axisList) a->stop();
      for (int i = 0; i < arm::kNumAxes; i++) _pendingTarget[i] = NAN;
      shared::logEvent("no setpoints for %d ms - axes stopped", COMM_TIMEOUT_MS);
    }
  }

  // ---------------------------------------------------------------------------------------
  // Commands (network extension and serial console)
  // ---------------------------------------------------------------------------------------
  void handleCommands() {
    MotionCommand cmd;
    while (xQueueReceive(shared::commandQueue, &cmd, 0) == pdTRUE) {
      arm::CommandResult result = execute(cmd);
      if (cmd.reportResult) {
        _lastCommandSeq = cmd.seq;
        _lastCommandResult = result;
      }
    }
  }

  arm::CommandResult execute(const MotionCommand& cmd) {
    uint32_t now = millis();
    bool axisValid = cmd.axis >= 1 && cmd.axis <= arm::kNumAxes;
    switch (cmd.type) {
      case arm::Command::Stop:
        for (Axis* a : Axis::axisList) a->stop();
        for (int i = 0; i < arm::kNumAxes; i++) _pendingTarget[i] = NAN;
        shared::logEvent("stop");
        return arm::CommandResult::Accepted;

      case arm::Command::Jog: {
        if (!axisValid) return arm::CommandResult::BadArgument;
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
        disableDrivers();
        for (int i = 0; i < arm::kNumAxes; i++) _pendingTarget[i] = NAN;
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
    s.endstopMask = _endstops.triggeredMask();
    s.endstopEnabledMask = _endstops.enabledMask();
    if (_endstopStopMask) s.faultFlags |= arm::kFaultEndstopHit;
    if (_commTimedOut) s.faultFlags |= arm::kFaultCommTimeout;
    if (_driversEnabled && !_enableNetHigh && driversAwake(now)) s.faultFlags |= arm::kFaultEStopOpen;
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
