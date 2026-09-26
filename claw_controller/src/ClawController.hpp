#ifndef CLAW_CONTROLLER_HPP
#define CLAW_CONTROLLER_HPP

#include <Arduino.h>
#include <Wire.h>

#include "ClawShared.hpp"
#include "Configuration.h"
#include "Finger.hpp"
#include "Ina219.hpp"
#include "MotorDriver.hpp"
#include "StatusLeds.hpp"

// Runs in the high-priority control task every CONTROL_PERIOD_MS: commands, finger state
// machines, status. The only code that touches the motors.
class ClawController {
 public:
  void begin() {
    for (Finger& f : _fingers) f.begin();
    _leds.begin();
    Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ_HZ);
    initSensors(millis(), true);
  }

  void tick(uint32_t now) {
    sampleCurrents(now);
    handleCommands(now);
    for (Finger& f : _fingers) f.update(now);
    publishStatus();
    updateLeds(now);
  }

 private:
  MotorChannel _motors[claw::kNumFingers] = {
      MotorChannel(A_IN1, A_IN2, A_PWM, FINGER_A_INVERT),
      MotorChannel(B_IN1, B_IN2, B_PWM, FINGER_B_INVERT),
      MotorChannel(C_IN1, C_IN2, C_PWM, FINGER_C_INVERT),
  };
  Finger _fingers[claw::kNumFingers] = {Finger('A', _motors[0]), Finger('B', _motors[1]),
                                        Finger('C', _motors[2])};
  Ina219 _sensors[claw::kNumFingers] = {Ina219(INA_ADDR_A), Ina219(INA_ADDR_B), Ina219(INA_ADDR_C)};
  float _currentmA[claw::kNumFingers] = {};  // filtered |current|
  float _peakmA[claw::kNumFingers] = {};     // since the last command for that finger
  uint32_t _lastSensorRetryMs = 0;
  bool _sensorWasOk[claw::kNumFingers] = {};
  StatusLeds _leds;
  uint8_t _lastCommandSeq = 0;
  claw::CommandResult _lastCommandResult = claw::CommandResult::None;

  static uint8_t maskOrAll(uint8_t mask) { return mask ? (mask & 0x07) : 0x07; }

  void initSensors(uint32_t now, bool report) {
    for (int i = 0; i < claw::kNumFingers; i++) {
      Ina219& ina = _sensors[i];
      if (ina.ok()) continue;
      bool ok = ina.begin(Wire, SHUNT_OHMS, INA_MAX_CURRENT_A);
      if (ok)
        shared::logEvent("INA219 %c (0x%02X) ok: range %.0f mA, resolution %.3f mA", 'A' + i,
                         ina.address(), ina.rangemA(), ina.lsbmA());
      else if (report)
        shared::logEvent("INA219 %c (0x%02X) NOT FOUND - finger %c locked out", 'A' + i,
                         ina.address(), 'A' + i);
      _fingers[i].setSensorOk(ok, now);
    }
    _lastSensorRetryMs = now;
  }

  void sampleCurrents(uint32_t now) {
    bool anyFailed = false;
    for (int i = 0; i < claw::kNumFingers; i++) {
      Ina219& ina = _sensors[i];
      if (ina.ok()) ina.sample();
      if (!ina.ok()) {
        if (_sensorWasOk[i]) shared::logEvent("INA219 %c stopped responding", 'A' + i);
        _sensorWasOk[i] = false;
        _fingers[i].setSensorOk(false, now);
        anyFailed = true;
        continue;
      }
      _sensorWasOk[i] = true;
      float mag = fabsf(ina.currentmA());
      _currentmA[i] += CURRENT_FILTER_ALPHA * (mag - _currentmA[i]);
      _peakmA[i] = max(_peakmA[i], _currentmA[i]);
    }
    if (anyFailed && now - _lastSensorRetryMs >= SENSOR_RETRY_MS) initSensors(now, false);
  }

  void resetPeaks(uint8_t mask) {
    for (int i = 0; i < claw::kNumFingers; i++)
      if (mask & (1 << i)) _peakmA[i] = 0;
  }

  void handleCommands(uint32_t now) {
    ClawCommand cmd;
    while (xQueueReceive(shared::commandQueue, &cmd, 0) == pdTRUE) {
      claw::CommandResult r = execute(cmd, now);
      if (r != claw::CommandResult::Accepted) shared::logEvent("command %d rejected", (int)cmd.type);
      if (cmd.reportResult) {
        _lastCommandSeq = cmd.seq;
        _lastCommandResult = r;
      }
    }
  }

  claw::CommandResult execute(const ClawCommand& cmd, uint32_t now) {
    uint8_t mask = maskOrAll(cmd.fingerMask);
    switch (cmd.type) {
      case claw::Command::MoveFinger:
        if (cmd.arg == 0 || cmd.effortPct == 0 || cmd.durationMs == 0)
          return claw::CommandResult::BadArgument;
        resetPeaks(mask);
        for (int i = 0; i < claw::kNumFingers; i++)
          if (mask & (1 << i)) _fingers[i].move(cmd.arg, cmd.effortPct / 100.0f, cmd.durationMs, now);
        shared::logEvent("move mask %d %s %d%% for %d ms", mask, cmd.arg > 0 ? "close" : "open",
                         cmd.effortPct, cmd.durationMs);
        return claw::CommandResult::Accepted;

      case claw::Command::Stop:
        for (int i = 0; i < claw::kNumFingers; i++)
          if (mask & (1 << i)) _fingers[i].stop(now);
        shared::logEvent("stop");
        return claw::CommandResult::Accepted;

      default:
        return claw::CommandResult::Rejected;
    }
  }

  void publishStatus() {
    ClawStatus s;
    for (int i = 0; i < claw::kNumFingers; i++) {
      s.fingerState[i] = _fingers[i].state();
      s.currentmA[i] = _currentmA[i];
      s.peakmA[i] = _peakmA[i];
      if (!_sensors[i].ok()) {
        s.flags |= claw::kFlagSensorFault;
        s.fingerFault[i] = claw::FingerFault::NoSensor;
      }
    }
    s.state = summaryState(s);
    s.lastCommandSeq = _lastCommandSeq;
    s.lastCommandResult = _lastCommandResult;
    shared::writeStatus(s);
  }

  static claw::ClawState summaryState(const ClawStatus& s) {
    bool moving = false;
    for (int i = 0; i < claw::kNumFingers; i++) moving |= s.fingerState[i] == claw::FingerState::Moving;
    return moving ? claw::ClawState::Moving : claw::ClawState::Idle;
  }

  void updateLeds(uint32_t now) {
    ClawStatus s = shared::readStatus();
    using P = StatusLeds::Pattern;
    P green = s.state == claw::ClawState::Moving ? P::FastBlink : P::On;
    P red = (s.flags & claw::kFlagSensorFault) ? P::SlowBlink : P::Off;
    _leds.set(green, red);
    _leds.update(now);
  }
};

#endif
