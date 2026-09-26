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
    for (int i = 0; i < claw::kNumFingers; i++) _fingers[i].update(now, _currentmA[i]);
    trackMoveStats(now);
    streamCalibration(now);
    publishStatus();
    updateLeds(now);
  }

 private:
  MotorChannel _motors[claw::kNumFingers] = {
      MotorChannel(A_IN1, A_IN2, A_PWM, FINGER_A_INVERT),
      MotorChannel(B_IN1, B_IN2, B_PWM, FINGER_B_INVERT),
      MotorChannel(C_IN1, C_IN2, C_PWM, FINGER_C_INVERT),
  };
  static_assert(GRIP_CONFIRM_MS < HARD_LIMIT_CONFIRM_MS, "a closing finger must be able to grip before the hard limit trips");
  static_assert(FINGER_A_GRIP_THRESHOLD_MA < FINGER_A_HARD_LIMIT_MA, "A: grip threshold must be below the hard limit");
  static_assert(FINGER_B_GRIP_THRESHOLD_MA < FINGER_B_HARD_LIMIT_MA, "B: grip threshold must be below the hard limit");
  static_assert(FINGER_C_GRIP_THRESHOLD_MA < FINGER_C_HARD_LIMIT_MA, "C: grip threshold must be below the hard limit");
  Finger _fingers[claw::kNumFingers] = {
      Finger('A', _motors[0], {FINGER_A_GRIP_THRESHOLD_MA, FINGER_A_HARD_LIMIT_MA}),
      Finger('B', _motors[1], {FINGER_B_GRIP_THRESHOLD_MA, FINGER_B_HARD_LIMIT_MA}),
      Finger('C', _motors[2], {FINGER_C_GRIP_THRESHOLD_MA, FINGER_C_HARD_LIMIT_MA}),
  };
  bool _gripperState = false;  // legacy: last open (0) / close (1) command
  Ina219 _sensors[claw::kNumFingers] = {Ina219(INA_ADDR_A), Ina219(INA_ADDR_B), Ina219(INA_ADDR_C)};
  float _currentmA[claw::kNumFingers] = {};  // filtered |current|
  float _peakmA[claw::kNumFingers] = {};     // since the last command for that finger
  uint32_t _lastSensorRetryMs = 0;
  bool _sensorWasOk[claw::kNumFingers] = {};
  bool _calibration = false;
  uint32_t _lastCalSampleMs = 0;

  // Per-finger statistics of the current manual move, reported when it ends.
  struct MoveStats {
    bool active = false;
    int dir = 0;
    uint8_t pct = 0;
    uint32_t startMs = 0;
    float sum = 0;
    uint32_t count = 0;
    float peak = 0;
  } _moveStats[claw::kNumFingers];

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

  void trackMoveStats(uint32_t now) {
    for (int i = 0; i < claw::kNumFingers; i++) {
      MoveStats& m = _moveStats[i];
      if (!m.active) continue;
      if (_fingers[i].state() == claw::FingerState::Moving) {
        if (now - m.startMs >= INRUSH_BLANK_MS) {
          m.sum += _currentmA[i];
          m.count++;
        }
        m.peak = max(m.peak, _currentmA[i]);
        continue;
      }
      m.active = false;
      shared::logEvent("%c %s %u%% %lu ms: avg %.0f mA after inrush, peak %.0f mA (incl. inrush)",
                       'A' + i, m.dir > 0 ? "close" : "open", m.pct, (unsigned long)(now - m.startMs),
                       m.count ? m.sum / m.count : 0.0f, m.peak);
    }
  }

  void streamCalibration(uint32_t now) {
    if (!_calibration || now - _lastCalSampleMs < 1000 / CAL_STREAM_HZ) return;
    _lastCalSampleMs = now;
    CalibrationSample c;
    c.ms = now;
    for (int i = 0; i < claw::kNumFingers; i++) {
      c.rawmA[i] = _sensors[i].currentmA();
      c.filteredmA[i] = _currentmA[i];
      c.state[i] = _fingers[i].state();
    }
    xQueueSend(shared::calibrationQueue, &c, 0);  // drop samples rather than block
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
        for (int i = 0; i < claw::kNumFingers; i++) {
          if (!(mask & (1 << i))) continue;
          _fingers[i].move(cmd.arg, cmd.effortPct / 100.0f, cmd.durationMs, now);
          _moveStats[i] = MoveStats();
          _moveStats[i].active = _fingers[i].state() == claw::FingerState::Moving;
          _moveStats[i].dir = cmd.arg;
          _moveStats[i].pct = cmd.effortPct;
          _moveStats[i].startMs = now;
        }
        shared::logEvent("move mask %d %s %d%% for %d ms", mask, cmd.arg > 0 ? "close" : "open",
                         cmd.effortPct, cmd.durationMs);
        return claw::CommandResult::Accepted;

      case claw::Command::Open:
        resetPeaks(mask);
        for (int i = 0; i < claw::kNumFingers; i++)
          if (mask & (1 << i)) _fingers[i].open(now);
        _gripperState = false;
        shared::logEvent("open");
        return claw::CommandResult::Accepted;

      case claw::Command::Close:
      case claw::Command::CloseToForce: {
        bool force = cmd.type == claw::Command::CloseToForce;
        if (force && cmd.targetCurrentmA == 0) return claw::CommandResult::BadArgument;
        resetPeaks(mask);
        for (int i = 0; i < claw::kNumFingers; i++) {
          if (!(mask & (1 << i))) continue;
          Finger& f = _fingers[i];
          if (force) {
            // Keep the target safely below the hard limit.
            float target = min((float)cmd.targetCurrentmA, 0.9f * f.params().hardLimitmA);
            f.close(now, target, FORCE_CLOSE_EFFORT_PCT);
          } else {
            f.close(now);
          }
        }
        _gripperState = true;
        if (force) shared::logEvent("close to %u mA", cmd.targetCurrentmA);
        else shared::logEvent("close");
        return claw::CommandResult::Accepted;
      }

      case claw::Command::ResetFaults:
        for (int i = 0; i < claw::kNumFingers; i++)
          if (mask & (1 << i)) _fingers[i].clearFault(now);
        shared::logEvent("faults cleared");
        return claw::CommandResult::Accepted;

      case claw::Command::Stop:
        for (int i = 0; i < claw::kNumFingers; i++)
          if (mask & (1 << i)) _fingers[i].stop(now);
        shared::logEvent("stop");
        return claw::CommandResult::Accepted;

      case claw::Command::SetCalibration:
        _calibration = cmd.arg != 0;
        shared::logEvent("calibration mode %s", _calibration ? "ON" : "off");
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
      s.fingerFault[i] = _fingers[i].fault();
      if (!_sensors[i].ok()) {
        s.flags |= claw::kFlagSensorFault;
        s.fingerFault[i] = claw::FingerFault::NoSensor;
      }
    }
    s.gripperState = _gripperState;
    if (_calibration) s.flags |= claw::kFlagCalibration;
    s.state = summaryState(s);
    s.lastCommandSeq = _lastCommandSeq;
    s.lastCommandResult = _lastCommandResult;
    shared::writeStatus(s);
  }

  // One state for the whole claw, most urgent first.
  static claw::ClawState summaryState(const ClawStatus& s) {
    int count[9] = {};
    int usable = 0;
    for (int i = 0; i < claw::kNumFingers; i++) {
      count[(int)s.fingerState[i]]++;
      if (s.fingerState[i] != claw::FingerState::Disabled) usable++;
    }
    using F = claw::FingerState;
    if (count[(int)F::Fault]) return claw::ClawState::Fault;
    if (count[(int)F::Closing]) return claw::ClawState::Closing;
    if (count[(int)F::Opening]) return claw::ClawState::Opening;
    if (count[(int)F::Moving]) return claw::ClawState::Moving;
    if (count[(int)F::Gripped]) return claw::ClawState::Gripped;
    if (usable && count[(int)F::Open] == usable) return claw::ClawState::Open;
    return claw::ClawState::Idle;
  }

  void updateLeds(uint32_t now) {
    ClawStatus s = shared::readStatus();
    using P = StatusLeds::Pattern;
    P green = P::On;
    switch (s.state) {
      case claw::ClawState::Closing:
      case claw::ClawState::Opening:
      case claw::ClawState::Moving: green = P::FastBlink; break;
      case claw::ClawState::Gripped: green = P::DoubleBlink; break;
      default: break;
    }
    P red = P::Off;
    if (s.state == claw::ClawState::Fault) red = P::On;
    if (s.flags & claw::kFlagSensorFault) red = P::SlowBlink;
    if (s.flags & claw::kFlagCalibration) green = red = P::Alternate;
    _leds.set(green, red);
    _leds.update(now);
  }
};

#endif
