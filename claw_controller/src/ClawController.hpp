#ifndef CLAW_CONTROLLER_HPP
#define CLAW_CONTROLLER_HPP

#include <Arduino.h>

#include "ClawShared.hpp"
#include "Configuration.h"
#include "Finger.hpp"
#include "MotorDriver.hpp"
#include "StatusLeds.hpp"

// Runs in the high-priority control task every CONTROL_PERIOD_MS: commands, finger state
// machines, status. The only code that touches the motors.
class ClawController {
 public:
  void begin() {
    for (Finger& f : _fingers) f.begin();
    _leds.begin();
  }

  void tick(uint32_t now) {
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
  StatusLeds _leds;
  uint8_t _lastCommandSeq = 0;
  claw::CommandResult _lastCommandResult = claw::CommandResult::None;

  static uint8_t maskOrAll(uint8_t mask) { return mask ? (mask & 0x07) : 0x07; }

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
    for (int i = 0; i < claw::kNumFingers; i++) s.fingerState[i] = _fingers[i].state();
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
    _leds.set(green, P::Off);
    _leds.update(now);
  }
};

#endif
