#ifndef SERIAL_CONSOLE_HPP
#define SERIAL_CONSOLE_HPP

#include <Arduino.h>
#include <Wire.h>

#include "ClawShared.hpp"
#include "Configuration.h"

// USB serial console (115200, newline-terminated). Runs in loop() at low priority and prints
// the control task's event log.
class SerialConsole {
 public:
  void poll() {
    EventText ev;
    while (xQueueReceive(shared::eventQueue, &ev, 0) == pdTRUE) Serial.printf("[claw] %s\n", ev.text);

    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\r') continue;
      if (c == '\n') {
        _line[_len] = 0;
        if (_len) handleLine(_line);
        _len = 0;
      } else if (_len < sizeof(_line) - 1) {
        _line[_len++] = c;
      }
    }
  }

  static void printHelp() {
    Serial.println(
        "Commands (finger = a, b, c or all):\n"
        "  status                                  finger states and currents\n"
        "  i2c                                     scan the I2C bus\n"
        "  move <finger> <close|open> <pct> <ms>   timed move at <pct>% PWM (max 5 s)\n"
        "  stop [finger]                           brake\n"
        "  help");
  }

 protected:
  char _line[96];
  size_t _len = 0;

  static const char* fingerStateName(claw::FingerState s) {
    switch (s) {
      case claw::FingerState::Idle: return "idle";
      case claw::FingerState::Braked: return "braked";
      case claw::FingerState::Opening: return "opening";
      case claw::FingerState::Open: return "open";
      case claw::FingerState::Closing: return "closing";
      case claw::FingerState::Gripped: return "GRIPPED";
      case claw::FingerState::Moving: return "moving";
      case claw::FingerState::Fault: return "FAULT";
      case claw::FingerState::Disabled: return "disabled";
    }
    return "?";
  }

  // "a"/"b"/"c"/"all" -> mask, 0 on error
  static uint8_t parseFinger(const char* s) {
    String f = s;
    f.toLowerCase();
    if (f == "a") return 1;
    if (f == "b") return 2;
    if (f == "c") return 4;
    if (f == "all") return 7;
    return 0;
  }

  static void send(const ClawCommand& cmd) {
    if (!shared::postCommand(cmd)) Serial.println("command queue full");
  }

  void handleLine(char* line) {
    char* argv[6];
    int argc = 0;
    for (char* tok = strtok(line, " \t"); tok && argc < 6; tok = strtok(nullptr, " \t")) argv[argc++] = tok;
    if (!argc) return;
    String cmd = argv[0];
    cmd.toLowerCase();
    ClawCommand c;

    if (cmd == "help" || cmd == "?") {
      printHelp();
    } else if (cmd == "status") {
      printStatus();
    } else if (cmd == "i2c") {
      // Read-only probe. Runs at low priority while the control task also uses the bus;
      // the Wire driver serialises transactions.
      Serial.print("I2C devices:");
      for (uint8_t a = 1; a < 127; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) Serial.printf(" 0x%02X", a);
      }
      Serial.println(" (expect 0x40 0x41 0x44)");
    } else if (cmd == "move" && argc == 5) {
      c.type = claw::Command::MoveFinger;
      c.fingerMask = parseFinger(argv[1]);
      String dir = argv[2];
      c.arg = dir == "close" ? 1 : dir == "open" ? -1 : 0;
      c.effortPct = constrain(atoi(argv[3]), 0, 100);
      c.durationMs = constrain(atoi(argv[4]), 0, MAX_MANUAL_MOVE_MS);
      if (!c.fingerMask || !c.arg) Serial.println("usage: move <a|b|c|all> <close|open> <pct> <ms>");
      else send(c);
    } else if (cmd == "stop" || cmd == "s") {
      c.type = claw::Command::Stop;
      c.fingerMask = argc > 1 ? parseFinger(argv[1]) : 0;
      send(c);
    } else {
      Serial.println("unknown command or wrong arguments - type 'help'");
    }
  }

  static void printStatus() {
    ClawStatus s = shared::readStatus();
    Serial.printf("WiFi %s\n", shared::wifiConnected ? "connected" : "not connected");
    for (int i = 0; i < claw::kNumFingers; i++)
      Serial.printf("  %c: %-8s %7.1f mA (peak %7.1f)%s\n", 'A' + i, fingerStateName(s.fingerState[i]),
                    s.currentmA[i], s.peakmA[i],
                    s.fingerFault[i] == claw::FingerFault::NoSensor ? "  NO SENSOR" : "");
  }
};

#endif
