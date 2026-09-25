#ifndef SERIAL_CONSOLE_HPP
#define SERIAL_CONSOLE_HPP

#include <Arduino.h>

#include "Configuration.h"
#include "OledDisplay.hpp"
#include "SharedState.hpp"

// USB serial console (115200 baud, newline-terminated commands) for bench testing without ROS.
// Runs in a low-priority task on core 0; also prints the motion loop's event log.
class SerialConsole {
 public:
  void poll() {
    EventText ev;
    while (xQueueReceive(shared::eventQueue, &ev, 0) == pdTRUE) Serial.printf("[arm] %s\n", ev.text);

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

    if (_monitorEndstops && millis() - _lastMonitorMs >= 200) {
      _lastMonitorMs = millis();
      ArmStatus s = shared::readStatus();
      Serial.print("[endstops] ");
      for (int a = 1; a <= 6; a++) {
        uint8_t bit = 1 << (a - 1);
        Serial.printf("A%d:%s ", a,
                      !(s.endstopEnabledMask & bit) ? "off" : (s.endstopMask & bit) ? "HIT" : "ok");
      }
      Serial.printf(" net:%s\n", s.enableNetHigh ? "HIGH" : "LOW");
    }
  }

  static void printHelp() {
    Serial.println(
        "Commands:\n"
        "  status              show state, positions, endstops, faults\n"
        "  endstops            toggle live endstop monitor (5 Hz)\n"
        "  enable | disable    wake / sleep all drivers\n"
        "  jog <axis> <deg>    relative move of one axis (soft limits apply)\n"
        "  stop                decelerate all axes to a stop\n"
        "  help");
  }

 private:
  char _line[96];
  size_t _len = 0;
  bool _monitorEndstops = false;
  uint32_t _lastMonitorMs = 0;

  static void send(arm::Command type, uint8_t axis = 0, int32_t p0 = 0, int32_t p1 = 0,
                   int32_t p2 = 0) {
    MotionCommand cmd;
    cmd.type = type;
    cmd.axis = axis;
    cmd.param[0] = p0;
    cmd.param[1] = p1;
    cmd.param[2] = p2;
    if (!shared::postCommand(cmd)) Serial.println("command queue full");
  }

  void handleLine(char* line) {
    char* argv[6];
    int argc = 0;
    for (char* tok = strtok(line, " \t"); tok && argc < 6; tok = strtok(nullptr, " \t")) argv[argc++] = tok;
    if (!argc) return;
    String cmd = argv[0];
    cmd.toLowerCase();

    if (cmd == "help" || cmd == "?") {
      printHelp();
    } else if (cmd == "status") {
      printStatus();
    } else if (cmd == "endstops") {
      _monitorEndstops = !_monitorEndstops;
      Serial.printf("endstop monitor %s\n", _monitorEndstops ? "on" : "off");
    } else if (cmd == "enable") {
      send(arm::Command::EnableDrivers);
    } else if (cmd == "disable") {
      send(arm::Command::DisableDrivers);
    } else if (cmd == "stop" || cmd == "s") {
      send(arm::Command::Stop);
    } else if (cmd == "jog" && argc == 3) {
      send(arm::Command::Jog, atoi(argv[1]), (int32_t)lround(atof(argv[2]) * 1000.0));
    } else {
      Serial.println("unknown command or wrong arguments - type 'help'");
    }
  }

  static void printStatus() {
    ArmStatus s = shared::readStatus();
    Serial.printf("state %s, drivers %s, ENABLE net %s, PC active %d, WiFi %d\n",
                  OledDisplay::stateName(s.state), s.driversEnabled ? "on" : "off",
                  s.enableNetHigh ? "HIGH" : "LOW", s.active, (int)shared::wifiConnected);
    for (int a = 1; a <= 6; a++) {
      uint8_t bit = 1 << (a - 1);
      Serial.printf("  A%d %9.3f deg  homed:%s  endstop:%s\n", a, s.positionMilliDeg[a - 1] * 1e-3,
                    (s.homedMask & bit) ? "yes" : "no",
                    !(s.endstopEnabledMask & bit) ? "off" : (s.endstopMask & bit) ? "HIT" : "ok");
    }
    Serial.printf("  faults 0x%02X\n", s.faultFlags);
  }
};

#endif
