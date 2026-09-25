#ifndef OledDisplay_HPP_
#define OledDisplay_HPP_

#include <Arduino.h>
#include <Wire.h>

#include "Configuration.h"
#include "Homing.hpp"
#include "SSD1306Wire.h"
#include "SharedState.hpp"

// Renders the status snapshot. Called from its own low-priority task at OLED_REFRESH_MS:
// the original redrew the display inside the network callback on every message, which stalled
// the network task for tens of milliseconds per frame.
class OledDisplay {
 public:
  OledDisplay() : _display(OLED_ADDRESS, OLED_SDA, OLED_SCL) {}

  void begin() {
    _display.init();
    if (OLED_FLIP_VERTICAL) _display.flipScreenVertically();
    _display.setFont(ArialMT_Plain_10);
    _display.setTextAlignment(TEXT_ALIGN_LEFT);
  }

  void showMessage(const char* line1, const char* line2 = "") {
    _display.clear();
    _display.drawString(0, 0, line1);
    _display.drawString(0, 12, line2);
    _display.display();
  }

  void render(const ArmStatus& s, bool wifi) {
    char line[40];
    _display.clear();

    snprintf(line, sizeof(line), "%s  %s  %s", stateName(s.state), wifi ? "WiFi" : "no WiFi",
             s.active ? "PC on" : "PC off");
    _display.drawString(0, 0, line);

    statusLine(s, line, sizeof(line));
    _display.drawString(0, 12, line);

    for (int row = 0; row < 3; row++) {
      int a = row * 2;
      snprintf(line, sizeof(line), "A%d %7.2f  A%d %7.2f", a + 1, s.positionMilliDeg[a] * 1e-3,
               a + 2, s.positionMilliDeg[a + 1] * 1e-3);
      _display.drawString(0, 24 + row * 13, line);
    }
    _display.display();
  }

  static const char* stateName(arm::State st) {
    switch (st) {
      case arm::State::Disabled: return "OFF";
      case arm::State::Ready: return "READY";
      case arm::State::Homing: return "HOMING";
      case arm::State::SpeedTest: return "SPEEDTEST";
      case arm::State::EStop: return "E-STOP";
    }
    return "?";
  }

 private:
  SSD1306Wire _display;

  // Second line: the most important fault, otherwise endstop / homing overview.
  static void statusLine(const ArmStatus& s, char* out, size_t len) {
    if (s.faultFlags & arm::kFaultSoftEStop) {
      snprintf(out, len, "PC E-STOP: send reset");
    } else if (s.faultFlags & arm::kFaultEStopLatched) {
      snprintf(out, len, "E-STOP: close it, reset");
    } else if (s.faultFlags & arm::kFaultEStopOpen) {
      snprintf(out, len, "E-STOP OPEN");
    } else if (s.state == arm::State::Homing) {
      snprintf(out, len, "Homing A%d: %s", s.homingAxis, Homing::phaseName(s.homingPhase));
    } else if (s.faultFlags & arm::kFaultHomingFailed) {
      snprintf(out, len, "HOMING FAILED A%d", s.homingAxis);
    } else if (s.faultFlags & arm::kFaultCommTimeout) {
      snprintf(out, len, "PC stream lost - holding");
    } else if (s.faultFlags & arm::kFaultEndstopHit) {
      snprintf(out, len, "Endstop hit: %s", axisList(s.endstopMask).c_str());
    } else {
      snprintf(out, len, "H:%s E:%s", homedSummary(s).c_str(), endstopSummary(s).c_str());
    }
  }

  static String axisList(uint8_t mask) {
    String r;
    for (int a = 1; a <= 6; a++)
      if (mask & (1 << (a - 1))) r += String(a) + " ";
    return r.length() ? r : String("-");
  }

  // One character per axis: axis number if homed, '.' if not.
  static String homedSummary(const ArmStatus& s) {
    String r;
    for (int a = 0; a < 6; a++) r += (s.homedMask & (1 << a)) ? char('1' + a) : '.';
    return r;
  }

  // One character per axis: '.' ok, 'X' triggered, '-' not fitted.
  static String endstopSummary(const ArmStatus& s) {
    String r;
    for (int a = 0; a < 6; a++) {
      if (!(s.endstopEnabledMask & (1 << a))) r += '-';
      else r += (s.endstopMask & (1 << a)) ? 'X' : '.';
    }
    return r;
  }
};

#endif
