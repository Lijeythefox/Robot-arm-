#ifndef STATUS_LEDS_HPP
#define STATUS_LEDS_HPP

#include <Arduino.h>

#include "Configuration.h"

// Red and green LEDs, each showing one pattern. Patterns are derived from the claw state by
// the caller (see README for the meaning of each one).
class StatusLeds {
 public:
  enum class Pattern : uint8_t {
    Off,
    On,
    SlowBlink,    // 1 Hz
    FastBlink,    // 5 Hz
    DoubleBlink,  // two short flashes per second
    Alternate,    // 2 Hz, green and red take turns (set on both LEDs)
  };

  void begin() {
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, LOW);
  }

  void set(Pattern green, Pattern red) {
    _green = green;
    _red = red;
  }

  void update(uint32_t now) {
    digitalWrite(LED_GREEN, level(_green, now, false));
    digitalWrite(LED_RED, level(_red, now, true));
  }

 private:
  Pattern _green = Pattern::Off;
  Pattern _red = Pattern::Off;

  static bool level(Pattern p, uint32_t now, bool isRed) {
    switch (p) {
      case Pattern::Off: return false;
      case Pattern::On: return true;
      case Pattern::SlowBlink: return (now % 1000) < 500;
      case Pattern::FastBlink: return (now % 200) < 100;
      case Pattern::DoubleBlink: {
        uint32_t t = now % 1000;
        return t < 80 || (t >= 200 && t < 280);
      }
      case Pattern::Alternate: return ((now % 500) < 250) != isRed;
    }
    return false;
  }
};

#endif
