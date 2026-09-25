#ifndef ENDSTOPS_HPP
#define ENDSTOPS_HPP

#include <Arduino.h>

#include "Configuration.h"

// Six normally-closed endstops: LOW = OK, HIGH = at limit or broken wire.
// update() is called once per control tick; a new level only counts after it has been read
// ENDSTOP_DEBOUNCE_MS times in a row. Disabled endstops always read "not triggered".
class Endstops {
 public:
  static constexpr int kCount = 6;

  struct Input {
    uint8_t pin;
    bool enabled;
    bool pullup;
    int8_t homeDir;  // joint direction in which the switch is reached
  };

  void begin() {
    for (int i = 0; i < kCount; i++) {
      const Input& in = kInputs[i];
      if (!in.enabled) continue;
      pinMode(in.pin, in.pullup ? INPUT_PULLUP : INPUT);
      _raw[i] = digitalRead(in.pin);
      _stable[i] = _raw[i];  // trust the first reading; the RC filter has settled by now
      _count[i] = 0;
    }
  }

  void update() {
    for (int i = 0; i < kCount; i++) {
      if (!kInputs[i].enabled) continue;
      bool level = digitalRead(kInputs[i].pin);
      _raw[i] = level;
      if (level == _stable[i]) {
        _count[i] = 0;
      } else if (++_count[i] >= ENDSTOP_DEBOUNCE_MS / CONTROL_TICK_MS) {
        _stable[i] = level;
        _count[i] = 0;
      }
    }
  }

  // axis is 1..6
  bool enabled(int axis) const { return kInputs[axis - 1].enabled; }
  bool triggered(int axis) const { return kInputs[axis - 1].enabled && _stable[axis - 1]; }
  bool rawLevel(int axis) const { return _raw[axis - 1]; }
  int homeDir(int axis) const { return kInputs[axis - 1].homeDir; }

  uint8_t triggeredMask() const {
    uint8_t m = 0;
    for (int a = 1; a <= kCount; a++)
      if (triggered(a)) m |= 1 << (a - 1);
    return m;
  }
  uint8_t enabledMask() const {
    uint8_t m = 0;
    for (int a = 1; a <= kCount; a++)
      if (enabled(a)) m |= 1 << (a - 1);
    return m;
  }

 private:
  static constexpr Input kInputs[kCount] = {
      {AXIS_1_ENDSTOP_PIN, AXIS_1_ENDSTOP_ENABLED, AXIS_1_ENDSTOP_PULLUP, AXIS_1_HOME_DIR},
      {AXIS_2_ENDSTOP_PIN, AXIS_2_ENDSTOP_ENABLED, AXIS_2_ENDSTOP_PULLUP, AXIS_2_HOME_DIR},
      {AXIS_3_ENDSTOP_PIN, AXIS_3_ENDSTOP_ENABLED, AXIS_3_ENDSTOP_PULLUP, AXIS_3_HOME_DIR},
      {AXIS_4_ENDSTOP_PIN, AXIS_4_ENDSTOP_ENABLED, AXIS_4_ENDSTOP_PULLUP, AXIS_4_HOME_DIR},
      {AXIS_5_ENDSTOP_PIN, AXIS_5_ENDSTOP_ENABLED, AXIS_5_ENDSTOP_PULLUP, AXIS_5_HOME_DIR},
      {AXIS_6_ENDSTOP_PIN, AXIS_6_ENDSTOP_ENABLED, AXIS_6_ENDSTOP_PULLUP, AXIS_6_HOME_DIR},
  };
  static_assert(ENDSTOP_DEBOUNCE_MS >= CONTROL_TICK_MS, "debounce shorter than one tick");

  bool _raw[kCount] = {};
  bool _stable[kCount] = {};
  uint8_t _count[kCount] = {};
};

#endif
