#ifndef HOMING_HPP
#define HOMING_HPP

#include <Arduino.h>
#include <math.h>

#include "Axis.hpp"
#include "Configuration.h"
#include "Endstops.hpp"
#include "SharedState.hpp"

enum class HomingPhase : uint8_t {
  Idle = 0,
  ClearSwitch,   // started on the switch: back off until it releases
  FastApproach,
  FastStop,      // switch seen: wait for the stop to complete
  Backoff,
  SlowApproach,
  Latch,         // switch seen on the slow approach: wait for the stop, then set the position
  FinalBackoff,  // leave the switch released
  Park,
  Failed,
};

// Homes one axis or a list of axes, one at a time:
// fast approach -> back off -> slow re-approach -> set position -> back off (-> park).
// tick() must only be called while the drivers are awake.
class Homing {
 public:
  struct Params {
    double switchDeg, fastDegS, slowDegS, backoffDeg, maxTravelDeg, parkDeg;
    float maxSpeed, maxAccel;  // the unhomed limits; homing never exceeds them
  };

  Homing(Endstops& endstops, uint8_t& homedMask) : _endstops(endstops), _homedMask(homedMask) {}

  // Returns false (and logs why) if none of the requested axes can be homed.
  bool start(const uint8_t* axes, int count) {
    _count = 0;
    for (int i = 0; i < count && _count < 6; i++) {
      int n = axes[i];
      if (n < 1 || n > 6) continue;
      if (!_endstops.enabled(n)) {
        shared::logEvent("A%d has no endstop enabled - skipped", n);
        continue;
      }
      _queue[_count++] = n;
    }
    _index = 0;
    _failed = false;
    if (_count == 0) {
      shared::logEvent("homing: nothing to home");
      return false;
    }
    beginAxis(_queue[0]);
    return true;
  }

  void abort(const char* reason) {
    if (!active()) return;
    Axis::axisList[_axis - 1]->stop();
    fail("homing aborted: %s", reason);
  }

  bool active() const { return _phase != HomingPhase::Idle && _phase != HomingPhase::Failed; }
  bool failed() const { return _failed; }
  uint8_t axis() const { return _axis; }
  HomingPhase phase() const { return _phase; }

  // Advance the state machine. Returns true on the tick the whole sequence finishes
  // (successfully or not).
  bool tick() {
    if (!active()) return false;
    Axis& a = *Axis::axisList[_axis - 1];
    const Params& p = kParams[_axis - 1];
    int dir = _endstops.homeDir(_axis);
    bool hit = _endstops.triggered(_axis);
    double travelled = fabs(a.getPosition() - _startDeg);

    switch (_phase) {
      case HomingPhase::ClearSwitch:
        if (a.isRunning()) break;
        if (hit) return fail("A%d switch still pressed after backing off (stuck or broken wire?)", _axis);
        startFastApproach(a);
        break;

      case HomingPhase::FastApproach:
        if (hit) {
          a.hardStop();
          _phase = HomingPhase::FastStop;
        } else if (travelled > p.maxTravelDeg) {
          a.hardStop();
          return fail("A%d switch not found within %.0f deg", _axis, p.maxTravelDeg);
        }
        break;

      case HomingPhase::FastStop:
        if (a.isRunning()) break;
        a.moveRelative(-dir * p.backoffDeg);
        _phase = HomingPhase::Backoff;
        break;

      case HomingPhase::Backoff:
        if (a.isRunning()) break;
        if (hit) return fail("A%d switch did not release after %.1f deg back-off", _axis, p.backoffDeg);
        setSpeedDegS(a, p.slowDegS);
        _startDeg = a.getPosition();
        a.runContinuous(dir);
        _phase = HomingPhase::SlowApproach;
        break;

      case HomingPhase::SlowApproach:
        if (hit) {
          _triggerDeg = a.getPosition();
          a.hardStop();
          _phase = HomingPhase::Latch;
        } else if (travelled > 2 * p.backoffDeg + 2) {
          a.hardStop();
          return fail("A%d switch not found on slow approach", _axis);
        }
        break;

      case HomingPhase::Latch:
        // A hard stop may still emit a few queued steps; count them from the trigger point.
        if (a.isRunning()) break;
        a.setCurrentPosition(p.switchDeg + (a.getPosition() - _triggerDeg));
        _homedMask |= 1 << (_axis - 1);
        shared::logEvent("A%d homed (switch at %.2f deg)", _axis, p.switchDeg);
        setSpeedDegS(a, p.fastDegS);
        a.moveRelative(-dir * p.backoffDeg);
        _phase = HomingPhase::FinalBackoff;
        break;

      case HomingPhase::FinalBackoff:
        if (a.isRunning()) break;
        if (HOMING_PARK_AFTER) {
          a.moveToPosition(p.parkDeg);
          _phase = HomingPhase::Park;
          break;
        }
        return nextAxis();

      case HomingPhase::Park:
        if (a.isRunning()) break;
        return nextAxis();

      default:
        break;
    }
    return false;
  }

  static const char* phaseName(uint8_t phase) {
    switch ((HomingPhase)phase) {
      case HomingPhase::ClearSwitch: return "clear";
      case HomingPhase::FastApproach: return "fast";
      case HomingPhase::FastStop: return "fast";
      case HomingPhase::Backoff: return "back off";
      case HomingPhase::SlowApproach: return "slow";
      case HomingPhase::Latch: return "latch";
      case HomingPhase::FinalBackoff: return "back off";
      case HomingPhase::Park: return "park";
      case HomingPhase::Failed: return "FAILED";
      default: return "";
    }
  }

 private:
  static constexpr Params kParams[6] = {
      {AXIS_1_HOME_SWITCH_DEG, AXIS_1_HOME_FAST_DEG_S, AXIS_1_HOME_SLOW_DEG_S, AXIS_1_HOME_BACKOFF_DEG, AXIS_1_HOME_MAX_TRAVEL_DEG, AXIS_1_HOME_PARK_DEG, AXIS_1_UNHOMED_VEL_MAX, AXIS_1_UNHOMED_ACC_MAX},
      {AXIS_2_HOME_SWITCH_DEG, AXIS_2_HOME_FAST_DEG_S, AXIS_2_HOME_SLOW_DEG_S, AXIS_2_HOME_BACKOFF_DEG, AXIS_2_HOME_MAX_TRAVEL_DEG, AXIS_2_HOME_PARK_DEG, AXIS_2_UNHOMED_VEL_MAX, AXIS_2_UNHOMED_ACC_MAX},
      {AXIS_3_HOME_SWITCH_DEG, AXIS_3_HOME_FAST_DEG_S, AXIS_3_HOME_SLOW_DEG_S, AXIS_3_HOME_BACKOFF_DEG, AXIS_3_HOME_MAX_TRAVEL_DEG, AXIS_3_HOME_PARK_DEG, AXIS_3_UNHOMED_VEL_MAX, AXIS_3_UNHOMED_ACC_MAX},
      {AXIS_4_HOME_SWITCH_DEG, AXIS_4_HOME_FAST_DEG_S, AXIS_4_HOME_SLOW_DEG_S, AXIS_4_HOME_BACKOFF_DEG, AXIS_4_HOME_MAX_TRAVEL_DEG, AXIS_4_HOME_PARK_DEG, AXIS_4_UNHOMED_VEL_MAX, AXIS_4_UNHOMED_ACC_MAX},
      {AXIS_5_HOME_SWITCH_DEG, AXIS_5_HOME_FAST_DEG_S, AXIS_5_HOME_SLOW_DEG_S, AXIS_5_HOME_BACKOFF_DEG, AXIS_5_HOME_MAX_TRAVEL_DEG, AXIS_5_HOME_PARK_DEG, AXIS_5_UNHOMED_VEL_MAX, AXIS_5_UNHOMED_ACC_MAX},
      {AXIS_6_HOME_SWITCH_DEG, AXIS_6_HOME_FAST_DEG_S, AXIS_6_HOME_SLOW_DEG_S, AXIS_6_HOME_BACKOFF_DEG, AXIS_6_HOME_MAX_TRAVEL_DEG, AXIS_6_HOME_PARK_DEG, AXIS_6_UNHOMED_VEL_MAX, AXIS_6_UNHOMED_ACC_MAX},
  };

  Endstops& _endstops;
  uint8_t& _homedMask;
  uint8_t _queue[6] = {};
  int _count = 0;
  int _index = 0;
  uint8_t _axis = 0;
  HomingPhase _phase = HomingPhase::Idle;
  bool _failed = false;
  double _startDeg = 0;
  double _triggerDeg = 0;

  void setSpeedDegS(Axis& a, double degPerSec) {
    const Params& p = kParams[_axis - 1];
    float steps = min((float)(degPerSec * a.stepsPerDegree()), p.maxSpeed);
    // Reach the approach speed in ~0.25 s, but never faster than the unhomed limit allows.
    a.setSpeedLimits(steps, min(steps * 4, p.maxAccel));
  }

  void beginAxis(uint8_t n) {
    _axis = n;
    _homedMask &= ~(1 << (n - 1));  // the old position frame is no longer trusted
    Axis& a = *Axis::axisList[n - 1];
    const Params& p = kParams[n - 1];
    setSpeedDegS(a, p.fastDegS);
    shared::logEvent("homing A%d", n);
    if (_endstops.triggered(n)) {
      a.moveRelative(-_endstops.homeDir(n) * 2 * p.backoffDeg);
      _phase = HomingPhase::ClearSwitch;
    } else {
      startFastApproach(a);
    }
  }

  void startFastApproach(Axis& a) {
    setSpeedDegS(a, kParams[_axis - 1].fastDegS);
    _startDeg = a.getPosition();
    a.runContinuous(_endstops.homeDir(_axis));
    _phase = HomingPhase::FastApproach;
  }

  bool nextAxis() {
    if (++_index < _count) {
      beginAxis(_queue[_index]);
      return false;
    }
    _phase = HomingPhase::Idle;
    shared::logEvent("homing complete");
    return true;
  }

  bool fail(const char* fmt, ...) __attribute__((format(printf, 2, 3))) {
    EventText ev;
    va_list args;
    va_start(args, fmt);
    vsnprintf(ev.text, sizeof(ev.text), fmt, args);
    va_end(args);
    shared::logEvent("HOMING FAILED: %s", ev.text);
    _phase = HomingPhase::Failed;
    _failed = true;
    return true;
  }
};

#endif
