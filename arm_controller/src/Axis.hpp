#ifndef AXIS_HPP
#define AXIS_HPP

#include <Arduino.h>
#include <AccelStepper.h>

#include <vector>

#include "Configuration.h"

// One joint driven by a DRV8825 (STEP/DIR). The public interface of the original class
// (constructor, getPosition, moveToPosition, runMotor, runAll, axisList) is unchanged; the
// extra methods are what homing and the safety checks need.
//
// Every method must be called from the motion loop (core 1) only. Other tasks read positions
// from the shared status snapshot instead.
class Axis {
 private:
  AccelStepper _stepper;
  bool _invertDirection = false;
  double _gearing = -1;
  double _microsteps = -1;
  double _stepsPerRev = -1;
  int _axisNumber = -1;  // 1...6, -1 = not initialised
  double _minPos;
  double _maxPos;
  float _maxSpeed = 0;
  float _acceleration = 0;
  int8_t _continuousDir = 0;  // joint direction of a runContinuous() move, 0 otherwise

  int _sign() const { return _invertDirection ? -1 : 1; }

 public:
  Axis(int number, uint8_t stepPin, uint8_t directionPin, double gearing, int stepsPerRev,
       int microsteps, int minPos, int maxPos, int velMax, int accMax,
       bool invertDirection = false)
      : _stepper(AccelStepper::DRIVER, stepPin, directionPin) {
    _axisNumber = number;
    _invertDirection = invertDirection;
    _gearing = gearing;
    _microsteps = microsteps;
    _stepsPerRev = stepsPerRev;
    _minPos = minPos;
    _maxPos = maxPos;
    setSpeedLimits(velMax, accMax);
    Axis::axisList.push_back(this);
  }

  // Hardware setup that cannot run in a global constructor. Nothing to do for AccelStepper.
  bool begin() { return true; }
  static bool beginAll() {
    bool ok = true;
    for (Axis* axis : axisList) ok &= axis->begin();
    return ok;
  }

  int number() const { return _axisNumber; }
  double minPosition() const { return _minPos; }
  double maxPosition() const { return _maxPos; }
  double stepsPerDegree() const { return _stepsPerRev * _microsteps * _gearing / 360.0; }
  long degreesToSteps(double deg) const { return lround(deg * stepsPerDegree() * _sign()); }
  double stepsToDegrees(long steps) const { return steps / stepsPerDegree() * _sign(); }

  // Current position in degrees (from counted steps, not measured).
  double getPosition() { return stepsToDegrees(_stepper.currentPosition()); }

  // Move to an absolute position in degrees, clamped to the soft limits.
  void moveToPosition(double position) {
    if (position > _maxPos) position = _maxPos;
    else if (position < _minPos) position = _minPos;
    moveToPositionUnlimited(position);
  }

  // Homing and speed tests need to go past the soft limits.
  void moveToPositionUnlimited(double position) {
    _continuousDir = 0;
    _stepper.moveTo(degreesToSteps(position));
  }

  void moveRelative(double degrees) { moveToPositionUnlimited(getPosition() + degrees); }

  // Run in one joint direction (+1/-1) until stopped. Used for the homing approach.
  void runContinuous(int jointDir) {
    _continuousDir = jointDir > 0 ? 1 : -1;
    long far = 1000000000L * _continuousDir * _sign();
    _stepper.moveTo(_stepper.currentPosition() + far);
  }

  // Decelerate to a stop with the configured acceleration.
  void stop() {
    _continuousDir = 0;
    _stepper.stop();
  }

  // Stop without deceleration (endstop hit, E-stop). Position stays valid.
  void hardStop() {
    _continuousDir = 0;
    _stepper.setCurrentPosition(_stepper.currentPosition());  // also zeroes the speed
  }

  bool isRunning() { return _stepper.isRunning(); }

  // Direction of travel in joint degrees: +1, -1, or 0 when idle.
  int directionOfTravel() {
    if (!_stepper.isRunning()) return 0;
    if (_continuousDir != 0) return _continuousDir;
    long toGo = _stepper.distanceToGo();
    if (toGo == 0) return 0;
    return ((toGo > 0) ? 1 : -1) * _sign();
  }

  // Redefine the current position (homing). Stops any motion.
  void setCurrentPosition(double degrees) {
    _continuousDir = 0;
    _stepper.setCurrentPosition(degreesToSteps(degrees));
  }

  void setSpeedLimits(float stepsPerSecond, float acceleration) {
    _maxSpeed = stepsPerSecond;
    _acceleration = acceleration;
    _stepper.setMaxSpeed(stepsPerSecond);
    _stepper.setAcceleration(acceleration);
  }
  float maxSpeed() const { return _maxSpeed; }

  // Polling function controlling the motor (one step at most per call):
  void runMotor() { _stepper.run(); }

  // Static function to poll all the existing motors:
  static void runAll() {
    for (Axis* axis : Axis::axisList) axis->runMotor();
  }

  // Static list of pointers to all axis instances, in construction order (axis 1 first).
  static std::vector<Axis*> axisList;
};

inline std::vector<Axis*> Axis::axisList;

#endif
