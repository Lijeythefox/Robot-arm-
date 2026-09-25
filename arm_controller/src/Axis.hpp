#ifndef AXIS_HPP
#define AXIS_HPP

#include <Arduino.h>

#include <vector>

#include "Configuration.h"

// Step generation backend:
//   default           FastAccelStepper - step pulses come from the ESP32's MCPWM/PCNT
//                     hardware, so timing does not depend on how often the loop runs
//   USE_ACCELSTEPPER  the original AccelStepper - pulses are bit-banged from loop();
//                     build the "esp32dev_accelstepper" environment to use it
#ifdef USE_ACCELSTEPPER
#include <AccelStepper.h>
#else
#include <FastAccelStepper.h>
#endif

// One joint driven by a DRV8825 (STEP/DIR). The public interface of the original class
// (constructor, getPosition, moveToPosition, runMotor, runAll, axisList) is unchanged; the
// extra methods are what homing, the speed test and the safety checks need.
//
// Every method must be called from the motion loop (core 1) only. Other tasks read positions
// from the shared status snapshot instead.
class Axis {
 private:
#ifdef USE_ACCELSTEPPER
  AccelStepper _stepper;
#else
  FastAccelStepper* _stepper = nullptr;
  static FastAccelStepperEngine& engine() {
    static FastAccelStepperEngine instance;
    return instance;
  }
#endif
  uint8_t _stepPin, _dirPin;
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
#ifdef USE_ACCELSTEPPER
      : _stepper(AccelStepper::DRIVER, stepPin, directionPin)
#endif
  {
    _stepPin = stepPin;
    _dirPin = directionPin;
    _axisNumber = number;
    _invertDirection = invertDirection;
    _gearing = gearing;
    _microsteps = microsteps;
    _stepsPerRev = stepsPerRev;
    _minPos = minPos;
    _maxPos = maxPos;
    _maxSpeed = velMax;
    _acceleration = accMax;
#ifdef USE_ACCELSTEPPER
    setSpeedLimits(velMax, accMax);
#endif
    Axis::axisList.push_back(this);
  }

  // Hardware setup that cannot run in a global constructor. Call once from setup().
  bool begin() {
#ifdef USE_ACCELSTEPPER
    return true;
#else
    _stepper = engine().stepperConnectToPin(_stepPin);
    if (!_stepper) return false;
    _stepper->setDirectionPin(_dirPin);  // DIR HIGH = counting up, same as AccelStepper
    setSpeedLimits(_maxSpeed, _acceleration);
    return true;
#endif
  }

  static bool beginAll() {
#ifndef USE_ACCELSTEPPER
    // The queue-filling task runs at top priority on core 1, away from the WiFi stack.
    engine().init(1);
#endif
    bool ok = true;
    for (Axis* axis : axisList) ok &= axis->begin();
    return ok;
  }

  int number() const { return _axisNumber; }
  double minPosition() const { return _minPos; }
  double maxPosition() const { return _maxPos; }
  double stepsPerDegree() const { return _stepsPerRev * _microsteps * _gearing / 360.0; }
  double stepsPerMotorRev() const { return _stepsPerRev * _microsteps; }
  long degreesToSteps(double deg) const { return lround(deg * stepsPerDegree() * _sign()); }
  double stepsToDegrees(long steps) const { return steps / stepsPerDegree() * _sign(); }

  long currentSteps() {
#ifdef USE_ACCELSTEPPER
    return _stepper.currentPosition();
#else
    return _stepper->getCurrentPosition();
#endif
  }

  // Current position in degrees (from counted steps, not measured).
  double getPosition() { return stepsToDegrees(currentSteps()); }

  // Move to an absolute position in degrees, clamped to the soft limits.
  void moveToPosition(double position) {
    if (position > _maxPos) position = _maxPos;
    else if (position < _minPos) position = _minPos;
    moveToPositionUnlimited(position);
  }

  // Homing and speed tests need to go past the soft limits.
  void moveToPositionUnlimited(double position) {
    _continuousDir = 0;
    moveToSteps(degreesToSteps(position));
  }

  void moveRelative(double degrees) { moveToPositionUnlimited(getPosition() + degrees); }

  // Run in one joint direction (+1/-1) until stopped. Used for the homing approach.
  void runContinuous(int jointDir) {
    _continuousDir = jointDir > 0 ? 1 : -1;
#ifdef USE_ACCELSTEPPER
    _stepper.moveTo(_stepper.currentPosition() + 1000000000L * _continuousDir * _sign());
#else
    if (_continuousDir * _sign() > 0) _stepper->runForward();
    else _stepper->runBackward();
#endif
  }

  // Decelerate to a stop with the configured acceleration.
  void stop() {
    _continuousDir = 0;
#ifdef USE_ACCELSTEPPER
    _stepper.stop();
#else
    _stepper->stopMove();
#endif
  }

  // Stop without deceleration (endstop hit, E-stop). The counted position stays valid.
  // FastAccelStepper finishes the few steps already queued (a few ms), so wait for
  // isRunning() == false before relying on the position.
  void hardStop() {
    _continuousDir = 0;
#ifdef USE_ACCELSTEPPER
    _stepper.setCurrentPosition(_stepper.currentPosition());  // also zeroes the speed
#else
    _stepper->forceStop();
#endif
  }

  bool isRunning() {
#ifdef USE_ACCELSTEPPER
    return _stepper.isRunning();
#else
    return _stepper->isRunning();
#endif
  }

  // Signed speed in joint direction, microsteps/s.
  float currentSpeed() {
#ifdef USE_ACCELSTEPPER
    return _stepper.speed() * _sign();
#else
    return _stepper->getCurrentSpeedInMilliHz() / 1000.0f * _sign();
#endif
  }

  // Direction of travel in joint degrees: +1, -1, or 0 when idle.
  int directionOfTravel() {
    if (!isRunning()) return 0;
    if (_continuousDir != 0) return _continuousDir;
    float v = currentSpeed();
    if (v != 0) return v > 0 ? 1 : -1;
#ifdef USE_ACCELSTEPPER
    long toGo = _stepper.distanceToGo();
#else
    long toGo = _stepper->targetPos() - _stepper->getCurrentPosition();
#endif
    if (toGo == 0) return 0;
    return ((toGo > 0) ? 1 : -1) * _sign();
  }

  // Redefine the current position (homing). Only call while stopped.
  void setCurrentPosition(double degrees) {
    _continuousDir = 0;
#ifdef USE_ACCELSTEPPER
    _stepper.setCurrentPosition(degreesToSteps(degrees));
#else
    _stepper->setCurrentPosition(degreesToSteps(degrees));
#endif
  }

  void setSpeedLimits(float stepsPerSecond, float acceleration) {
    _maxSpeed = stepsPerSecond;
    _acceleration = acceleration;
#ifdef USE_ACCELSTEPPER
    _stepper.setMaxSpeed(stepsPerSecond);
    _stepper.setAcceleration(acceleration);
#else
    if (!_stepper) return;  // applied in begin()
    _stepper->setSpeedInHz((uint32_t)max(1.0f, stepsPerSecond));
    _stepper->setAcceleration((int32_t)max(1.0f, acceleration));
    if (_stepper->isRunning()) _stepper->applySpeedAcceleration();
#endif
  }
  float maxSpeed() const { return _maxSpeed; }

  // Polling function controlling the motor (one step at most per call). FastAccelStepper
  // generates steps in hardware, so there is nothing to poll.
  void runMotor() {
#ifdef USE_ACCELSTEPPER
    _stepper.run();
#endif
  }

  // Static function to poll all the existing motors:
  static void runAll() {
    for (Axis* axis : Axis::axisList) axis->runMotor();
  }

  static constexpr bool kHardwareStepping =
#ifdef USE_ACCELSTEPPER
      false;
#else
      true;
#endif

 private:
  void moveToSteps(long steps) {
#ifdef USE_ACCELSTEPPER
    _stepper.moveTo(steps);
#else
    _stepper->moveTo((int32_t)steps);
#endif
  }

 public:
  // Static list of pointers to all axis instances, in construction order (axis 1 first).
  static std::vector<Axis*> axisList;
};

inline std::vector<Axis*> Axis::axisList;

#endif
