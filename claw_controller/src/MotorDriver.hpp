#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include <Arduino.h>

#include "Configuration.h"

// One TB6612FNG channel. Truth table (STBY high):
//   IN1 H, IN2 L -> forward      IN1 L, IN2 H -> reverse
//   IN1 H, IN2 H -> short brake  IN1 L, IN2 L -> stop (coast)
// With IN1 != IN2, PWM low means short brake, so the motor current keeps flowing through the
// shunt during the PWM off-time and the INA219 sees the real average motor current.
//
// Direction here is finger direction: +1 = close, -1 = open (the invert flag maps it).
class MotorChannel {
 public:
  MotorChannel(uint8_t in1, uint8_t in2, uint8_t pwm, bool invert)
      : _in1(in1), _in2(in2), _pwm(pwm), _invert(invert) {}

  void begin() {
    // TB6612 inputs have internal pull-downs, so the motor coasts until this runs.
    pinMode(_in1, OUTPUT);
    pinMode(_in2, OUTPUT);
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, LOW);
    ledcAttach(_pwm, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_BITS);
    ledcWrite(_pwm, 0);
  }

  // dir: +1 close, -1 open. duty 0..1
  void drive(int dir, float duty) {
    bool forward = (dir > 0) != _invert;
    digitalWrite(_in1, forward ? HIGH : LOW);
    digitalWrite(_in2, forward ? LOW : HIGH);
    setDuty(duty);
    _dir = dir > 0 ? 1 : -1;
    _mode = Mode::Drive;
  }

  void setDuty(float duty) {
    duty = constrain(duty, 0.0f, 1.0f);
    _duty = duty;
    ledcWrite(_pwm, (uint32_t)lroundf(duty * kMaxDuty));
  }

  void brake() {
    digitalWrite(_in1, HIGH);
    digitalWrite(_in2, HIGH);
    ledcWrite(_pwm, kMaxDuty);
    _duty = 0;
    _dir = 0;
    _mode = Mode::Brake;
  }

  void coast() {
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, LOW);
    ledcWrite(_pwm, 0);
    _duty = 0;
    _dir = 0;
    _mode = Mode::Coast;
  }

  enum class Mode : uint8_t { Coast, Brake, Drive };
  Mode mode() const { return _mode; }
  int direction() const { return _dir; }
  float duty() const { return _duty; }
  // +1 if positive shunt current (IN+ -> IN-, i.e. "forward") corresponds to closing.
  int closeSign() const { return _invert ? -1 : 1; }

 private:
  static constexpr uint32_t kMaxDuty = (1u << MOTOR_PWM_BITS) - 1;
  uint8_t _in1, _in2, _pwm;
  bool _invert;
  Mode _mode = Mode::Coast;
  int _dir = 0;
  float _duty = 0;
};

#endif
