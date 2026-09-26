#ifndef INA219_HPP
#define INA219_HPP

#include <Arduino.h>
#include <Wire.h>

#include "Configuration.h"

// Minimal INA219 driver tuned for fast current sampling.
//
// Setup picks the smallest shunt-voltage range (PGA) that covers maxCurrentA * shuntOhms and
// programs the calibration register so the current register reads directly in amps. It then
// runs the shunt ADC continuously (bus voltage is not needed) and leaves the register pointer
// on the current register, so each sample is a single 2-byte read with no address write.
class Ina219 {
 public:
  Ina219(uint8_t address) : _address(address) {}

  bool begin(TwoWire& wire, float shuntOhms, float maxCurrentA) {
    _wire = &wire;
    _shuntOhms = shuntOhms;

    // PGA ranges: /1 = 40 mV, /2 = 80 mV, /4 = 160 mV, /8 = 320 mV
    float needed = maxCurrentA * shuntOhms;
    uint16_t pga = 3;
    float rangeV = 0.320f;
    const float ranges[4] = {0.040f, 0.080f, 0.160f, 0.320f};
    for (uint16_t i = 0; i < 4; i++) {
      if (needed <= ranges[i]) {
        pga = i;
        rangeV = ranges[i];
        break;
      }
    }
    _rangeA = rangeV / shuntOhms;

    // Current LSB: smallest "round" step that still covers the range in 15 bits.
    _currentLsbA = _rangeA / 32768.0f;
    uint16_t cal = (uint16_t)min(0.04096f / (_currentLsbA * shuntOhms), 65534.0f) & 0xFFFE;
    _currentLsbA = 0.04096f / (cal * shuntOhms);  // exact LSB for the truncated cal value

    // BRNG = 0 (16 V bus range, VM is 5 V), PGA, BADC = 12 bit, SADC per config, mode 101 =
    // shunt voltage continuous.
    uint16_t config = (pga << 11) | (0x3 << 7) | ((INA219_SHUNT_ADC & 0xF) << 3) | 0x5;
    _config = config;

    if (!writeRegister(kRegConfig, 0x8000)) return fail();  // reset
    delay(1);
    if (!writeRegister(kRegCalibration, cal)) return fail();
    if (!writeRegister(kRegConfig, config)) return fail();
    uint16_t readBack = 0;
    if (!readRegister(kRegConfig, readBack) || readBack != config) return fail();
    if (!selectRegister(kRegCurrent)) return fail();
    _ok = true;
    _errors = 0;
    return true;
  }

  // One sample. Returns false on an I2C error (the previous value is kept).
  bool sample() {
    if (!_ok) return false;
    if (_wire->requestFrom(_address, (uint8_t)2) != 2) {
      if (++_errors >= kMaxConsecutiveErrors) _ok = false;
      return false;
    }
    int16_t raw = (int16_t)((_wire->read() << 8) | _wire->read());
    _currentmA = raw * _currentLsbA * 1000.0f;
    _errors = 0;
    return true;
  }

  bool ok() const { return _ok; }
  uint8_t address() const { return _address; }
  float currentmA() const { return _currentmA; }  // signed: + = IN+ -> IN- ("forward")
  float rangemA() const { return _rangeA * 1000.0f; }
  float lsbmA() const { return _currentLsbA * 1000.0f; }

 private:
  static constexpr uint8_t kRegConfig = 0x00;
  static constexpr uint8_t kRegCurrent = 0x04;
  static constexpr uint8_t kRegCalibration = 0x05;
  static constexpr uint8_t kMaxConsecutiveErrors = 10;

  uint8_t _address;
  TwoWire* _wire = nullptr;
  float _shuntOhms = 0.1f;
  float _rangeA = 0;
  float _currentLsbA = 0;
  uint16_t _config = 0;
  bool _ok = false;
  uint8_t _errors = 0;
  float _currentmA = 0;

  bool fail() {
    _ok = false;
    return false;
  }

  bool writeRegister(uint8_t reg, uint16_t value) {
    _wire->beginTransmission(_address);
    _wire->write(reg);
    _wire->write(value >> 8);
    _wire->write(value & 0xFF);
    return _wire->endTransmission() == 0;
  }

  bool selectRegister(uint8_t reg) {
    _wire->beginTransmission(_address);
    _wire->write(reg);
    return _wire->endTransmission() == 0;
  }

  bool readRegister(uint8_t reg, uint16_t& value) {
    if (!selectRegister(reg)) return false;
    if (_wire->requestFrom(_address, (uint8_t)2) != 2) return false;
    value = (_wire->read() << 8) | _wire->read();
    return true;
  }
};

#endif
