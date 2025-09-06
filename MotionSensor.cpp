#include "MotionSensor.h"

// Definitions for MotionSensor declared in MotionSensor.h

MotionSensor::MotionSensor(uint8_t pin, unsigned long debounceMs)
  : _pin(pin), _state(false), _lastChange(0), _debounceMs(debounceMs), _cb(nullptr) {}

void MotionSensor::begin() {
  pinMode(_pin, INPUT);
  _state = digitalRead(_pin);
  _lastChange = millis();
}

bool MotionSensor::isMotion() const {
  return _state;
}

bool MotionSensor::readRaw() {
  return digitalRead(_pin);
}

void MotionSensor::setCallback(Callback cb) {
  _cb = cb;
}

void MotionSensor::update() {
  bool raw = readRaw();
  unsigned long now = millis();
  if (raw != _state) {
    if (now - _lastChange >= _debounceMs) {
      _state = raw;
      _lastChange = now;
      if (_cb) _cb(_state);
    }
  } else {
    _lastChange = now;
  }
}
