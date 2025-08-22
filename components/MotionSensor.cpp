#include "MotionSensor.h"

MotionSensor::MotionSensor()
  : _pin(255), _activeHigh(true), _state(false), _lastChange(0) {}

void MotionSensor::begin(uint8_t pin, bool activeHigh) {
  _pin = pin;
  _activeHigh = activeHigh;
  pinMode(_pin, INPUT);
  _state = false;
  _lastChange = millis();
}

bool MotionSensor::isMotion() {
  // return last known state
  return _state;
}

void MotionSensor::update() {
  if (_pin == 255) return;
  bool raw = digitalRead(_pin) == HIGH;
  bool active = _activeHigh ? raw : !raw;
  unsigned long now = millis();
  if (active != _state && (now - _lastChange) > _debounceMs) {
    _state = active;
    _lastChange = now;
  }
}
