#ifndef COMPONENTS_MOTION_SENSOR_H
#define COMPONENTS_MOTION_SENSOR_H

#include <Arduino.h>

// Simple 3-wire PIR / motion sensor helper for ESP32
// Usage:
//   MotionSensor pir(PIN);
//   pir.begin();
//   in loop(): pir.update(); if (pir.isMotion()) ...

class MotionSensor {
public:
  MotionSensor(uint8_t pin, unsigned long debounceMs = 200);
  void begin();
  // Call frequently from loop to maintain debounced state
  void update();

  // Immediate raw read (digitalRead)
  bool readRaw();

  // Debounced motion state
  bool isMotion() const { return _state; }

  // Set a simple callback that will be called when state changes.
  // Callback signature: void callback(bool newState)
  using Callback = void(*)(bool);
  void setCallback(Callback cb) { _cb = cb; }

private:
  uint8_t _pin;
  bool _state;
  unsigned long _lastChange;
  unsigned long _debounceMs;
  Callback _cb;
};

// Inline implementations
inline MotionSensor::MotionSensor(uint8_t pin, unsigned long debounceMs)
  : _pin(pin), _state(false), _lastChange(0), _debounceMs(debounceMs), _cb(nullptr) {}

inline void MotionSensor::begin() {
  pinMode(_pin, INPUT);
  _state = digitalRead(_pin);
  _lastChange = millis();
}

inline bool MotionSensor::readRaw() {
  return digitalRead(_pin);
}

inline void MotionSensor::update() {
  bool raw = readRaw();
  unsigned long now = millis();
  if (raw != _state) {
    // potential change, check debounce
    if (now - _lastChange >= _debounceMs) {
      _state = raw;
      _lastChange = now;
      if (_cb) _cb(_state);
    }
  } else {
    _lastChange = now;
  }
}

#endif // COMPONENTS_MOTION_SENSOR_H

