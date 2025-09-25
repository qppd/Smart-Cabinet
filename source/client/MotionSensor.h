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
  bool isMotion() const;

  // Set a simple callback that will be called when state changes.
  // Callback signature: void callback(bool newState)
  using Callback = void(*)(bool);
  void setCallback(Callback cb);

private:
  uint8_t _pin;
  bool _state;
  unsigned long _lastChange;
  unsigned long _debounceMs;
  Callback _cb;
};

// Implementations moved to MotionSensor.cpp

#endif // COMPONENTS_MOTION_SENSOR_H