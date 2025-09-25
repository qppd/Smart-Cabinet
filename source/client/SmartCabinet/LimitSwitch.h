#ifndef COMPONENTS_LIMIT_SWITCH_H
#define COMPONENTS_LIMIT_SWITCH_H

#include <Arduino.h>

// Simple debounced limit switch helper for ESP32
// Usage:
//  LimitSwitch sw(pin, activeLow=true, debounceMs=50);
//  sw.begin(); sw.update(); if (sw.isPressed()) ...

class LimitSwitch {
public:
  LimitSwitch(uint8_t pin, bool activeLow = true, unsigned long debounceMs = 50);
  void begin();
  void update();
  bool isPressed() const;

  using Callback = void(*)(bool pressed);
  void setCallback(Callback cb);

private:
  uint8_t _pin;
  bool _activeLow;
  bool _state;
  int _lastRaw;
  unsigned long _lastChange;
  unsigned long _debounceMs;
  Callback _cb;
};

#endif // COMPONENTS_LIMIT_SWITCH_H