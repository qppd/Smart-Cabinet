#include "LimitSwitch.h"

LimitSwitch::LimitSwitch(uint8_t pin, bool activeLow, unsigned long debounceMs)
  : _pin(pin), _activeLow(activeLow), _state(false), _lastRaw(-1), _lastChange(0), _debounceMs(debounceMs), _cb(nullptr) {}

void LimitSwitch::begin() {
  if (_activeLow) pinMode(_pin, INPUT_PULLUP);
  else pinMode(_pin, INPUT);
  _lastRaw = digitalRead(_pin);
  // interpret raw according to active level
  _state = (_activeLow ? (_lastRaw == LOW) : (_lastRaw == HIGH));
  _lastChange = millis();
}

void LimitSwitch::update() {
  int raw = digitalRead(_pin);
  bool pressed = _activeLow ? (raw == LOW) : (raw == HIGH);
  unsigned long now = millis();
  if (pressed != _state) {
    if (now - _lastChange >= _debounceMs) {
      _state = pressed;
      _lastChange = now;
      if (_cb) _cb(_state);
    }
  } else {
    _lastChange = now;
  }
}

bool LimitSwitch::isPressed() const { return _state; }

void LimitSwitch::setCallback(Callback cb) { _cb = cb; }