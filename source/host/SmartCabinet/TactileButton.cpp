#include "TactileButton.h"

TactileButton::TactileButton(uint8_t pin, bool pullup, unsigned long debounceTime) 
    : _pin(pin), _pullup(pullup), _debounceTime(debounceTime),
      _currentState(false), _lastState(false), _pressed(false), _released(false),
      _lastDebounceTime(0), _pressStartTime(0), _pressedDuration(0) {
}

void TactileButton::begin() {
    if (_pullup) {
        pinMode(_pin, INPUT_PULLUP);
        // With pullup, button press = LOW
        _currentState = digitalRead(_pin) == LOW;
    } else {
        pinMode(_pin, INPUT);
        // Without pullup, button press = HIGH
        _currentState = digitalRead(_pin) == HIGH;
    }
    _lastState = _currentState;
    
    Serial.print("[TactileButton] Initialized on pin ");
    Serial.print(_pin);
    Serial.println(_pullup ? " (INPUT_PULLUP)" : " (INPUT)");
}

void TactileButton::update() {
    // Read the current button state
    bool reading;
    if (_pullup) {
        reading = digitalRead(_pin) == LOW;  // Button pressed when LOW (pullup)
    } else {
        reading = digitalRead(_pin) == HIGH; // Button pressed when HIGH (no pullup)
    }
    
    // Reset flags
    _pressed = false;
    _released = false;
    
    // Debounce logic
    if (reading != _lastState) {
        _lastDebounceTime = millis();
    }
    
    if ((millis() - _lastDebounceTime) > _debounceTime) {
        // State has been stable for debounce time
        if (reading != _currentState) {
            _currentState = reading;
            
            if (_currentState) {
                // Button just pressed
                _pressed = true;
                _pressStartTime = millis();
            } else {
                // Button just released
                _released = true;
                _pressedDuration = millis() - _pressStartTime;
            }
        }
    }
    
    _lastState = reading;
}

bool TactileButton::isPressed() {
    return _currentState;
}

bool TactileButton::wasPressed() {
    return _pressed;
}

bool TactileButton::wasReleased() {
    return _released;
}

bool TactileButton::isHeld(unsigned long duration) {
    if (_currentState && _pressStartTime > 0) {
        return (millis() - _pressStartTime) >= duration;
    }
    return false;
}

unsigned long TactileButton::getPressedDuration() {
    if (_currentState && _pressStartTime > 0) {
        return millis() - _pressStartTime;
    }
    return _pressedDuration;
}
