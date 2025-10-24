#ifndef TACTILE_BUTTON_H
#define TACTILE_BUTTON_H

#include <Arduino.h>

class TactileButton {
public:
    TactileButton(uint8_t pin, bool pullup = true, unsigned long debounceTime = 50);
    void begin();
    void update();
    bool isPressed();
    bool wasPressed();
    bool wasReleased();
    bool isHeld(unsigned long duration = 1000);
    unsigned long getPressedDuration();
    
private:
    uint8_t _pin;
    bool _pullup;
    unsigned long _debounceTime;
    
    bool _currentState;
    bool _lastState;
    bool _pressed;
    bool _released;
    
    unsigned long _lastDebounceTime;
    unsigned long _pressStartTime;
    unsigned long _pressedDuration;
};

#endif // TACTILE_BUTTON_H
