#ifndef REED_SWITCH_H
#define REED_SWITCH_H

#include <Arduino.h>

class ReedSwitch {
private:
    uint8_t pin;
    bool state;

public:
    ReedSwitch(uint8_t pin);
    void begin();
    bool isClosed();
};

#endif
