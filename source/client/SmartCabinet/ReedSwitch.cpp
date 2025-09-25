#include "ReedSwitch.h"

ReedSwitch::ReedSwitch(uint8_t pin) : pin(pin), state(false) {}

void ReedSwitch::begin() {
    pinMode(pin, INPUT_PULLUP);
}

bool ReedSwitch::isClosed() {
    state = digitalRead(pin) == LOW;
    return state;
}