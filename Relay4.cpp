#include "Relay4.h"

Relay4::Relay4(uint8_t r1, uint8_t r2, uint8_t r3, uint8_t r4, bool activeHigh)
  : _pins{r1, r2, r3, r4}, _activeHigh(activeHigh) {
  for (int i = 0; i < 4; ++i) _state[i] = false;
}

void Relay4::begin() {
  for (int i = 0; i < 4; ++i) {
    pinMode(_pins[i], OUTPUT);
    digitalWrite(_pins[i], _activeHigh ? LOW : HIGH);
  }
}

void Relay4::set(uint8_t channel, bool on) {
  if (channel < 1 || channel > 4) return;
  uint8_t idx = channel - 1;
  _state[idx] = on;
  digitalWrite(_pins[idx], (_activeHigh ? (on ? HIGH : LOW) : (on ? LOW : HIGH)));
}

bool Relay4::get(uint8_t channel) const {
  if (channel < 1 || channel > 4) return false;
  return _state[channel - 1];
}

void Relay4::allOff() {
  for (int i = 1; i <= 4; ++i) set(i, false);
}
