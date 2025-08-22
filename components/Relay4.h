#ifndef COMPONENTS_RELAY4_H
#define COMPONENTS_RELAY4_H

#include <Arduino.h>

// Simple 4-channel relay board helper (active HIGH by default)
class Relay4 {
public:
  Relay4(uint8_t r1, uint8_t r2, uint8_t r3, uint8_t r4, bool activeHigh = true);
  void begin();
  void set(uint8_t channel, bool on);
  bool get(uint8_t channel) const;
  void allOff();

private:
  uint8_t _pins[4];
  bool _state[4];
  bool _activeHigh;
};

#endif // COMPONENTS_RELAY4_H
