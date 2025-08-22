#ifndef COMPONENTS_TB6600_H
#define COMPONENTS_TB6600_H

#include <Arduino.h>

// Simple TB6600 stepper driver helper for single motor control
// Uses DIR and STEP pins and optional ENABLE pin.
class TB6600 {
public:
  TB6600(uint8_t dirPin, uint8_t stepPin, uint8_t enablePin = 255);
  void begin();
  void enable(bool en);
  // single step (blocking pulse)
  void stepOnce(unsigned int pulseUs = 100);
  // step multiple times with microsecond delay between pulses (blocking)
  void stepMany(unsigned long steps, unsigned int pulseUs = 100, unsigned int gapUs = 800);
  void setDirection(bool dir);

private:
  uint8_t _dirPin, _stepPin, _enablePin;
  bool _dir;
};

#endif // COMPONENTS_TB6600_H
