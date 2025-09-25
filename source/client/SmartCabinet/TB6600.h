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

  // Immediate emergency stop: disable driver and cancel any ongoing non-blocking steps
  void emergencyStop();

  // Non-blocking API
  // Start a non-blocking stepping operation. Returns true if started.
  bool startSteps(unsigned long steps, unsigned int pulseUs = 100, unsigned int gapUs = 800);
  // Call frequently from loop to drive the steps
  void update();
  // true while non-blocking stepping is in progress
  bool isBusy() const { return _busy; }

private:
  uint8_t _dirPin, _stepPin, _enablePin;
  bool _dir;
  // non-blocking state
  volatile unsigned long _remainingSteps;
  unsigned int _pulseUs;
  unsigned int _gapUs;
  unsigned long _nextToggleMicros;
  bool _pulseState;
  bool _busy;
};

#endif // COMPONENTS_TB6600_H