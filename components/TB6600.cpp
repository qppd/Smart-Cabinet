#include "TB6600.h"

TB6600::TB6600(uint8_t dirPin, uint8_t stepPin, uint8_t enablePin)
  : _dirPin(dirPin), _stepPin(stepPin), _enablePin(enablePin), _dir(false) {}

void TB6600::begin() {
  pinMode(_dirPin, OUTPUT);
  pinMode(_stepPin, OUTPUT);
  if (_enablePin != 255) pinMode(_enablePin, OUTPUT);
  enable(false);
}

void TB6600::enable(bool en) {
  if (_enablePin == 255) return;
  digitalWrite(_enablePin, en ? LOW : HIGH); // many drivers use LOW to enable
}

void TB6600::setDirection(bool dir) {
  _dir = dir;
  digitalWrite(_dirPin, dir ? HIGH : LOW);
}

void TB6600::stepOnce(unsigned int pulseUs) {
  digitalWrite(_stepPin, HIGH);
  delayMicroseconds(pulseUs);
  digitalWrite(_stepPin, LOW);
}

void TB6600::stepMany(unsigned long steps, unsigned int pulseUs, unsigned int gapUs) {
  for (unsigned long i = 0; i < steps; ++i) {
    stepOnce(pulseUs);
    if (gapUs) delayMicroseconds(gapUs);
  }
}
