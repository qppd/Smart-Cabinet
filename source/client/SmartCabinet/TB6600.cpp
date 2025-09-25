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

bool TB6600::startSteps(unsigned long steps, unsigned int pulseUs, unsigned int gapUs) {
  if (steps == 0 || _busy) return false;
  _remainingSteps = steps;
  _pulseUs = pulseUs;
  _gapUs = gapUs;
  _pulseState = false;
  _nextToggleMicros = micros();
  _busy = true;
  return true;
}

void TB6600::update() {
  if (!_busy) return;
  unsigned long now = micros();
  if (!_pulseState) {
    // wait for gap then start pulse
    if ((long)(now - _nextToggleMicros) >= 0) {
      digitalWrite(_stepPin, HIGH);
      _pulseState = true;
      _nextToggleMicros = now + _pulseUs;
    }
  } else {
    if ((long)(now - _nextToggleMicros) >= 0) {
      digitalWrite(_stepPin, LOW);
      _pulseState = false;
      if (_remainingSteps > 0) --_remainingSteps;
      if (_remainingSteps == 0) {
        _busy = false;
      } else {
        _nextToggleMicros = now + _gapUs;
      }
    }
  }
}

void TB6600::emergencyStop() {
  // disable driver immediately
  if (_enablePin != 255) digitalWrite(_enablePin, HIGH); // assume HIGH disables
  // cancel any ongoing non-blocking operation
  _busy = false;
  _remainingSteps = 0;
}