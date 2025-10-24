#include <Arduino.h>
#include "Buzzer.h"

Buzzer::Buzzer(uint8_t pin, uint8_t ledcChannel, uint32_t freq, bool activeHigh)
  : _pin(pin), _ledcChannel(ledcChannel), _freqHz(freq), _activeHigh(activeHigh), _active(false), _beepEndMs(0) {}

void Buzzer::begin() {
  // configure ledc timer and channel
  // Use 8-bit resolution for simplicity
  ledcAttach(_pin, _freqHz, 8);
  off();
}

void Buzzer::on() {
  _active = true;
  // 50% duty cycle works better for most piezo buzzers
  ledcWrite(_pin, 128);
}

void Buzzer::off() {
  _active = false;
  ledcWrite(_pin, 0);
}

void Buzzer::beep(unsigned int ms, uint32_t freqHz) {
  if (freqHz != _freqHz) {
    _freqHz = freqHz;
    ledcChangeFrequency(_pin, _freqHz, 8);
  }
  on();
  _beepEndMs = millis() + ms;
}

void Buzzer::update() {
  if (_active && _beepEndMs != 0 && millis() >= _beepEndMs) {
    off();
    _beepEndMs = 0;
  }
}

