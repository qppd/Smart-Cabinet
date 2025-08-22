
#include "Buzzer.h"
#include <driver/ledc.h>

Buzzer::Buzzer(uint8_t pin, uint8_t ledcChannel, uint32_t freq, bool activeHigh)
  : _pin(pin), _ledcChannel(ledcChannel), _freqHz(freq), _activeHigh(activeHigh), _active(false), _beepEndMs(0) {}

void Buzzer::begin() {
  // configure ledc timer and channel
  // Use 8-bit resolution for simplicity
  ledcSetup(_ledcChannel, _freqHz, 8);
  ledcAttachPin(_pin, _ledcChannel);
  off();
}

void Buzzer::on() {
  _active = true;
  // full duty cycle (255 / 8-bit)
  ledcWrite(_ledcChannel, 200);
}

void Buzzer::off() {
  _active = false;
  ledcWrite(_ledcChannel, 0);
}

void Buzzer::beep(unsigned int ms, uint32_t freqHz) {
  if (freqHz != _freqHz) {
    _freqHz = freqHz;
    ledcSetup(_ledcChannel, _freqHz, 8);
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

