#ifndef COMPONENTS_BUZZER_H
#define COMPONENTS_BUZZER_H

#include <Arduino.h>

// Non-blocking piezo buzzer helper for ESP32 using LEDC (hardware PWM)
// Usage:
//   Buzzer buzzer(pin); // optional channel/frequency
//   buzzer.begin();
//   buzzer.beep(200, 2000); // play 2kHz for 200ms (non-blocking)
//   in loop(): buzzer.update();

class Buzzer {
public:
  // ledcChannel: 0-15 (ESP32 supports up to 16 channels). freq default in Hz.
  Buzzer(uint8_t pin, uint8_t ledcChannel = 0, uint32_t freq = 2000, bool activeHigh = true);
  void begin();

  // start continuous tone at configured frequency
  void on();
  // stop tone
  void off();

  // non-blocking beep: play 'freq' (Hz) for ms milliseconds
  void beep(unsigned int ms, uint32_t freqHz = 2000);

  // Call regularly from main loop to handle stopping scheduled beeps
  void update();

  bool isOn() const { return _active; }

private:
  uint8_t _pin;
  uint8_t _ledcChannel;
  uint32_t _freqHz;
  bool _activeHigh;
  bool _active;
  unsigned long _beepEndMs;
};

#endif // COMPONENTS_BUZZER_H
