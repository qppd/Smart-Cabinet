#ifndef COMPONENTS_FINGERPRINT_AS608_H
#define COMPONENTS_FINGERPRINT_AS608_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <Adafruit_Fingerprint.h>

// Wrapper for AS608 fingerprint module using a HardwareSerial instance
class FingerprintAS608 {
public:
  // Provide the HardwareSerial port (e.g., Serial2) and RX/TX pins are configured externally
  FingerprintAS608(HardwareSerial &serial, uint32_t baud = 57600);
  void begin();
  bool verifySensor();
  int getTemplateCount();
  // Enroll a finger to an ID (blocking, returns true on success)
  bool enroll(uint16_t id);
  // Search for fingerprint, returns -1 on error, 0 if not found, or matching ID
  int search();

private:
  HardwareSerial &_serial;
  Adafruit_Fingerprint _finger;
  uint32_t _baud;
};

#endif // COMPONENTS_FINGERPRINT_AS608_H
