/*
  Main.ino - Blank template for Smart Cabinet project
  Created: 2025-08-23

  This file is intentionally minimal. Replace or extend the
  setup() and loop() functions below with your application code.
*/

// Include component wrappers
#include "components/MotionSensor.h"
#include "components/I2CLcd.h"
#include "components/Buzzer.h"
#include "components/FingerprintAS608.h"
#include "components/TB6600.h"
#include "components/Relay4.h"

// Default ESP32 pin assignments (change to match your wiring)
// Motion sensor (PIR) - 3-wire: OUT to GPIO
const uint8_t PIR_PIN = 34; // input-only pin

// I2C LCD - SDA / SCL use default Wire pins (GPIO 21 SDA, 22 SCL)
const uint8_t LCD_ADDR = 0x27;

// Buzzer
const uint8_t BUZZER_PIN = 25;

// Fingerprint module: use Serial2 (RX2=16, TX2=17) by default
HardwareSerial FingerSerial(2);

// TB6600 drivers (example pins)
TB6600 stepper1(2, 4, 15); // DIR, STEP, ENABLE
TB6600 stepper2(18, 19, 5);

// 4-channel relay
Relay4 relays(12, 13, 14, 27, false); // activeLow typical for ESP32 relay boards

// Components instances
MotionSensor pir(PIR_PIN);
I2CLcd lcd(LCD_ADDR, 20, 4);
Buzzer buzzer(BUZZER_PIN, 0, 2000);
FingerprintAS608 finger(FingerSerial, 57600);

void setup() {
  // initialize once
  Serial.begin(115200);
  Serial.println("Smart Cabinet components init...");

  // Motion sensor
  pir.begin();

  // I2C LCD
  Wire.begin(); // default SDA=21, SCL=22 on most ESP32 boards
  lcd.begin();
  lcd.print(0, 0, "Smart Cabinet");
  lcd.print(0, 1, "Init components...");

  // Buzzer
  buzzer.begin();
  buzzer.beep(100, 1500);

  // Fingerprint serial
  finger.begin();
  if (finger.verifySensor()) Serial.println("Fingerprint sensor OK");
  else Serial.println("Fingerprint sensor NOT found");

  // Steppers
  stepper1.begin();
  stepper2.begin();

  // Relays
  relays.begin();
  relays.allOff();

  Serial.println("Initialization complete");
}

void loop() {
  // Non-blocking updates for components
  pir.update();
  buzzer.update();

  // Example: display motion state on Serial and LCD (no control logic)
  static bool lastPir = false;
  if (pir.isMotion() != lastPir) {
    lastPir = pir.isMotion();
    Serial.print("PIR motion: "); Serial.println(lastPir ? "YES" : "NO");
    lcd.clear();
    lcd.print(0, 0, "Smart Cabinet");
    lcd.print(0, 1, lastPir ? "Motion detected" : "No motion");
    if (lastPir) buzzer.beep(150, 2000);
  }

  // Keep loop light and responsive
  delay(20);
}
