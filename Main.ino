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
#include "components/LimitSwitch.h"

// Centralized pin assignments
#include "pins.h"

// Serial for fingerprint module
HardwareSerial FingerSerial(2);

// TB6600 drivers
TB6600 stepper1(TB1_DIR, TB1_STEP, TB1_ENABLE);
TB6600 stepper2(TB2_DIR, TB2_STEP, TB2_ENABLE);

// 4-channel relay
Relay4 relays(RELAY1_PIN, RELAY2_PIN, RELAY3_PIN, RELAY4_PIN, false); // activeLow typical for ESP32 relay boards

// Limit switches
LimitSwitch limitMin(LIMIT_MIN_PIN, true);
LimitSwitch limitMax(LIMIT_MAX_PIN, true);

// Emergency flags set by ISRs
volatile bool emergencyMin = false;
volatile bool emergencyMax = false;

// ISR handlers (keep them tiny)
void IRAM_ATTR isrLimitMin() {
  emergencyMin = true;
  // immediately disable stepper drivers
  stepper1.emergencyStop();
  stepper2.emergencyStop();
}

void IRAM_ATTR isrLimitMax() {
  emergencyMax = true;
  stepper1.emergencyStop();
  stepper2.emergencyStop();
}

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

  // Limit switches
  limitMin.begin();
  limitMax.begin();
  limitMin.setCallback([](bool pressed){
    Serial.print("Limit MIN: "); Serial.println(pressed ? "PRESSED" : "RELEASED");
    if (pressed) {
      lcd.print(0, 2, "Limit MIN: PRESSED  ");
    } else {
      lcd.print(0, 2, "Limit MIN: RELEASED ");
    }
  });
  limitMax.setCallback([](bool pressed){
    Serial.print("Limit MAX: "); Serial.println(pressed ? "PRESSED" : "RELEASED");
    if (pressed) {
      lcd.print(0, 3, "Limit MAX: PRESSED  ");
    } else {
      lcd.print(0, 3, "Limit MAX: RELEASED ");
    }
  });

  // Attach hardware interrupts for emergency stop (falling edge for activeLow)
  attachInterrupt(digitalPinToInterrupt(LIMIT_MIN_PIN), isrLimitMin, FALLING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_MAX_PIN), isrLimitMax, FALLING);

  Serial.println("Initialization complete");
}

void loop() {
  // Non-blocking updates for components
  pir.update();
  buzzer.update();
  stepper1.update();
  stepper2.update();
  finger.update();
  limitMin.update();
  limitMax.update();

  // Handle emergency flags in main loop (non-ISR safe actions here)
  if (emergencyMin) {
    emergencyMin = false;
    Serial.println("Emergency stop: MIN limit triggered");
    lcd.print(0, 2, "EMERGENCY: MIN STOP ");
    // ensure relays or other actuators are safe
    relays.allOff();
  }
  if (emergencyMax) {
    emergencyMax = false;
    Serial.println("Emergency stop: MAX limit triggered");
    lcd.print(0, 3, "EMERGENCY: MAX STOP ");
    relays.allOff();
  }

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
