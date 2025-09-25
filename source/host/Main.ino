#include "MotionSensor.h"
#include "I2CLcd.h"
#include "DS1302Rtc.h"
#include "Buzzer.h"
#include "FingerprintAS608.h"
#include "TB6600.h"
#include "Relay4.h"
#include "LimitSwitch.h"
#include "ReedSwitch.h"

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

// Reed switch
ReedSwitch reedSwitch(REED_SWITCH_PIN);

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
//MotionSensor pir(PIR_PIN);
I2CLcd lcd(LCD_ADDR, 20, 4);
DS1302Rtc rtc(DS1302_CE_PIN, DS1302_SCK_PIN, DS1302_IO_PIN);
//Buzzer buzzer(BUZZER_PIN, 0, 2000);
//FingerprintAS608 finger(FingerSerial, 57600);

void setup() {
  // initialize once
  Serial.begin(115200);
  Serial.println("Smart Cabinet components init...");

  // Motion sensor
  //pir.begin();

  // I2C LCD
  Wire.begin(); // default SDA=21, SCL=22 on most ESP32 boards
  lcd.begin();
  lcd.print(0, 0, "Smart Cabinet");
  lcd.print(0, 1, "Init components...");

  // RTC
  rtc.begin();

  // Buzzer
  //buzzer.begin();
  //buzzer.beep(100, 1500);

  // Fingerprint serial
  //finger.begin();
  //if (finger.verifySensor()) Serial.println("Fingerprint sensor OK");
  //else Serial.println("Fingerprint sensor NOT found");

  // Steppers
  //stepper1.begin();
  //stepper2.begin();

  // Relays
  //relays.begin();
  //relays.allOff();

  // Limit switches
  //limitMin.begin();
  // limitMax.begin();
  // limitMin.setCallback([](bool pressed){
  //   Serial.print("Limit MIN: "); Serial.println(pressed ? "PRESSED" : "RELEASED");
  //   if (pressed) {
  //     lcd.print(0, 2, "Limit MIN: PRESSED  ");
  //   } else {
  //     lcd.print(0, 2, "Limit MIN: RELEASED ");
  //   }
  // });
  // limitMax.setCallback([](bool pressed){
  //   Serial.print("Limit MAX: "); Serial.println(pressed ? "PRESSED" : "RELEASED");
  //   if (pressed) {
  //     lcd.print(0, 3, "Limit MAX: PRESSED  ");
  //   } else {
  //     lcd.print(0, 3, "Limit MAX: RELEASED ");
  //   }
  // });

  // Initialize reed switch
  //reedSwitch.begin()

  // Attach hardware interrupts for emergency stop (falling edge for activeLow)
  // attachInterrupt(digitalPinToInterrupt(LIMIT_MIN_PIN), isrLimitMin, FALLING);
  // attachInterrupt(digitalPinToInterrupt(LIMIT_MAX_PIN), isrLimitMax, FALLING);

  Serial.println("Initialization complete");
}

void loop() {
  static unsigned long lastUpdate = 0;
  static int counter = 0;
  if (millis() - lastUpdate > 1000) {
    lastUpdate = millis();
    rtc.updateTime();
    lcd.print(0, 2, "Counter: " + String(counter) + "    ");
    lcd.clear();
    rtc.displayTime(lcd, 0, 3);
    counter++;
    if (counter > 9999) counter = 0;
  }
}
