#ifndef SMART_CABINET_PINS_H
#define SMART_CABINET_PINS_H

// Pin assignments for ESP32 - change to match your wiring

// PIR motion sensor
static const uint8_t PIR_PIN = 34; // input-only

// I2C LCD
static const uint8_t LCD_ADDR = 0x27; // I2C address (SDA=21, SCL=22 by default)

// Buzzer
static const uint8_t BUZZER_PIN = 25;

// Fingerprint module (Serial2)
static const uint8_t FINGER_RX_PIN = 16; // RX2
static const uint8_t FINGER_TX_PIN = 17; // TX2

// TB6600 drivers (DIR, STEP, ENABLE)
static const uint8_t TB1_DIR = 2;
static const uint8_t TB1_STEP = 4;
static const uint8_t TB1_ENABLE = 15;

static const uint8_t TB2_DIR = 18;
static const uint8_t TB2_STEP = 19;
static const uint8_t TB2_ENABLE = 5;

// 4-channel relay pins
static const uint8_t RELAY1_PIN = 12;
static const uint8_t RELAY2_PIN = 13;
static const uint8_t RELAY3_PIN = 14;
static const uint8_t RELAY4_PIN = 27;

// Limit switches
static const uint8_t LIMIT_MIN_PIN = 32;
static const uint8_t LIMIT_MAX_PIN = 33;

#endif // SMART_CABINET_PINS_H
