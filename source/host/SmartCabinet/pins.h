#ifndef SMART_CABINET_HOST_PINS_H
#define SMART_CABINET_HOST_PINS_H

// ===========================================
// HOST DEVICE PIN CONFIGURATION
// ===========================================
// This configuration is for the host ESP32 that handles:
// - Fingerprint authentication
// - User interface (LCD, Buzzer)
// - Real-time clock
// - WebSocket server communication
// - System coordination and control

// I2C LCD Display
static const uint8_t LCD_ADDR = 0x27; // I2C address (SDA=21, SCL=22 by default)

// Buzzer for Audio Feedback
static const uint8_t BUZZER_PIN = 5;

// Fingerprint Sensor Module (Serial2)
static const uint8_t FINGER_RX_PIN = 16; // RX2
static const uint8_t FINGER_TX_PIN = 17; // TX2

// Enrollment Button
static const uint8_t ENROLL_BUTTON_PIN = 23; // Tactile button for fingerprint enrollment

// DS1302 Real-Time Clock Module
static const uint8_t DS1302_CE_PIN = 0;   // RST/CE pin
static const uint8_t DS1302_SCK_PIN = 19; // CLK/SCK pin  
static const uint8_t DS1302_IO_PIN = 18;   // DAT/IO pin

// System Constants
static const unsigned long HOST_DISPLAY_UPDATE_INTERVAL = 1000;    // LCD update frequency
static const unsigned long HOST_FINGERPRINT_CHECK_INTERVAL = 500;  // Fingerprint scan frequency
static const unsigned long HOST_STATUS_BROADCAST_INTERVAL = 5000;  // Status broadcast frequency

// Fingerprint Authentication Settings
static const uint8_t MAX_FINGERPRINT_ATTEMPTS = 3;       // Max failed attempts before lockout
static const unsigned long LOCKOUT_DURATION = 30000;     // Lockout duration (30 seconds)
static const uint8_t MAX_ENROLLED_FINGERPRINTS = 50;     // Maximum users

// Audio Feedback Settings
static const unsigned int SUCCESS_BEEP_FREQ = 1500;      // Success tone frequency
static const unsigned int ERROR_BEEP_FREQ = 800;         // Error tone frequency
static const unsigned int FEEDBACK_BEEP_FREQ = 2000;     // General feedback frequency

#endif // SMART_CABINET_HOST_PINS_H
