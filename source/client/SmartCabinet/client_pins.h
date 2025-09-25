#ifndef SMART_CABINET_CLIENT_PINS_H
#define SMART_CABINET_CLIENT_PINS_H

// ===========================================
// CLIENT DEVICE PIN CONFIGURATION
// ===========================================
// This configuration is for the client ESP32 that handles:
// - Motion detection and user interface
// - Motor control and mechanical operations
// - Relay control for actuators
// - Safety switches and sensors
// - WebSocket communication with host

// PIR Motion Sensor
static const uint8_t CLIENT_PIR_PIN = 34; // Input-only pin for PIR sensor

// TB6600 Stepper Motor Driver #1 (Door/Drawer Motor)
static const uint8_t CLIENT_TB1_DIR = 2;      // Direction pin
static const uint8_t CLIENT_TB1_STEP = 4;     // Step pulse pin
static const uint8_t CLIENT_TB1_ENABLE = 15;  // Enable pin (LOW = enabled)

// TB6600 Stepper Motor Driver #2 (Lock/Mechanism Motor)  
static const uint8_t CLIENT_TB2_DIR = 32;     // Direction pin
static const uint8_t CLIENT_TB2_STEP = 33;    // Step pulse pin
static const uint8_t CLIENT_TB2_ENABLE = 5;   // Enable pin (LOW = enabled)

// 4-Channel Relay Module (Controls solenoids, LED strips, etc.)
static const uint8_t CLIENT_RELAY1_PIN = 12;  // Solenoid lock control
static const uint8_t CLIENT_RELAY2_PIN = 13;  // LED strip power
static const uint8_t CLIENT_RELAY3_PIN = 14;  // Auxiliary device 1
static const uint8_t CLIENT_RELAY4_PIN = 27;  // Auxiliary device 2

// Limit Switches (Safety/Homing)
static const uint8_t CLIENT_LIMIT_MIN_PIN = 35;  // Minimum position limit switch
static const uint8_t CLIENT_LIMIT_MAX_PIN = 36;  // Maximum position limit switch

// Reed Switch (Door closed detection)
static const uint8_t CLIENT_REED_SWITCH_PIN = 39;  // Reed switch for door state

// Communication & Networking
// WiFi credentials for WebSocket client connection
static const char* CLIENT_WIFI_SSID = "YourWiFiSSID";
static const char* CLIENT_WIFI_PASSWORD = "YourWiFiPassword";
static const char* CLIENT_WEBSOCKET_HOST = "192.168.1.100";  // Host ESP32 IP
static const uint16_t CLIENT_WEBSOCKET_PORT = 81;
static const char* CLIENT_WEBSOCKET_PATH = "/";

// Timing Constants
static const unsigned long CLIENT_MOTION_TIMEOUT = 60000;      // 60 seconds no motion = auto-close
static const unsigned long CLIENT_MOTOR_TIMEOUT = 30000;       // 30 seconds max motor operation
static const unsigned long CLIENT_WEBSOCKET_RETRY = 5000;      // 5 seconds between connection retries
static const unsigned long CLIENT_STATUS_UPDATE_INTERVAL = 1000; // Send status every second

// Motor Configuration
static const unsigned long CLIENT_DOOR_OPEN_STEPS = 2000;      // Steps to fully open door
static const unsigned long CLIENT_DOOR_CLOSE_STEPS = 2000;     // Steps to fully close door
static const unsigned long CLIENT_LOCK_ENGAGE_STEPS = 400;     // Steps to engage lock
static const unsigned long CLIENT_LOCK_RELEASE_STEPS = 400;    // Steps to release lock

// Motor Speed Settings (microseconds)
static const unsigned int CLIENT_MOTOR_PULSE_US = 100;         // Pulse width
static const unsigned int CLIENT_MOTOR_GAP_US = 800;           // Gap between pulses (speed control)

#endif // SMART_CABINET_CLIENT_PINS_H