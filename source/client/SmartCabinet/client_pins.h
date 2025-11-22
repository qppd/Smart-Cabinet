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
// - ESP-NOW communication with host
//
// NOTES ON PIN SELECTION:
// - Avoided GPIO 0, 2, 5, 12, 15 (strapping pins - can cause boot issues)
// - GPIO 6-11 are flash pins (NEVER USE)
// - GPIO 34-39 are input-only (OK for sensors)
// - Safe pins: 4, 13, 14, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33

// PIR Motion Sensor
static const uint8_t CLIENT_PIR_PIN = 19; // Input-only pin - OK for sensor

// TB6600 Stepper Motor Driver #1 (Door/Drawer Motor)
static const uint8_t CLIENT_TB1_DIR = 25;     // Direction pin (safe pin)
static const uint8_t CLIENT_TB1_STEP = 26;    // Step pulse pin (safe pin)
static const uint8_t CLIENT_TB1_ENABLE = 27;  // Enable pin (LOW = enabled)

// TB6600 Stepper Motor Driver #2 (Lock/Mechanism Motor)  
static const uint8_t CLIENT_TB2_DIR = 32;     // Direction pin (safe pin)
static const uint8_t CLIENT_TB2_STEP = 33;    // Step pulse pin (safe pin)
static const uint8_t CLIENT_TB2_ENABLE = 39;  // Enable pin (LOW = enabled)

// 4-Channel Relay Module (Controls solenoids, LED strips, etc.)
static const uint8_t CLIENT_RELAY1_PIN = 34;  // Solenoid lock control
static const uint8_t CLIENT_RELAY2_PIN = 13;  // LED strip power
static const uint8_t CLIENT_RELAY3_PIN = 22;  // Auxiliary device 1
static const uint8_t CLIENT_RELAY4_PIN = 23;  // Auxiliary device 2

// Limit Switches (Safety/Homing)
static const uint8_t CLIENT_LIMIT_MIN_PIN = 5;  // Minimum position limit switch - Input-only, OK
static const uint8_t CLIENT_LIMIT_MAX_PIN = 36;  // Maximum position limit switch - Input-only, OK

// Reed Switch (Door closed detection)
static const uint8_t CLIENT_REED_SWITCH_PIN = 18;  // Reed switch - Input-only, OK

// Timing Constants
static const unsigned long CLIENT_MOTION_TIMEOUT = 60000;      // 60 seconds no motion = auto-close
static const unsigned long CLIENT_MOTOR_TIMEOUT = 30000;       // 30 seconds max motor operation
static const unsigned long CLIENT_STATUS_UPDATE_INTERVAL = 1000; // Send status every second

// Motor Configuration
// NEMA23 Stepper Motor: 3.0A, 1.8°/step (200 steps/revolution)
// 1 full rotation = 200 steps
// With 1/8 microstepping on TB6600 = 1600 steps/revolution
static const unsigned long CLIENT_DOOR_OPEN_STEPS = 52000;     // Steps to fully open door (adjust based on your cabinet)
static const unsigned long CLIENT_DOOR_CLOSE_STEPS = 60000;    // Max steps for closing (will stop at limit switch)
static const unsigned long CLIENT_LOCK_ENGAGE_STEPS = 400;     // Steps to engage lock
static const unsigned long CLIENT_LOCK_RELEASE_STEPS = 400;    // Steps to release lock

// Motor Speed Settings (microseconds)
// NEMA23 3.0A motor can handle faster speeds than smaller motors
static const unsigned int CLIENT_MOTOR_PULSE_US = 50;          // Pulse width (min ~5us for TB6600)
static const unsigned int CLIENT_MOTOR_GAP_US = 500;           // Gap between pulses (faster = lower value)

#endif // SMART_CABINET_CLIENT_PINS_H