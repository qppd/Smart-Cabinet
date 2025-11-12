# Smart Cabinet Host Controller

## Overview

The Smart Cabinet Host Controller is the authentication and control unit of the Smart Cabinet system. It manages user authentication, system control logic, user interface, and communication with the client controller. The host handles all security-related operations and provides user interaction through biometric authentication and visual feedback.

## Architecture

```
USER INTERACTION (Fingerprint + LCD + Button)
           |
HOST CONTROLLER (Authentication & Logic)
           |
    Communication Layer (WebSocket/ESP-NOW)
           |
CLIENT CONTROLLER (Physical Operations)
```

## Components Managed

### Authentication System
- **Fingerprint Sensor (AS608)**: Biometric user authentication
- **User Enrollment**: Store and manage authorized fingerprints
- **Verification System**: Real-time fingerprint matching

### User Interface
- **I2C LCD Display (16x2 or 20x4)**: System status and user prompts
- **Tactile Button**: User input and enrollment trigger
- **Visual Feedback**: Status messages and authentication results

### Time Management
- **NTP Time Client**: Network time synchronization
- **RTC Support**: Optional real-time clock integration
- **Timestamp Logging**: Event tracking with accurate time

### Communication
- **ESP-NOW Communication**: Direct ESP32-to-ESP32 messaging
- **WebSocket Server**: Real-time client communication
- **JSON Messaging**: Structured command protocol

### Audio Feedback
- **Buzzer Module**: Audio alerts and status indicators
- **Configurable Tones**: Success, error, and warning sounds

## Files Structure

```
source/host/SmartCabinet/
├── SmartCabinet.ino          # Main controller logic
├── pins.h                    # Pin configuration and constants
├── FingerprintAS608.h/.cpp   # Fingerprint sensor interface
├── I2CLcd.h/.cpp             # LCD display control
├── TactileButton.h/.cpp      # Button input handling
├── Buzzer.h/.cpp             # Audio feedback
├── NTPTime.h/.cpp            # Time synchronization
├── ESPNowComm.h/.cpp         # ESP-NOW communication
├── ButtonTest.ino.example    # Button testing example
└── README.md                 # This documentation
```

## Hardware Requirements

### Microcontroller
- **ESP32 Development Board** (recommended)
- Minimum 4MB Flash memory
- Built-in WiFi capability

### Authentication & Input
- **AS608 Fingerprint Sensor** (optical or capacitive)
- **Tactile Push Button** (momentary switch)
- **Pull-up/Pull-down Resistor** for button (if not using internal pull-up)

### Display & Feedback
- **I2C LCD Display** (16x2 or 20x4 character)
  - Compatible with HD44780 controller
  - I2C interface (PCF8574 backpack)
- **Passive or Active Buzzer** (5V compatible)

### Communication
- **WiFi Network** for NTP time sync
- **Local Network** for WebSocket communication (optional)
- **Direct ESP-NOW** for client communication (alternative)

### Power Requirements
- **5V 2A** for ESP32 and peripherals
- **Regulated 3.3V** for fingerprint sensor (if required)
- **Stable Power Supply** for reliable fingerprint readings

## Pin Configuration

Pin assignments are defined in `pins.h`. Refer to that file for specific GPIO mappings for:
- Fingerprint sensor UART (TX/RX)
- I2C LCD (SDA/SCL)
- Tactile button input
- Buzzer output
- Status indicators (optional)

## Software Dependencies

### Arduino Libraries Required
```cpp
#include <WiFi.h>              // ESP32 WiFi (built-in)
#include <Wire.h>              // I2C communication (built-in)
#include <LiquidCrystal_I2C.h> // I2C LCD display
#include <Adafruit_Fingerprint.h> // AS608 fingerprint sensor
#include <NTPClient.h>         // Network time protocol
#include <ArduinoJson.h>       // JSON processing (optional)
```

### Installation Commands
```bash
# In Arduino IDE Library Manager, install:
# - LiquidCrystal I2C by Frank de Brabander
# - Adafruit Fingerprint Sensor Library
# - NTPClient by Fabrice Weinberg
# - ArduinoJson by Benoit Blanchon (if using WebSocket)
```

## Configuration

### Network Settings
Update WiFi credentials for NTP time synchronization:
```cpp
const char* WIFI_SSID = "YourWiFiNetwork";
const char* WIFI_PASSWORD = "YourPassword";
```

### I2C LCD Configuration
Set your LCD dimensions and I2C address:
```cpp
#define LCD_COLUMNS 16
#define LCD_ROWS 2
#define LCD_ADDRESS 0x27  // Common: 0x27 or 0x3F
```

### Fingerprint Sensor Settings
Configure UART communication:
```cpp
#define FINGERPRINT_RX_PIN 16
#define FINGERPRINT_TX_PIN 17
#define FINGERPRINT_BAUD 57600
```

## System States

The host operates in the following states:

1. **IDLE**: Waiting for user interaction (button press or fingerprint)
2. **AUTHENTICATING**: Processing fingerprint scan
3. **AUTHENTICATED**: Valid user identified, sending unlock command
4. **ENROLLING**: Adding new fingerprint to database
5. **ERROR**: Authentication failed or system error
6. **LOCKED**: System in secure state

## Operation Sequence

### Authentication Process
1. **Idle Display** → LCD shows "Place Finger" prompt
2. **Button Press** → Optional wake-up trigger
3. **Finger Detection** → Sensor detects finger placement
4. **Image Capture** → Fingerprint image acquired
5. **Template Matching** → Compare against stored templates
6. **Authentication Result**:
   - **Success** → LCD shows "Access Granted", buzzer beep, send unlock command
   - **Failure** → LCD shows "Access Denied", error tone

### Enrollment Process
1. **Long Button Press** → Enter enrollment mode (5+ seconds)
2. **LCD Prompt** → "Enroll: Place Finger"
3. **First Scan** → Capture initial fingerprint image
4. **Remove Finger** → LCD shows "Remove Finger"
5. **Second Scan** → Confirm fingerprint match
6. **Template Storage** → Save to next available slot
7. **Completion** → LCD shows "Enrolled: ID #X"

### Time Synchronization
1. **WiFi Connection** → Connect to configured network
2. **NTP Query** → Request time from NTP server
3. **Time Update** → Synchronize system clock
4. **Periodic Sync** → Auto-update every 24 hours

## Component Testing Status

### Verified Working Components ✓

#### **Fingerprint Sensor (AS608)** - TESTED & WORKING
- **Model**: AS608 Optical/Capacitive Fingerprint Scanner
- **Communication**: UART interface with ESP32
- **Features Verified**:
  - Fingerprint enrollment (template creation and storage)
  - Fingerprint authentication (template matching)
  - Image capture and processing
  - Template database management
- **Performance**: Fast recognition (<1 second), reliable accuracy
- **Status**: ✅ OPERATIONAL - Successfully enrolling and authenticating users

#### **I2C LCD Display** - TESTED & WORKING
- **Configuration**: I2C interface with PCF8574 backpack
- **Display Type**: 16x2 or 20x4 character LCD
- **Features Verified**:
  - Text display and positioning
  - Cursor control and visibility
  - Backlight control
  - Multi-line text updates
  - Custom character support (if needed)
- **Performance**: Clear text rendering, responsive updates
- **Status**: ✅ OPERATIONAL - Displaying system messages and user prompts correctly

#### **Tactile Button** - TESTED & WORKING
- **Configuration**: Digital input with internal pull-up resistor
- **Features Verified**:
  - Button press detection
  - Debouncing algorithm
  - Long-press detection (enrollment mode trigger)
  - State change callbacks
- **Performance**: Reliable press detection, no false triggers
- **Status**: ✅ OPERATIONAL - Accurate input detection with proper debouncing

### Components Pending Testing

#### **Buzzer Module** - STATUS UNKNOWN
- **Purpose**: Audio feedback for authentication and system events
- **Expected Functionality**: Beep tones for success, error, and warnings
- **Status**: ⏳ TESTING NEEDED

#### **NTP Time Client** - STATUS UNKNOWN
- **Purpose**: Network time synchronization for event logging
- **Expected Functionality**: WiFi connection and NTP server queries
- **Status**: ⏳ TESTING NEEDED

#### **ESP-NOW Communication** - STATUS UNKNOWN
- **Purpose**: Direct communication with client controller
- **Expected Functionality**: Send unlock commands and receive status
- **Status**: ⏳ TESTING NEEDED

### Testing Summary

The core authentication and user interface components have been successfully validated:

1. **Fingerprint Sensor**: Verified enrollment and authentication workflows
2. **LCD Display**: Confirmed text display and user feedback capability
3. **Tactile Button**: Validated input detection and enrollment trigger

These working components form the complete user interaction layer, enabling secure biometric access control with visual feedback.

## Fingerprint Management

### Enrolling New Users
1. Long-press the tactile button (5+ seconds)
2. Follow LCD prompts to scan finger twice
3. Note the assigned ID number
4. Test authentication immediately

### Deleting Users
Use serial commands or implement delete function:
```cpp
deleteFingerprint(ID);  // Remove specific user
emptyDatabase();        // Clear all users
```

### Viewing Stored Templates
```cpp
getFingerprintCount();  // Get number of enrolled users
getTemplateCount();     // Check storage capacity
```

## Troubleshooting

### Fingerprint Sensor Issues
```
Problem: Sensor not responding
Check: UART wiring (TX/RX, ensure crossover)
Check: Baud rate configuration (typically 57600)
Check: Power supply voltage (3.3V or 5V depending on model)
Check: Serial monitor for initialization messages
```

### LCD Display Issues
```
Problem: LCD shows no text or garbled characters
Check: I2C address (scan using I2C scanner sketch)
Check: SDA/SCL connections
Check: Contrast adjustment (potentiometer on I2C backpack)
Check: Power supply (5V to LCD module)
```

### Button Issues
```
Problem: Button not responding
Check: Pull-up resistor configuration
Check: Pin mode (INPUT_PULLUP)
Check: Physical button connection
Check: Debounce timing in code
```

## Maintenance

### Regular Checks
- **Weekly**: Clean fingerprint sensor surface
- **Monthly**: Verify all enrolled users still authenticate
- **Quarterly**: Check LCD display clarity and backlight
- **Annually**: Re-enroll users if recognition degrades

### Fingerprint Sensor Maintenance
- Keep sensor surface clean and dry
- Avoid excessive force during scanning
- Store templates periodically if sensor supports it
- Replace sensor if optical element degrades

## Development and Customization

### Adding Authentication Logging
Implement event logging with timestamps:
```cpp
void logAuthenticationEvent(int userID, bool success) {
  String timestamp = getFormattedTime();
  String event = success ? "GRANTED" : "DENIED";
  // Log to SD card, EEPROM, or send to server
}
```

### Custom LCD Messages
Modify display messages in code:
```cpp
lcd.setCursor(0, 0);
lcd.print("Custom Message");
```

### Multi-Level Access Control
Implement user permission levels:
```cpp
enum AccessLevel { ADMIN, USER, GUEST };
AccessLevel userPermissions[MAX_USERS];
```

## Security Considerations

### Best Practices
- **Enrollment Security**: Restrict enrollment mode to authorized personnel
- **Template Protection**: Fingerprint templates are stored locally on sensor
- **Physical Security**: Protect ESP32 and sensor from tampering
- **Network Security**: Use encrypted communication for remote management

### Privacy Notes
- Fingerprint templates cannot be reverse-engineered to original fingerprint
- No biometric data is transmitted over network
- Templates stored in sensor's flash memory only

## Support and Contact

For technical support, customization, or questions:

- **Email**: quezon.province.pd@gmail.com
- **GitHub**: [github.com/qppd](https://github.com/qppd)
- **Portfolio**: [sajed-mendoza.onrender.com](https://sajed-mendoza.onrender.com)

## License and Credits

**Author**: QPPD (Sajed Mendoza)  
**Project**: Smart Cabinet System - Host Controller  
**Version**: 1.0  
**Year**: 2025  

This project is part of the Smart Cabinet automation system designed for secure, intelligent storage solutions with biometric access control.
