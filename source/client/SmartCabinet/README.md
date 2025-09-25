# Smart Cabinet Client Controller

## Overview

The Smart Cabinet Client Controller is the mechanical control unit of the Smart Cabinet system. It handles all physical operations including motor control, sensor monitoring, safety systems, and mechanical actuations. The client communicates with the host controller via WebSocket connection.

## Architecture

```
HOST CONTROLLER (Authentication & Logic)
           |
    WebSocket Connection
           |
CLIENT CONTROLLER (Physical Operations)
           |
    Physical Hardware Layer
```

## Components Managed

### Motion Detection & User Interface
- **PIR Motion Sensor**: Detects user presence and movement
- **Auto-close Timer**: Automatically closes cabinet after inactivity

### Motor Control Systems
- **TB6600 Stepper Driver #1**: Door/drawer opening mechanism
- **TB6600 Stepper Driver #2**: Lock engagement/disengagement
- **Non-blocking Motor Control**: Allows concurrent operations
- **Emergency Stop System**: Immediate motor shutdown capability

### Relay-Controlled Actuators
- **Relay 1**: Solenoid lock control (failsafe locking)
- **Relay 2**: LED strip illumination
- **Relay 3**: Auxiliary device 1 (customizable)
- **Relay 4**: Auxiliary device 2 (customizable)

### Safety & Position Sensing
- **Limit Switches**: Emergency stops and motor homing
- **Reed Switch**: Door closed/open detection
- **Position Monitoring**: Ensures mechanical consistency

### Network Communication
- **WebSocket Client**: Real-time communication with host
- **JSON Messaging**: Structured command and status exchange
- **Auto-reconnection**: Maintains connection reliability

## Files Structure

```
source/client/
├── SmartCabinetClient.ino     # Main controller logic
├── ClientModules.ino          # Extended functions and utilities
├── client_pins.h              # Pin configuration and constants
├── LimitSwitch.h/.cpp         # Limit switch handling
├── ReedSwitch.h/.cpp          # Reed switch monitoring
├── Relay4.h/.cpp              # 4-channel relay control
├── TB6600.h/.cpp              # Stepper motor driver
├── MotionSensor.h/.cpp        # PIR motion detection
├── WebSocketClient.h/.cpp     # Network communication
└── README.md                  # This documentation
```

## Hardware Requirements

### Microcontroller
- **ESP32 Development Board** (recommended)
- Minimum 4MB Flash memory
- Built-in WiFi capability

### Motor Control
- **2x TB6600 Stepper Motor Drivers**
- **2x NEMA 17 Stepper Motors** (or compatible)
- **Separate 12V-24V Power Supply** for motors (2-5A capacity)

### Sensors & Switches
- **PIR Motion Sensor** (HC-SR501 or similar)
- **2x Limit Switches** (normally open, mechanical)
- **Reed Switch with Magnet** (door position sensing)

### Actuators & Output
- **4-Channel Relay Module** (5V or 3.3V compatible)
- **12V Solenoid Lock** (fail-secure type recommended)
- **12V LED Strip** (optional, for cabinet illumination)

### Power Requirements
- **5V 2A** for ESP32 and logic components
- **12V-24V 2-5A** for stepper motors (depends on motor specifications)
- **12V 1A** for solenoids and LED strips

## Pin Configuration

The pin assignments are defined in `client_pins.h`:

```cpp
// Motion Detection
#define CLIENT_PIR_PIN              34

// Stepper Motor Control
#define CLIENT_TB1_DIR              2    // Door motor direction
#define CLIENT_TB1_STEP             4    // Door motor step
#define CLIENT_TB1_ENABLE           15   // Door motor enable

#define CLIENT_TB2_DIR              32   // Lock motor direction  
#define CLIENT_TB2_STEP             33   // Lock motor step
#define CLIENT_TB2_ENABLE           5    // Lock motor enable

// Relay Control
#define CLIENT_RELAY1_PIN           12   // Solenoid lock
#define CLIENT_RELAY2_PIN           13   // LED strip
#define CLIENT_RELAY3_PIN           14   // Auxiliary 1
#define CLIENT_RELAY4_PIN           27   // Auxiliary 2

// Safety Switches
#define CLIENT_LIMIT_MIN_PIN        35   // Minimum position
#define CLIENT_LIMIT_MAX_PIN        36   // Maximum position
#define CLIENT_REED_SWITCH_PIN      39   // Door closed sensor
```

## Software Dependencies

### Arduino Libraries Required
```cpp
#include <WiFi.h>              // ESP32 WiFi (built-in)
#include <ArduinoJson.h>       // JSON processing
#include <ArduinoWebsockets.h> // WebSocket communication
```

### Installation Commands
```bash
# In Arduino IDE Library Manager, install:
# - ArduinoJson by Benoit Blanchon
# - ArduinoWebsockets by Gil Maimon
```

## Configuration

### Network Settings
Update the following in `client_pins.h`:

```cpp
static const char* CLIENT_WIFI_SSID = "YourWiFiNetwork";
static const char* CLIENT_WIFI_PASSWORD = "YourPassword";
static const char* CLIENT_WEBSOCKET_HOST = "192.168.1.100";  // Host IP
static const uint16_t CLIENT_WEBSOCKET_PORT = 81;
```

### Motor Configuration
Adjust motor parameters based on your hardware:

```cpp
// Steps for full door operation
static const unsigned long CLIENT_DOOR_OPEN_STEPS = 2000;
static const unsigned long CLIENT_DOOR_CLOSE_STEPS = 2000;

// Steps for lock operation  
static const unsigned long CLIENT_LOCK_ENGAGE_STEPS = 400;
static const unsigned long CLIENT_LOCK_RELEASE_STEPS = 400;

// Motor speed (microseconds)
static const unsigned int CLIENT_MOTOR_PULSE_US = 100;  // Pulse width
static const unsigned int CLIENT_MOTOR_GAP_US = 800;    // Speed control
```

### Timing Parameters
```cpp
static const unsigned long CLIENT_MOTION_TIMEOUT = 60000;      // Auto-close timer
static const unsigned long CLIENT_MOTOR_TIMEOUT = 30000;       // Motor operation limit
static const unsigned long CLIENT_WEBSOCKET_RETRY = 5000;      // Reconnection interval
```

## System States

The client operates in the following states:

1. **CLIENT_IDLE**: Waiting for commands from host
2. **CLIENT_OPENING**: Executing door opening sequence
3. **CLIENT_OPEN**: Door is open, monitoring for motion
4. **CLIENT_CLOSING**: Executing door closing sequence  
5. **CLIENT_LOCKED**: Door is closed and locked (secure state)
6. **CLIENT_ERROR**: Emergency condition, all operations stopped

## Operation Sequence

### Door Opening Process
1. **Authentication Request** → Host validates fingerprint
2. **Unlock Command** → Client receives open command
3. **Solenoid Release** → Relay 1 energizes solenoid
4. **Mechanical Unlock** → Lock motor disengages mechanism
5. **Door Opening** → Door motor opens cabinet
6. **Illumination** → LED strip turns on
7. **Motion Monitoring** → PIR sensor monitors activity

### Auto-Close Process
1. **Motion Timer** → No motion detected for set period
2. **LED Warning** → Brief LED flash (optional)
3. **Door Closing** → Door motor closes cabinet
4. **Position Check** → Reed switch confirms closure
5. **Mechanical Lock** → Lock motor engages mechanism
6. **Solenoid Lock** → Relay 1 de-energizes solenoid
7. **Secure State** → Return to locked state

## Safety Features

### Emergency Stop System
- **Limit Switch Triggers**: Immediate motor shutdown
- **Motor Timeouts**: Prevents stuck motor conditions
- **Emergency Command**: Remote emergency stop via WebSocket
- **Power Failure**: Fail-secure solenoid locks cabinet

### Position Monitoring
- **Reed Switch**: Confirms door position
- **Limit Switches**: Prevents over-travel
- **Consistency Checks**: Validates expected vs actual states

### Error Recovery
- **Automatic Retry**: Network reconnection attempts
- **State Recovery**: Resume from last known good state
- **Manual Override**: Emergency unlock procedures

## Communication Protocol

### WebSocket Message Format
```json
{
  "type": "command|status|diagnostic|error",
  "timestamp": 1234567890,
  "command": "unlock_and_open|force_close|emergency_stop",
  "data": { ... }
}
```

### Status Updates
The client sends periodic status updates including:
- Current system state
- Door and lock positions
- Sensor readings
- Motor status
- Network connection quality

### Command Processing
Supported commands from host:
- `unlock_and_open`: Start door opening sequence
- `force_close`: Immediate door closing
- `emergency_stop`: Stop all operations
- `test_motors`: Run motor diagnostics
- `calibrate_door`: Position calibration
- `get_diagnostics`: Request system diagnostics

## Troubleshooting

### Motor Issues
```
Problem: Motors not moving
Check: Power supply connections
Check: TB6600 wiring (DIR, STEP, ENABLE pins)
Check: Motor power supply voltage
Check: Enable pin logic (LOW = enabled)
```

### Network Issues
```
Problem: WebSocket connection failed
Check: WiFi credentials in client_pins.h
Check: Host IP address and port
Check: Network firewall settings
Check: Host controller running and listening
```

### Sensor Issues
```
Problem: Reed switch not detecting door
Check: Magnet alignment with reed switch
Check: Wiring and pin configuration
Check: Switch type (normally open vs normally closed)

Problem: Motion sensor false triggers
Check: PIR sensor sensitivity adjustment
Check: Mounting position and angle
Check: Debounce timing in code
```

### Safety Switch Issues
```
Problem: Limit switches triggering unexpectedly
Check: Switch mounting and alignment
Check: Mechanical interference
Check: Wiring and pull-up resistors
Check: Switch type configuration (normally open/closed)
```

## Testing and Calibration

### Initial Setup Test
1. Upload code to ESP32
2. Check serial monitor for initialization messages
3. Verify WiFi connection
4. Test WebSocket connection to host
5. Run motor test sequence
6. Verify all sensor readings

### Motor Calibration
1. Use `calibrate_door` command
2. Manually check door positions
3. Adjust step counts in configuration
4. Test opening/closing sequences
5. Verify limit switch operation

### Safety Testing
1. Test emergency stop function
2. Verify limit switch operation
3. Check failsafe lock behavior
4. Test power failure recovery

## Maintenance

### Regular Checks
- **Weekly**: Verify door operation and sensor function
- **Monthly**: Check motor alignment and lubrication
- **Quarterly**: Inspect wiring connections
- **Annually**: Replace mechanical wear components

### Software Updates
- Monitor system logs for errors
- Update WiFi credentials if network changes
- Adjust timing parameters based on usage patterns
- Update motor parameters if hardware changes

## Integration Notes

### Host Controller Requirements
The client requires a host controller running:
- WebSocket server on configured port
- Fingerprint authentication system
- Command processing and user management
- System monitoring and logging

### Network Requirements
- Stable WiFi connection
- Local network access between host and client
- Firewall configuration for WebSocket port
- Optional: Static IP assignment for reliability

## Development and Customization

### Adding New Sensors
1. Create new sensor class files (.h/.cpp)
2. Add pin definitions to client_pins.h
3. Initialize sensor in setup()
4. Update sensor readings in loop()
5. Add sensor data to status messages

### Motor Speed Tuning
Adjust these parameters for optimal performance:
```cpp
CLIENT_MOTOR_PULSE_US  // Shorter = faster, but may cause missed steps
CLIENT_MOTOR_GAP_US    // Shorter = faster, but may cause motor stall
```

### Adding Custom Commands
1. Add command processing in ClientModules.ino
2. Update processCommand() function
3. Add corresponding response messages
4. Test command via WebSocket

## Support and Contact

For technical support, customization, or questions:

- **Email**: quezon.province.pd@gmail.com
- **GitHub**: [github.com/qppd](https://github.com/qppd)
- **Portfolio**: [sajed-mendoza.onrender.com](https://sajed-mendoza.onrender.com)

## License and Credits

**Author**: QPPD (Sajed Mendoza)  
**Project**: Smart Cabinet System  
**Version**: 1.0  
**Year**: 2025  

This project is part of the Smart Cabinet automation system designed for secure, intelligent storage solutions.