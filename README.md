# Smart Cabinet

## Table of Contents
1. [Introduction](#introduction)
2. [Features](#features)
3. [System Architecture](#system-architecture)
4. [Setup Instructions](#setup-instructions)
5. [Operation](#operation)
6. [Testing and Commissioning](#testing-and-commissioning)
7. [Troubleshooting](#troubleshooting)
8. [Files of Interest](#files-of-interest)

---

## Introduction
The Smart Cabinet is an advanced storage solution designed to enhance security and automation. It integrates fingerprint authentication, motion detection, and automated mechanisms to ensure secure and efficient operation. This project is ideal for applications requiring controlled access and intelligent automation.

---

## Features
- **Secure Access**: Fingerprint authentication using the Adafruit AS608 sensor.
- **Automated Mechanisms**: Stepper motors for opening/closing doors, controlled by TB6600 drivers.
- **Real-Time Feedback**: I2C LCD for user messages and a buzzer for audible alerts.
- **Energy Efficiency**: Auto-close functionality with motion detection.
- **Safety Mechanisms**: Reed switch for door-closed detection and limit switches for motor homing.

---

## System Architecture
### Hardware Components
- **Microcontroller**: ESP32 (recommended).
- **Sensors**: Fingerprint sensor, PIR motion sensor, reed switch, and limit switches.
- **Actuators**: Stepper motors, solenoids, and an LED strip.
- **Drivers**: TB6600 for stepper motors, relay board for solenoids and LED strip.
- **Display**: I2C LCD for user interaction.
- **Feedback**: Buzzer for alerts.

### Software Components
- **Main Control**: `Main.ino` orchestrates the system.
- **Helper Classes**: Modular components for sensors and actuators in the `components/` directory.
- **Libraries**:
  - `LiquidCrystal_I2C`
  - `Adafruit Fingerprint Sensor Library`

---

## Setup Instructions
### Wiring
- Ensure common grounds between all components.
- Use a separate power supply for TB6600 drivers and stepper motors.
- Refer to `pins.h` for pin mappings.

### Software Setup
1. Install the required libraries in the Arduino IDE or PlatformIO.
2. Open `Main.ino` and configure the `pins.h` file for your hardware setup.
3. Upload the code to the ESP32.

---

## Operation
### Authentication
1. Place a finger on the sensor.
2. If authenticated, the system unlocks and opens the cabinet.
3. LED strip turns on, and a motion inactivity timer starts.

### Enrollment
1. Trigger enrollment mode via a button or special fingerprint sequence.
2. Follow LCD prompts to enroll a new fingerprint.

### Auto-Close
1. If no motion is detected for 60 seconds, the cabinet closes automatically.
2. Reed switch confirms the door is fully closed.

---

## Testing and Commissioning
1. Verify pin mappings in `pins.h`.
2. Test individual components (e.g., fingerprint sensor, motors, relays).
3. Simulate workflows to ensure proper operation.
4. Test auto-close functionality with motion detection.

---

## Troubleshooting
- **Motor Issues**: Check TB6600 connections and power supply.
- **Fingerprint Sensor Not Responding**: Verify wiring and test with `verifySensor()`.
- **Reed Switch Not Triggering**: Ensure proper alignment and wiring.

---

## Files of Interest
- `Main.ino`: Main control logic.
- `pins.h`: Pin mappings and constants.
- `components/`: Modular classes for sensors and actuators.

---

For further assistance, refer to the detailed comments in the codebase or contact the project maintainer.
