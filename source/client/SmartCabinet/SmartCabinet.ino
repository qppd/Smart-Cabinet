/*
  ===============================================
  SMART CABINET CLIENT CONTROLLER
  ===============================================
  
  This ESP32 acts as the client controller for the Smart Cabinet system.
  It handles:
  - Motion detection and user interaction
  - Stepper motor control for door/drawer operations
  - Relay control for solenoids and LED strips
  - Safety switches (limit switches, reed switch)
  - WebSocket communication with the host controller
  
  Components managed:
  - PIR Motion Sensor
  - TB6600 Stepper Motor Drivers (x2)
  - 4-Channel Relay Module
  - Limit Switches (safety/homing)
  - Reed Switch (door position)
  - WebSocket Client
  
  Author: QPPD
  Version: 1.0
  Date: 2025
*/

#include <WiFi.h>
#include <ArduinoJson.h>
#include "client_pins.h"
#include "MotionSensor.h"
#include "TB6600.h" 
#include "Relay4.h"
#include "LimitSwitch.h"
#include "ReedSwitch.h"
#include "WebSocketClient.h"

// ===========================================
// COMPONENT INSTANCES
// ===========================================

// Motion detection
MotionSensor motionSensor(CLIENT_PIR_PIN);

// Motor controllers
TB6600 doorMotor(CLIENT_TB1_DIR, CLIENT_TB1_STEP, CLIENT_TB1_ENABLE);
TB6600 lockMotor(CLIENT_TB2_DIR, CLIENT_TB2_STEP, CLIENT_TB2_ENABLE);

// Relay control (only relay 1 on pin 13)
Relay4 relayBoard(13, CLIENT_RELAY2_PIN, CLIENT_RELAY3_PIN, CLIENT_RELAY4_PIN);

// Safety switches (only one limit switch on pin 35)
LimitSwitch limitSwitch(35);
ReedSwitch reedSwitch(CLIENT_REED_SWITCH_PIN);

// Network communication
WebSocketClient wsClient;

// ===========================================
// SYSTEM STATE VARIABLES
// ===========================================

enum ClientState {
  CLIENT_IDLE,
  CLIENT_OPENING,
  CLIENT_OPEN,
  CLIENT_CLOSING,
  CLIENT_LOCKED,
  CLIENT_ERROR
};

ClientState currentState = CLIENT_IDLE;
ClientState previousState = CLIENT_IDLE;

// Timing variables
unsigned long lastMotionTime = 0;
unsigned long lastStatusUpdate = 0;
unsigned long motorStartTime = 0;
unsigned long lastWifiCheck = 0;

// Flags
bool doorIsOpen = false;
bool lockEngaged = true;
bool motionDetected = false;
bool emergencyStop = false;
bool systemEnabled = true;

// Motor operation flags
bool doorMotorRunning = false;
bool lockMotorRunning = false;

// ===========================================
// CALLBACK FUNCTIONS
// ===========================================

void onMotionDetected(bool motion) {
  motionDetected = motion;
  if (motion) {
    lastMotionTime = millis();
    Serial.println("[CLIENT] Motion detected!");
    
    // Send motion event to host
    if (wsClient.isConnected()) {
      DynamicJsonDocument doc(200);
      doc["type"] = "motion";
      doc["detected"] = motion;
      doc["timestamp"] = millis();
      String message;
      serializeJson(doc, message);
      wsClient.sendMessage(message);
    }
  }
}

void onLimitSwitchTriggered(bool pressed) {
  if (pressed) {
    Serial.println("[CLIENT] EMERGENCY: Limit switch triggered!");
    emergencyStop = true;
    stopAllMotors();
  }
}

// ===========================================
// INITIALIZATION
// ===========================================

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("===============================================");
  Serial.println("SMART CABINET CLIENT CONTROLLER STARTING...");
  Serial.println("===============================================");
  
  // Initialize WiFi
  initializeWiFi();
  
  // Initialize hardware components
  initializeHardware();
  
  // Initialize WebSocket connection
  initializeWebSocket();
  
  Serial.println("[CLIENT] System initialization complete!");
  Serial.println("===============================================");
  
  // Print test commands help
  Serial.println();
  printTestHelp();
  
  // Initial state
  currentState = CLIENT_LOCKED;
  sendStatusUpdate();
}

// ===========================================
// MAIN LOOP
// ===========================================

void loop() {
  // Handle serial commands for testing
  handleSerialCommands();
  
  // Update all sensors and components
  updateComponents();
  
  // Handle state machine
  handleStateMachine();
  
  // Handle emergency conditions
  handleEmergencyConditions();
  
  // Maintain network connection
  maintainConnections();
  
  // Send periodic status updates
  sendPeriodicUpdates();
  
  // Small delay to prevent overwhelming the system
  delay(10);
}

// ===========================================
// INITIALIZATION FUNCTIONS
// ===========================================

void initializeWiFi() {
  Serial.print("[CLIENT] Connecting to WiFi: ");
  Serial.println(CLIENT_WIFI_SSID);
  
  WiFi.begin(CLIENT_WIFI_SSID, CLIENT_WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("[CLIENT] WiFi connected! IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("[CLIENT] WiFi connection failed! Operating in standalone mode.");
  }
}

void initializeHardware() {
  Serial.println("[CLIENT] Initializing hardware components...");
  
  // Initialize motion sensor
  motionSensor.begin();
  motionSensor.setCallback(onMotionDetected);
  Serial.println("[CLIENT] Motion sensor initialized");
  
  // Initialize motors
  doorMotor.begin();
  lockMotor.begin();
  doorMotor.enable(false); // Start disabled
  lockMotor.enable(false); // Start disabled  
  Serial.println("[CLIENT] Stepper motors initialized");
  
  // Initialize relay board
  relayBoard.begin();
  relayBoard.allOff(); // Start with all relays off
  Serial.println("[CLIENT] Relay board initialized");
  
  // Initialize switches
  limitSwitch.begin();
  limitSwitch.setCallback(onLimitSwitchTriggered);
  reedSwitch.begin();
  Serial.println("[CLIENT] Safety switches initialized");
  
  // Check initial door state
  doorIsOpen = !reedSwitch.isClosed();
  Serial.print("[CLIENT] Initial door state: ");
  Serial.println(doorIsOpen ? "OPEN" : "CLOSED");
}

void initializeWebSocket() {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[CLIENT] Connecting to WebSocket server...");
    wsClient.begin(CLIENT_WEBSOCKET_HOST, CLIENT_WEBSOCKET_PORT, CLIENT_WEBSOCKET_PATH);
  }
}

// ===========================================
// COMPONENT UPDATE FUNCTIONS  
// ===========================================

void updateComponents() {
  // Update all sensors and components
  motionSensor.update();
  doorMotor.update();
  lockMotor.update();
  limitSwitch.update();
  wsClient.loop();
  
  // Check motor completion status
  checkMotorCompletion();
}

void checkMotorCompletion() {
  // Check if door motor finished
  if (doorMotorRunning && !doorMotor.isBusy()) {
    doorMotorRunning = false;
    doorMotor.enable(false);
    Serial.println("[CLIENT] Door motor operation completed");
    
    // Update door state based on operation
    doorIsOpen = !reedSwitch.isClosed();
  }
  
  // Check if lock motor finished  
  if (lockMotorRunning && !lockMotor.isBusy()) {
    lockMotorRunning = false;
    lockMotor.enable(false);
    Serial.println("[CLIENT] Lock motor operation completed");
    
    // Update lock state (you may need additional sensors for this)
  }
}

// ===========================================
// STATE MACHINE
// ===========================================

void handleStateMachine() {
  switch (currentState) {
    case CLIENT_IDLE:
      handleIdleState();
      break;
      
    case CLIENT_OPENING:
      handleOpeningState();
      break;
      
    case CLIENT_OPEN:
      handleOpenState();
      break;
      
    case CLIENT_CLOSING:
      handleClosingState();
      break;
      
    case CLIENT_LOCKED:
      handleLockedState();
      break;
      
    case CLIENT_ERROR:
      handleErrorState();
      break;
  }
  
  // Send state change notifications
  if (currentState != previousState) {
    Serial.print("[CLIENT] State changed: ");
    Serial.print(stateToString(previousState));
    Serial.print(" -> ");
    Serial.println(stateToString(currentState));
    
    sendStatusUpdate();
    previousState = currentState;
  }
}

void handleIdleState() {
  // Waiting for authentication signal from host
  // Motion detection is active but no action taken until authorized
}

void handleOpeningState() {
  if (!doorMotorRunning) {
    // Start door opening sequence
    Serial.println("[CLIENT] Starting door opening sequence...");
    
    // Release solenoid lock first
    relayBoard.set(1, true); // Energize solenoid to release lock
    delay(500); // Give solenoid time to actuate
    
    // Disengage mechanical lock
    if (lockEngaged) {
      lockMotor.enable(true);
      lockMotor.setDirection(false); // Direction for unlocking
      lockMotor.startSteps(CLIENT_LOCK_RELEASE_STEPS, CLIENT_MOTOR_PULSE_US, CLIENT_MOTOR_GAP_US);
      lockMotorRunning = true;
      lockEngaged = false;
    }
    
    // Start door motor
    doorMotor.enable(true);
    doorMotor.setDirection(true); // Direction for opening
    doorMotor.startSteps(CLIENT_DOOR_OPEN_STEPS, CLIENT_MOTOR_PULSE_US, CLIENT_MOTOR_GAP_US);
    doorMotorRunning = true;
    motorStartTime = millis();
    
    // Turn on LED strip
    relayBoard.set(2, true);
  }
  
  // Check if door is fully open
  if (!doorMotorRunning && reedSwitch.isClosed() == false) {
    currentState = CLIENT_OPEN;
    lastMotionTime = millis(); // Reset motion timer
  }
  
  // Check for timeout
  if (millis() - motorStartTime > CLIENT_MOTOR_TIMEOUT) {
    Serial.println("[CLIENT] ERROR: Door opening timeout!");
    currentState = CLIENT_ERROR;
  }
}

void handleOpenState() {
  // Door is open, monitor for motion and auto-close timer
  
  // Check for auto-close condition (no motion for specified time)
  if (millis() - lastMotionTime > CLIENT_MOTION_TIMEOUT) {
    Serial.println("[CLIENT] Auto-close triggered - no motion detected");
    currentState = CLIENT_CLOSING;
  }
}

void handleClosingState() {
  if (!doorMotorRunning) {
    Serial.println("[CLIENT] Starting door closing sequence...");
    
    // Start door motor in closing direction
    doorMotor.enable(true);
    doorMotor.setDirection(false); // Direction for closing
    doorMotor.startSteps(CLIENT_DOOR_CLOSE_STEPS, CLIENT_MOTOR_PULSE_US, CLIENT_MOTOR_GAP_US);
    doorMotorRunning = true;
    motorStartTime = millis();
    
    // Turn off LED strip
    relayBoard.set(2, false);
  }
  
  // Check if door is fully closed
  if (!doorMotorRunning && reedSwitch.isClosed()) {
    // Engage mechanical lock
    lockMotor.enable(true);
    lockMotor.setDirection(true); // Direction for locking  
    lockMotor.startSteps(CLIENT_LOCK_ENGAGE_STEPS, CLIENT_MOTOR_PULSE_US, CLIENT_MOTOR_GAP_US);
    lockMotorRunning = true;
    lockEngaged = true;
    
    // Engage solenoid lock
    relayBoard.set(1, false); // De-energize solenoid to engage lock
    
    currentState = CLIENT_LOCKED;
    doorIsOpen = false;
  }
  
  // Check for timeout
  if (millis() - motorStartTime > CLIENT_MOTOR_TIMEOUT) {
    Serial.println("[CLIENT] ERROR: Door closing timeout!");
    currentState = CLIENT_ERROR;
  }
}

void handleLockedState() {
  // Door is locked, system is secure
  // Wait for unlock command from host
}

void handleErrorState() {
  // Stop all operations
  stopAllMotors();
  relayBoard.allOff();
  
  // Flash LED strip as error indicator
  static unsigned long lastFlash = 0;
  static bool ledState = false;
  
  if (millis() - lastFlash > 500) {
    ledState = !ledState;
    relayBoard.set(2, ledState);
    lastFlash = millis();
  }
  
  // System requires manual reset or host command to clear error
}

// ===========================================
// MOTOR CONTROL FUNCTIONS
// ===========================================

void stopAllMotors() {
  Serial.println("[CLIENT] EMERGENCY STOP - All motors stopped!");
  
  doorMotor.emergencyStop();
  lockMotor.emergencyStop();
  doorMotorRunning = false;
  lockMotorRunning = false;
  
  // Send emergency stop notification
  if (wsClient.isConnected()) {
    DynamicJsonDocument doc(200);
    doc["type"] = "emergency_stop";
    doc["timestamp"] = millis();
    String message;
    serializeJson(doc, message);
    wsClient.sendMessage(message);
  }
}

// ===========================================
// EMERGENCY HANDLING
// ===========================================

void handleEmergencyConditions() {
  // Check for emergency stop conditions
  if (emergencyStop) {
    currentState = CLIENT_ERROR;
    return;
  }
  
  // Check limit switches during motor operations
  if (doorMotorRunning || lockMotorRunning) {
    if (limitSwitch.isPressed()) {
      emergencyStop = true;
      return;
    }
  }
  
  // Check motor timeouts
  if ((doorMotorRunning || lockMotorRunning) && 
      (millis() - motorStartTime > CLIENT_MOTOR_TIMEOUT)) {
    Serial.println("[CLIENT] Motor operation timeout!");
    emergencyStop = true;
    return;
  }
}

// ===========================================
// NETWORK COMMUNICATION
// ===========================================

void maintainConnections() {
  // Check WiFi connection
  if (millis() - lastWifiCheck > 30000) { // Check every 30 seconds
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[CLIENT] WiFi disconnected, attempting reconnection...");
      WiFi.reconnect();
    }
    lastWifiCheck = millis();
  }
  
  // Check WebSocket connection
  if (!wsClient.isConnected() && WiFi.status() == WL_CONNECTED) {
    static unsigned long lastWSRetry = 0;
    if (millis() - lastWSRetry > CLIENT_WEBSOCKET_RETRY) {
      Serial.println("[CLIENT] Attempting WebSocket reconnection...");
      wsClient.begin(CLIENT_WEBSOCKET_HOST, CLIENT_WEBSOCKET_PORT, CLIENT_WEBSOCKET_PATH);
      lastWSRetry = millis();
    }
  }
}

void sendPeriodicUpdates() {
  if (millis() - lastStatusUpdate > CLIENT_STATUS_UPDATE_INTERVAL) {
    sendStatusUpdate();
    lastStatusUpdate = millis();
  }
}

void sendStatusUpdate() {
  if (!wsClient.isConnected()) return;
  
  DynamicJsonDocument doc(500);
  doc["type"] = "client_status";
  doc["timestamp"] = millis();
  doc["state"] = stateToString(currentState);
  doc["door_open"] = doorIsOpen;
  doc["lock_engaged"] = lockEngaged;
  doc["motion_detected"] = motionDetected;
  doc["reed_switch"] = reedSwitch.isClosed();
  doc["limit_switch"] = limitSwitch.isPressed();
  doc["door_motor_running"] = doorMotorRunning;
  doc["lock_motor_running"] = lockMotorRunning;
  doc["emergency_stop"] = emergencyStop;
  doc["wifi_rssi"] = WiFi.RSSI();
  
  String message;
  serializeJson(doc, message);
  wsClient.sendMessage(message);
}

// ===========================================
// UTILITY FUNCTIONS
// ===========================================

String stateToString(ClientState state) {
  switch (state) {
    case CLIENT_IDLE: return "IDLE";
    case CLIENT_OPENING: return "OPENING";
    case CLIENT_OPEN: return "OPEN";
    case CLIENT_CLOSING: return "CLOSING";
    case CLIENT_LOCKED: return "LOCKED";
    case CLIENT_ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}

// ===========================================
// PUBLIC API FUNCTIONS
// ===========================================

void unlockAndOpen() {
  if (currentState == CLIENT_LOCKED && !emergencyStop) {
    Serial.println("[CLIENT] Unlock and open command received");
    currentState = CLIENT_OPENING;
  }
}

void forceClose() {
  if (currentState == CLIENT_OPEN && !emergencyStop) {
    Serial.println("[CLIENT] Force close command received");  
    currentState = CLIENT_CLOSING;
  }
}

void clearEmergencyStop() {
  if (currentState == CLIENT_ERROR) {
    Serial.println("[CLIENT] Emergency stop cleared");
    emergencyStop = false;
    currentState = CLIENT_IDLE;
  }
}

void enableSystem(bool enable) {
  systemEnabled = enable;
  Serial.print("[CLIENT] System ");
  Serial.println(enable ? "ENABLED" : "DISABLED");
}

// ===========================================
// SERIAL COMMAND TESTING FUNCTIONS
// ===========================================

void handleSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();
    
    Serial.println();
    Serial.println("========================================");
    Serial.print("[TEST] Command received: ");
    Serial.println(command);
    Serial.println("========================================");
    
    if (command == "test_relay") {
      testRelay();
    } else if (command == "test_motor1") {
      testMotor(1);
    } else if (command == "test_motor2") {
      testMotor(2);
    } else if (command == "test_limit") {
      testLimitSwitch();
    } else if (command == "test_motion") {
      testMotionSensor();
    } else if (command == "test_reed") {
      testReedSwitch();
    } else if (command == "help") {
      printTestHelp();
    } else if (command != "") {
      Serial.println("[TEST] Unknown command. Type 'help' for available commands.");
    }
  }
}

void printTestHelp() {
  Serial.println();
  Serial.println("╔════════════════════════════════════════════════════════════╗");
  Serial.println("║          SMART CABINET TEST COMMANDS                       ║");
  Serial.println("╚════════════════════════════════════════════════════════════╝");
  Serial.println();
  Serial.println("Available test commands:");
  Serial.println("  test_relay    - Test relay 1 (Pin 13) - ON/OFF sequence");
  Serial.println("  test_motor1   - Test Motor 1 (Door Motor) - 2s CW, 2s CCW");
  Serial.println("  test_motor2   - Test Motor 2 (Lock Motor) - 2s CW, 2s CCW");
  Serial.println("  test_limit    - Test limit switch (Pin 35) - press 3x to pass");
  Serial.println("  test_motion   - Test motion sensor (trigger to pass)");
  Serial.println("  test_reed     - Test reed switch (trigger to pass)");
  Serial.println("  help          - Show this help message");
  Serial.println();
  Serial.println("════════════════════════════════════════════════════════════");
  Serial.println();
}

void testRelay() {
  Serial.println();
  Serial.println("┌────────────────────────────────────────┐");
  Serial.println("│      RELAY TEST STARTED                │");
  Serial.println("└────────────────────────────────────────┘");
  Serial.println();
  
  Serial.println("[TEST] Testing Relay 1 (Pin 13)...");
  
  // Turn ON
  Serial.println("  → Turning relay ON");
  relayBoard.set(1, true);
  delay(2000);
  
  // Turn OFF
  Serial.println("  → Turning relay OFF");
  relayBoard.set(1, false);
  delay(500);
  
  Serial.println("[TEST] Relay 1 test completed ✓");
  Serial.println();
  
  Serial.println("┌────────────────────────────────────────┐");
  Serial.println("│    RELAY TEST COMPLETED - SUCCESS ✓    │");
  Serial.println("└────────────────────────────────────────┘");
  Serial.println();
}

void testMotor(int motorNum) {
  Serial.println();
  Serial.print("┌────────────────────────────────────────┐");
  Serial.println();
  Serial.print("│      MOTOR ");
  Serial.print(motorNum);
  Serial.println(" TEST STARTED              │");
  Serial.println("└────────────────────────────────────────┘");
  Serial.println();
  
  TB6600* motor = (motorNum == 1) ? &doorMotor : &lockMotor;
  String motorName = (motorNum == 1) ? "Door Motor" : "Lock Motor";
  
  Serial.print("[TEST] Testing ");
  Serial.println(motorName);
  Serial.println();
  
  // Enable motor
  motor->enable(true);
  Serial.println("[TEST] Motor enabled");
  
  // Test CW direction
  Serial.println("[TEST] Running Clockwise for 2 seconds...");
  motor->setDirection(true);
  unsigned long startTime = millis();
  while (millis() - startTime < 2000) {
    motor->stepOnce(CLIENT_MOTOR_PULSE_US);
    delayMicroseconds(CLIENT_MOTOR_GAP_US);
  }
  Serial.println("[TEST] Clockwise rotation completed ✓");
  delay(500);
  
  // Test CCW direction
  Serial.println("[TEST] Running Counter-Clockwise for 2 seconds...");
  motor->setDirection(false);
  startTime = millis();
  while (millis() - startTime < 2000) {
    motor->stepOnce(CLIENT_MOTOR_PULSE_US);
    delayMicroseconds(CLIENT_MOTOR_GAP_US);
  }
  Serial.println("[TEST] Counter-Clockwise rotation completed ✓");
  
  // Disable motor
  motor->enable(false);
  Serial.println("[TEST] Motor disabled");
  Serial.println();
  
  Serial.print("┌────────────────────────────────────────┐");
  Serial.println();
  Serial.print("│   MOTOR ");
  Serial.print(motorNum);
  Serial.println(" TEST COMPLETED - SUCCESS ✓ │");
  Serial.println("└────────────────────────────────────────┘");
  Serial.println();
}

void testLimitSwitch() {
  Serial.println();
  Serial.println("┌────────────────────────────────────────┐");
  Serial.println("│    LIMIT SWITCH TEST STARTED           │");
  Serial.println("└────────────────────────────────────────┘");
  Serial.println();
  Serial.println("[TEST] Limit Switch wiring: COM-NO (Normally Open)");
  Serial.println("[TEST] NOT pressed = No continuity (HIGH)");
  Serial.println("[TEST] PRESSED = Continuity (LOW)");
  Serial.println("[TEST] Pin: 35");
  Serial.println();
  
  // Test Limit Switch
  Serial.println("[TEST] Testing Limit Switch (Pin 35)...");
  Serial.println("[TEST] Please press the limit switch (3 times)");
  int pressCount = 0;
  bool lastState = false;
  
  while (pressCount < 3) {
    limitSwitch.update();
    bool currentState = limitSwitch.isPressed();
    
    if (currentState && !lastState) {
      pressCount++;
      Serial.print("  → Press detected: ");
      Serial.print(pressCount);
      Serial.println("/3");
    }
    lastState = currentState;
    delay(50);
  }
  Serial.println("[TEST] Limit Switch test completed ✓");
  Serial.println();
  
  Serial.println("┌────────────────────────────────────────┐");
  Serial.println("│  LIMIT SWITCH TEST COMPLETED - SUCCESS ✓│");
  Serial.println("└────────────────────────────────────────┘");
  Serial.println();
}

void testMotionSensor() {
  Serial.println();
  Serial.println("┌────────────────────────────────────────┐");
  Serial.println("│    MOTION SENSOR TEST STARTED          │");
  Serial.println("└────────────────────────────────────────┘");
  Serial.println();
  Serial.println("[TEST] Motion sensor initialization...");
  Serial.println("[TEST] Please keep still and wait for sensor to stabilize...");
  Serial.println();
  
  // Wait for sensor to stabilize (warm-up period)
  for (int i = 5; i > 0; i--) {
    Serial.print("[TEST] Stabilizing... ");
    Serial.print(i);
    Serial.println(" seconds");
    delay(1000);
  }
  
  Serial.println();
  Serial.println("[TEST] Sensor stabilized!");
  Serial.println("[TEST] No motion should be detected now...");
  
  // Check that there's no motion
  bool motionClear = true;
  for (int i = 0; i < 20; i++) {
    motionSensor.update();
    if (motionSensor.isMotion()) {
      motionClear = false;
      Serial.println("[TEST] WARNING: Motion detected during stabilization!");
      Serial.println("[TEST] Please restart test and remain still.");
      return;
    }
    delay(100);
  }
  
  if (motionClear) {
    Serial.println("[TEST] ✓ No motion detected - Baseline established");
    Serial.println();
    Serial.println("[TEST] Now testing motion detection...");
    Serial.println("[TEST] Please move in front of the sensor!");
    Serial.println();
    
    // Wait for motion detection
    bool motionDetected = false;
    unsigned long testStart = millis();
    unsigned long timeout = 30000; // 30 second timeout
    
    while (!motionDetected && (millis() - testStart < timeout)) {
      motionSensor.update();
      if (motionSensor.isMotion()) {
        motionDetected = true;
        Serial.println("[TEST] ✓✓✓ MOTION DETECTED! ✓✓✓");
        break;
      }
      
      // Print waiting indicator every 2 seconds
      if ((millis() - testStart) % 2000 < 50) {
        Serial.println("[TEST] Waiting for motion...");
      }
      delay(50);
    }
    
    Serial.println();
    if (motionDetected) {
      Serial.println("┌────────────────────────────────────────┐");
      Serial.println("│  MOTION SENSOR TEST COMPLETED - SUCCESS ✓│");
      Serial.println("└────────────────────────────────────────┘");
    } else {
      Serial.println("┌────────────────────────────────────────┐");
      Serial.println("│  MOTION SENSOR TEST FAILED - TIMEOUT   │");
      Serial.println("└────────────────────────────────────────┘");
    }
    Serial.println();
  }
}

void testReedSwitch() {
  Serial.println();
  Serial.println("┌────────────────────────────────────────┐");
  Serial.println("│      REED SWITCH TEST STARTED          │");
  Serial.println("└────────────────────────────────────────┘");
  Serial.println();
  Serial.println("[TEST] Reed switch initialization...");
  Serial.println("[TEST] Reed Switch wiring: COM-NO (Normally Open)");
  Serial.println("[TEST] Magnet FAR = Continuity (LOW)");
  Serial.println("[TEST] Magnet CLOSE = No continuity (HIGH)");
  Serial.println("[TEST] Please ensure magnet is FAR from the sensor...");
  Serial.println();
  
  // Wait for sensor to stabilize
  for (int i = 3; i > 0; i--) {
    Serial.print("[TEST] Stabilizing... ");
    Serial.print(i);
    Serial.println(" seconds");
    delay(1000);
  }
  
  Serial.println();
  Serial.println("[TEST] Sensor stabilized!");
  
  // Read initial state - we expect continuity (closed circuit, LOW) when magnet is far
  bool initialState = reedSwitch.isClosed();
  
  Serial.print("[TEST] Initial state: ");
  Serial.print(initialState ? "CLOSED/LOW (continuity - magnet far)" : "OPEN/HIGH (no continuity - magnet close)");
  Serial.println();
  
  // We want the magnet to be FAR initially (continuity = closed = LOW)
  if (!initialState) {
    Serial.println("[TEST] WARNING: No continuity detected! Magnet might be too close.");
    Serial.println("[TEST] Please move magnet away and restart test.");
    return;
  }
  
  Serial.println("[TEST] ✓ Continuity detected - Magnet is far (baseline established)");
  Serial.println();
  Serial.println("[TEST] Now testing reed switch trigger...");
  Serial.println("[TEST] Please bring magnet CLOSE to the sensor!");
  Serial.println("[TEST] (Continuity should break when magnet is close)");
  Serial.println();
  
  // Wait for reed switch trigger - looking for open circuit (no continuity, HIGH)
  bool switchTriggered = false;
  unsigned long testStart = millis();
  unsigned long timeout = 30000; // 30 second timeout
  
  while (!switchTriggered && (millis() - testStart < timeout)) {
    // When magnet is close, continuity breaks, so isClosed() returns false
    if (!reedSwitch.isClosed()) {
      switchTriggered = true;
      Serial.println("[TEST] ✓✓✓ REED SWITCH TRIGGERED! ✓✓✓");
      Serial.println("[TEST] Continuity broken - Magnet detected close!");
      break;
    }
    
    // Print waiting indicator every 2 seconds
    if ((millis() - testStart) % 2000 < 50) {
      Serial.println("[TEST] Waiting for magnet...");
    }
    delay(50);
  }
  
  Serial.println();
  if (switchTriggered) {
    Serial.println("┌────────────────────────────────────────┐");
    Serial.println("│  REED SWITCH TEST COMPLETED - SUCCESS ✓ │");
    Serial.println("└────────────────────────────────────────┘");
  } else {
    Serial.println("┌────────────────────────────────────────┐");
    Serial.println("│   REED SWITCH TEST FAILED - TIMEOUT    │");
    Serial.println("└────────────────────────────────────────┘");
  }
  Serial.println();
}
