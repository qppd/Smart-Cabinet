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

// Relay control
Relay4 relayBoard(CLIENT_RELAY1_PIN, CLIENT_RELAY2_PIN, CLIENT_RELAY3_PIN, CLIENT_RELAY4_PIN);

// Safety switches
LimitSwitch limitSwitchMin(CLIENT_LIMIT_MIN_PIN);
LimitSwitch limitSwitchMax(CLIENT_LIMIT_MAX_PIN);
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
  
  // Initial state
  currentState = CLIENT_LOCKED;
  sendStatusUpdate();
}

// ===========================================
// MAIN LOOP
// ===========================================

void loop() {
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
  limitSwitchMin.begin();
  limitSwitchMax.begin();
  limitSwitchMin.setCallback(onLimitSwitchTriggered);
  limitSwitchMax.setCallback(onLimitSwitchTriggered);
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
  limitSwitchMin.update();
  limitSwitchMax.update();
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
    if (limitSwitchMin.isPressed() || limitSwitchMax.isPressed()) {
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
  doc["limit_min"] = limitSwitchMin.isPressed();
  doc["limit_max"] = limitSwitchMax.isPressed();
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
