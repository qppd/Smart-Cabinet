/*
  ===============================================
  SMART CABINET CLIENT MODULES
  ===============================================
  
  This file contains additional functions and modules for the Smart Cabinet Client.
  It provides extended functionality, command processing, diagnostics, and utilities
  that complement the main client controller.
  
  Functions included:
  - WebSocket message processing
  - Command execution functions  
  - Diagnostic and testing functions
  - Configuration management
  - Advanced motor control routines
  - Error handling and recovery
  - System calibration functions
  
  Author: QPPD
  Version: 1.0
  Date: 2025
*/

// ===========================================
// WEBSOCKET MESSAGE PROCESSING
// ===========================================

void processWebSocketMessage(String message) {
  Serial.print("[CLIENT] Processing WebSocket message: ");
  Serial.println(message);
  
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, message);
  
  if (error) {
    Serial.print("[CLIENT] JSON parsing failed: ");
    Serial.println(error.c_str());
    return;
  }
  
  String command = doc["command"] | "";
  String type = doc["type"] | "";
  
  // Process different message types
  if (type == "command") {
    processCommand(command, doc);
  } else if (type == "config") {
    processConfigUpdate(doc);
  } else if (type == "diagnostic") {
    processDiagnosticRequest(doc);
  } else if (type == "authentication") {
    processAuthenticationResult(doc);
  }
}

void processCommand(String command, DynamicJsonDocument& doc) {
  Serial.print("[CLIENT] Executing command: ");
  Serial.println(command);
  
  if (command == "unlock_and_open") {
    unlockAndOpen();
  } 
  else if (command == "force_close") {
    forceClose();
  }
  else if (command == "emergency_stop") {
    emergencyStop = true;
  }
  else if (command == "clear_emergency") {
    clearEmergencyStop();
  }
  else if (command == "enable_system") {
    bool enable = doc["enable"] | true;
    enableSystem(enable);
  }
  else if (command == "test_motors") {
    testMotors();
  }
  else if (command == "test_relays") {
    testRelays();
  }
  else if (command == "calibrate_door") {
    calibrateDoorPosition();
  }
  else if (command == "home_motors") {
    homeMotors();
  }
  else if (command == "get_diagnostics") {
    sendDiagnostics();
  }
  else if (command == "update_config") {
    updateConfiguration(doc);
  }
  else {
    Serial.print("[CLIENT] Unknown command: ");
    Serial.println(command);
    sendErrorResponse("Unknown command: " + command);
  }
}

void processAuthenticationResult(DynamicJsonDocument& doc) {
  bool authenticated = doc["authenticated"] | false;
  String userId = doc["user_id"] | "unknown";
  
  if (authenticated) {
    Serial.print("[CLIENT] Authentication successful for user: ");
    Serial.println(userId);
    
    if (currentState == CLIENT_LOCKED || currentState == CLIENT_IDLE) {
      unlockAndOpen();
    }
  } else {
    Serial.println("[CLIENT] Authentication failed");
    
    // Flash LED briefly to indicate failed authentication
    relayBoard.set(2, true);
    delay(200);
    relayBoard.set(2, false);
  }
}

void processConfigUpdate(DynamicJsonDocument& doc) {
  Serial.println("[CLIENT] Processing configuration update");
  
  // Update timing parameters if provided
  if (doc.containsKey("motion_timeout")) {
    // Note: In a real implementation, you'd want to store these in EEPROM
    Serial.print("[CLIENT] Motion timeout updated to: ");
    Serial.println(doc["motion_timeout"].as<unsigned long>());
  }
  
  if (doc.containsKey("motor_speed")) {
    Serial.print("[CLIENT] Motor speed updated to: ");
    Serial.println(doc["motor_speed"].as<unsigned int>());
  }
  
  sendConfigConfirmation();
}

void processDiagnosticRequest(DynamicJsonDocument& doc) {
  String diagnosticType = doc["diagnostic_type"] | "full";
  
  if (diagnosticType == "full") {
    sendFullDiagnostics();
  } else if (diagnosticType == "sensors") {
    sendSensorDiagnostics();
  } else if (diagnosticType == "motors") {
    sendMotorDiagnostics();
  } else if (diagnosticType == "network") {
    sendNetworkDiagnostics();
  }
}

// ===========================================
// TESTING AND DIAGNOSTICS
// ===========================================

void testMotors() {
  Serial.println("[CLIENT] Starting motor test sequence...");
  
  if (currentState != CLIENT_IDLE && currentState != CLIENT_LOCKED) {
    sendErrorResponse("Cannot test motors - system not in safe state");
    return;
  }
  
  // Test door motor
  Serial.println("[CLIENT] Testing door motor...");
  doorMotor.enable(true);
  
  // Small movement test
  doorMotor.setDirection(true);
  doorMotor.stepMany(100, CLIENT_MOTOR_PULSE_US, CLIENT_MOTOR_GAP_US);
  delay(500);
  
  doorMotor.setDirection(false);  
  doorMotor.stepMany(100, CLIENT_MOTOR_PULSE_US, CLIENT_MOTOR_GAP_US);
  
  doorMotor.enable(false);
  
  // Test lock motor
  Serial.println("[CLIENT] Testing lock motor...");
  lockMotor.enable(true);
  
  lockMotor.setDirection(true);
  lockMotor.stepMany(50, CLIENT_MOTOR_PULSE_US, CLIENT_MOTOR_GAP_US);
  delay(500);
  
  lockMotor.setDirection(false);
  lockMotor.stepMany(50, CLIENT_MOTOR_PULSE_US, CLIENT_MOTOR_GAP_US);
  
  lockMotor.enable(false);
  
  Serial.println("[CLIENT] Motor test completed");
  sendTestResults("motors", "PASSED");
}

void testRelays() {
  Serial.println("[CLIENT] Starting relay test sequence...");
  
  // Test each relay individually
  for (int relay = 1; relay <= 4; relay++) {
    Serial.print("[CLIENT] Testing relay ");
    Serial.println(relay);
    
    relayBoard.set(relay, true);
    delay(500);
    relayBoard.set(relay, false);
    delay(200);
  }
  
  // Test all relays together
  Serial.println("[CLIENT] Testing all relays together...");
  relayBoard.set(1, true);
  relayBoard.set(2, true);  
  relayBoard.set(3, true);
  relayBoard.set(4, true);
  delay(1000);
  relayBoard.allOff();
  
  Serial.println("[CLIENT] Relay test completed");
  sendTestResults("relays", "PASSED");
}

void calibrateDoorPosition() {
  Serial.println("[CLIENT] Starting door position calibration...");
  
  if (currentState != CLIENT_IDLE) {
    sendErrorResponse("Cannot calibrate - system not in idle state");
    return;
  }
  
  // Move to closed position using reed switch
  doorMotor.enable(true);
  doorMotor.setDirection(false); // Closing direction
  
  unsigned long calibrationStart = millis();
  bool foundClosedPosition = false;
  
  while (!foundClosedPosition && (millis() - calibrationStart < 30000)) {
    doorMotor.stepOnce(CLIENT_MOTOR_PULSE_US);
    delay(5); // Slow movement for precise calibration
    
    if (reedSwitch.isClosed()) {
      foundClosedPosition = true;
      Serial.println("[CLIENT] Found closed position");
    }
    
    // Check for limit switches
    if (limitSwitchMin.isPressed() || limitSwitchMax.isPressed()) {
      Serial.println("[CLIENT] Calibration stopped - limit switch hit");
      break;
    }
  }
  
  doorMotor.enable(false);
  
  if (foundClosedPosition) {
    Serial.println("[CLIENT] Door calibration completed successfully");
    sendTestResults("calibration", "PASSED");
  } else {
    Serial.println("[CLIENT] Door calibration failed");
    sendTestResults("calibration", "FAILED");
  }
}

void homeMotors() {
  Serial.println("[CLIENT] Homing motors to reference positions...");
  
  // Home door motor to closed position
  calibrateDoorPosition();
  
  // Home lock motor (if position sensors are available)
  // This would depend on your specific hardware setup
  
  Serial.println("[CLIENT] Motor homing completed");
}

// ===========================================
// DIAGNOSTIC REPORTING
// ===========================================

void sendFullDiagnostics() {
  DynamicJsonDocument doc(1000);
  doc["type"] = "diagnostics";
  doc["diagnostic_type"] = "full";
  doc["timestamp"] = millis();
  
  // System state
  doc["system"]["state"] = stateToString(currentState);
  doc["system"]["emergency_stop"] = emergencyStop;
  doc["system"]["system_enabled"] = systemEnabled;
  doc["system"]["uptime"] = millis();
  
  // Door and lock status  
  doc["door"]["is_open"] = doorIsOpen;
  doc["door"]["lock_engaged"] = lockEngaged;
  doc["door"]["reed_switch"] = reedSwitch.isClosed();
  
  // Motor status
  doc["motors"]["door_running"] = doorMotorRunning;
  doc["motors"]["lock_running"] = lockMotorRunning;
  doc["motors"]["door_busy"] = doorMotor.isBusy();
  doc["motors"]["lock_busy"] = lockMotor.isBusy();
  
  // Safety switches
  doc["safety"]["limit_min"] = limitSwitchMin.isPressed();
  doc["safety"]["limit_max"] = limitSwitchMax.isPressed();
  
  // Motion detection
  doc["motion"]["detected"] = motionDetected;
  doc["motion"]["last_motion"] = lastMotionTime;
  doc["motion"]["time_since_motion"] = millis() - lastMotionTime;
  
  // Relay status
  for (int i = 1; i <= 4; i++) {
    doc["relays"]["relay_" + String(i)] = relayBoard.get(i);
  }
  
  // Network status
  doc["network"]["wifi_connected"] = (WiFi.status() == WL_CONNECTED);
  doc["network"]["wifi_rssi"] = WiFi.RSSI();
  doc["network"]["websocket_connected"] = wsClient.isConnected();
  doc["network"]["ip_address"] = WiFi.localIP().toString();
  
  String message;
  serializeJson(doc, message);
  wsClient.sendMessage(message);
}

void sendSensorDiagnostics() {
  DynamicJsonDocument doc(500);
  doc["type"] = "diagnostics";
  doc["diagnostic_type"] = "sensors";
  doc["timestamp"] = millis();
  
  // Read all sensor states
  doc["motion_sensor"]["raw"] = motionSensor.readRaw();
  doc["motion_sensor"]["debounced"] = motionSensor.isMotion();
  doc["reed_switch"]["closed"] = reedSwitch.isClosed();
  doc["limit_switches"]["min"] = limitSwitchMin.isPressed();
  doc["limit_switches"]["max"] = limitSwitchMax.isPressed();
  
  String message;
  serializeJson(doc, message);
  wsClient.sendMessage(message);
}

void sendMotorDiagnostics() {
  DynamicJsonDocument doc(400);
  doc["type"] = "diagnostics";
  doc["diagnostic_type"] = "motors";
  doc["timestamp"] = millis();
  
  doc["door_motor"]["running"] = doorMotorRunning;
  doc["door_motor"]["busy"] = doorMotor.isBusy();
  doc["lock_motor"]["running"] = lockMotorRunning;
  doc["lock_motor"]["busy"] = lockMotor.isBusy();
  
  String message;
  serializeJson(doc, message);
  wsClient.sendMessage(message);
}

void sendNetworkDiagnostics() {
  DynamicJsonDocument doc(400);
  doc["type"] = "diagnostics";
  doc["diagnostic_type"] = "network";
  doc["timestamp"] = millis();
  
  doc["wifi"]["connected"] = (WiFi.status() == WL_CONNECTED);
  doc["wifi"]["ssid"] = WiFi.SSID();
  doc["wifi"]["rssi"] = WiFi.RSSI();
  doc["wifi"]["ip"] = WiFi.localIP().toString();
  doc["wifi"]["mac"] = WiFi.macAddress();
  
  doc["websocket"]["connected"] = wsClient.isConnected();
  doc["websocket"]["host"] = CLIENT_WEBSOCKET_HOST;
  doc["websocket"]["port"] = CLIENT_WEBSOCKET_PORT;
  
  String message;
  serializeJson(doc, message);
  wsClient.sendMessage(message);
}

// ===========================================
// RESPONSE FUNCTIONS
// ===========================================

void sendErrorResponse(String errorMessage) {
  DynamicJsonDocument doc(300);
  doc["type"] = "error";
  doc["timestamp"] = millis();
  doc["message"] = errorMessage;
  doc["state"] = stateToString(currentState);
  
  String message;
  serializeJson(doc, message);
  wsClient.sendMessage(message);
  
  Serial.print("[CLIENT] Error: ");
  Serial.println(errorMessage);
}

void sendTestResults(String testType, String result) {
  DynamicJsonDocument doc(300);
  doc["type"] = "test_result";
  doc["timestamp"] = millis();
  doc["test_type"] = testType;
  doc["result"] = result;
  
  String message;
  serializeJson(doc, message);
  wsClient.sendMessage(message);
}

void sendConfigConfirmation() {
  DynamicJsonDocument doc(200);
  doc["type"] = "config_updated";
  doc["timestamp"] = millis();
  doc["status"] = "success";
  
  String message;
  serializeJson(doc, message);
  wsClient.sendMessage(message);
}

// ===========================================
// ADVANCED MOTOR CONTROL
// ===========================================

bool moveToPosition(TB6600& motor, unsigned long steps, bool direction, unsigned long timeoutMs = 30000) {
  Serial.print("[CLIENT] Moving motor ");
  Serial.print(steps);
  Serial.print(" steps in direction ");
  Serial.println(direction ? "forward" : "reverse");
  
  motor.enable(true);
  motor.setDirection(direction);
  
  if (!motor.startSteps(steps, CLIENT_MOTOR_PULSE_US, CLIENT_MOTOR_GAP_US)) {
    Serial.println("[CLIENT] Failed to start motor movement");
    motor.enable(false);
    return false;
  }
  
  unsigned long startTime = millis();
  
  while (motor.isBusy() && (millis() - startTime < timeoutMs)) {
    motor.update();
    
    // Check for emergency conditions
    if (limitSwitchMin.isPressed() || limitSwitchMax.isPressed()) {
      Serial.println("[CLIENT] Motor movement stopped - limit switch");
      motor.emergencyStop();
      return false;
    }
    
    delay(1);
  }
  
  motor.enable(false);
  
  if (millis() - startTime >= timeoutMs) {
    Serial.println("[CLIENT] Motor movement timeout");
    return false;
  }
  
  Serial.println("[CLIENT] Motor movement completed successfully");
  return true;
}

void performControlledDoorOpen() {
  Serial.println("[CLIENT] Performing controlled door opening...");
  
  // Step 1: Release solenoid lock
  relayBoard.set(1, true);
  delay(200);
  
  // Step 2: Disengage mechanical lock
  if (!moveToPosition(lockMotor, CLIENT_LOCK_RELEASE_STEPS, false, 10000)) {
    sendErrorResponse("Failed to release mechanical lock");
    return;
  }
  lockEngaged = false;
  
  // Step 3: Open door gradually
  if (!moveToPosition(doorMotor, CLIENT_DOOR_OPEN_STEPS, true, 30000)) {
    sendErrorResponse("Failed to open door");
    return;
  }
  
  // Step 4: Turn on LED strip
  relayBoard.set(2, true);
  
  doorIsOpen = true;
  Serial.println("[CLIENT] Controlled door opening completed");
}

void performControlledDoorClose() {
  Serial.println("[CLIENT] Performing controlled door closing...");
  
  // Step 1: Turn off LED strip
  relayBoard.set(2, false);
  
  // Step 2: Close door
  if (!moveToPosition(doorMotor, CLIENT_DOOR_CLOSE_STEPS, false, 30000)) {
    sendErrorResponse("Failed to close door");
    return;
  }
  
  // Step 3: Wait for door to settle
  delay(500);
  
  // Step 4: Engage mechanical lock
  if (!moveToPosition(lockMotor, CLIENT_LOCK_ENGAGE_STEPS, true, 10000)) {
    sendErrorResponse("Failed to engage mechanical lock");
    return;
  }
  lockEngaged = true;
  
  // Step 5: Engage solenoid lock  
  relayBoard.set(1, false);
  
  doorIsOpen = false;
  Serial.println("[CLIENT] Controlled door closing completed");
}

// ===========================================
// CONFIGURATION MANAGEMENT
// ===========================================

void updateConfiguration(DynamicJsonDocument& config) {
  Serial.println("[CLIENT] Updating system configuration...");
  
  // In a real implementation, these would be stored in EEPROM
  // For now, we'll just acknowledge the update
  
  if (config.containsKey("motion_timeout")) {
    Serial.print("[CLIENT] Motion timeout: ");
    Serial.println(config["motion_timeout"].as<unsigned long>());
  }
  
  if (config.containsKey("motor_speed")) {
    Serial.print("[CLIENT] Motor speed: ");
    Serial.println(config["motor_speed"].as<unsigned int>());
  }
  
  if (config.containsKey("auto_close_enabled")) {
    Serial.print("[CLIENT] Auto-close enabled: ");
    Serial.println(config["auto_close_enabled"].as<bool>());
  }
  
  sendConfigConfirmation();
}

// ===========================================
// SYSTEM HEALTH MONITORING
// ===========================================

void checkSystemHealth() {
  static unsigned long lastHealthCheck = 0;
  
  if (millis() - lastHealthCheck < 10000) return; // Check every 10 seconds
  lastHealthCheck = millis();
  
  Serial.println("[CLIENT] Performing system health check...");
  
  bool healthIssues = false;
  
  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[CLIENT] Health: WiFi disconnected");
    healthIssues = true;
  }
  
  // Check WebSocket connection
  if (!wsClient.isConnected()) {
    Serial.println("[CLIENT] Health: WebSocket disconnected");
    healthIssues = true;
  }
  
  // Check if motors are stuck
  if (doorMotorRunning && (millis() - motorStartTime > CLIENT_MOTOR_TIMEOUT)) {
    Serial.println("[CLIENT] Health: Door motor timeout");
    healthIssues = true;
    emergencyStop = true;
  }
  
  if (lockMotorRunning && (millis() - motorStartTime > CLIENT_MOTOR_TIMEOUT)) {
    Serial.println("[CLIENT] Health: Lock motor timeout");  
    healthIssues = true;
    emergencyStop = true;
  }
  
  // Check sensor consistency
  // For example, if door should be closed but reed switch says open
  if (currentState == CLIENT_LOCKED && !reedSwitch.isClosed()) {
    Serial.println("[CLIENT] Health: Door position inconsistency");
    healthIssues = true;
  }
  
  if (!healthIssues) {
    Serial.println("[CLIENT] System health: OK");
  }
}

// ===========================================
// UTILITY AND HELPER FUNCTIONS
// ===========================================

void printSystemStatus() {
  Serial.println("===============================================");
  Serial.println("SMART CABINET CLIENT STATUS");
  Serial.println("===============================================");
  Serial.print("State: "); Serial.println(stateToString(currentState));
  Serial.print("Door Open: "); Serial.println(doorIsOpen ? "YES" : "NO");
  Serial.print("Lock Engaged: "); Serial.println(lockEngaged ? "YES" : "NO");
  Serial.print("Motion Detected: "); Serial.println(motionDetected ? "YES" : "NO");
  Serial.print("Emergency Stop: "); Serial.println(emergencyStop ? "YES" : "NO");
  Serial.print("System Enabled: "); Serial.println(systemEnabled ? "YES" : "NO");
  Serial.println("-----------------------------------------------");
  Serial.print("Reed Switch: "); Serial.println(reedSwitch.isClosed() ? "CLOSED" : "OPEN");
  Serial.print("Limit Min: "); Serial.println(limitSwitchMin.isPressed() ? "PRESSED" : "NORMAL");
  Serial.print("Limit Max: "); Serial.println(limitSwitchMax.isPressed() ? "PRESSED" : "NORMAL");
  Serial.println("-----------------------------------------------");
  Serial.print("Door Motor Running: "); Serial.println(doorMotorRunning ? "YES" : "NO");
  Serial.print("Lock Motor Running: "); Serial.println(lockMotorRunning ? "YES" : "NO");
  Serial.println("-----------------------------------------------");
  Serial.print("WiFi: "); Serial.println(WiFi.status() == WL_CONNECTED ? "CONNECTED" : "DISCONNECTED");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("IP: "); Serial.println(WiFi.localIP());
    Serial.print("RSSI: "); Serial.println(WiFi.RSSI());
  }
  Serial.print("WebSocket: "); Serial.println(wsClient.isConnected() ? "CONNECTED" : "DISCONNECTED");
  Serial.println("===============================================");
}