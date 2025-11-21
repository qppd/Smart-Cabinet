#include "I2CLcd.h"
#include "Buzzer.h"
#include "FingerprintAS608.h"
#include "ESPNowComm.h"
#include "TactileButton.h"

// Centralized pin assignments
#include "pins.h"

// Client ESP32 MAC Address - REPLACE WITH YOUR CLIENT'S MAC ADDRESS
uint8_t clientMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Serial for fingerprint module
HardwareSerial FingerSerial(2);

// Component instances
I2CLcd lcd(LCD_ADDR, 20, 4);
Buzzer buzzer(BUZZER_PIN, 5, 2000);
FingerprintAS608 finger(FingerSerial, 57600);
ESPNowComm espNow;
TactileButton enrollButton(ENROLL_BUTTON_PIN);

// Uptime tracking
unsigned long startTime = 0;

void setup() {
  // Initialize host controller
  Serial.begin(115200);
  Serial.println("===============================================");
  Serial.println("SMART CABINET HOST CONTROLLER STARTING...");
  Serial.println("===============================================");

  // I2C LCD
  Wire.begin(); // default SDA=21, SCL=22 on most ESP32 boards
  lcd.begin();
  lcd.print(0, 0, "Smart Cabinet Host");
  lcd.print(0, 1, "Initializing...");

  // Buzzer
  buzzer.begin();
  buzzer.beep(50, 1500); // Quick beep
  Serial.println("[HOST] Buzzer initialized");

  // Enrollment Button
  enrollButton.begin();
  Serial.println("[HOST] Enrollment button initialized");

  // Fingerprint serial - Configure RX/TX pins
  FingerSerial.begin(57600, SERIAL_8N1, FINGER_RX_PIN, FINGER_TX_PIN);
  Serial.print("[HOST] Fingerprint serial configured on RX:");
  Serial.print(FINGER_RX_PIN);
  Serial.print(" TX:");
  Serial.println(FINGER_TX_PIN);
  
  finger.begin();
  if (finger.verifySensor()) {
    Serial.println("[HOST] Fingerprint sensor OK");
    lcd.print(0, 2, "Fingerprint: OK");
  } else {
    Serial.println("[HOST] Fingerprint sensor NOT found");
    lcd.print(0, 2, "Fingerprint: ERROR");
  }

  // Initialize ESP-NOW Communication
  lcd.print(0, 3, "ESP-NOW: Init...");
  espNow.begin();
  espNow.setPeerAddress(clientMAC);
  Serial.println("[HOST] ESP-NOW initialized");
  lcd.print(0, 3, "ESP-NOW: Ready");
  delay(1000);

  // Record start time
  startTime = millis();
  
  lcd.clear();
  lcd.print(0, 0, "Smart Cabinet Host");
  lcd.print(0, 1, "System Ready");

  Serial.println("[HOST] Host controller initialization complete");
  Serial.println("===============================================");
  
  // Single welcome beep
  buzzer.beep(100, 1500);
}

void loop() {
  static unsigned long lastUpdate = 0;
  static unsigned long lastFingerprintCheck = 0;
  static bool displayInitialized = false;
  
  // Update buzzer state (required for beep timing)
  buzzer.update();
  
  // Update button state
  enrollButton.update();
  
  // Check for button press to start enrollment
  if (enrollButton.wasPressed()) {
    Serial.println("[HOST] Enrollment button pressed!");
    lcd.print(0, 3, "Starting Enroll...");
    buzzer.beep(100, 2000);
    delay(500);
    enrollFingerprint();
  }
  
  // Initialize static display content once
  if (!displayInitialized) {
    lcd.clear();
    lcd.print(0, 0, "Smart Cabinet Host");
    lcd.print(0, 3, "Place finger...");
    displayInitialized = true;
  }
  
  // Update display every second (only changing parts)
  if (millis() - lastUpdate > 1000) {
    lastUpdate = millis();
    
    // Update uptime (line 1)
    unsigned long uptime = (millis() - startTime) / 1000;
    unsigned long hours = uptime / 3600;
    unsigned long minutes = (uptime % 3600) / 60;
    unsigned long seconds = uptime % 60;
    char uptimeStr[21];
    sprintf(uptimeStr, "Up: %02luh %02lum %02lus", hours, minutes, seconds);
    lcd.print(0, 1, uptimeStr);
    
    // Update system status (line 2)
    if (espNow.hasConnectedPeer()) {
      lcd.print(0, 2, "Client: Connected ");
    } else {
      lcd.print(0, 2, "Client: Waiting...");
    }
  }
  
  // Check for fingerprint every 500ms (more responsive)
  if (millis() - lastFingerprintCheck > 500) {
    lastFingerprintCheck = millis();
    checkFingerprint();
  }
}

void checkFingerprint() {
  // Try to authenticate directly (authenticate() already checks for finger)
  int result = finger.authenticate();
  
  if (result >= 0) {
    // Authentication successful
    Serial.print("[HOST] Authentication successful! User ID: ");
    Serial.println(result);
    
    lcd.print(0, 3, "Access Granted!   ");
    buzzer.beep(200, 2000); // Single success beep
    
    // Send unlock command to client via ESP-NOW
    espNow.sendUnlockCommand(result);
    
    delay(2000); // Display result for 2 seconds
    lcd.print(0, 3, "Place finger...");
    
  } else if (result == -2) {
    // Finger detected but not matched (wrong finger or not enrolled)
    Serial.println("[HOST] Authentication failed - No match found");
    lcd.print(0, 3, "Access Denied!    ");
    
    // Single error beep
    buzzer.beep(200, 800);
    
    // Send authentication failure to client via ESP-NOW
    espNow.sendAuthenticationResult(false, -1);
    
    delay(2000);
    lcd.print(0, 3, "Place finger...");
  }
  // result == -1: No finger detected or sensor error (silent, no action needed)
}

void enrollFingerprint() {
  // Get the next available ID
  int templateCount = finger.getTemplateCount();
  uint16_t enrollID = templateCount + 1;
  
  // Check if we've reached max capacity
  if (enrollID > MAX_ENROLLED_FINGERPRINTS) {
    Serial.println("[HOST] Maximum fingerprint capacity reached!");
    lcd.print(0, 3, "DB Full!          ");
    
    // Single error beep
    buzzer.beep(200, 800);
    delay(2000);
    lcd.print(0, 3, "Place finger...");
    return;
  }
  
  Serial.print("[HOST] Starting enrollment for ID #");
  Serial.println(enrollID);
  
  // Step 1: Place finger first time
  lcd.clear();
  lcd.print(0, 0, "ENROLLMENT MODE");
  lcd.print(0, 1, "ID: " + String(enrollID));
  lcd.print(0, 2, "Place finger");
  lcd.print(0, 3, "on sensor...");
  
  buzzer.beep(100, 1500);
  
  // Wait for finger and get first image
  bool success = false;
  unsigned long timeout = millis() + 30000; // 30 second timeout
  
  while (millis() < timeout && !success) {
    if (finger.isFingerDetected()) {
      lcd.print(0, 3, "Hold still...    ");
      buzzer.beep(50, 2000);
      delay(1000); // Give time to hold finger
      
      // Step 2: Remove finger
      lcd.print(0, 2, "Remove finger");
      lcd.print(0, 3, "               ");
      buzzer.beep(100, 1000);
      delay(2000);
      
      // Step 3: Place same finger again
      lcd.print(0, 2, "Place SAME finger");
      lcd.print(0, 3, "again...        ");
      buzzer.beep(100, 1500);
      
      // Call the blocking enroll function
      if (finger.enroll(enrollID)) {
        // Success!
        Serial.print("[HOST] Enrollment successful! ID #");
        Serial.println(enrollID);
        
        lcd.clear();
        lcd.print(0, 1, "ENROLLMENT SUCCESS!");
        lcd.print(0, 2, "User ID: " + String(enrollID));
        
        // Single success beep
        buzzer.beep(300, 2000);
        
        success = true;
      } else {
        // Failed
        Serial.println("[HOST] Enrollment failed!");
        
        lcd.clear();
        lcd.print(0, 1, "ENROLLMENT FAILED!");
        lcd.print(0, 2, "Try again...");
        
        // Single error beep
        buzzer.beep(200, 800);
      }
      
      delay(3000);
      break;
    }
    delay(100);
  }
  
  if (!success && millis() >= timeout) {
    Serial.println("[HOST] Enrollment timeout!");
    lcd.clear();
    lcd.print(0, 1, "ENROLLMENT TIMEOUT!");
    buzzer.beep(200, 800);
    delay(2000);
  }
  
  // Return to normal display
  lcd.clear();
  lcd.print(0, 0, "Smart Cabinet Host");
  lcd.print(0, 3, "Place finger...");
}
