#include "I2CLcd.h"
#include "NTPTime.h"
#include "Buzzer.h"
#include "FingerprintAS608.h"
#include "WebSocketServer.h"

// Centralized pin assignments
#include "pins.h"

// Serial for fingerprint module
HardwareSerial FingerSerial(2);

// Component instances
I2CLcd lcd(LCD_ADDR, 20, 4);
NTPTime ntpTime;
Buzzer buzzer(BUZZER_PIN, 5, 2000);  // Use LEDC channel 5 to avoid WiFi conflicts
FingerprintAS608 finger(FingerSerial, 57600);
WebSocketServer wsServer;

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
  buzzer.beep(100, 1500);
  Serial.println("[HOST] Buzzer initialized");

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

  // Connect to WiFi
  lcd.print(0, 3, "WiFi: Connecting...");
  wsServer.setWiFiCredentials(HOST_WIFI_SSID, HOST_WIFI_PASSWORD);
  if (wsServer.connectWiFi()) {
    lcd.clear();
    lcd.print(0, 0, "Smart Cabinet Host");
    lcd.print(0, 1, "WiFi: Connected");
    lcd.print(0, 2, wsServer.getIPAddress());
  } else {
    lcd.print(0, 3, "WiFi: Failed!");
  }
  delay(2000);

  // Initialize NTP Time (requires WiFi)
  ntpTime.begin();
  Serial.println("[HOST] NTP time initialized");

  // Initialize WebSocket Server
  wsServer.begin();
  Serial.println("[HOST] WebSocket server started");
  
  // Display date and time from NTP
  ntpTime.updateTime();
  lcd.clear();
  lcd.print(0, 0, "Smart Cabinet Host");
  ntpTime.displayTime(lcd, 0, 3);

  Serial.println("[HOST] Host controller initialization complete");
  Serial.println("===============================================");
  
  // Welcome beep sequence
  buzzer.beep(100, 1000);
  delay(100);
  buzzer.beep(100, 1500);
}

void loop() {
  static unsigned long lastUpdate = 0;
  static unsigned long lastFingerprintCheck = 0;
  
  // Update buzzer state (required for beep timing)
  buzzer.update();
  
  // Handle WebSocket communication
  //wsServer.loop();
  
  // Update display every second
  if (millis() - lastUpdate > 1000) {
    lastUpdate = millis();
    ntpTime.updateTime();
    
    // Clear LCD and update display
    lcd.clear();
    lcd.print(0, 0, "Smart Cabinet Host");
    
    // Show current time
    ntpTime.displayTime(lcd, 0, 1);
    
    // Show system status
    if (wsServer.hasConnectedClients()) {
      lcd.print(0, 2, "Client: Connected");
    } else {
      lcd.print(0, 2, "Client: Waiting...");
    }
    
    // Show fingerprint status
    lcd.print(0, 3, "Place finger...");
  }
  
  // Check for fingerprint every 500ms (more responsive)
  if (millis() - lastFingerprintCheck > 500) {
    lastFingerprintCheck = millis();
    checkFingerprint();
  }
}

void checkFingerprint() {
  // Check if finger is detected
  if (finger.isFingerDetected()) {
    Serial.println("[HOST] Finger detected, starting authentication...");
    lcd.print(0, 3, "Authenticating...");
    buzzer.beep(50, 2000); // Quick beep for feedback
    
    int result = finger.authenticate();
    
    if (result >= 0) {
      // Authentication successful
      Serial.print("[HOST] Authentication successful! User ID: ");
      Serial.println(result);
      
      lcd.print(0, 3, "Access Granted!   ");
      buzzer.beep(200, 1500); // Success tone
      delay(100);
      buzzer.beep(200, 2000);
      
      // Send unlock command to client
      wsServer.sendUnlockCommand(result);
      
    } else {
      // Authentication failed
      Serial.println("[HOST] Authentication failed");
      lcd.print(0, 3, "Access Denied!    ");
      
      // Error beep sequence
      for (int i = 0; i < 3; i++) {
        buzzer.beep(100, 800);
        delay(100);
      }
      
      // Send authentication failure to client
      wsServer.sendAuthenticationResult(false, -1);
    }
    
    delay(2000); // Display result for 2 seconds
    lcd.print(0, 3, "Place finger...");
  }
}
