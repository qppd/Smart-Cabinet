#include "I2CLcd.h"
#include "NTPTime.h"
#include "Buzzer.h"
#include "FingerprintAS608.h"
#include "ESPNowComm.h"

// Centralized pin assignments
#include "pins.h"

// Client ESP32 MAC Address - REPLACE WITH YOUR CLIENT'S MAC ADDRESS
uint8_t clientMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Serial for fingerprint module
HardwareSerial FingerSerial(2);

// Component instances
I2CLcd lcd(LCD_ADDR, 20, 4);
NTPTime ntpTime;
Buzzer buzzer(BUZZER_PIN, 5, 2000);  // Use LEDC channel 5 to avoid WiFi conflicts
FingerprintAS608 finger(FingerSerial, 57600);
ESPNowComm espNow;

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

  // Initialize ESP-NOW Communication
  lcd.print(0, 3, "ESP-NOW: Init...");
  espNow.begin();
  espNow.setPeerAddress(clientMAC);
  Serial.println("[HOST] ESP-NOW initialized");
  lcd.print(0, 3, "ESP-NOW: Ready");
  delay(1000);

  // Connect to WiFi for NTP only
  lcd.print(0, 3, "WiFi: Connecting...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(HOST_WIFI_SSID, HOST_WIFI_PASSWORD);
  
  int wifiRetries = 0;
  while (WiFi.status() != WL_CONNECTED && wifiRetries < 15) {
    delay(1000);
    wifiRetries++;
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[HOST] WiFi connected");
    Serial.print("[HOST] IP address: ");
    Serial.println(WiFi.localIP());
    lcd.print(0, 3, "WiFi: Connected");
  } else {
    Serial.println("\n[HOST] WiFi connection failed");
    lcd.print(0, 3, "WiFi: Failed!");
  }
  delay(1000);

  // Initialize NTP Time (requires WiFi)
  ntpTime.begin();
  Serial.println("[HOST] NTP time initialized");
  
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
  static bool displayInitialized = false;
  
  // Update buzzer state (required for beep timing)
  buzzer.update();
  
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
    ntpTime.updateTime();
    
    // Update time (line 1) - no clear needed
    ntpTime.displayTime(lcd, 0, 1);
    
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
      
      // Send unlock command to client via ESP-NOW
      espNow.sendUnlockCommand(result);
      
    } else {
      // Authentication failed
      Serial.println("[HOST] Authentication failed");
      lcd.print(0, 3, "Access Denied!    ");
      
      // Error beep sequence
      for (int i = 0; i < 3; i++) {
        buzzer.beep(100, 800);
        delay(100);
      }
      
      // Send authentication failure to client via ESP-NOW
      espNow.sendAuthenticationResult(false, -1);
    }
    
    delay(2000); // Display result for 2 seconds
    lcd.print(0, 3, "Place finger...");
  }
}
