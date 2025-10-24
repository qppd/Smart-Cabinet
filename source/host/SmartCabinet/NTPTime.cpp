#include "NTPTime.h"

NTPTime::NTPTime() : synced(false) {
    memset(&timeinfo, 0, sizeof(struct tm));
    memset(timeBuffer, 0, sizeof(timeBuffer));
}

void NTPTime::begin() {
    Serial.println("[NTPTime] Initializing NTP time synchronization...");
    
    // Set timezone to GMT+8 (28800 seconds = 8 hours)
    // configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2, ntpServer3)
    configTime(28800, 0, "pool.ntp.org", "time.nist.gov");

    Serial.print("[NTPTime] Waiting for NTP time sync");
    int retries = 0;
    while (!getLocalTime(&timeinfo) && retries < 15) {
        Serial.print(".");
        delay(1000);
        retries++;
    }

    if (retries >= 15) {
        Serial.println("\n[NTPTime] Failed to get time from NTP");
        synced = false;
    } else {
        Serial.println("\n[NTPTime] Time synced from NTP successfully!");
        synced = true;
        
        // Print current time for verification
        strftime(timeBuffer, sizeof(timeBuffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
        Serial.print("[NTPTime] Current time: ");
        Serial.println(timeBuffer);
    }
}

void NTPTime::updateTime() {
    if (getLocalTime(&timeinfo)) {
        synced = true;
    } else {
        synced = false;
        Serial.println("[NTPTime] Warning: Failed to update time");
    }
}

void NTPTime::displayTime(I2CLcd &lcd, uint8_t col, uint8_t row) {
    if (!synced) {
        lcd.print(col, row, "Time: Not synced");
        return;
    }
    
    // Format: DD/MM/YYYY HH:MM:SS
    strftime(timeBuffer, sizeof(timeBuffer), "%d/%m/%Y %H:%M:%S", &timeinfo);
    lcd.print(col, row, String(timeBuffer));
}

String NTPTime::getFormattedDateTime() {
    if (!synced) {
        return "NOT SYNCED";
    }
    
    strftime(timeBuffer, sizeof(timeBuffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
    return String(timeBuffer);
}

unsigned long NTPTime::getTimestamp() {
    if (!synced) {
        return 0;
    }
    
    time_t timestamp = mktime(&timeinfo);
    return (unsigned long)timestamp;
}

bool NTPTime::isSynced() {
    return synced;
}
