#ifndef NTP_TIME_H
#define NTP_TIME_H

#include <Arduino.h>
#include <WiFi.h>
#include <time.h>
#include "I2CLcd.h"

class NTPTime {
public:
    NTPTime();
    void begin();
    void updateTime();
    void displayTime(I2CLcd &lcd, uint8_t col = 0, uint8_t row = 0);
    String getFormattedDateTime();
    unsigned long getTimestamp();
    bool isSynced();

private:
    struct tm timeinfo;
    char timeBuffer[50];
    bool synced;
};

#endif // NTP_TIME_H
