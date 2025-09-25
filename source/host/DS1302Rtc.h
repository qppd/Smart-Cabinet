#ifndef DS1302RTC_H
#define DS1302RTC_H

#include <RTClib.h>
#include "I2CLcd.h"
#include "pins.h"

class DS1302Rtc {
public:
    DS1302Rtc(uint8_t ce_pin = DS1302_CE_PIN, uint8_t sck_pin = DS1302_SCK_PIN, uint8_t io_pin = DS1302_IO_PIN);
    void begin();
    void updateTime();
    void displayTime(I2CLcd &lcd, uint8_t col = 0, uint8_t row = 0);
    DateTime now();
private:
    DS1302 rtc;
    DateTime currentTime;
    char buf[20];
};

#endif // DS1302RTC_H
