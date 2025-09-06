#include "DS1302Rtc.h"

DS1302Rtc::DS1302Rtc(uint8_t ce_pin, uint8_t sck_pin, uint8_t io_pin)
    : rtc(ce_pin, sck_pin, io_pin) {}

void DS1302Rtc::begin() {
    rtc.begin();
    if (!rtc.isrunning()) {
        rtc.adjust(DateTime(__DATE__, __TIME__));
    }
}

void DS1302Rtc::updateTime() {
    currentTime = rtc.now();
}

void DS1302Rtc::displayTime(I2CLcd &lcd, uint8_t col, uint8_t row) {
    currentTime.tostr(buf);
    lcd.print(col, row, String(buf));
}

DateTime DS1302Rtc::now() {
    return currentTime;
}
