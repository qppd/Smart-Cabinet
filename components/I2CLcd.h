#ifndef COMPONENTS_I2C_LCD_H
#define COMPONENTS_I2C_LCD_H

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Wrapper for a 20x4 I2C LCD at address 0x27
class I2CLcd {
public:
  I2CLcd(uint8_t addr = 0x27, uint8_t cols = 20, uint8_t rows = 4);
  void begin();
  void clear();
  void print(uint8_t col, uint8_t row, const String &text);
  void setBacklight(bool on);

private:
  LiquidCrystal_I2C _lcd;
};

#endif // COMPONENTS_I2C_LCD_H
