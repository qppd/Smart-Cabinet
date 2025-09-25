#include "I2CLcd.h"

I2CLcd::I2CLcd(uint8_t addr, uint8_t cols, uint8_t rows)
  : _lcd(addr, cols, rows) {}

void I2CLcd::begin() {
  Wire.begin();
  // Use begin(cols, rows) for Frank de Brabander's LiquidCrystal_I2C
  _lcd.begin(20, 4); // or use member variables if you want dynamic size
  _lcd.backlight();
  _lcd.clear();
}

void I2CLcd::clear() {
  _lcd.clear();
}

void I2CLcd::print(uint8_t col, uint8_t row, const String &text) {
  _lcd.setCursor(col, row);
  _lcd.print(text);
}

void I2CLcd::setBacklight(bool on) {
  if (on) _lcd.backlight(); else _lcd.noBacklight();
}
