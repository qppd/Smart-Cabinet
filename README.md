# Smart Cabinet - Component libraries

This folder contains reusable component wrappers for an ESP32-based smart cabinet project.

Required Arduino libraries (Arduino IDE)
- LiquidCrystal I2C (for 20x4 I2C LCD)
- Adafruit Fingerprint Sensor Library

Install via Arduino IDE Library Manager:

1. Open Arduino IDE
2. Sketch -> Include Library -> Manage Libraries...
3. Search and install these packages:

- "LiquidCrystal I2C" (pick a popular fork compatible with HD44780 I2C backpacks)
- "Adafruit Fingerprint Sensor Library" by Adafruit

PlatformIO (optional) - add to `platformio.ini` under `lib_deps`:

lib_deps =
  johnrickman/LiquidCrystal_I2C@^1.1.4 ; or another compatible fork
  adafruit/Adafruit Fingerprint Sensor Library@^1.3.0

Notes
- The fingerprint wrapper uses `HardwareSerial` (e.g., `Serial2`) for ESP32. Wire RX/TX accordingly.
- The buzzer uses the ESP32 LEDC API. No external library required.
- If you prefer another LCD driver (e.g., `LiquidCrystal_PCF8574` or `U8g2`), swap the `I2CLcd` implementation accordingly.

If you want, I can add PlatformIO-ready examples showing how to wire and initialize each component.
