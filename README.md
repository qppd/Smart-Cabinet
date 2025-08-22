# Smart Cabinet - Full annotated walkthrough (Arduino IDE)

This README contains a comprehensive, line-by-line explanation of `Main.ino` and every component wrapper in `components/` so you can use this project from the Arduino IDE with minimal friction.

If you prefer a shorter quick-start, tell me and I'll provide one.

---

## Short summary of repository changes

- `MotionSensor` was converted to a clean `.h`/`.cpp` split (declarations in `MotionSensor.h`, definitions in `MotionSensor.cpp`).
- I checked call sites: `Main.ino` already matched the new API (`MotionSensor pir(PIR_PIN); pir.begin();`) so no updates were needed elsewhere.

---

## How to build (Arduino IDE)

1. Install required libraries: Sketch -> Include Library -> Manage Libraries...
   - `LiquidCrystal I2C` (choose a compatible fork)
   - `Adafruit Fingerprint Sensor Library` (by Adafruit)
2. Select the correct ESP32 board under Tools -> Board.
3. Open `Main.ino` and click Upload.

---

## Annotated: `Main.ino` (high level)

The main sketch wires together the components. Important lines are shown in `<code>` tags below with a short explanation. See the file itself for full context.

<code>#include "components/MotionSensor.h"</code>
Motion sensor wrapper used for PIR sensors.

<code>#include "components/I2CLcd.h"</code>
I2C LCD wrapper for a 20x4 display.

<code>#include "components/Buzzer.h"</code>
Buzzer wrapper using ESP32 LEDC.

<code>#include "components/FingerprintAS608.h"</code>
Fingerprint module wrapper using `HardwareSerial` and Adafruit library.

<code>#include "components/TB6600.h"</code>
Stepper driver helper for TB6600-based drivers.

<code>#include "components/Relay4.h"</code>
4-channel relay wrapper.

<code>#include "components/LimitSwitch.h"</code>
Debounced limit switch helper with callbacks.

<code>#include "pins.h"</code>
Centralized pin mapping file; change pins here for your board.

Construction examples:

<code>MotionSensor pir(PIR_PIN);</code>
Construct PIR sensor object with pin and default debounce.

<code>I2CLcd lcd(LCD_ADDR, 20, 4);</code>
Construct the LCD object for 20x4.

<code>Buzzer buzzer(BUZZER_PIN, 0, 2000);</code>
Buzzer with channel 0 and default 2000Hz frequency.

<code>FingerprintAS608 finger(FingerSerial, 57600);</code>
Fingerprint wrapper using `Serial2` at 57600 baud.

In `setup()` the sketch initializes Serial, components, and attaches ISRs for limit switches. In `loop()` it calls `update()` on each non-blocking component and processes emergency flags and UI updates.

---

## Full component annotations (Arduino IDE)

Below are concise annotated descriptions for each component wrapper located in `components/`. For each component I list the purpose, the main public API lines, and implementation/usage notes.

### `Buzzer` (Buzzer.h / Buzzer.cpp)

Purpose: non-blocking piezo buzzer helper for ESP32 using LEDC (hardware PWM).

Public API highlights:

<code>Buzzer(uint8_t pin, uint8_t ledcChannel = 0, uint32_t freq = 2000, bool activeHigh = true);</code>
- Constructor: pin number, ledc channel (0-15), frequency in Hz, active-high.

<code>void begin();</code>
- Configures LEDC timer and attaches pin.

<code>void on(); void off();</code>
- Start and stop continuous tone.

<code>void beep(unsigned int ms, uint32_t freqHz = 2000);</code>
- Non-blocking beep: sets a stop time and plays tone.

<code>void update();</code>
- Call in loop(); if current time >= scheduled beep end, stops buzzer.

Implementation notes:
- Uses ledcSetup(_ledcChannel, _freqHz, 8) and ledcAttachPin.
- Maintains _beepEndMs to schedule stop; update() checks millis().

Usage snippet:
<code>Buzzer buz(BUZZER_PIN); void setup(){buz.begin();} void loop(){buz.update();}</code>

---

### `FingerprintAS608` (FingerprintAS608.h / FingerprintAS608.cpp)

Purpose: lightweight wrapper around Adafruit_Fingerprint to provide blocking and simple async search.

Public API highlights:

<code>FingerprintAS608(HardwareSerial &serial, uint32_t baud = 57600);</code>
- Constructor uses a `HardwareSerial` instance (e.g., `Serial2`).

<code>void begin(); bool verifySensor(); int getTemplateCount(); bool enroll(uint16_t id); int search();</code>
- begin(): starts serial port.
- verifySensor(): wraps Adafruit's verifyPassword().
- search(): blocking fingerprint search; returns ID, 0 not found, -1 error.

<code>using SearchCallback = void(*)(int); bool startSearch(SearchCallback cb); void update();</code>
- startSearch() and update() implement a simple async search: update() polls _finger.getImage() and proceeds when an image is ready.

Implementation notes:
- Relies on Adafruit_Fingerprint. Enrollment implementation is simplified and should be replaced by the full flow in production.
- Use blocking `search()` for simple flows or `startSearch()`/`update()` for non-blocking patterns.

Usage snippet:
<code>FingerprintAS608 finger(Serial2); void setup(){finger.begin(); if(finger.verifySensor()) Serial.println("OK");}</code>

---

### `I2CLcd` (I2CLcd.h / I2CLcd.cpp)

Purpose: wrapper around LiquidCrystal_I2C for a 20x4 I2C LCD.

Public API highlights:

<code>I2CLcd(uint8_t addr = 0x27, uint8_t cols = 20, uint8_t rows = 4);</code>
<code>void begin(); void clear(); void print(uint8_t col, uint8_t row, const String &text); void setBacklight(bool on);</code>

Implementation notes:
- begin() calls Wire.begin(), _lcd.begin(), _lcd.backlight(), and _lcd.clear().
- print() sets cursor and prints the provided String.

Usage snippet:
<code>I2CLcd lcd(0x27, 20, 4); void setup(){lcd.begin(); lcd.print(0,0,"Hello");}</code>

---

### `LimitSwitch` (LimitSwitch.h / LimitSwitch.cpp)

Purpose: debounce limit switches and provide a callback when the pressed state changes.

Public API highlights:

<code>LimitSwitch(uint8_t pin, bool activeLow = true, unsigned long debounceMs = 50);</code>
<code>void begin(); void update(); bool isPressed() const; void setCallback(Callback cb);</code>

Implementation notes:
- begin() configures input mode. If activeLow, INPUT_PULLUP is used.
- update() reads raw digital value, maps it to pressed/not pressed using activeLow, and flips _state only after debounceMs of stable change.
- On change, the optional callback is invoked with the new pressed state.

Usage snippet:
<code>LimitSwitch sw(LIMIT_PIN, true); void setup(){sw.begin(); sw.setCallback([](bool p){Serial.println(p?"PRESSED":"RELEASED");});} void loop(){sw.update();}</code>

---

### `Relay4` (Relay4.h / Relay4.cpp)

Purpose: control a 4-channel relay module, keeping track of logical states and handling active-high/low wiring differences.

Public API highlights:

<code>Relay4(uint8_t r1, uint8_t r2, uint8_t r3, uint8_t r4, bool activeHigh = true);</code>
<code>void begin(); void set(uint8_t channel, bool on); bool get(uint8_t channel) const; void allOff();</code>

Implementation notes:
- begin() sets pins to OUTPUT and writes the safe default state (off).
- set() accepts channel 1..4 and writes appropriate HIGH/LOW depending on activeHigh.

Usage snippet:
<code>Relay4 rel(RELAY1,RELAY2,RELAY3,RELAY4,false); void setup(){rel.begin(); rel.allOff();}</code>

---

### `TB6600` (TB6600.h / TB6600.cpp)

Purpose: basic TB6600 stepper helper for DIR/STEP drivers. Offers both blocking and non-blocking stepping.

Public API highlights:

<code>TB6600(uint8_t dirPin, uint8_t stepPin, uint8_t enablePin = 255);</code>
<code>void begin(); void enable(bool en); void setDirection(bool dir); void stepOnce(unsigned int pulseUs = 100); void stepMany(unsigned long steps, unsigned int pulseUs = 100, unsigned int gapUs = 800);</code>

Non-blocking API:

<code>bool startSteps(unsigned long steps, unsigned int pulseUs = 100, unsigned int gapUs = 800); void update(); bool isBusy() const;</code>

Implementation notes:
- Uses micros() timing in update() to toggle the step pin with microsecond accuracy.
- emergencyStop() writes the enable pin to disable the driver and cancels non-blocking operations.

Usage examples:

Blocking:
<code>stepper.setDirection(true); stepper.stepMany(200);</code>

Non-blocking:
<code>if(!stepper.isBusy()) stepper.startSteps(1000); void loop(){ stepper.update(); }</code>

---

### `MotionSensor` (MotionSensor.h / MotionSensor.cpp)

Purpose: debounce PIR motion sensor input and optionally notify on changes.

Public API highlights:

<code>MotionSensor(uint8_t pin, unsigned long debounceMs = 200); void begin(); void update(); bool readRaw(); bool isMotion() const; void setCallback(Callback cb);</code>

Implementation notes:
- begin() configures the pin and captures the initial raw state as the debounced state.
- update() must be called frequently; it implements a typical debounce window where a candidate change must be stable for debounceMs before being accepted. If accepted, the callback is invoked.

Usage snippet:
<code>MotionSensor pir(PIR_PIN); void setup(){ pir.begin(); } void loop(){ pir.update(); if(pir.isMotion()) { /* motion detected */ } }</code>

---

## Final notes

- All component wrappers are intentionally small and dependency-light to make them easy to inspect and modify inside Arduino IDE.
- If you'd like, I can:
  - Generate a wiring diagram (text + suggested connections) per component.
  - Add a brief quick-start `GETTING_STARTED.md` with a minimal wiring checklist and upload steps for Arduino IDE.
  - Run a grep across the repo to ensure no remaining code expects the old `MotionSensor` API and optionally add a compatibility shim.

Tell me which of these you'd like next and I'll implement it.
