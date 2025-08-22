# Smart Cabinet — Full annotated walkthrough (GitHub README)

This README is written as Markdown but uses HTML `<code>` tags and `<pre><code>` blocks so GitHub will render the tags and the annotated content correctly.

> Note: GitHub strips full HTML documents (doctype/head/body). Don't use a full HTML document inside `README.md` — use Markdown with inline HTML tags like `<code>` and `<pre>` instead. This file follows that approach.

---

## Short summary of repository changes

- `<code>MotionSensor</code>` was converted to a proper `.h`/`.cpp` split.
- Call sites were checked; `Main.ino` already matches the new API: `<code>MotionSensor pir(PIR_PIN); pir.begin();</code>`.

---

## How to build (Arduino IDE)

1. Install required libraries: *Sketch → Include Library → Manage Libraries...*
   - `<code>LiquidCrystal I2C</code>` (choose a compatible fork)
   - `<code>Adafruit Fingerprint Sensor Library</code>` (by Adafruit)
2. Select the correct ESP32 board under *Tools → Board*.
3. Open `<code>Main.ino</code>` and click *Upload*.

---

## Annotated: `<code>Main.ino</code>` (high level)

The main sketch wires together the components. Important lines are shown in `<code>` tags below with short explanations.

<pre><code>#include "components/MotionSensor.h"</code></pre>
<p><strong>Motion sensor wrapper used for PIR sensors.</strong></p>

<pre><code>#include "components/I2CLcd.h"</code></pre>
<p><strong>I2C LCD wrapper for a 20x4 display.</strong></p>

<pre><code>#include "components/Buzzer.h"</code></pre>
<p><strong>Buzzer wrapper using ESP32 LEDC.</strong></p>

<pre><code>#include "components/FingerprintAS608.h"</code></pre>
<p><strong>Fingerprint module wrapper using <code>HardwareSerial</code> and Adafruit library.</strong></p>

<pre><code>#include "components/TB6600.h"</code></pre>
<p><strong>Stepper driver helper for TB6600-based drivers.</strong></p>

<pre><code>#include "components/Relay4.h"</code></pre>
<p><strong>4-channel relay wrapper.</strong></p>

<pre><code>#include "components/LimitSwitch.h"</code></pre>
<p><strong>Debounced limit switch helper with callbacks.</strong></p>

<pre><code>#include "pins.h"</code></pre>
<p><strong>Centralized pin mapping file — change pins here for your board.</strong></p>

### Construction examples

<pre><code>MotionSensor pir(PIR_PIN);
I2CLcd lcd(LCD_ADDR, 20, 4);
Buzzer buzzer(BUZZER_PIN, 0, 2000);
FingerprintAS608 finger(FingerSerial, 57600);</code></pre>

> In `<code>setup()</code>` the sketch initializes Serial, components, and attaches ISRs. In `<code>loop()</code>` it calls `<code>update()</code>` on each non-blocking component and processes emergency flags and UI updates.

---

## Full component annotations (Markdown with `<code>` tags)

Each component wrapper in `<code>components/</code>` is annotated below: purpose, main public API, implementation notes, and a short usage snippet.

### `Buzzer` (`Buzzer.h` / `Buzzer.cpp`)

**Purpose:** non-blocking piezo buzzer helper for ESP32 using LEDC (hardware PWM).

**API highlights:**

<pre><code>Buzzer(uint8_t pin, uint8_t ledcChannel = 0, uint32_t freq = 2000, bool activeHigh = true);
void begin();
void on(); void off();
void beep(unsigned int ms, uint32_t freqHz = 2000);
void update();</code></pre>

**Notes:** uses `<code>ledcSetup</code>` / `<code>ledcAttachPin</code>`, tracks `_beepEndMs` and stops the tone in `<code>update()</code>`.

**Usage:**

<pre><code>Buzzer buz(BUZZER_PIN);
void setup(){ buz.begin(); }
void loop(){ buz.update(); }</code></pre>

---

### `FingerprintAS608` (`FingerprintAS608.h` / `FingerprintAS608.cpp`)

**Purpose:** wrapper around Adafruit_Fingerprint to provide blocking and simple async search.

**API highlights:**

<pre><code>FingerprintAS608(HardwareSerial &serial, uint32_t baud = 57600);
void begin(); bool verifySensor(); int getTemplateCount(); bool enroll(uint16_t id); int search();
using SearchCallback = void(*)(int); bool startSearch(SearchCallback cb); void update();</code></pre>

**Notes:** enrollment is simplified; use Adafruit examples for robust enrollment. `startSearch` + `update` implement a basic async flow.

**Usage:**

<pre><code>FingerprintAS608 finger(Serial2);
void setup(){ finger.begin(); if (finger.verifySensor()) Serial.println("OK"); }</code></pre>

---

### `I2CLcd` (`I2CLcd.h` / `I2CLcd.cpp`)

**Purpose:** wrapper around `LiquidCrystal_I2C` for a 20x4 I2C LCD.

**API highlights:**

<pre><code>I2CLcd(uint8_t addr = 0x27, uint8_t cols = 20, uint8_t rows = 4);
void begin(); void clear(); void print(uint8_t col, uint8_t row, const String &text); void setBacklight(bool on);</code></pre>

**Notes:** `begin()` calls `Wire.begin()`, `_lcd.begin()`, `_lcd.backlight()`, and `_lcd.clear()`.

**Usage:**

<pre><code>I2CLcd lcd(0x27, 20, 4);
void setup(){ lcd.begin(); lcd.print(0,0,"Hello"); }</code></pre>

---

### `LimitSwitch` (`LimitSwitch.h` / `LimitSwitch.cpp`)

**Purpose:** debounce mechanical limit switches and provide a callback when the pressed state changes.

**API highlights:**

<pre><code>LimitSwitch(uint8_t pin, bool activeLow = true, unsigned long debounceMs = 50);
void begin(); void update(); bool isPressed() const; void setCallback(Callback cb);</code></pre>

**Notes:** if `activeLow` is true, `begin()` uses `INPUT_PULLUP`. `update()` only accepts a state change after `debounceMs` milliseconds of stable readings.

**Usage:**

<pre><code>LimitSwitch sw(LIMIT_PIN, true);
void setup(){ sw.begin(); sw.setCallback([](bool p){ Serial.println(p?"PRESSED":"RELEASED"); }); }
void loop(){ sw.update(); }</code></pre>

---

### `Relay4` (`Relay4.h` / `Relay4.cpp`)

**Purpose:** control a 4-channel relay module, handling active-high/low wiring differences.

**API highlights:**

<pre><code>Relay4(uint8_t r1, uint8_t r2, uint8_t r3, uint8_t r4, bool activeHigh = true);
void begin(); void set(uint8_t channel, bool on); bool get(uint8_t channel) const; void allOff();</code></pre>

**Notes:** `begin()` sets GPIOs to `OUTPUT` and writes a safe off state. `set()` maps logical `on` to the correct physical level depending on `activeHigh`.

**Usage:**

<pre><code>Relay4 rel(RELAY1,RELAY2,RELAY3,RELAY4,false);
void setup(){ rel.begin(); rel.allOff(); }</code></pre>

---

### `TB6600` (`TB6600.h` / `TB6600.cpp`)

**Purpose:** basic TB6600 stepper helper for DIR/STEP drivers with both blocking and non-blocking APIs.

**API highlights:**

<pre><code>TB6600(uint8_t dirPin, uint8_t stepPin, uint8_t enablePin = 255);
void begin(); void enable(bool en); void setDirection(bool dir); void stepOnce(unsigned int pulseUs = 100); void stepMany(unsigned long steps, unsigned int pulseUs = 100, unsigned int gapUs = 800);
bool startSteps(unsigned long steps, unsigned int pulseUs = 100, unsigned int gapUs = 800); void update(); bool isBusy() const;</code></pre>

**Notes:** uses `micros()` in `update()` to generate microsecond-accurate pulses. `emergencyStop()` disables driver and cancels scheduled non-blocking operations.

**Usage (non-blocking):**

<pre><code>if (!stepper.isBusy()) stepper.startSteps(1000);
void loop(){ stepper.update(); }</code></pre>

---

### `MotionSensor` (`MotionSensor.h` / `MotionSensor.cpp`)

**Purpose:** debounce PIR motion sensor input and optionally notify on changes.

**API highlights:**

<pre><code>MotionSensor(uint8_t pin, unsigned long debounceMs = 200);
void begin(); void update(); bool readRaw(); bool isMotion() const; void setCallback(Callback cb);</code></pre>

**Notes:** `update()` requires frequent calls; candidate changes are only accepted after `debounceMs` milliseconds of steady reading. A callback (if installed) is invoked on accepted changes.

**Usage:**

<pre><code>MotionSensor pir(PIR_PIN);
void setup(){ pir.begin(); }
void loop(){ pir.update(); if(pir.isMotion()){ /* motion detected */ } }</code></pre>

---

## Final notes

- This `README.md` uses inline HTML `<code>` and `<pre>` tags so GitHub will render the exact tags you wanted while keeping the document in Markdown format.
- If you want the README shortened, or exported as a single HTML file for publishing elsewhere, tell me which format you prefer and I will produce it.
