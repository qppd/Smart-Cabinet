<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>Smart Cabinet - Full annotated walkthrough</title>
  <style>
    body { font-family: system-ui, -apple-system, Arial, sans-serif; line-height:1.5; padding:24px; max-width:980px; margin:auto; }
    pre { background:#f6f8fa; padding:12px; overflow:auto; border-radius:6px; }
    code { background:#eef2f7; padding:2px 6px; border-radius:4px; }
    h1,h2,h3 { color:#111827 }
    .muted { color:#6b7280 }
    .section { margin-bottom:24px }
  </style>
</head>
<body>
  <h1>Smart Cabinet — Full annotated walkthrough (Arduino IDE)</h1>
  <p class="muted">This HTML file contains a complete, annotated walkthrough of <code>Main.ino</code> and every component wrapper in <code>components/</code>. It's formatted for easy browsing in a web browser or VS Code preview.</p>

  <section class="section">
    <h2>Short summary of repository changes</h2>
    <ul>
      <li><code>MotionSensor</code> was converted to a proper <code>.h</code>/<code>.cpp</code> split.</li>
      <li>Call sites were checked; <code>Main.ino</code> already matches the new API (<code>MotionSensor pir(PIR_PIN); pir.begin();</code>).</li>
    </ul>
  </section>

  <section class="section">
    <h2>How to build (Arduino IDE)</h2>
    <ol>
      <li>Install required libraries: <em>Sketch &rarr; Include Library &rarr; Manage Libraries...</em>
        <ul>
          <li><code>LiquidCrystal I2C</code> (choose a compatible fork)</li>
          <li><code>Adafruit Fingerprint Sensor Library</code> (by Adafruit)</li>
        </ul>
      </li>
      <li>Select the correct ESP32 board under <em>Tools &rarr; Board</em>.</li>
      <li>Open <code>Main.ino</code> and click <em>Upload</em>.</li>
    </ol>
  </section>

  <section class="section">
    <h2>Annotated: <code>Main.ino</code> (high level)</h2>
    <p>The main sketch wires together the components. Important lines are shown below with short explanations.</p>

    <pre><code>#include "components/MotionSensor.h"</code></pre>
    <p>Motion sensor wrapper used for PIR sensors.</p>

    <pre><code>#include "components/I2CLcd.h"</code></pre>
    <p>I2C LCD wrapper for a 20x4 display.</p>

    <pre><code>#include "components/Buzzer.h"</code></pre>
    <p>Buzzer wrapper using ESP32 LEDC.</p>

    <pre><code>#include "components/FingerprintAS608.h"</code></pre>
    <p>Fingerprint module wrapper using <code>HardwareSerial</code> and Adafruit library.</p>

    <pre><code>#include "components/TB6600.h"</code></pre>
    <p>Stepper driver helper for TB6600-based drivers.</p>

    <pre><code>#include "components/Relay4.h"</code></pre>
    <p>4-channel relay wrapper.</p>

    <pre><code>#include "components/LimitSwitch.h"</code></pre>
    <p>Debounced limit switch helper with callbacks.</p>

    <pre><code>#include "pins.h"</code></pre>
    <p>Centralized pin mapping file; change pins here for your board.</p>

    <h3>Construction examples</h3>
    <pre><code>MotionSensor pir(PIR_PIN);
I2CLcd lcd(LCD_ADDR, 20, 4);
Buzzer buzzer(BUZZER_PIN, 0, 2000);
FingerprintAS608 finger(FingerSerial, 57600);</code></pre>

    <p>In <code>setup()</code> the sketch initializes Serial, components, and attaches ISRs for limit switches. In <code>loop()</code> it calls <code>update()</code> on each non-blocking component and processes emergency flags and UI updates.</p>
  </section>

  <section class="section">
    <h2>Full component annotations (Arduino IDE)</h2>
    <p>Each component wrapper in <code>components/</code> is annotated below: purpose, main public API, implementation notes, and a short usage snippet.</p>

    <h3>Buzzer (Buzzer.h / Buzzer.cpp)</h3>
    <p><strong>Purpose:</strong> non-blocking piezo buzzer helper for ESP32 using LEDC (hardware PWM).</p>
    <pre><code>Buzzer(uint8_t pin, uint8_t ledcChannel = 0, uint32_t freq = 2000, bool activeHigh = true);
void begin();
void on(); void off();
void beep(unsigned int ms, uint32_t freqHz = 2000);
void update();</code></pre>
    <p><strong>Notes:</strong> uses <code>ledcSetup</code> / <code>ledcAttachPin</code>, tracks <code>_beepEndMs</code> and stops the tone in <code>update()</code>.</p>
    <pre><code>Buzzer buz(BUZZER_PIN);
void setup(){ buz.begin(); }
void loop(){ buz.update(); }</code></pre>

    <h3>FingerprintAS608 (FingerprintAS608.h / FingerprintAS608.cpp)</h3>
    <p><strong>Purpose:</strong> lightweight wrapper around Adafruit_Fingerprint for blocking and simple async search.</p>
    <pre><code>FingerprintAS608(HardwareSerial &serial, uint32_t baud = 57600);
void begin(); bool verifySensor(); int getTemplateCount(); bool enroll(uint16_t id); int search();
using SearchCallback = void(*)(int); bool startSearch(SearchCallback cb); void update();</code></pre>
    <p><strong>Notes:</strong> enrollment is simplified; use Adafruit examples for production flows. <code>startSearch</code> + <code>update</code> implement a basic async flow.</p>
    <pre><code>FingerprintAS608 finger(Serial2);
void setup(){ finger.begin(); if (finger.verifySensor()) Serial.println("OK"); }</code></pre>

    <h3>I2CLcd (I2CLcd.h / I2CLcd.cpp)</h3>
    <p><strong>Purpose:</strong> wrapper around <code>LiquidCrystal_I2C</code> for a 20x4 I2C LCD.</p>
    <pre><code>I2CLcd(uint8_t addr = 0x27, uint8_t cols = 20, uint8_t rows = 4);
void begin(); void clear(); void print(uint8_t col, uint8_t row, const String &text); void setBacklight(bool on);</code></pre>
    <p><strong>Notes:</strong> <code>begin()</code> calls <code>Wire.begin()</code>, <code>_lcd.begin()</code>, <code>_lcd.backlight()</code> and <code>_lcd.clear()</code>.</p>
    <pre><code>I2CLcd lcd(0x27, 20, 4);
void setup(){ lcd.begin(); lcd.print(0,0,"Hello"); }</code></pre>

    <h3>LimitSwitch (LimitSwitch.h / LimitSwitch.cpp)</h3>
    <p><strong>Purpose:</strong> debounce limit switches and provide callbacks on state change.</p>
    <pre><code>LimitSwitch(uint8_t pin, bool activeLow = true, unsigned long debounceMs = 50);
void begin(); void update(); bool isPressed() const; void setCallback(Callback cb);</code></pre>
    <p><strong>Notes:</strong> if <code>activeLow</code>, <code>begin()</code> uses <code>INPUT_PULLUP</code>. <code>update()</code> implements a debounce window.</p>
    <pre><code>LimitSwitch sw(LIMIT_PIN, true);
void setup(){ sw.begin(); sw.setCallback([](bool p){ Serial.println(p?"PRESSED":"RELEASED"); }); }
void loop(){ sw.update(); }</code></pre>

    <h3>Relay4 (Relay4.h / Relay4.cpp)</h3>
    <p><strong>Purpose:</strong> control a 4-channel relay module, handling active-high/low differences.</p>
    <pre><code>Relay4(uint8_t r1, uint8_t r2, uint8_t r3, uint8_t r4, bool activeHigh = true);
void begin(); void set(uint8_t channel, bool on); bool get(uint8_t channel) const; void allOff();</code></pre>
    <p><strong>Notes:</strong> <code>begin()</code> sets pins to <code>OUTPUT</code> and writes safe default off state.</p>
    <pre><code>Relay4 rel(RELAY1,RELAY2,RELAY3,RELAY4,false);
void setup(){ rel.begin(); rel.allOff(); }</code></pre>

    <h3>TB6600 (TB6600.h / TB6600.cpp)</h3>
    <p><strong>Purpose:</strong> basic TB6600 stepper helper for DIR/STEP drivers with blocking and non-blocking APIs.</p>
    <pre><code>TB6600(uint8_t dirPin, uint8_t stepPin, uint8_t enablePin = 255);
void begin(); void enable(bool en); void setDirection(bool dir); void stepOnce(unsigned int pulseUs = 100); void stepMany(unsigned long steps, unsigned int pulseUs = 100, unsigned int gapUs = 800);
bool startSteps(unsigned long steps, unsigned int pulseUs = 100, unsigned int gapUs = 800); void update(); bool isBusy() const;</code></pre>
    <p><strong>Notes:</strong> uses <code>micros()</code> in <code>update()</code> to generate microsecond pulses for STEP pin. <code>emergencyStop()</code> disables driver and cancels non-blocking operations.</p>
    <pre><code>stepper.setDirection(true);
stepper.stepMany(200);

// non-blocking
if (!stepper.isBusy()) stepper.startSteps(1000);
void loop(){ stepper.update(); }</code></pre>

    <h3>MotionSensor (MotionSensor.h / MotionSensor.cpp)</h3>
    <p><strong>Purpose:</strong> debounce PIR motion sensor input and optionally notify on changes.</p>
    <pre><code>MotionSensor(uint8_t pin, unsigned long debounceMs = 200);
void begin(); void update(); bool readRaw(); bool isMotion() const; void setCallback(Callback cb);</code></pre>
    <p><strong>Notes:</strong> <code>update()</code> requires frequent calls; candidate changes are only accepted after <code>debounceMs</code> milliseconds of steady reading. A callback (if installed) is invoked on accepted changes.</p>
    <pre><code>MotionSensor pir(PIR_PIN);
void setup(){ pir.begin(); }
void loop(){ pir.update(); if(pir.isMotion()){ /* motion detected */ } }</code></pre>

  </section>

  <section class="section">
    <h2>Final notes</h2>
    <ul>
      <li>These wrappers are intentionally small and dependency-light for easy inspection inside Arduino IDE.</li>
      <li>Options I can implement next:
        <ul>
          <li>Wiring diagrams (text/ASCII) per component</li>
          <li>A short <code>GETTING_STARTED.md</code> with wiring checklist + Arduino IDE steps</li>
          <li>A compatibility shim for older <code>MotionSensor</code> API if you have external code that expects it</li>
        </ul>
      </li>
    </ul>
  </section>

</body>
</html>
