# Smart Cabinet — Full annotated walkthrough (GitHub README)

This README is written as Markdown but uses HTML `<code>` tags and `<pre><code>` blocks so GitHub will render the tags and the annotated content correctly.

> Note: GitHub strips full HTML documents (doctype/head/body). Don't use a full HTML document inside `README.md` — use Markdown with inline HTML tags like `<code>` and `<pre>` instead. This file follows that approach.
## Smart Cabinet — Project Plan and Developer README 🚪🤖

This README describes the Smart Cabinet hardware, software architecture, wiring notes, operation flows (authenticate/enroll/delete), timing rules (auto-close), failure and safety modes, and commissioning/test steps.

Keep `pins.h` as the canonical pin map for your board. Update pins there before wiring or testing.

---

## 📝 Checklist (Requirements)

- ✅ **Relay** to control 2 solenoids and LED strip
- ✅ **Two TB6600 drivers** for two steppers
- ✅ **Fingerprint sensor** for authenticate/enroll/delete
- ✅ **I2C LCD** for user messages
- ✅ **Buzzer** for audible feedback
- ✅ **Two limit switches** for homing two steppers
- ✅ **Reed switch** for door-closed detection
- ✅ **Motion sensor** for auto-closing
- ✅ **LED strip** on while open, off after closed
- ✅ **Auto-close** after 60s no motion, then close, turn off LED, and shut solenoids after reed confirms

---

## 🛠️ System Overview

The Smart Cabinet uses one main microcontroller (ESP32 recommended) to coordinate:

- 🔒 **Fingerprint authentication** (Adafruit AS608 wrapper)
- ⚙️ **Two stepper motors** (TB6600 drivers) to open/close two doors/drawers
- 🔌 **Relay board** (4 channels) to toggle two solenoids and an LED strip (+ spare)
- 👁️ **PIR motion sensor** for auto-close
- 🧲 **Reed switch** to confirm fully-closed door state
- 🛑 **Two limit switches** (one per stepper) for homing
- 📟 **I2C LCD** for user messages
- 🔊 **Buzzer** for audible feedback

---

## 📐 Component Mapping and Suggested Pin Roles

- **Fingerprint**: `FingerSerial` (HardwareSerial), Rx/Tx to sensor; use `Adafruit_Fingerprint` library.
- **TB6600 #1**: DIR1, STEP1, ENABLE1 (pins mapped in `pins.h`)
- **TB6600 #2**: DIR2, STEP2, ENABLE2
- **Relay4 channels**:
  - Relay1 = Solenoid A
  - Relay2 = Solenoid B
  - Relay3 = LED strip
  - Relay4 = Spare (or master power)
- **Limit switch A**: Home for stepper A
- **Limit switch B**: Home for stepper B
- **Reed switch**: Door closed detect — wired to a digital input with pull-up/down
- **PIR motion sensor**: Digital input
- **I2C LCD**: SDA / SCL (LCD I2C address in `pins.h`)
- **Buzzer**: PWM-capable pin

---

## ⚡ Wiring Notes and Safety

- Use common grounds between the controller, TB6600 drivers, relay module, sensors, and solenoids where applicable.
- TB6600 drivers and stepper motors should use a separate 24V (or motor-rated) supply. Tie grounds.
- Relay board should be powered as per its spec. If relays are driven by the ESP32, check whether the module is active-low or active-high and set `Relay4(activeHigh)` accordingly.
- Solenoids draw significant current — ensure MOSFETs/relays are rated for inrush and continuous current.
- Add flyback diodes and TVS as required for inductive loads if not built into the relay board.
- Use level shifting on serial lines to the fingerprint module if voltage domains differ.

---

## 🔄 Operation Flows

### 🔓 Authentication (Open)
1. Finger placed on sensor → `FingerprintAS608` search.
2. If match: LCD shows "Access granted", buzzer short beep, relays energize solenoid(s) to unlock, and TB6600 steppers move to the configured OPEN position.
3. When steppers reach open position (either by step count or sensor), set `isOpen = true` and turn on LED strip relay.
4. Start/refresh a motion inactivity timer.

### 🖊️ Enrollment
1. Start enrollment mode via a physical button or special fingerprint sequence (documented in code UI).
2. Follow LCD prompts to enroll a finger to a free template slot. Confirm success on LCD + buzzer.

### 🗑️ Deletion
1. Delete by ID from the UI or provide an admin fingerprint that triggers deletion mode.
2. Confirm deletion on LCD and clear template from the sensor storage.

### 🤖 Auto-Close Logic (Motion-Based)
- If `isOpen` and PIR reports no motion continuously for 60 seconds: start close sequence.
- Close sequence: move steppers to CLOSE position. When reed switch indicates closed (or limit switches indicate closed), deactivate solenoids and turn LED strip off.
- If motion is detected during closing, abort closing and reopen to safe position.

---

## 🧪 Testing and Commissioning Checklist

1. Verify all pin mappings in `pins.h`.
2. Power the controller (no motors powered) and verify serial logs, LCD, and buzzer behavior.
3. Test fingerprint sensor: `verifySensor()` and `getTemplateCount()`.
4. Enroll a test fingerprint and verify authentication.
5. With motors powered, test homing for each stepper; confirm limit switches trigger and home positions set.
6. Test open/close sequences without solenoids first (steppers only) to confirm positions and reed detection.
7. Test relays with low-power loads, then with solenoids, verifying current and heat.
8. Test motion-based auto-close: open, enable PIR, stop motion, wait 60s, confirm close and relays off.

---

## 🛠️ Build and Upload

- Open `Main.ino` in the Arduino IDE (or PlatformIO). Ensure the board is set to the correct ESP32 board and the correct `pins.h` mapping is selected.
- Required libraries:
  - `LiquidCrystal_I2C` (compatible fork)
  - `Adafruit Fingerprint Sensor Library`

---

## 🛡️ Troubleshooting

- **Motor not moving**: Check TB6600 enable pin, motor supply, and step pulse polarity.
- **Fingerprint not responding**: Verify baud/power and RX/TX wiring and test with `verifySensor()`.
- **Reed never triggers**: Confirm wiring and that the magnet position aligns to trigger only when fully closed.

---

## 📂 Files of Interest

- `Main.ino` — main sketch and high-level loop
- `pins.h` — canonical pin map and constants
- `components/` — helpers: `Buzzer`, `FingerprintAS608`, `I2CLcd`, `TB6600`, `Relay4`, `LimitSwitch`, `MotionSensor`

---

If you want, I can:
- Generate an example `pins.h` with suggested pin numbers for an ESP32.
- Add a simple enrollment UI flow in `Main.ino` and commit it.
- Create short unit tests for `LimitSwitch` and `TB6600` update loops.

---

Completion status
- README updated with a comprehensive plan, wiring notes, flows, and test checklist. Map each requirement to the status is shown in the checklist above.
