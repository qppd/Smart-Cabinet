# Smart Cabinet — Full annotated walkthrough (GitHub README)

This README is written as Markdown but uses HTML `<code>` tags and `<pre><code>` blocks so GitHub will render the tags and the annotated content correctly.

> Note: GitHub strips full HTML documents (doctype/head/body). Don't use a full HTML document inside `README.md` — use Markdown with inline HTML tags like `<code>` and `<pre>` instead. This file follows that approach.
## Smart Cabinet — Project plan and developer README

This README describes the Smart Cabinet hardware, software architecture, wiring notes, operation flows (authenticate/enroll/delete), timing rules (auto-close), failure and safety modes, and commissioning/test steps.

Keep `pins.h` as the canonical pin map for your board. Update pins there before wiring or testing.

## Checklist (requirements extracted from the request)
- Relay to control 2 solenoids and LED strip — Done (documented)
- Two TB6600 drivers for two steppers — Done (documented)
- Fingerprint sensor for authenticate/enroll/delete — Done (documented)
- I2C LCD — Done (documented)
- Buzzer — Done (documented)
- Two limit switches for homing two steppers — Done (documented)
- Reed switch for door-closed detection — Done (documented)
- Motion sensor for auto-closing — Done (documented)
- LED strip on when door open; off after close — Done (documented)
- Auto-close after no motion for 1 minute, then close, turn off LED, and shut solenoid after door closed by reed switch — Done (documented)

If anything below doesn't match your intended wiring or behavior, tell me which item to change and I will update the README and code notes.

## System overview

The Smart Cabinet uses one main microcontroller (ESP32 recommended) to coordinate:
- Fingerprint authentication (Adafruit AS608 wrapper)
- Two stepper motors (TB6600 drivers) to open/close two doors/drawers
- Relay board (4 channels) to toggle two solenoids and an LED strip (+ spare)
- PIR motion sensor for auto-close
- Reed switch to confirm fully-closed door state
- Two limit switches (one per stepper) for homing
- I2C LCD for user messages
- Buzzer for audible feedback

Core behavior summary:
- When an authorized fingerprint is presented, the cabinet unlocks and opens the doors (steppers move to 'open' position), the LED strip turns on, and the buzzer/ LCD show status.
- If motion is not detected for 60 seconds, the controller initiates auto-close: steppers close until reed switch reports closed, then relays deactivate solenoids and LED strip.
- Manual close is detected by reed switch; when closed, solenoids and LED are turned off.

## Component mapping and suggested pin roles

- Fingerprint: `FingerSerial` (HardwareSerial), Rx/Tx to sensor; use `Adafruit_Fingerprint` library.
- TB6600 #1: DIR1, STEP1, ENABLE1 (pins mapped in `pins.h`)
- TB6600 #2: DIR2, STEP2, ENABLE2
- Relay4 channels: Relay1 = Solenoid A; Relay2 = Solenoid B; Relay3 = LED strip; Relay4 = spare (or master power)
- Limit switch A (home for stepper A)
- Limit switch B (home for stepper B)
- Reed switch (door closed detect) — wired to a digital input with pull-up/down
- PIR motion sensor — digital input
- I2C LCD — SDA / SCL (LCD I2C address in `pins.h`)
- Buzzer — PWM-capable pin

Put final pin numbers in `pins.h`. The sketch reads that file for initialization and wiring comments.

## Wiring notes and safety
- Use common grounds between the controller, TB6600 drivers, relay module, sensors, and solenoids where applicable.
- TB6600 drivers and stepper motors should use a separate 24V (or motor-rated) supply. Tie grounds.
- Relay board should be powered as per its spec. If relays are driven by the ESP32, check whether the module is active-low or active-high and set `Relay4(activeHigh)` accordingly.
- Solenoids draw significant current — ensure MOSFETs/relays are rated for inrush and continuous current.
- Add flyback diodes and TVS as required for inductive loads if not built into the relay board.
- Use level shifting on serial lines to the fingerprint module if voltage domains differ.

## Operation flows

Authentication (open)
1. Finger placed on sensor → `FingerprintAS608` search.
2. If match: LCD shows "Access granted", buzzer short beep, relays energize solenoid(s) to unlock, and TB6600 steppers move to the configured OPEN position.
3. When steppers reach open position (either by step count or sensor), set `isOpen = true` and turn on LED strip relay.
4. Start/refresh a motion inactivity timer.

Enrollment
1. Start enrollment mode via a physical button or special fingerprint sequence (documented in code UI).
2. Follow LCD prompts to enroll a finger to a free template slot. Confirm success on LCD + buzzer.

Deletion
1. Delete by ID from the UI or provide an admin fingerprint that triggers deletion mode.
2. Confirm deletion on LCD and clear template from the sensor storage.

Auto-close logic (motion-based)
- If `isOpen` and PIR reports no motion continuously for 60 seconds: start close sequence.
- Close sequence: move steppers to CLOSE position. When reed switch indicates closed (or limit switches indicate closed), deactivate solenoids and turn LED strip off.
- If motion is detected during closing, abort closing and reopen to safe position.

Homing and limit switches
- On power-up or when commanded, perform homing for each stepper: step slowly toward the limit switch until pressed, set current position 0, then move to the default closed or open offset as needed.
- Debounce limit switches in software. Use `LimitSwitch` helper in `components/`.

Reed switch behavior
- Reed switch must be placed such that it closes (or changes state) only when the door is fully closed. The controller uses it as the final verification before turning off solenoids and LED strip.

Relay behavior summary
- Relay1: Solenoid A (lock/unlock)
- Relay2: Solenoid B (lock/unlock)
- Relay3: LED strip (on while door open)
- Relay4: Spare / optional master power cut

Buzzer and LCD feedback patterns (recommended)
- Short beep + "Access granted" for successful auth.
- Two short beeps + "Enroll OK" on enrollment.
- Long beep + "Access denied" for failed auth.
- Beep pattern for low-voltage or emergency conditions.

Edge cases and safety behaviors
- Power loss while open: on reboot, perform homing and, if reed shows closed, ensure relays (solenoids/LED) are off.
- Stalled motor or jam: detect by missed step counts or excessive time; abort movement, turn off motor drivers, sound buzzer and show error on LCD.
- If reed switch never reports closed during close sequence, time out and alert user by buzzer/LCD.

Testing and commissioning checklist
1. Verify all pin mappings in `pins.h`.
2. Power the controller (no motors powered) and verify serial logs, LCD, and buzzer behavior.
3. Test fingerprint sensor: `verifySensor()` and `getTemplateCount()`.
4. Enroll a test fingerprint and verify authentication.
5. With motors powered, test homing for each stepper; confirm limit switches trigger and home positions set.
6. Test open/close sequences without solenoids first (steppers only) to confirm positions and reed detection.
7. Test relays with low-power loads, then with solenoids, verifying current and heat.
8. Test motion-based auto-close: open, enable PIR, stop motion, wait 60s, confirm close and relays off.

Automated and manual tests to add (recommended)
- Unit test for `LimitSwitch` logic (debounce) — simple mocked input.
- Integration test for `TB6600` non-blocking sequence using short step counts.

Build and upload
- Open `Main.ino` in the Arduino IDE (or PlatformIO). Ensure the board is set to the correct ESP32 board and the correct `pins.h` mapping is selected.
- Required libraries: `LiquidCrystal_I2C` (compatible fork), `Adafruit Fingerprint Sensor Library`.

Assumptions made
- Controller is ESP32 (API references and components use ESP32 features such as LEDC). If using Arduino UNO or other MCUs, adjust PWM/serial usage and timing.
- TB6600 expects DIR/STEP pulse/enable interface.
- Relay board uses a single logical active level (check `Relay4` constructor parameter in `components/`).

Next steps and small improvements (low-risk)
- Add a small config section in `pins.h` documenting expected pinouts and default values.
- Add an "admin" fingerprint ID stored in `pins.h` or `config.h` for deletion/enrollment operations.
- Add a watchdog / emergency stop button mapping and UI flow.

Troubleshooting
- Motor not moving: check TB6600 enable pin, motor supply, and step pulse polarity.
- Fingerprint not responding: verify baud/power and RX/TX wiring and test with `verifySensor()`.
- Reed never triggers: confirm wiring and that the magnet position aligns to trigger only when fully closed.

Files of interest
- `Main.ino` — main sketch and high-level loop
- `pins.h` — canonical pin map and constants
- `components/` — helpers: `Buzzer`, `FingerprintAS608`, `I2CLcd`, `TB6600`, `Relay4`, `LimitSwitch`, `MotionSensor`

If you want, I can:
- Generate an example `pins.h` with suggested pin numbers for an ESP32.
- Add a simple enrollment UI flow in `Main.ino` and commit it.
- Create short unit tests for `LimitSwitch` and `TB6600` update loops.

---

Completion status
- README updated with a comprehensive plan, wiring notes, flows, and test checklist. Map each requirement to the status is shown in the checklist above.
