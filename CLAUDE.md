# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build / Upload

This is an **Arduino sketch** targeting **Arduino Uno (ATmega328P, 16 MHz)** — there is no `Makefile`, `package.json`, or test harness. Builds happen via the Arduino IDE or `arduino-cli`:

```bash
# Compile
arduino-cli compile --fqbn arduino:avr:uno .

# Upload (replace port)
arduino-cli upload  --fqbn arduino:avr:uno -p /dev/cu.usbmodemXXXX .

# Serial monitor at 9600 baud — matches Serial.begin in setup()
arduino-cli monitor -p /dev/cu.usbmodemXXXX -c baudrate=9600
```

The sketch folder name (`GoandBack_fix`) must match the `.ino` filename — do not rename one without the other.

Required Arduino libraries (install via Library Manager): **MFRC522**, **Servo**, plus the bundled **SPI** and **EEPROM**.

`.vscode/c_cpp_properties.json` is preconfigured for **Windows** Arduino IDE paths (`%LOCALAPPDATA%/Arduino15/...`). On macOS/Linux IntelliSense will be broken until those paths are updated — the actual build is unaffected.

## Architecture

Three files form a single state-machine-driven robot controller:

- **`GoandBack_fix.ino`** — Arduino entry point. Instantiates one global `Controller ctrlr`, calls `ctrlr.init()` in `setup()`, and `ctrlr.RunOnce()` every `loop()` iteration. All logic lives in `Controller`.
- **`Controller.h`** — pin map, tunables (`SERVO_UP/DOWN`, `LINEDETECT_THRESHOLD_MIN`, `OBSTACLE_THRESHOLD`, `Power`, P-controller `Kp`/`maxCorrection`), the `POSITION` / `APP_STATE` enums, and the city-RFID UID strings.
- **`Controller.cpp`** — full implementation.

### Runtime state machine

The robot moves between three logical positions, gated by RFID tag reads:

```
eInitialPosition ──(Start tag)──► eWareHousePosition ──(City tag)──► eTargetPosition ──(City tag)──► eWareHousePosition ...
```

`ProcessRFIDRead()` in `Controller.cpp` is the **dispatcher**: it reads a tag, then runs a fixed sequence of `DoLineTrace(n)` + `PivotTurnLeft/Right()` calls keyed on `currentPosition` and the matched UID string. Each city (Seoul, Incheon, Sejong, Daejeon, Daegu, Gwangju, Chuncheon, Jeju) has its own hardcoded path. **Adding a new destination = adding a new `else if` branch in both the `eWareHousePosition` and `case 2:` (target → warehouse return) blocks** — the two blocks are NOT symmetric, edit both.

The `case 2:` branch is the return-trip handler (it should logically be `case eTargetPosition`); the magic number is intentional, not a bug to "fix" without checking the enum ordering.

### Line tracing

`DoLineTrace(targetCount)` loops on `LineTracer()` until `nLineCounter == targetCount` crossings have been counted, while polling `CheckObstacle()` between iterations.

- `LineTrace()` runs a **P-controller** (`Kp`, `maxCorrection`) over normalized bottom-sensor readings. Base speed drops by 20 when `currentPosition == eWareHousePosition` (carrying a payload).
- A "crossing" = both bottom sensors simultaneously read above `LINEDETECT_THRESHOLD_MIN` (730). `bSignalHigh` debounces so one physical crossing increments `nLineCounter` exactly once.
- On reaching `targetCount`, `LineTracer()` does a deliberate **reverse-then-forward re-alignment**: back up ~400 ms to fully clear the line, then creep forward at reduced power until both sensors hit the line again. Tweak the `delay(400)` in `LineTracer` if the bot under- or over-shoots realignment.

### Obstacle bypass

When `CheckObstacle()` (front-center IR < `OBSTACLE_THRESHOLD`, with one debounce re-read) trips mid-`DoLineTrace`:

1. `enableObstacleAvoidance` is temporarily cleared to prevent re-entrancy.
2. `nLineCounter` is saved, then `ReverseToPreviousNode()` backs up until both bottom sensors find a line.
3. A hardcoded detour fires: `PivotTurnLeft → DoLineTrace(1) → PivotTurnRight → DoLineTrace(2) → PivotTurnRight → DoLineTrace(1) → PivotTurnLeft`.
4. `targetCount` is decremented by 1, `nLineCounter` is restored, and the outer `DoLineTrace` resumes.

Because `DoLineTrace` recurses into itself during the bypass, **don't add state that assumes a single live invocation** without first guarding against the bypass re-entry.

### Calibration via EEPROM

`readData()` pulls six values from EEPROM starting at `START_ADDRESS = 240`:

| Offset | Type    | Field            | Purpose                                          |
|--------|---------|------------------|--------------------------------------------------|
| +0     | int16_t | `_rightWhite`    | Right bottom sensor reading on white surface     |
| +2     | int16_t | `_leftWhite`     | Left bottom sensor reading on white surface      |
| +4     | int16_t | `_rightBlack`    | Right bottom sensor reading on black line        |
| +6     | int16_t | `_leftBlack`     | Left bottom sensor reading on black line         |
| +8     | float   | `_motorCalibR`   | Right motor PWM multiplier (drift compensation)  |
| +12    | float   | `_motorCalibL`   | Left motor PWM multiplier                        |

These are written by a **separate calibration sketch** (not in this repo) — flashing this sketch does not overwrite them, but flashing a stock sketch can. `normalizeLeft/Right()` maps raw analog reads to a 0–1000 scale using these, and `drive()` multiplies PWM output by the calib floats, so a fresh board with zeroed EEPROM will not move correctly.

### Hardware pin map (from `Controller.h`)

Digital: RFID SS=2, Buzzer=3, RFID RST=4, Left PWM=5, Right PWM=6, Left Dir=7, Right Dir=8, Lift Servo=9
Analog: Front Center=A0, Front Left=A1, Front Right=A2, User Button=A3, Bottom Left=A6, Bottom Right=A7

Note `FORWARD`/`BACKWARD` direction bits are **inverted between the two motors** inside `drive()` — the wheels are mounted mirrored. Don't "fix" the asymmetry without rewiring.
