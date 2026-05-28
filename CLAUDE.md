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

The sketch folder name (`NavigateObstacle`) must match the `.ino` filename — do not rename one without the other.

Required Arduino libraries (install via Library Manager): **MFRC522**, **Servo**, plus the bundled **SPI** and **EEPROM**.

`.vscode/c_cpp_properties.json` is preconfigured for **Windows** Arduino IDE paths (`%LOCALAPPDATA%/Arduino15/...`). On macOS/Linux IntelliSense will be broken until those paths are updated — the actual build is unaffected.

## Architecture

Three files form a single state-machine-driven robot controller:

- **`NavigateObstacle.ino`** — Arduino entry point. Instantiates one global `Controller ctrlr`, calls `ctrlr.init()` in `setup()`, and `ctrlr.RunOnce()` every `loop()` iteration. All logic lives in `Controller`.
- **`Controller.h`** — pin map, tunables (`SERVO_UP/DOWN`, `LINEDETECT_THRESHOLD_MIN`, `OBSTACLE_THRESHOLD`, `Power`, P-controller `Kp`/`maxCorrection`), the `POSITION` / `APP_STATE` enums, and the city-RFID UID strings.
- **`Controller.cpp`** — full implementation.

### Runtime state machine

The robot operates in two logical positions, gated by RFID tag reads:

```
eInitialPosition ──(Start tag)──► eWareHousePosition ──(City tag, full round-trip)──► eWareHousePosition ...
```

`ProcessRFIDRead()` in `Controller.cpp` is the **dispatcher**. After the start RFID, control enters `eWareHousePosition`; each subsequent city tag triggers a full round-trip via the **coordinate-based navigator** (no more hardcoded per-city paths).

### Coordinate-based navigation

A 4-column × 8-row **full grid** (`col 0~3, row 0~7`, every cell connected to its 4 cardinal neighbors). Warehouse at `WAREHOUSE_X / WAREHOUSE_Y` (default `(3, 0)`), start RFID at `INIT_START_X / INIT_START_Y` (default `(0, 3)` heading `INIT_START_HEADING = HD_EAST`), cities along `row 7`. Bot tracks `currentPose = {x, y, heading}` and at each crossing decides the next direction by:

1. Compute `(dx, dy)` to target.
2. Look up the current crossing's `conn` bitmask via `lookupConn(x, y)` — programmatic, returns N|E|S|W minus any grid-boundary directions (controlled by `GRID_COLS=4`, `GRID_ROWS=8`).
3. **Y-first greedy + perpendicular fallback**: prefer N/S when `dy != 0` and that direction is in `conn`; otherwise E/W if `dx != 0`. If the direct direction is masked (e.g., obstacle blocked it) **and the target lies straight along that axis (dx=0 or dy=0)**, fall back to whichever perpendicular direction (E/W or N/S) is available so the bot can detour and return to the target column/row later.
4. `rotateToHeading()` issues a `PivotTurn*`/`TurnHalf` if needed, then `DoLineTrace(1)` advances one crossing.

Adding a new city: just add a `CityCoord` mapping RFID UID → `(x, y)` in `CITY_COORDS[]`. **No path code or map edit needed** — the navigator finds the route. If a future layout has a gap in the grid (some cell missing a particular direction), add an override inside `lookupConn()`. If the navigator hits a true dead-end (every direction blocked), it prints `"Nav STUCK at (x,y)"`, returns `false`, and the dispatch handles the failure (see next paragraph).

`navigateTo()` returns `bool` — `true` on reaching target, `false` on STUCK. The `eWareHousePosition` dispatch checks the return: on success, runs the drop-cargo + return-trip sequence normally; on failure, **skips `LifterDown`/`TurnHalf`** and just navigates back to `(1, 0)` with cargo still on the lifter. This avoids the previous bug where a failed forward trip silently triggered the "arrived at city" choreography in the middle of the map.

**Obstacle bypass** (single unified mode): when `CheckObstacle()` trips during `DoLineTrace(1)`, the bot does `ReverseToPreviousNode()` and returns `false`. The navigator records the blocked `(x, y, direction)` and re-runs `desiredHeading()` with that direction masked out of `conn`, so the next iteration picks an alternative (perpendicular fallback if the target is on the same axis). Block clears on any successful move. Init sequence uses the same navigator (`navigateTo(WAREHOUSE_X, WAREHOUSE_Y)` from the start RFID position), so obstacles encountered during the bring-up trip are handled the same way.

**Layout requirement**: the blocked crossing must have at least one other valid direction toward the target, otherwise the navigator prints `"Nav STUCK"` and stops.

**Precise realign** (the reverse-then-forward dance in `LineTracer()`) only runs when arriving at `y == 0` (warehouse row) or `y == 7` (city row) — the rows where a clean turn matters. Intermediate crossings just count and pass through with a brief stop. Controlled by `_preciseRealign` member, set per call by the `precise` parameter of `DoLineTrace()`.

Initial pose is set in the `eInitialPosition` case after the start-tag sequence: `currentPose = {1, 0, HD_NORTH}`. If the physical heading after init differs, the first `navigateTo()` will issue a `TurnHalf` to correct.

The `eTargetPosition` enum value is unused; `currentPosition` only ever holds `eInitialPosition` or `eWareHousePosition`.

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
