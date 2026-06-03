# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build / Upload

This is an **Arduino sketch** targeting **Arduino Nano (ATmega328P, 16 MHz)** — there is no `Makefile`, `package.json`, or test harness. Builds happen via the Arduino IDE or `arduino-cli`:

```bash
# Compile
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328old .

# Upload (replace port) — Nano uses the old bootloader variant
arduino-cli upload  --fqbn arduino:avr:nano:cpu=atmega328old -p /dev/cu.usbmodemXXXX .

# Serial monitor at 9600 baud — matches Serial.begin in setup()
arduino-cli monitor -p /dev/cu.usbmodemXXXX -c baudrate=9600
```

The sketch folder name (`arduino-line`) must match the `.ino` filename (`arduino-line.ino`) — do not rename one without the other.

Required Arduino libraries (install via Library Manager): **MFRC522**, **Servo**, plus the bundled **SPI** and **EEPROM**.

`.vscode/c_cpp_properties.json` is preconfigured for **Windows** Arduino IDE paths (`%LOCALAPPDATA%/Arduino15/...`). On macOS/Linux IntelliSense will be broken until those paths are updated — the actual build is unaffected.

## Architecture

Five files form a single state-machine-driven robot controller:

- **`arduino-line.ino`** — Arduino entry point. Instantiates one global `Controller ctrlr`, calls `ctrlr.init()` in `setup()`, and `ctrlr.RunOnce()` every `loop()` iteration. All logic lives in `Controller`.
- **`Controller.h`** — pin map, the `POSITION` / `APP_STATE` enums, city-RFID UID strings, and forward declarations. Tunables have moved to `Settings_robot*.h`.
- **`Controller.cpp`** — full implementation.
- **`Settings.h`** — 27-line `ROBOT_ID` selector: `#define ROBOT_ID 1` then `#include "Settings_robot1.h"` or `"Settings_robot2.h"`. Switch robots by changing `ROBOT_ID` here or passing `-DROBOT_ID=2` as a build flag.
- **`Settings_robot1.h`** / **`Settings_robot2.h`** — all tunables (speeds, PID, servo angles, turn delays, thresholds, map coordinates). Edit the file for your robot.

### Runtime state machine

The robot operates in two logical positions, gated by RFID tag reads:

```
eInitialPosition ──(Start tag)──► eWareHousePosition ──(City tag, full round-trip)──► eWareHousePosition ...
```

`ProcessRFIDRead()` in `Controller.cpp` is the **dispatcher**. After the start RFID, control enters `eWareHousePosition`; each subsequent city tag triggers a full round-trip via the **coordinate-based navigator** (no more hardcoded per-city paths).

### Coordinate-based navigation

**Robot 1**: 4-column × 8-row grid (`col 0~3, row 0~7`). Warehouse default `(2, 0)`. Start RFID at `(-1, 3)` heading `HD_EAST` (one cell off-grid west of column 0). Cities along `row 7`, cols 0–3.

**Robot 2**: 8-column × 8-row grid (`col 0~7`, `GRID_COLS=8`). Robot 2 starts at `(-1, INIT_START_Y)` heading east, transits map 1 (cols 0–3) to reach its warehouse in cols 4–7, then is confined to cols ≥ 4 via `NAV_MIN_X=4` (activated after the initial transit). Cities along `row 7`, cols 4–7 (대구/광주/춘천/제주).

Both robots: every cell is connected to its 4 cardinal neighbors (full grid). The bot tracks `currentPose = {x, y, heading}` and navigates with **BFS shortest-path, re-planned at every crossing**:

1. `navigateTo(tx, ty)` loops until the pose reaches the target. The off-grid start pad (`x < 0`) is handled first: rotate `HD_EAST` and step one cell onto the grid.
2. `computeBfsPath(tx, ty)` runs a breadth-first search from `currentPose` over `lookupConn()` connectivity — minus statically/dynamically blocked neighbors (`maskBlockedNeighbors`), the west isolation boundary (`NAV_MIN_X`), and the last temporary one-direction mask — filling `_pathX/_pathY` with the shortest route (start cell excluded, target included). Returns `false` if no path exists.
3. The bot drives that path one cell at a time: `rotateToHeading()` issues a `PivotTurn*`/`TurnHalf` as needed, then `DoLineTrace(1)` advances one crossing.
4. If `DoLineTrace(1)` returns `false` (obstacle), the bot registers the would-be cell via `addDynBlockedCell()`, records a temporary directional mask, `break`s out of the path loop, and re-runs BFS from the current cell — immediately taking the new shortest detour. A loop guard (`GRID_COLS*GRID_ROWS*4 + 8` replans) backstops infinite loops.

(This replaced the old greedy + DFS-backtracking navigator, which could wander when the cell just before the target was blocked on a single-file approach. `desiredHeading()`/`maskCellsOnPath()`/visit-counter logic and the `NAVLOG_TAG_DEADEND` tag were removed.)

Adding a new city: just add a `CityCoord` mapping RFID UID → `(x, y)` in `CITY_COORDS[]` (in `Settings_robot*.h`). **No path code or map edit needed** — BFS finds the route. If a future layout has a gap in the grid (some cell missing a direction), add an override inside `lookupConn()`.

`navigateTo()` returns `bool` — `true` on reaching the target, `false` on STUCK (prints `"Nav STUCK: no path (BFS)."`, `"... guard exceeded."`, or `"... obstacle at start-pad exit."`). The `eWareHousePosition` dispatch checks the return: on success, runs the drop-cargo + return-trip sequence normally; on failure, **keeps cargo on the lifter** (skips the city `LifterDown` choreography and the final warehouse `LifterUp`) and still navigates back to `(WAREHOUSE_X, WAREHOUSE_Y)`. This avoids the old bug where a failed forward trip triggered the "arrived at city" choreography mid-map.

**Map isolation** (`NAV_MIN_X`): robot 1 leaves it at `-128` (inert). Robot 2 sets it to `4` immediately after the initial transit (`eInitialPosition`), and `computeBfsPath` drops the west (`CONN_W`) edge whenever `x - 1 < _navMinX`, structurally confining robot 2 to cols ≥ 4 for all later round-trips. (Robot 1 is confined by `GRID_COLS = 4`.)

**Serial status map**: typing `m`/`p` into the serial monitor calls `PrintStatusMap()` — an ASCII grid (`@`=pose, `#`=warehouse, `!`=dynamic block, `x`=static block, `C`=city, `:`=isolation off-limits). Only processed while idle at the warehouse, since `navigateTo` blocks the loop while driving. The last trip's BFS evaluations (`Eval`) and dynamic blocks (`DynBlock`) are also logged to EEPROM (NavLog, addresses `[0, NAVLOG_ENTRIES*8+1]`) and auto-dumped on boot.

**Precise realign** (the reverse-then-forward dance in `LineTracer()`) only runs when arriving at `y == 0` (warehouse row) or `y == 7` (city row) — the rows where a clean turn matters. Intermediate crossings just count and pass through with a brief stop. Controlled by the `_preciseRealign` member, set per call by the `precise` parameter of `DoLineTrace()`.

Initial pose is set in the `eInitialPosition` case from the start-tag: `currentPose = {INIT_START_X, INIT_START_Y, INIT_START_HEADING}` (robot 1: `(-1, 3, HD_EAST)`). `navigateTo(WAREHOUSE_X, WAREHOUSE_Y)` then drives onto the grid and to the warehouse, and `_navMinX` is set to `NAV_MIN_X` immediately after.

The `eTargetPosition` enum value is unused; `currentPosition` only ever holds `eInitialPosition` or `eWareHousePosition`.

### Line tracing

`DoLineTrace(targetCount)` loops on `LineTracer()` until `nLineCounter == targetCount` crossings have been counted, while polling `CheckObstacle()` between iterations.

- `LineTrace()` runs a **PD-controller** (`PID_KP`, `PID_KD`, `PID_MAX_CORRECTION`) over normalized bottom-sensor readings for *steering*. The *base PWM* comes from a **straight motion profile** (a slew-rate-limited setpoint in `_drivePwm`, set per call from `Settings_robot*.h`): a run starts at `DRIVE_START_PWM` (kick), accelerates over `DRIVE_ACCEL_MS` toward cruise (`MOTOR_POWER`, or `MOTOR_POWER_CARGO` with payload), then in the run's last `DRIVE_BRAKE_CELLS` cell(s) decelerates over `DRIVE_DECEL_MS` toward `DRIVE_END_PWM`. This yields a trapezoid on long runs / triangle on 1-cell runs (no encoders — PWM≈speed). The start/cruise/end PWMs and accel/decel times each have a `*_CARGO` variant for payload (`DRIVE_BRAKE_CELLS` is shared). (This replaced the old `CROSSING_APPROACH_*` step-decel; `CROSSING_PASS_POWER` was renamed `DRIVE_END_PWM`.)
- A "crossing" = both bottom sensors simultaneously read above `LINEDETECT_NORM_MIN` (normalized 0–1000 scale, default 700). `bSignalHigh` debounces so one physical crossing increments `nLineCounter` exactly once. At a crossing the bot drives forward for `CROSSING_PASS_MS` (a blind window, no sampling → prevents double-count) — at cruise (`_drivePwm`) for intermediate crossings, at `DRIVE_END_PWM` for the run's final crossing. **Intermediate crossings are passed at cruise with no speed dip** (the no-stop run behavior); deceleration happens only in the last cell.
- On reaching `targetCount` **at `y == 0`/`y == 7`** (precise realign), `LineTracer()` does a deliberate **reverse-then-forward re-alignment**: back up `REALIGN_BACKUP_MS` (~240 ms) to fully clear the line, then creep forward at reduced power until both sensors hit the line again (`REALIGN_CREEP_MS`). Tweak `REALIGN_BACKUP_MS` in `Settings_robot*.h` if the bot under- or over-shoots. Intermediate crossings skip the dance (brief stop only). The whole dance can be disabled with `PRECISE_REALIGN_ENABLE 0` (falls back to a brief stop). (These constants, plus `CROSSING_PASS_MS` for the crossing-pass forward nudge, were extracted from hardcoded values when the always-`1.0` `SPEED_SCALE` global was removed.)

### Obstacle handling

When `CheckObstacle()` (front-center / front-left / front-right IR each below `OBSTACLE_THRESHOLD` / `OBSTACLE_THRESHOLD_SIDE`, with one debounce re-read) trips mid-`DoLineTrace(1)`:

1. The bot `Stop()`s, then `ReverseToPreviousNode()` backs up until both bottom sensors re-find a line, creeps forward to realign, and stops.
2. `DoLineTrace(1)` returns `false`.
3. `navigateTo` registers the would-be cell via `addDynBlockedCell()` (persisted for the rest of the power cycle so future trips pre-avoid it), records a temporary one-direction mask for the immediate replan, and re-runs BFS from the current cell → new shortest detour.

There is **no hardcoded detour choreography** anymore — the old `PivotTurnLeft → DoLineTrace → ...` sequence was removed with the BFS rewrite, and `DoLineTrace` no longer recurses. Avoidance emerges entirely from re-planning, so the dynamic-block list (`g_dynBlocked`, capped at `MAX_DYN_BLOCKED`) is the only obstacle state to reason about.

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

These are written by the `MotorCalibration/` sketch in this repo. The `MotorCalibration/` folder also has a `Settings.h` / `Settings_robot1.h` / `Settings_robot2.h` split (same `ROBOT_ID` selector pattern) holding `CALIB_R` / `CALIB_L` constants for the calibration routine. Flashing the main sketch does not overwrite EEPROM calibration, but flashing a stock sketch can. `normalizeLeft/Right()` maps raw analog reads to a 0–1000 scale using these values, and `drive()` multiplies PWM output by the calib floats, so a fresh board with zeroed EEPROM will not move correctly.

### Hardware pin map (from `Controller.h`)

Digital: RFID SS=2, Buzzer=3, RFID RST=4, Left PWM=5, Right PWM=6, Left Dir=7, Right Dir=8, Lift Servo=9
Analog: Front Center=A0, Front Left=A1, Front Right=A2, User Button=A3, Bottom Left=A6, Bottom Right=A7

Note `FORWARD`/`BACKWARD` direction bits are **inverted between the two motors** inside `drive()` — the wheels are mounted mirrored. Don't "fix" the asymmetry without rewiring.
