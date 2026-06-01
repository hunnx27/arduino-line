# 듀얼 로봇 빌드 분기 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `arduino-line` 스케치를 빌드타임 `ROBOT_ID`(1/2)로 분기시켜 두 로봇이 단일 8열 격자에서 각자 맵을 돌게 하되, 로봇1 거동은 무손실 보존한다.

**Architecture:** `Settings.h` 를 얇은 셀렉터로 바꾸고 `Settings_robot1.h`(기존 그대로) / `Settings_robot2.h`(맵2 완본)로 완전 분리. 네비게이터는 이미 `GRID_COLS`/`WAREHOUSE_*`/`CITY_COORDS` 등 매크로만으로 임의 격자를 풀므로, 로직 변경은 3곳뿐 — `NAV_PATH_MAX` 격자 자동산출, `NAV_MIN_X` 서쪽 격리, 부팅 `ROBOT_ID` 로그.

**Tech Stack:** Arduino C++ (ATmega328P / Arduino **Nano**, 16MHz), arduino-cli 또는 Arduino IDE. 단위 테스트 프레임워크 없음 → 검증은 컴파일 성공 + 온디바이스 시리얼 트레이스.

설계 근거: [docs/superpowers/specs/2026-06-01-dual-robot-settings-split-design.md](../specs/2026-06-01-dual-robot-settings-split-design.md)

---

## File Structure

| 파일 | 역할 | 변경 |
|---|---|---|
| `Settings.h` | `ROBOT_ID` 셀렉터 (값 없음) | 전면 교체 |
| `Settings_robot1.h` | 로봇1 맵+튜닝 완본 (= 기존 Settings.h) | 신규 (복사) |
| `Settings_robot2.h` | 로봇2 맵+튜닝 완본 | 신규 |
| `Controller.h` | `NAV_PATH_MAX` 자동산출, `_navMinX` 멤버 | 2줄 수정/추가 |
| `Controller.cpp` | 부팅 로그, `navigateTo` 서쪽 마스킹, init 후 격리 활성화 | 3곳 수정 |
| `arduino-line.ino` | — | 변경 없음 |

> **컴파일 검증 명령(공통).** 각 태스크 끝에서 사용.
> - 로봇1(기본): `arduino-cli compile --fqbn arduino:avr:nano .`
> - 로봇2: `Settings.h` 의 `#define ROBOT_ID 1` 을 임시로 `2` 로 바꿔 `arduino-cli compile --fqbn arduino:avr:nano .` 후 **반드시 1로 되돌림**. (또는 `--build-property "build.extra_flags=-DROBOT_ID=2"`.)
> - `arduino-cli` 가 없으면 Arduino IDE 로 두 경우 모두 "Verify(컴파일)" 만 수행. 설치는 하지 말 것.

---

## Task 1: Settings 셀렉터 분리 (로봇1 무손실)

**Files:**
- Create: `Settings_robot1.h` (현재 `Settings.h` 복사 기반)
- Modify: `Settings.h` (전면 교체)

- [ ] **Step 1: 현재 Settings.h 를 robot1 완본으로 복사**

```bash
cd "c:/develop/workspace_lecture/arduino-line"
cp Settings.h Settings_robot1.h
```

- [ ] **Step 2: `Settings_robot1.h` 헤더가드 변경**

`Settings_robot1.h` 의 맨 위/아래 가드를 바꾼다.

맨 위 2줄:
```cpp
#ifndef SETTINGS_H
#define SETTINGS_H
```
→
```cpp
#ifndef SETTINGS_ROBOT1_H
#define SETTINGS_ROBOT1_H
```

맨 아래:
```cpp
#endif // SETTINGS_H
```
→
```cpp
#endif // SETTINGS_ROBOT1_H
```

- [ ] **Step 3: `Settings_robot1.h` 에 맵 격리 매크로 추가 (로봇1=무제한)**

`물류창고 위치` 블록(`#define WAREHOUSE_Y 0` 줄) 바로 아래에 새 블록 삽입:

```cpp
// -------------------- 맵 격리 경계 --------------------
// 횡단(최초 진입) 이후 이 열 미만으로 서쪽 이동 금지 → 자기 맵에 격리.
// 로봇1 은 단일 맵이므로 -128(무제한) = 격리 안 함.
#define NAV_MIN_X -128
```

- [ ] **Step 4: `Settings.h` 를 셀렉터로 전면 교체**

`Settings.h` 전체 내용을 아래로 덮어쓴다(Write):

```cpp
#ifndef SETTINGS_H
#define SETTINGS_H

// =====================================================================
// Settings.h — 빌드 대상 로봇 선택 셀렉터.
// 실제 튜닝/맵 값은 Settings_robot1.h / Settings_robot2.h 에 있다.
//
// 전환 방법:
//   1) 아래 ROBOT_ID 를 1 또는 2 로 바꿔 업로드, 또는
//   2) 파일 수정 없이 빌드플래그: -DROBOT_ID=2
//      (예) arduino-cli compile --fqbn arduino:avr:nano \
//             --build-property "build.extra_flags=-DROBOT_ID=2" .
// =====================================================================

#ifndef ROBOT_ID
  #define ROBOT_ID 1
#endif

#if   ROBOT_ID == 1
  #include "Settings_robot1.h"
#elif ROBOT_ID == 2
  #include "Settings_robot2.h"
#else
  #error "ROBOT_ID must be 1 or 2"
#endif

#endif // SETTINGS_H
```

- [ ] **Step 5: 컴파일 검증 (로봇1)**

Run: `arduino-cli compile --fqbn arduino:avr:nano .`
Expected: 성공. RAM/Flash 리포트가 변경 전과 동일(순수 리팩터).

- [ ] **Step 6: 커밋**

```bash
git add Settings.h Settings_robot1.h
git commit -m "리팩터: Settings.h 를 ROBOT_ID 셀렉터로 분리 (robot1 완본 추출)"
```

---

## Task 2: `Settings_robot2.h` 생성 (맵2 완본)

**Files:**
- Create: `Settings_robot2.h` (`Settings_robot1.h` 복사 기반)

- [ ] **Step 1: robot1 완본을 robot2 로 복사**

```bash
cd "c:/develop/workspace_lecture/arduino-line"
cp Settings_robot1.h Settings_robot2.h
```

- [ ] **Step 2: 헤더가드 변경**

`Settings_robot2.h` 의 가드:
```cpp
#ifndef SETTINGS_ROBOT1_H
#define SETTINGS_ROBOT1_H
```
→
```cpp
#ifndef SETTINGS_ROBOT2_H
#define SETTINGS_ROBOT2_H
```
맨 아래 `#endif // SETTINGS_ROBOT1_H` → `#endif // SETTINGS_ROBOT2_H`.

- [ ] **Step 3: 격자 폭 변경**

```cpp
#define GRID_COLS 4
```
→
```cpp
#define GRID_COLS 8
```

- [ ] **Step 4: 출발 행 (현장 입력값)**

```cpp
#define INIT_START_Y         3
```
→
```cpp
#define INIT_START_Y         3   // TODO: 로봇2 출발 행 — 현장 측정값으로 교체 (로봇1 과 다른 행)
```

- [ ] **Step 5: 시작 카드 UID (현장 입력값)**

```cpp
#define START_RFID_UID "647AB573"
```
→
```cpp
#define START_RFID_UID "00000000"   // TODO: 로봇2 시작 카드 UID — 태그 스캔 후 교체
```

- [ ] **Step 6: 창고 열 (현장 입력값)**

```cpp
#define WAREHOUSE_X 2
```
→
```cpp
#define WAREHOUSE_X 6   // TODO: 로봇2 창고 열 (4~7 중 하나) — 현장값으로 교체
```

- [ ] **Step 7: 도시 좌표 (col 4~7, 현장 UID)**

```cpp
static const CityCoord CITY_COORDS[] = {
    {"647AB573",         0, 7},  // Seoul   (col 0)
    {"",         1, 7},  // Incheon (col 1)
    {"",         2, 7},  // Sejong  (col 2 — 메인 라인. 창고 바로 아래)
    {"148EC573", 3, 7},  // Daejeon (col 3)
};
```
→
```cpp
static const CityCoord CITY_COORDS[] = {
    {"", 4, 7},  // 맵2 도시 col 4 — TODO: 실 RFID UID
    {"", 5, 7},  // 맵2 도시 col 5 — TODO: 실 RFID UID
    {"", 6, 7},  // 맵2 도시 col 6 — TODO: 실 RFID UID
    {"", 7, 7},  // 맵2 도시 col 7 — TODO: 실 RFID UID
};
```

- [ ] **Step 8: 정적 차단 셀 (맵1 패턴 +4 시프트 기본값)**

```cpp
static const BlockedCell BLOCKED_CELLS[] = {
    {2, 3}, {3, 5}, {1, 5}
};
```
→
```cpp
static const BlockedCell BLOCKED_CELLS[] = {
    // 맵1 {2,3}{3,5}{1,5} 를 +4 열 시프트한 기본값. TODO: 맵2 실제 장애물로 확인/교체.
    // (횡단 통로 행에 맵1 장애물이 걸리면 그 좌표도 여기 추가.)
    {6, 3}, {7, 5}, {5, 5}
};
```

- [ ] **Step 9: 맵 격리 경계 (로봇2=4)**

```cpp
#define NAV_MIN_X -128
```
→
```cpp
#define NAV_MIN_X 4   // 로봇2: 횡단 후 cols 4~7 에 고정 (맵1 침범 금지)
```

- [ ] **Step 10: 컴파일 검증 (로봇2)**

`Settings.h` 의 `#define ROBOT_ID 1` 을 임시로 `2` 로 변경.
Run: `arduino-cli compile --fqbn arduino:avr:nano .`
Expected: 성공. 끝나면 `Settings.h` 를 `ROBOT_ID 1` 로 되돌린다.

- [ ] **Step 11: 커밋**

```bash
git add Settings_robot2.h
git commit -m "추가: Settings_robot2.h (맵2 완본, 현장값 TODO 표기)"
```

---

## Task 3: `NAV_PATH_MAX` 격자 크기 자동산출

**Files:**
- Modify: `Controller.h:149`

- [ ] **Step 1: 고정 34 → 격자 산출로 변경**

```cpp
    static const uint8_t NAV_PATH_MAX = 34;
```
→
```cpp
    static const uint8_t NAV_PATH_MAX = GRID_COLS * GRID_ROWS + 2;  // 로봇1:34, 로봇2:66
```

- [ ] **Step 2: 컴파일 검증 (양쪽)**

Run (로봇1): `arduino-cli compile --fqbn arduino:avr:nano .` → 성공.
Run (로봇2): `Settings.h` ROBOT_ID=2 로 임시 변경 후 컴파일 → 성공, 되돌리기.
Expected: 둘 다 성공. 로봇1 RAM 변동 없음(34=34), 로봇2 RAM 약간 증가.

- [ ] **Step 3: 커밋**

```bash
git add Controller.h
git commit -m "변경: NAV_PATH_MAX 를 GRID_COLS*GRID_ROWS+2 로 자동산출 (8열 대응)"
```

---

## Task 4: 부팅 `ROBOT_ID` 로그

**Files:**
- Modify: `Controller.cpp` `init()` (캘리브 출력 블록 직후, 현재 293행 근처)

- [ ] **Step 1: 캘리브 출력 다음에 로봇 식별 로그 추가**

다음 블록을 찾는다:
```cpp
    Serial.print("rW="); Serial.print(_rightWhite);
    Serial.print(" lW="); Serial.print(_leftWhite);
    Serial.print(" rB="); Serial.print(_rightBlack);
    Serial.print(" lB="); Serial.println(_leftBlack);
```
바로 아래에 삽입:
```cpp
    Serial.print(F("ROBOT_ID=")); Serial.print(ROBOT_ID);
    Serial.print(F(" GRID=")); Serial.print(GRID_COLS); Serial.print(F("x")); Serial.print(GRID_ROWS);
    Serial.print(F(" WH=(")); Serial.print(WAREHOUSE_X); Serial.print(F(",")); Serial.print(WAREHOUSE_Y); Serial.println(F(")"));
```

- [ ] **Step 2: 컴파일 검증**

Run: `arduino-cli compile --fqbn arduino:avr:nano .`
Expected: 성공.

- [ ] **Step 3: 커밋**

```bash
git add Controller.cpp
git commit -m "추가: 부팅 시 ROBOT_ID / GRID / 창고 좌표 시리얼 로그"
```

---

## Task 5: `NAV_MIN_X` 서쪽 격리 (멤버 + 마스킹 + 활성화)

**Files:**
- Modify: `Controller.h` (`_navMinX` 멤버 추가, 현재 143행 근처)
- Modify: `Controller.cpp` `navigateTo()` (서쪽 마스킹, 현재 922행 근처)
- Modify: `Controller.cpp` `ProcessRFIDRead()` eInitialPosition (활성화, 현재 437행 근처)

- [ ] **Step 1: `Controller.h` 에 `_navMinX` 멤버 추가**

다음 블록을 찾는다:
```cpp
    int8_t _blockedAtX = -128;              // 직전에 장애물로 막힌 좌표 (-128 = 없음)
    int8_t _blockedAtY = -128;
    uint8_t _blockedDirBit = 0;             // 막힌 방향의 CONN_* 비트
```
바로 아래에 삽입:
```cpp
    // 맵 격리 경계: 이 열 미만으로 서쪽 이동 금지. 부팅 시 무제한(-128),
    // eInitialPosition 최초 창고 도착 후 NAV_MIN_X 로 설정 (로봇2=4, 로봇1=-128 무효).
    int8_t _navMinX = -128;
```

- [ ] **Step 2: `navigateTo()` 에 서쪽 경계 마스킹 추가**

다음 줄을 찾는다:
```cpp
        uint8_t fwdConn = maskCellsOnPath(currentPose.x, currentPose.y, conn);
```
바로 아래에 삽입:
```cpp

        // 맵 격리: _navMinX 미만 열로 가는 서쪽 이동 차단.
        // (로봇2 격리용. 로봇1 은 _navMinX=-128 이라 절대 안 걸림.)
        if ((fwdConn & CONN_W) && (currentPose.x - 1 < _navMinX)) fwdConn &= ~CONN_W;
```

- [ ] **Step 3: 최초 창고 도착 후 격리 활성화**

`ProcessRFIDRead()` eInitialPosition 의 다음 줄을 찾는다:
```cpp
                navigateTo(WAREHOUSE_X, WAREHOUSE_Y);
                rotateToHeading(HD_SOUTH);  // 창고에서 도시 방향(+y, row 7 쪽) 으로 정렬
```
첫 줄과 둘째 줄 사이에 삽입:
```cpp
                navigateTo(WAREHOUSE_X, WAREHOUSE_Y);
                _navMinX = NAV_MIN_X;       // 횡단 완료 → 맵 격리 활성화 (로봇2: cols ≥4 고정)
                rotateToHeading(HD_SOUTH);  // 창고에서 도시 방향(+y, row 7 쪽) 으로 정렬
```

> 주의: eInitialPosition 안에 `navigateTo(WAREHOUSE_X, WAREHOUSE_Y)` 호출은 한 곳뿐이다(횡단용). eWareHousePosition 의 복귀 navigateTo 에는 넣지 말 것.

- [ ] **Step 4: 컴파일 검증 (양쪽)**

Run (로봇1): `arduino-cli compile --fqbn arduino:avr:nano .` → 성공.
Run (로봇2): `Settings.h` ROBOT_ID=2 임시 변경 후 컴파일 → 성공, 되돌리기.
Expected: 둘 다 성공.

- [ ] **Step 5: 커밋**

```bash
git add Controller.h Controller.cpp
git commit -m "추가: NAV_MIN_X 서쪽 격리 — 로봇2 횡단 후 자기 맵 고정"
```

---

## Task 6: 온디바이스 검증 (하드웨어 — 자동화 불가)

> 자동 테스트 프레임워크가 없으므로 시리얼 모니터(9600)로 거동을 직접 관찰한다. 커밋 없음. 각 항목은 관찰 기록만.

- [ ] **Step 1: 로봇1 회귀 — 펌웨어 업로드 & 거동 무변**

`ROBOT_ID=1` 빌드를 로봇1 보드에 업로드.
Run: `arduino-cli upload --fqbn arduino:avr:nano -p <PORT> .` 후 `arduino-cli monitor -p <PORT> -c baudrate=9600`.
Expected: 부팅 로그에 `ROBOT_ID=1 GRID=4x8 WH=(2,0)`. 시작 태그 후 기존과 동일하게 창고 이동 → 도시 왕복. **거동 변화 없음**.

- [ ] **Step 2: 로봇2 — 횡단 경로 시리얼 트레이스**

`Settings_robot2.h` 의 현장값(INIT_START_Y, WAREHOUSE_X, 도시 UID)을 실제 값으로 채운 뒤 `ROBOT_ID=2` 빌드를 로봇2 보드에 업로드.
Expected: 부팅 로그 `ROBOT_ID=2 GRID=8x8 WH=(<창고열>,0)`. 시작 후 `Nav:` / `Eval` 로그가 `(-1,y2)` → 맵1(col 0~3) 관통 → 맵2 창고 좌표 도달 순서로 찍힘. 미아/STUCK 없이 창고 도착.

- [ ] **Step 3: 로봇2 — 격리 확인**

횡단 후(첫 창고 도착 후) 임의 도시 왕복 1회 관찰. 의도적으로 맵2 서쪽에 장애물을 두면, `Eval` 로그의 `fwd=` 비트에 W(0b1000)가 col 4 에서 나타나지 않아야 함(서쪽 차단). col 3 이하로 진입하는 `Nav:` 좌표가 절대 안 나와야 함.

- [ ] **Step 4: 긴 횡단 드리프트 점검 (스펙 리스크 #2)**

로봇2 횡단 중 라인 이탈/오카운트 없이 직진하는지 육안+로그 확인. 문제 시: 횡단 행 한정 정밀정렬 추가를 별도 이슈로 기록(이 계획 범위 밖).

---

## Self-Review

**Spec coverage:**
- §3 셀렉터 구조 → Task 1. ✔
- §4 Settings_robot2 델타(GRID/INIT_START_Y/WAREHOUSE/CITY/BLOCKED/NAV_MIN_X/UID) → Task 2 Step 3~9. ✔
- §5.1 NAV_PATH_MAX → Task 3. ✔
- §5.2 NAV_MIN_X(멤버/마스킹/활성화) → Task 5. ✔
- §5.3 부팅 로그 → Task 4. ✔
- §6 격리 동작(서쪽 마스킹 규칙, 활성 시점) → Task 5 Step 2~3. ✔
- §11 검증(회귀/컴파일/횡단/격리/부팅로그) → 각 Task 컴파일 + Task 6. ✔
- §12 문서 정정(README/CLAUDE Uno→Nano, 듀얼구조) → 본 계획 범위 밖(후속). 명시.
- MotorCalibration CALIB 분리 → 별도 subagent 진행 중. 본 계획 범위 밖. 명시.

**Placeholder scan:** 〈현장 입력〉(INIT_START_Y/WAREHOUSE_X/도시UID/START_RFID_UID)은 코드 placeholder 가 아니라 하드웨어 측정값 — 명시적 TODO 주석 + 안전한 임시값(컴파일 가능)으로 처리. 설계 공백 아님. 그 외 "TBD/나중에" 없음.

**Type consistency:** `_navMinX`(int8_t) 선언(Task5 S1) ↔ 사용(S2 비교, S3 대입) 일치. `NAV_MIN_X` 매크로 robot1(-128)/robot2(4) 양쪽 정의(Task1 S3, Task2 S9) → Controller.cpp 참조(Task5) 시 항상 정의됨. `NAV_PATH_MAX` 식이 `GRID_COLS`/`GRID_ROWS`(Settings 매크로, Controller.h include 뒤) 참조 — 순서 OK.

---

## 후속 (이 계획 범위 밖)

- 문서 정정: `README.md` / `CLAUDE.md` 의 보드 Uno→Nano·FQBN, 듀얼 로봇 구조 반영.
- MotorCalibration CALIB_R/L 분리 (subagent 진행 중).
- (조건부) 횡단 행 드리프트 발견 시 정밀정렬 확장.
