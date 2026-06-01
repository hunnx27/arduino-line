# 듀얼 로봇 빌드 분기 설계 (Robot1 / Robot2)

작성일: 2026-06-01

## 1. 배경 / 목표

물류 로봇이 1대 → **2대**로 늘어난다. 두 로봇은 같은 규격의 맵을 좌우로 이어붙인 **단일 8열 격자** 위에서 동작한다.

- **로봇1**: 현재와 100% 동일. 맵1(col 0~3)에서만 창고↔도시 왕복.
- **로봇2**: 맵2(col 4~7)에서 창고↔도시 왕복. 단 **최초 출발 1회만** 맵1을 가로질러 맵2로 진입.

목표: **네비게이션/구동 로직(`Controller.cpp`/`.h`)은 거의 손대지 않고**, 맵·튜닝값을 로봇별로 완전히 분리해 빌드타임에 선택한다. 로봇1 거동은 무손실로 보존한다.

## 2. 물리 배치 & 좌표 모델

```
        맵1 (col 0~3)                    맵2 (col 4~7)
  ◉(-1, y2)  ← 로봇2 출발 (col -1, 행은 로봇1과 다름)
  ◉(-1, 3)   ← 로봇1 출발
        x=0  x=1  x=2  x=3   │   x=4  x=5  x=6  x=7
 y=0    .    .   ■창고1  .    │    .    .   ■창고2  .      ← 창고2 = col 4~7 중 하나
 y=3 ◉─(0,3)…………(3,3)──────┼──(4,3)…………(7,3)
 y=7  [도시 col0~3]          │   [도시 col4~7]
```

- **이음새 col 3 ↔ col 4 는 모든 행에서 라인 연결됨** → 풀그리드 가정 성립, `lookupConn` 예외 불필요.
- 두 로봇 모두 출발 패드는 `col = -1`(`INIT_START_X = -1`), **행(`INIT_START_Y`)만 다름**. heading 은 둘 다 `HD_EAST`.
- `HD_NORTH` = row 감소(y=0=창고행, 위쪽), `HD_SOUTH` = row 증가(y=7=도시행, 아래쪽).
- 로봇1은 `GRID_COLS=4` 라 구조적으로 동쪽(맵2)으로 못 나간다 → 자기 맵에 자동 격리.
- 로봇2는 `GRID_COLS=8`. 횡단 이후 맵1로 새지 않도록 `NAV_MIN_X` 로 격리(§6).

## 3. 아키텍처 — Settings 셀렉터 + 로봇별 완본 2개

기존 거동 튜닝이 전부 `Settings.h` 매크로로 빠져 있고, 네비게이터가 `GRID_COLS`/`GRID_ROWS`/`INIT_START_*`/`WAREHOUSE_*`/`CITY_COORDS` 만으로 임의 격자를 푼다는 점을 활용한다(검증 완료 — §10).

### 파일 구조

```
Settings.h            ← 얇은 셀렉터 (값 없음)
Settings_robot1.h     ← 현재 Settings.h 내용 전체 (맵1 + 튜닝). 로봇1 = 기존과 동일.
Settings_robot2.h     ← 전체 완본 (맵2 + 로봇2 튜닝)
```

**완전 분리**: 두 완본은 맵값뿐 아니라 PWM/PID/회전각 등 모든 튜닝 매크로를 각자 보유한다(로봇별 물리 편차 반영). 공유 섹션 없음.

### `Settings.h` (셀렉터)

```cpp
#ifndef SETTINGS_H
#define SETTINGS_H
// 빌드 대상 선택. 이 파일의 ROBOT_ID 를 바꾸거나, -DROBOT_ID=2 빌드플래그로 덮어쓴다.
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

- 분기점이 `Controller.h`/`Controller.cpp` 가 **공통으로 include하는 헤더 안**에 있어, 번역 단위 분리 문제(.ino 의 `#define` 이 `Controller.cpp` 에 전파 안 되는 문제)를 회피한다.
- `#ifndef ROBOT_ID` 가드 → 파일 수정 없이 `arduino-cli compile --build-property build.extra_flags=-DROBOT_ID=2` 로도 전환 가능.
- `Controller.h` 의 `#include "Settings.h"` 위치(타입 정의 뒤, class 앞)는 그대로. 두 완본도 같은 자리에서 펼쳐지므로 타입 의존성 순서 보존.
- `arduino-line.ino` 는 수정 불필요.
- 각 완본 파일은 자체 헤더가드(`SETTINGS_ROBOT1_H` / `SETTINGS_ROBOT2_H`).

## 4. `Settings_robot2.h` 델타 (로봇1 대비)

| 매크로 | 로봇1 | 로봇2 | 비고 |
|---|---|---|---|
| `GRID_COLS` | 4 | **8** | |
| `GRID_ROWS` | 8 | 8 | |
| `INIT_START_X` | -1 | -1 | 동일 |
| `INIT_START_Y` | 3 | **〈현장 입력〉** | 로봇2 출발 행 |
| `INIT_START_HEADING` | HD_EAST | HD_EAST | |
| `WAREHOUSE_X` | 2 | **〈현장 입력: 4~7〉** | |
| `WAREHOUSE_Y` | 0 | 0 | |
| `CITY_COORDS` | col 0~3, y=7 | **col 4~7, y=7 + 실 RFID UID** | 〈현장 태그 스캔〉 |
| `BLOCKED_CELLS` | 맵1 장애물 | 맵2 장애물(+ 횡단 통로상 맵1 장애물 있으면) | |
| `NAV_MIN_X` | **-128**(무제한) | **4** | §6 |
| `START_RFID_UID` | 〈로봇1 시작 태그〉 | 〈로봇2 시작 태그〉 | UID 중복 금지 |
| 튜닝값(PWM/PID/회전) | 로봇1 값 | 〈처음엔 복사 후 현장 튜닝〉 | |

> 〈현장 입력〉 항목은 물리 측정/태그 스캔이 필요한 하드웨어 값이다. 구현 시 명확한 `// TODO: 현장값` 주석 + 임시 기본값으로 채운다(설계 공백이 아니라 운영 입력값).

## 5. Settings 외 코드 변경 (3곳)

### 5.1 `Controller.h` — `NAV_PATH_MAX` 격자 크기 자동 산출

```cpp
// 기존: static const uint8_t NAV_PATH_MAX = 34;
static const uint8_t NAV_PATH_MAX = GRID_COLS * GRID_ROWS + 2;
```

- 로봇1: `4*8+2 = 34` → 기존과 동일(회귀 없음).
- 로봇2: `8*8+2 = 66` → 8×8 백트래킹 경로 스택 충분.
- `Settings.h` 가 class 정의 앞에서 include되므로 매크로 사용 가능. RAM 증가 ~+130B(경로 2배열 + g_visit), ATmega328P(Nano/Uno, SRAM 2KB)에서 안전.

### 5.2 `Controller.h` / `Controller.cpp` — `NAV_MIN_X` 격리 (§6)

- 멤버 추가: `int8_t _navMinX = -128;` (부팅 시 무제한 = 횡단 허용).
- `navigateTo()` 의 forward 후보 마스킹 파이프라인에 "서쪽 경계" 마스킹 1단계 추가.
- `ProcessRFIDRead()` `eInitialPosition` 에서 초기 `navigateTo(WAREHOUSE)` **성공 직후** `_navMinX = NAV_MIN_X;` 로 격리 활성화.

### 5.3 `Controller.cpp` `init()` — 부팅 로그 (무조건)

```cpp
Serial.print(F("ROBOT_ID=")); Serial.println(ROBOT_ID);
// (선택) GRID_COLS / WAREHOUSE_X,Y / INIT_START_Y 요약도 함께 출력해 맵 확인
```

엉뚱한 펌웨어를 엉뚱한 보드에 올렸을 때 즉시 육안 식별.

## 6. `NAV_MIN_X` 격리 동작 상세

목적: 횡단 이후 로봇2가 **구조적으로** cols 4~7 을 벗어나지 못하게 한다(로봇1이 `GRID_COLS=4` 로 동쪽에 못 가는 것과 대칭).

- `_navMinX` = 현재 허용되는 최소 열. 초기 `-128`(무제한).
- 마스킹 규칙: 현재 셀 `x` 에서 서쪽 이동 시 `x-1 < _navMinX` 이면 `CONN_W` 제거.
  - 로봇2 `_navMinX=4`: x=4 에서 west(→3) 차단, x=5 에서 west(→4) 허용 → cols ≥4 에 갇힘.
  - 로봇1 `_navMinX=-128`: 절대 안 걸림 → 무손실.
- **활성 시점**: `eInitialPosition` 에서 최초 `navigateTo(WAREHOUSE)` 성공 후. 그 전(횡단)에는 `-128` 이라 맵1 통과 허용.
- 백트래킹(부모로 후진)은 항상 직전 방문 셀(≥ `_navMinX`)로만 가므로 경계 위반 없음 → 별도 처리 불필요.

다운사이드 0: 로봇2는 횡단 후 맵1에 갈 정당한 이유가 없으므로, 막아도 정상 동작을 한 번도 방해하지 않는다.

## 7. 운영 제약 (코드 아님 — 운영에서 보장)

- **최초 횡단 구간은 두 맵의 공유 공간**이라 `NAV_MIN_X` 로 못 막는다(횡단 자체가 맵1 통과).
- 완화책: **"로봇2가 횡단을 끝낸 뒤 로봇1을 출발"** 시퀀싱을 운영에서 지킨다.
- 물리 백스톱: IR 장애물 감지가 하드 충돌은 막지만(앞에 다른 로봇 → 후진/우회), 횡단 중 마주치면 로봇2 길잃음 가능 → 시퀀싱으로 애초에 안 마주치게 한다.

## 8. 범위 밖 / 비포함 (v1)

- 두 로봇 간 통신/조율(런타임 상호 위치 인지) — 없음. 각자 독립 + 운영 시퀀싱.
- 횡단 시점 충돌을 막는 코드/하드웨어 안전장치 — 운영 시퀀싱으로 대체.
- 동적 맵 재구성, 3대 이상 확장.

## 9. 빌드 / 업로드

- 보드: Arduino **Nano** (ATmega328P, 16MHz). 컴파일 결과물은 Uno와 사실상 동일하나 **업로드 FQBN 은 `arduino:avr:nano`**(구형 부트로더면 `:cpu=atmega328old`).
- 로봇1: `ROBOT_ID=1`(기본) 빌드 → 로봇1 보드 업로드.
- 로봇2: `Settings.h` 의 `ROBOT_ID` 를 2로 (또는 `-DROBOT_ID=2`) → 로봇2 보드 업로드.
- **각 보드는 그 보드에서 별도 EEPROM 캘리브**(`Calibration.ino`/`MotorCalibration.ino`)를 먼저 수행해야 한다(흑/백 raw + 모터 calib). 미캘리브 보드는 정규화/구동이 망가짐.

## 10. 리스크 & 완화

| # | 리스크 | 상태/완화 |
|---|---|---|
| 1 | `lookupConn` 풀그리드 가정 vs 실제 라인 누락 | 이음새 모든 행 연결 확인 → **해소**. 향후 누락 생기면 `lookupConn` 예외 추가 |
| 2 | 긴 직선 횡단(7칸+) 헤딩 드리프트 누적(중간 행은 정밀정렬 없음) | **잔존**. 검증 필요. 문제 시 횡단 행 한정 정밀정렬 추가 검토 |
| 3 | 횡단 이후 로봇2가 맵1로 샘 → 충돌 | `NAV_MIN_X=4` 로 **구조적 차단** |
| 4 | 횡단 중 두 로봇 조우 | 운영 시퀀싱(로봇2 먼저) + IR 백스톱 |
| 5 | 두 완본 파일 동기화 누락(공통 로직-튜닝 개선 시) | 완전분리의 감수 비용. 각 파일에 "최종 동기화" 주석 권장 |
| 6 | 보드별 EEPROM 캘리브 누락 | 운영 체크리스트 |
| 7 | 엉뚱한 펌웨어 업로드 | 부팅 `ROBOT_ID` 로그 |
| 8 | RAM +130B | Nano 2KB 내 안전, 컴파일 리포트로 확인 |

## 11. 검증 방법

1. **로봇1 회귀**: `ROBOT_ID=1` 거동이 기존과 무변(`NAV_MIN_X=-128` 이라 마스킹 무효, `NAV_PATH_MAX=34` 로 동일 크기). 코드는 멤버/마스킹이 추가되므로 바이너리 동일은 아님 — 거동 동일이 판정 기준.
2. **컴파일**: `ROBOT_ID=1`/`2` 각각 컴파일 성공 + RAM/Flash 리포트 확인.
3. **로봇2 횡단(시리얼 트레이스)**: `Nav:`/`Eval` 로그로 `(-1,y2)→…→(WAREHOUSE_X,0)` 경로가 맵1 관통 후 맵2 도달하는지.
4. **로봇2 격리**: 횡단 후 `_navMinX=4` 활성. 의도적으로 맵2 서쪽에 장애물을 둬도 col 4 미만으로 안 가는지(`Eval` 의 `fwd` 비트에 W 없음) 확인.
5. **부팅 로그**: 각 보드에서 `ROBOT_ID=` 올바른 값 출력.

## 12. 후속 (문서 정정 — 별도)

- `CLAUDE.md` / `README.md`: 보드 Uno→**Nano**, FQBN, 듀얼 로봇 구조(Settings 셀렉터·`NAV_MIN_X`) 반영.
