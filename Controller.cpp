#include "Controller.h"

// === 좌표 맵 (4열 × 8행 풀 그리드) ===
// 창고: (2, 0), 도시 행: y=7. 모든 셀에 4방향(N/E/S/W) 라인이 있다고 가정 —
// 가장자리 셀은 격자 밖 방향만 자동 제외. 장애물 우회 시 좌/우로 빠질 라인 보장.
// 만약 layout 에 누락 구간이 생기면 lookupConn 안에서 예외 처리.
// GRID_COLS, GRID_ROWS, CITY_COORDS, BLOCKED_CELLS, INIT_* 등은 Settings.h 참조.

//        x=0         x=1           x=2           x=3
//        ┌──────┐    ┌──────┐      ┌──────────┐  ┌──────┐
//  y=0   │(0,0) │────│ (1,0)│──────│■(2,0)    │──│(3,0) │   ■ 창고
//        └──┬───┘    └──┬───┘      └────┬─────┘  └──┬───┘   WAREHOUSE
//           │           │                │            │
//  y=1     (0,1)───────(1,1)────────────(2,1)────────(3,1)
//           │           │                │            │
//  y=2     (0,2)───────(1,2)────────────(2,2)────────(3,2)
//           │           │                │            │
// ◉(-1,3)─(0,3)───────(1,3)────────────╳(2,3)╳──────(3,3)   ◉ 시작 RFID — y=3, 그리드 한 칸 뒤. heading=EAST
//           │           │              ✕ STATIC          │
//  y=4     (0,4)───────(1,4)────────────(2,4)────────(3,4)
//           │           │                │            │
//  y=5     (0,5)─────╳(1,5)╳───────────(2,5)──────╳(3,5)╳
//           │       ✕ STATIC              │       ✕ STATIC
//  y=6     (0,6)───────(1,6)────────────(2,6)─────────(3,6)
//           │           │                │            │
//  y=7   ┌──┴───┐    ┌──┴───┐      ┌────┴─────┐  ┌──┴───┐
//        │Seoul │────│Inchn │──────│Sejong ★ │──│Daejn │   ← 도시 라인 (CITY_COORDS in Settings.h)
//        │ (0,7)│    │ (1,7)│      │ (2,7)    │  │ (3,7)│
//        └──────┘    └──────┘      └──────────┘  └──────┘

// === 네비게이션 헬퍼 ===
static Heading opposite(Heading h) { return (Heading)((h + 2) % 4); }

static int8_t headingDx(Heading h) {
    switch (h) { case HD_EAST: return 1; case HD_WEST: return -1; default: return 0; }
}
// HD_NORTH = row 감소 방향 (사용자 mental model 의 "북쪽" = 격자 위쪽 = row 0 방향)
static int8_t headingDy(Heading h) {
    switch (h) { case HD_NORTH: return -1; case HD_SOUTH: return 1; default: return 0; }
}

static uint8_t lookupConn(int8_t x, int8_t y) {
    // 시작 RFID 셀 — 그리드 외부 한 칸 뒤. 동쪽으로만 진입 가능 (그리드의 (0, 3) 으로).
    if (x == INIT_START_X && y == INIT_START_Y) return CONN_E;

    if (x < 0 || x >= GRID_COLS || y < 0 || y >= GRID_ROWS) return 0;
    uint8_t conn = 0;
    if (y - 1 >= 0)        conn |= CONN_N;  // 위쪽(y-1) 으로 갈 수 있나
    if (y + 1 < GRID_ROWS) conn |= CONN_S;  // 아래쪽(y+1) 으로
    if (x + 1 < GRID_COLS) conn |= CONN_E;
    if (x - 1 >= 0)        conn |= CONN_W;
    return conn;
}

// 우선순위:
//   0) 현재 heading 이 도움 되면 그대로 유지 (불필요한 회전 제거)
//   1) Y(세로) → X(가로) 직접 방향
//   2) 직접 방향 막혔으면 perpendicular fallback (장애물 우회용)
static Heading desiredHeading(int8_t dx, int8_t dy, uint8_t conn, Heading currentHeading) {
    // 0) 현재 heading 유지 가능하면 우선
    switch (currentHeading) {
        case HD_NORTH: if (dy < 0 && (conn & CONN_N)) return HD_NORTH; break;
        case HD_SOUTH: if (dy > 0 && (conn & CONN_S)) return HD_SOUTH; break;
        case HD_EAST:  if (dx > 0 && (conn & CONN_E)) return HD_EAST;  break;
        case HD_WEST:  if (dx < 0 && (conn & CONN_W)) return HD_WEST;  break;
        default: break;
    }

    // 1) Y 우선 → X 직접 방향
    if (dy < 0 && (conn & CONN_N)) return HD_NORTH;
    if (dy > 0 && (conn & CONN_S)) return HD_SOUTH;
    if (dx > 0 && (conn & CONN_E)) return HD_EAST;
    if (dx < 0 && (conn & CONN_W)) return HD_WEST;

    // 2) Perpendicular fallback
    if (dy != 0) {
        if (conn & CONN_E) return HD_EAST;
        if (conn & CONN_W) return HD_WEST;
    } else if (dx != 0) {
        if (conn & CONN_N) return HD_NORTH;
        if (conn & CONN_S) return HD_SOUTH;
    }

    return (Heading)0xFF;
}

// === EEPROM 기반 NavLog ===
// 데드엔드/우회 디버깅용 — 매 Eval, DeadEnd 진입, DynBlock 추가를 EEPROM 에 기록.
// 부팅 시 init() 가 dump → clear 호출 → 시리얼에 직전 트립 흐름이 자동 출력.
//
// EEPROM 영역 (Settings.h 에서 NAVLOG_ENTRIES 만 튜닝):
//   [0, ENTRIES*8 - 1]   엔트리 데이터 (순환 버퍼)
//   [ENTRIES*8]          head (다음에 쓸 슬롯)
//   [ENTRIES*8 + 1]      count (현재 저장된 엔트리 수)
//   [240, 255]           기존 캘리브 — 변경 없음
#define NAVLOG_BASE       0
#define NAVLOG_ENTRY_SZ   8
#define NAVLOG_HEAD_ADDR  (NAVLOG_BASE + NAVLOG_ENTRIES * NAVLOG_ENTRY_SZ)
#define NAVLOG_COUNT_ADDR (NAVLOG_HEAD_ADDR + 1)

#define NAVLOG_TAG_EVAL     0x01
#define NAVLOG_TAG_DEADEND  0x02
#define NAVLOG_TAG_DYNBLOCK 0x03

static void navlogPush(uint8_t tag,
                       uint8_t b1, uint8_t b2, uint8_t b3,
                       uint8_t b4, uint8_t b5, uint8_t b6, uint8_t b7) {
    uint8_t head  = EEPROM.read(NAVLOG_HEAD_ADDR);
    uint8_t count = EEPROM.read(NAVLOG_COUNT_ADDR);
    if (head  >= NAVLOG_ENTRIES) head  = 0;   // 초기화 안 됨 (0xFF) → 처음부터
    if (count >  NAVLOG_ENTRIES) count = 0;

    uint16_t addr = NAVLOG_BASE + (uint16_t)head * NAVLOG_ENTRY_SZ;
    EEPROM.update(addr,     tag);
    EEPROM.update(addr + 1, b1);
    EEPROM.update(addr + 2, b2);
    EEPROM.update(addr + 3, b3);
    EEPROM.update(addr + 4, b4);
    EEPROM.update(addr + 5, b5);
    EEPROM.update(addr + 6, b6);
    EEPROM.update(addr + 7, b7);

    head = (head + 1) % NAVLOG_ENTRIES;
    if (count < NAVLOG_ENTRIES) count++;
    EEPROM.update(NAVLOG_HEAD_ADDR, head);
    EEPROM.update(NAVLOG_COUNT_ADDR, count);
}

static void navlogClear() {
    EEPROM.update(NAVLOG_HEAD_ADDR, 0);
    EEPROM.update(NAVLOG_COUNT_ADDR, 0);
}

static void navlogDump() {
    if (!Serial) return;
    uint8_t head  = EEPROM.read(NAVLOG_HEAD_ADDR);
    uint8_t count = EEPROM.read(NAVLOG_COUNT_ADDR);
    if (head >= NAVLOG_ENTRIES || count > NAVLOG_ENTRIES || count == 0) {
        Serial.println(F("=== NavLog: empty ==="));
        return;
    }
    Serial.print(F("=== NavLog dump: "));
    Serial.print(count);
    Serial.println(F(" entries (oldest → newest) ==="));

    // 순환 버퍼 정렬: count<MAX 면 0..count-1, count==MAX 면 head..head-1
    uint8_t startIdx = (count < NAVLOG_ENTRIES) ? 0 : head;
    for (uint8_t i = 0; i < count; i++) {
        uint8_t slot  = (startIdx + i) % NAVLOG_ENTRIES;
        uint16_t addr = NAVLOG_BASE + (uint16_t)slot * NAVLOG_ENTRY_SZ;
        uint8_t tag = EEPROM.read(addr);
        int8_t  x   = (int8_t)EEPROM.read(addr + 1);
        int8_t  y   = (int8_t)EEPROM.read(addr + 2);
        uint8_t b3  = EEPROM.read(addr + 3);
        uint8_t b4  = EEPROM.read(addr + 4);
        uint8_t b5  = EEPROM.read(addr + 5);
        uint8_t b6  = EEPROM.read(addr + 6);
        uint8_t b7  = EEPROM.read(addr + 7);

        Serial.print(F("[")); Serial.print(i); Serial.print(F("] "));
        switch (tag) {
            case NAVLOG_TAG_EVAL:
                Serial.print(F("Eval ("));
                Serial.print(x); Serial.print(F(","));
                Serial.print(y); Serial.print(F(") hd="));
                Serial.print(b3);
                Serial.print(F(" conn0=0b")); Serial.print(b4, BIN);
                Serial.print(F(" afterBlk=0b")); Serial.print(b5, BIN);
                Serial.print(F(" fwd=0b")); Serial.print(b6, BIN);
                Serial.print(F(" pathLen=")); Serial.println(b7);
                break;
            case NAVLOG_TAG_DEADEND:
                Serial.print(F("DeadEnd ("));
                Serial.print(x); Serial.print(F(","));
                Serial.print(y); Serial.print(F(") hd="));
                Serial.print(b3);
                Serial.print(F(" pathLen=")); Serial.println(b7);
                break;
            case NAVLOG_TAG_DYNBLOCK:
                Serial.print(F("DynBlock ("));
                Serial.print(x); Serial.print(F(","));
                Serial.print(y); Serial.print(F(") count="));
                Serial.println(b7);
                break;
            default:
                Serial.print(F("Unknown tag=0x"));
                Serial.println(tag, HEX);
                break;
        }
    }
    Serial.println(F("=== NavLog end ==="));
}

// === 런타임 동적 진입금지 셀 ===
// IR 장애물 감지 시 그 칸을 여기에 추가 → 이후 모든 navigateTo 가 사전 우회(재돌진 방지).
// 정적 BLOCKED_CELLS 와 달리 런타임에 채워지며 전원 OFF 전까지 유지(리부팅 시 초기화).
// 최대 저장 개수는 Settings.h 의 MAX_DYN_BLOCKED.
static BlockedCell g_dynBlocked[MAX_DYN_BLOCKED];
static uint8_t g_dynBlockedCount = 0;

static void addDynBlockedCell(int8_t x, int8_t y) {
    if (x < 0 || x >= GRID_COLS || y < 0 || y >= GRID_ROWS) return;  // 그리드 밖 무시
    for (uint8_t i = 0; i < g_dynBlockedCount; i++) {
        if (g_dynBlocked[i].x == x && g_dynBlocked[i].y == y) return;  // 이미 등록됨
    }
    if (g_dynBlockedCount >= MAX_DYN_BLOCKED) {
        if (Serial) Serial.println(F("Dyn-block list full — cell not stored."));
        return;  // 가득 차면 임시 마스킹(_blockedAt*)이 즉시 우회를 담당
    }
    g_dynBlocked[g_dynBlockedCount].x = x;
    g_dynBlocked[g_dynBlockedCount].y = y;
    g_dynBlockedCount++;
    if (Serial) {
        Serial.print(F("Dyn-blocked cell ("));
        Serial.print(x); Serial.print(F(",")); Serial.print(y);
        Serial.print(F(") count=")); Serial.println(g_dynBlockedCount);
    }
    navlogPush(NAVLOG_TAG_DYNBLOCK,
               (uint8_t)x, (uint8_t)y, 0, 0, 0, 0, g_dynBlockedCount);
}

static bool isBlockedCell(int8_t x, int8_t y) {
    for (uint8_t i = 0; i < BLOCKED_CELL_COUNT; i++) {
        if (BLOCKED_CELLS[i].x == x && BLOCKED_CELLS[i].y == y) return true;
    }
    for (uint8_t i = 0; i < g_dynBlockedCount; i++) {
        if (g_dynBlocked[i].x == x && g_dynBlocked[i].y == y) return true;
    }
    return false;
}

// 진입 금지 교차점에 해당하는 방향 비트를 conn 에서 제거.
static uint8_t maskBlockedNeighbors(int8_t x, int8_t y, uint8_t conn) {
    if ((conn & CONN_N) && isBlockedCell(x,     y - 1)) conn &= ~CONN_N;
    if ((conn & CONN_S) && isBlockedCell(x,     y + 1)) conn &= ~CONN_S;
    if ((conn & CONN_E) && isBlockedCell(x + 1, y    )) conn &= ~CONN_E;
    if ((conn & CONN_W) && isBlockedCell(x - 1, y    )) conn &= ~CONN_W;
    return conn;
}

// === 사이클 방지: 방문 카운터 ===
// 한 navigateTo 안에서 각 칸을 몇 번 거쳤는지 추적. VISIT_LIMIT 도달 시 그 칸 진입 차단.
// maskCellsOnPath 를 직속 부모만 마스킹으로 완화한 뒤 사이클이 생길 때 끊는 안전망.
// navigateTo 시작 시 reset, 매 pose 변경 시 increment.
// VISIT_LIMIT 은 Settings.h 에서 튜닝.
static uint8_t g_visit[GRID_COLS * GRID_ROWS];

static void resetVisit() {
    for (uint8_t i = 0; i < (uint8_t)(GRID_COLS * GRID_ROWS); i++) g_visit[i] = 0;
}

static void incrementVisit(int8_t x, int8_t y) {
    if (x < 0 || x >= GRID_COLS || y < 0 || y >= GRID_ROWS) return;
    uint8_t idx = (uint8_t)y * (uint8_t)GRID_COLS + (uint8_t)x;
    if (g_visit[idx] < 255) g_visit[idx]++;
}

static uint8_t maskOverVisited(int8_t x, int8_t y, uint8_t conn) {
    if ((conn & CONN_N) && (y - 1) >= 0          && g_visit[(uint8_t)(y - 1) * GRID_COLS + (uint8_t)x      ] >= VISIT_LIMIT) conn &= ~CONN_N;
    if ((conn & CONN_S) && (y + 1) < GRID_ROWS   && g_visit[(uint8_t)(y + 1) * GRID_COLS + (uint8_t)x      ] >= VISIT_LIMIT) conn &= ~CONN_S;
    if ((conn & CONN_E) && (x + 1) < GRID_COLS   && g_visit[(uint8_t)y       * GRID_COLS + (uint8_t)(x + 1)] >= VISIT_LIMIT) conn &= ~CONN_E;
    if ((conn & CONN_W) && (x - 1) >= 0          && g_visit[(uint8_t)y       * GRID_COLS + (uint8_t)(x - 1)] >= VISIT_LIMIT) conn &= ~CONN_W;
    return conn;
}

static bool lookupCityCoord(const String& uid, int8_t* outX, int8_t* outY) {
    for (uint8_t i = 0; i < CITY_COORD_COUNT; i++) {
        if (CITY_COORDS[i].uid[0] == '\0') continue;  // 미등록 UID skip
        if (uid.compareTo(CITY_COORDS[i].uid) == 0) {
            *outX = CITY_COORDS[i].x;
            *outY = CITY_COORDS[i].y;
            return true;
        }
    }
    return false;
}

void Controller::init() {
    pinMode(RightWheelDir, OUTPUT);
    pinMode(LeftWheelDir, OUTPUT);
    pinMode(RightWheelPWM, OUTPUT);
    pinMode(LeftWheelPWM, OUTPUT);
    pinMode(SensorBottomRight, INPUT);
    pinMode(SensorBottomLeft, INPUT);

    nLineCounter = 0;
    targetLineCount = 0;

    readData();
    Serial.print("rW="); Serial.print(_rightWhite);
    Serial.print(" lW="); Serial.print(_leftWhite);
    Serial.print(" rB="); Serial.print(_rightBlack);
    Serial.print(" lB="); Serial.println(_leftBlack);
    Serial.print(F("ROBOT_ID=")); Serial.print(ROBOT_ID);
    Serial.print(F(" GRID=")); Serial.print(GRID_COLS); Serial.print(F("x")); Serial.print(GRID_ROWS);
    Serial.print(F(" WH=(")); Serial.print(WAREHOUSE_X); Serial.print(F(",")); Serial.print(WAREHOUSE_Y); Serial.println(F(")"));

    // 직전 트립 NavLog 자동 출력 후 버퍼 초기화
    navlogDump();
    navlogClear();

    SPI.begin();
    mfrc522.PCD_Init();

    LifterUp();
    delay(300);
    LifterDown();

    state = STATE_RFIDREAD;
}

void Controller::RunOnce()
{
    if (state == STATE_NONE) {
    } else if (state == STATE_RFIDREAD) {
        ProcessRFIDRead();
    }
}

// 💡 왼쪽 센서 정규화 함수 (EEPROM 값을 기반으로 동적 정규화)
int Controller::normalizeLeft(int rawValue) {
    long diff = _leftBlack - _leftWhite;
    if (diff == 0) diff = 1; // 0 나누기 방지
    long normValue = (long)(rawValue - _leftWhite) * 1000 / diff;

    if (normValue < 0) normValue = 0;
    if (normValue > 1000) normValue = 1000;
    return (int)normValue;
}

// 💡 오른쪽 센서 정규화 함수 (EEPROM 값을 기반으로 동적 정규화)
int Controller::normalizeRight(int rawValue) {
    long diff = _rightBlack - _rightWhite;
    if (diff == 0) diff = 1; // 0 나누기 방지
    long normValue = (long)(rawValue - _rightWhite) * 1000 / diff;

    if (normValue < 0) normValue = 0;
    if (normValue > 1000) normValue = 1000;
    return (int)normValue;
}

// 양 바닥 센서가 검은선(교차로) 위인지 판정.
// 1순위: 흑/백 캘리브 정규화(흰0~검1000) > LINEDETECT_NORM_MIN → 로봇/바닥 독립.
// 폴백: 흑-백 격차가 너무 작으면(캘리브 무효/EEPROM 초기화) raw 임계값으로 판정.
bool Controller::onLine(int rawLeft, int rawRight) {
    long spanL = (long)_leftBlack  - _leftWhite;
    long spanR = (long)_rightBlack - _rightWhite;
    if (spanL > LINEDETECT_CALIB_MIN_SPAN && spanR > LINEDETECT_CALIB_MIN_SPAN) {
        return normalizeLeft(rawLeft)  > LINEDETECT_NORM_MIN
            && normalizeRight(rawRight) > LINEDETECT_NORM_MIN;
    }
    // 캘리브 무효 → raw 폴백 (로봇 종속, 비상용)
    return rawLeft  > LINEDETECT_RAW_FALLBACK
        && rawRight > LINEDETECT_RAW_FALLBACK;
}

bool Controller::CheckObstacle() {
    int center = analogRead(SensorFrontCenter);
    int left   = analogRead(SensorFrontLeft);
    int right  = analogRead(SensorFrontRight);
    Serial.print("sensor front C/L/R : ");
    Serial.print(center); Serial.print(" / ");
    Serial.print(left);   Serial.print(" / ");
    Serial.println(right);

    bool centerTrip = (center < OBSTACLE_THRESHOLD);
    bool leftTrip   = (left   < OBSTACLE_THRESHOLD_SIDE);
    bool rightTrip  = (right  < OBSTACLE_THRESHOLD_SIDE);

    if (centerTrip || leftTrip || rightTrip) {
        delay(2);
        bool centerTrip2 = (analogRead(SensorFrontCenter) < OBSTACLE_THRESHOLD);
        bool leftTrip2   = (analogRead(SensorFrontLeft)   < OBSTACLE_THRESHOLD_SIDE);
        bool rightTrip2  = (analogRead(SensorFrontRight)  < OBSTACLE_THRESHOLD_SIDE);
        if (centerTrip2 || leftTrip2 || rightTrip2) {
            return true;
        }
    }
    return false;
}

void Controller::ReverseToPreviousNode() {
    if (Serial) Serial.println("Obstacle! Reversing...");
    drive(BACKWARD, Power, BACKWARD, Power);\
    while (true) {
        if (onLine(GetLeft(), GetRight())) {
            break;
        }
    }
    Stop();
    delay(400);

    drive(FORWARD, Power - 40, FORWARD, Power - 40);
    while (true) {
        if (onLine(GetLeft(), GetRight())) break;
    }
    delay((unsigned long)(120 / SPEED_SCALE));  // 라인 올라탄 뒤 정렬 크리프(거리 보존). LineTracer 정렬과 동일 형식.

    Stop();
    delay(200); // 차체 안정화
}

bool Controller::DoLineTrace(uint16_t targetCount, bool precise)
{
    _preciseRealign = precise;  // LineTracer 가 도달 시점에 읽음
    _lastCrossingTime = millis();   // 사전 감속 타이머 시작 — 직전 교차로 시점 기준

    // 출발 라인 위에서 시작하면 그 라인은 카운트하지 않도록 래치 프라이밍.
    // (cold-start 시 봇을 시작 RFID/교차로 위에 올려놓아도 첫 교차로를 오인하지 않음.
    //  운행 중에도 정밀 정렬이 라인 위에서 끝나므로 동일하게 재카운트 방지.)
    _bSignalHigh = onLine(GetLeft(), GetRight()) ? 1 : 0;
    while (!LineTracer(targetCount)) {
        if (enableObstacleAvoidance && CheckObstacle()) {
            Stop();
            delay(500);
            if (Serial) Serial.println("Obstacle! Backing to prev node.");
            ReverseToPreviousNode();
            ResetLineCounter();
            return false;  // 네비게이터가 막힌 방향 기록 후 재계산
        }
    }
    return true;
}

void Controller::ProcessRFIDRead()
{
    if (RFIDRead()) {
        isBusy = true;
        mfrc522.PCD_AntennaOff();
        switch (currentPosition) {
        case eInitialPosition: {
            // 출발 조건: 플래그 OFF 면 아무 RFID 나, ON 이면 시작 카드 UID 일치 시에만.
            bool startOk = (REQUIRE_START_RFID_MATCH == 0) ||
                           (strRFID.compareTo(s_strRFIDUidForStart) == 0);
            if (startOk) {
                // 시작 RFID 위치에서 좌표계 시작 → 네비게이터로 창고까지 자동 이동.
                // 장애물 우회도 동일 메커니즘으로 처리됨.
                currentPose = {INIT_START_X, INIT_START_Y, INIT_START_HEADING};
                PlayMelody();               // 출발 신호 — "도-파-라"
                navigateTo(WAREHOUSE_X, WAREHOUSE_Y);
                _navMinX = NAV_MIN_X;       // 횡단 완료 → 맵 격리 활성화 (로봇2: cols ≥4 고정)
                rotateToHeading(HD_SOUTH);  // 창고에서 도시 방향(+y, row 7 쪽) 으로 정렬
                LifterUp();
                currentPosition = eWareHousePosition;
            }
            mfrc522.PCD_AntennaOn();
            break;
        }

        case eWareHousePosition: {
            // 좌표 기반 디스패치 — UID 로 목적지 좌표 찾고 navigateTo 로 왕복.
            int8_t tx, ty;
            if (!lookupCityCoord(strRFID, &tx, &ty)) {
                if (Serial) {
                    Serial.print(F("Unknown city UID: ["));
                    Serial.print(strRFID); Serial.println(F("]"));
                }
                mfrc522.PCD_AntennaOn();
                break;
            }

            // ── 1) 도시 가기 ──
            delay(500);
            bool reached = navigateTo(tx, ty);

            if (reached) {
                // 정상 도착 — 화물 내리고 창고 방향(-y, row 0 쪽 = HD_NORTH) 으로 정렬.
                LifterDown();
                Stop();
                delay(700);
                rotateToHeading(HD_NORTH);
                delay(1000);
            } else {
                // 도시 도달 실패 (장애물로 우회 경로 없음) — 화물 들고 그대로 창고 복귀
                if (Serial) Serial.println(F("Forward nav failed — returning to warehouse with cargo."));
                Stop();
                delay(500);
            }

            // ── 3) 물류창고로 복귀 (성공/실패 무관) ──
            navigateTo(WAREHOUSE_X, WAREHOUSE_Y);

            // 창고 도착 — 항상 도시 방향(N) 으로 정렬 (다음 RFID 태깅 대기 자세)
            Stop();
            rotateToHeading(HD_SOUTH);
            if (reached) {
                // 정상 흐름: 빈 손으로 복귀했으니 다음 화물 픽업
                LifterUp();
            }
            // 실패 흐름: 화물이 위에 있는 상태 그대로 (LifterUp 생략)

            mfrc522.PCD_AntennaOn();
            break;
        }
        }
        isBusy = false;
    }
}

// 목표각까지 SERVO_STEP_DEG 씩 단계적으로 써서 부드럽게 슬루.
// _servoAngle 로 현재 위치를 추적 → 어느 방향이든 한 스텝씩 이동.
void  Controller::LifterMove(int targetAngle)
{
    servo.attach(LiftServo);
    delay(10);

    int a = _servoAngle;
    int step = (targetAngle >= a) ? SERVO_STEP_DEG : -SERVO_STEP_DEG;
    while (a != targetAngle) {
        a += step;
        // 목표 넘어가면 클램프
        if ((step > 0 && a > targetAngle) || (step < 0 && a < targetAngle)) a = targetAngle;
        servo.write(a);
        delay(SERVO_STEP_MS);
    }
    _servoAngle = targetAngle;

    delay(SERVO_SETTLE_MS);   // 안착 대기 후 detach
    servo.detach();
    delay(10);
}

void  Controller::LifterUp()
{
    LifterMove(SERVO_UP);
    _hasPayload = true;    // 팔레트 적재 — 이후 cargo(느린/부드러운) 거동
}

void  Controller::LifterDown()
{
    LifterMove(SERVO_DOWN);
    _hasPayload = false;   // 팔레트 내림 — 이후 일반(빠른) 거동
}

bool Controller::RFIDRead()
{
    if (isBusy) return false;
    if (millis() - lastRFIDTime < 5000) return false;

    if (mfrc522.PICC_IsNewCardPresent()) {
        if (mfrc522.PICC_ReadCardSerial()) {
            strRFID = "";
            for (byte i = 0; i < mfrc522.uid.size; i++) {
                strRFID.concat(String(mfrc522.uid.uidByte[i] < 0x10 ? "0" : ""));
                strRFID.concat(String(mfrc522.uid.uidByte[i], HEX));
            }
            strRFID.toUpperCase();
            lastRFIDTime = millis();

            Serial.print("\nRFID 인식됨: [");
            Serial.print(strRFID);
            Serial.println("]");
            return true;
        }
    }
    return false;
}

// 💡 교체할 첫 번째 함수: 정지할 때 '역회전 브레이크' 적용
// 💡 교체할 첫 번째 함수: 도착 후 후진하여 라인에 완벽 정렬 (회원님 아이디어 적용!)
// 💡 교체할 함수: 뒤로 완전히 벗어난 뒤, 앞으로 오면서 선에 정렬하기!
bool Controller::LineTracer(uint16_t nTargetLineCounter)
{
    LineTrace();

    // 목표한 교차로 개수에 도달했을 때
    if (nTargetLineCounter == nLineCounter) {
        if (_preciseRealign) {
            // 정확한 turn 이 필요한 위치 (y=0 창고 / y=7 도시) — 후진/전진 dance
            if (Serial) Serial.println("LineCount Finished. Precise realign...");

            Stop();
            delay(150);

            // 거리 보존: PWM 은 drive() 에서 ×SPEED_SCALE 되므로, 같은 후진 거리를 유지하려면
            // 시간을 /SPEED_SCALE. 240ms 는 SPEED_SCALE=1.0 기준 튜닝값(= 옛 400×0.6).
            drive(BACKWARD, Power, BACKWARD, Power);
            delay((unsigned long)(240 / SPEED_SCALE));

            Stop();
            delay(100); // 기어 방향 전환 전 잠깐 대기

            drive(FORWARD, Power - 40, FORWARD, Power - 40);
            while (true) {
                if (onLine(GetLeft(), GetRight())) break;
            }
            delay((unsigned long)(120 / SPEED_SCALE));  // 라인 올라탄 뒤 정렬 크리프(거리 보존). 120ms = SPEED_SCALE=1.0 기준.

            Stop();
            delay(200); // 차체 안정화
        } else {
            // 중간 교차로 통과 — 정밀 정렬 불필요, 그냥 잠시 정지만
            Stop();
            delay(50);
        }

        ResetLineCounter();
        return true;
    }
    return false;
}

// 💡 교체할 두 번째 함수: P-제어 유지 및 선을 확실히 넘어가도록 세팅
void Controller::LineTrace() {
#if DEBUG_TRACE
    static unsigned long lastSensorLog = 0;
#endif

    int leftRaw = GetLeft();
    int rightRaw = GetRight();

    // 디버그: 200ms마다 좌/우 raw 값과 정규화 값 출력.
    // DEBUG_TRACE=0 이면 컴파일에서 제외 → Serial 블로킹 장님 구간 제거(교차로 인식 안정).
#if DEBUG_TRACE
    if (millis() - lastSensorLog > 200) {
        Serial.print("L_raw=");  Serial.print(leftRaw);
        Serial.print(" R_raw="); Serial.print(rightRaw);
        Serial.print(" | L_n="); Serial.print(normalizeLeft(leftRaw));
        Serial.print(" R_n=");   Serial.println(normalizeRight(rightRaw));
        lastSensorLog = millis();
    }
#endif

    // 교차로(검은선 2개 동시) 판단 — 캘리브 정규화 기준
    if (onLine(leftRaw, rightRaw)) {
        if (_bSignalHigh == 0) {
            nLineCounter++;
            if (Serial) {
                Serial.println(String("LINE!!! :") + String(nLineCounter));
            }
            _bSignalHigh = 1;
            _prevError = 0;             // 교차로 진입 — D 항 spike 방지
            _lastCrossingTime = millis(); // 사전 감속 타이머 리셋 — 다음 교차로 기준
        }
        tone(pinBuzzer, 1047);   // 도
        Forward(CrossingPassPower);   // 교차로 통과 시 감속 — overshoot 방지
        delay((unsigned long)(100 / SPEED_SCALE)); // 🌟 선을 완전히 넘어가도록 약간의 전진(거리 보존). 100ms = SPEED_SCALE=1.0 기준.
        noTone(pinBuzzer);
    }
    else {
        if (_bSignalHigh) {
            _bSignalHigh = 0;
            _prevError = 0;   // 교차로 이탈 — D 항 spike 방지
        }

        // 부드러운 PD-제어
        int leftNorm = normalizeLeft(leftRaw);
        int rightNorm = normalizeRight(rightRaw);

        int error = rightNorm - leftNorm;
        int dError = error - _prevError;
        float correction = Kp * error + Kd * dError;
        _prevError = error;

        if (correction > maxCorrection) correction = maxCorrection;
        if (correction < -maxCorrection) correction = -maxCorrection;

        // 사전 감속: 직전 교차로 후 일정 시간 지나면 base PWM 을 낮춤.
        // 화물 적재(eWareHousePosition) 는 느리므로 임계값/감속 PWM 따로 (Settings.h).
        bool cargo = _hasPayload;
        unsigned long approachThreshold = cargo ? CrossingApproachMsCargo : CrossingApproachMs;
        bool inApproach = (millis() - _lastCrossingTime) > approachThreshold;
#if DEBUG_APPROACH_TONE
        // 사전 감속 시작(전속→감속 전환) 순간에 한 번만 부저음. tone 의 duration 인자로
        // 비블로킹 재생(자동 정지) → 제어 루프 안 막음. 교차로마다 _lastCrossingTime 이
        // 리셋되며 inApproach 가 false 로 돌아가 자동 재무장.
        if (inApproach && !_inApproachPrev) {
            tone(pinBuzzer, DEBUG_APPROACH_TONE_HZ, 60);
        }
        _inApproachPrev = inApproach;
#endif
        int basePower;
        if (inApproach) {
            basePower = cargo ? CROSSING_APPROACH_POWER_CARGO : CrossingApproachPower;
        } else {
            basePower = cargo ? MOTOR_POWER_CARGO : Power;
        }

        float leftPower = basePower + correction;
        float rightPower = basePower - correction;

        drive(FORWARD, (int)leftPower, FORWARD, (int)rightPower);
    }
}


void Controller::ResetLineCounter()
{
    nLineCounter = 0;
}

void  Controller::drive(int dir1, int power1, int dir2, int power2)
{
    boolean dirHighLow1, dirHighLow2;

    if (dir1 == FORWARD)
        dirHighLow1 = HIGH;
    else
        dirHighLow1 = LOW;

    if (dir2 == FORWARD)
        dirHighLow2 = LOW;
    else
        dirHighLow2 = HIGH;

    // 💡 EEPROM에 저장된 각 모터의 편차(CalibL, CalibR)를 여기서 자동 적용함
    // SPEED_SCALE은 모든 drive() 경유 호출에 일괄 적용 (전역 속도 조절)
    digitalWrite(LeftWheelDir, dirHighLow1);
    analogWrite(LeftWheelPWM, power1 * _motorCalibL * SPEED_SCALE);

    digitalWrite(RightWheelDir, dirHighLow2);
    analogWrite(RightWheelPWM, power2 * _motorCalibR * SPEED_SCALE);
}

void  Controller::Forward(int power)
{
    drive(FORWARD, power, FORWARD, power);
}

void Controller::Stop()
{
    analogWrite(LeftWheelPWM, 0);
    analogWrite(RightWheelPWM, 0);
}

// 사다리꼴 속도 프로파일 회전 프리미티브.
//   dirL/dirR   : 각 바퀴 방향 (FORWARD/BACKWARD) — drive() 가 방향 반전/캘리브/스케일 처리.
//   cruiseL/R   : 정속 구간 목표 PWM (제자리 회전이면 동일, 피벗이면 강/약 비대칭).
//   holdMs      : 정속 유지 시간 — 회전각의 주 결정 요소.
// 가속(startPwm→cruise) → 정속 → 감속(cruise→startPwm) → 정지.
// 화물 적재(eWareHousePosition) 시 가감속 파라미터를 _CARGO 변형으로 전환 →
// 더 잘게·완만하게 돌아 팔레트 슬라이드/관성 흔들림을 줄인다.
void Controller::RampTurn(int dirL, int dirR, int cruiseL, int cruiseR, unsigned long holdMs)
{
    bool cargo  = _hasPayload;
    int  steps  = cargo ? TURN_RAMP_STEPS_CARGO   : TURN_RAMP_STEPS;
    int  stepMs = cargo ? TURN_RAMP_STEP_MS_CARGO : TURN_RAMP_STEP_MS;
    int  startP = cargo ? TURN_START_PWM_CARGO    : TURN_START_PWM;

    // SPEED_SCALE 보정: drive() PWM 은 ×SPEED_SCALE 이므로, 회전각(= PWM×시간)을 유지하려면
    // 시간(step·hold)을 /SPEED_SCALE 해야 한다 → 느려져도 같은 90°/180° 를 돈다.
    // (TURN_SETTLE_MS 는 진동 감쇠 대기라 속도와 무관 → 스케일 안 함.)
    unsigned long stepDelay = (unsigned long)(stepMs / SPEED_SCALE);
    unsigned long holdDelay = (unsigned long)(holdMs / SPEED_SCALE);

    // [1] 가속: startP 에서 cruise 까지 선형 증가.
    for (int i = 1; i <= steps; i++) {
        float f = (float)i / steps;
        int pl = startP + (int)((cruiseL - startP) * f);
        int pr = startP + (int)((cruiseR - startP) * f);
        drive(dirL, pl, dirR, pr);
        delay(stepDelay);
    }
    // [2] 정속 — 회전각은 이 구간 시간으로 결정.
    drive(dirL, cruiseL, dirR, cruiseR);
    delay(holdDelay);
    // [3] 감속: cruise 에서 startP 까지 선형 감소 → overshoot 완화.
    for (int i = steps - 1; i >= 1; i--) {
        float f = (float)i / steps;
        int pl = startP + (int)((cruiseL - startP) * f);
        int pr = startP + (int)((cruiseR - startP) * f);
        drive(dirL, pl, dirR, pr);
        delay(stepDelay);
    }
    Stop();
    delay(TURN_SETTLE_MS);   // 차체 안정화 — 다음 전진/정렬 전에 진동 가라앉힘
#if DEBUG_TURN_PAUSE_MS > 0
    delay(DEBUG_TURN_PAUSE_MS);   // [디버그] 회전각 측정용 추가 정지
#endif
}

void Controller::TurnHalf() {
    bool cargo = _hasPayload;
    // 화물 적재 시 각속도 ↓ (원심력으로 팔레트 슬라이드 방지). 각도 유지 위해 delay 살짝 ↑.
    int turnPwm             = cargo ? TURNHALF_PWM_CARGO      : TURNHALF_PWM;
    unsigned long turnDelay = cargo ? TURNHALF_DELAY_MS_CARGO : TURNHALF_DELAY_MS;
    // 제자리 좌회전(왼쪽 후진/오른쪽 전진) 180°.
    RampTurn(BACKWARD, FORWARD, turnPwm, turnPwm, turnDelay);
}

void Controller::PivotTurnLeft()
{
    if (Serial) Serial.println("Enter Pivot turn Left");
    bool cargo = _hasPayload;
    // 왼쪽 약, 오른쪽 강 → 좌회전 (왼쪽 후진/오른쪽 전진). 회전각은 holdMs 로 결정.
    int strongPwm           = cargo ? PIVOT_LEFT_STRONG_PWM_CARGO : PIVOT_LEFT_STRONG_PWM;
    int weakPwm             = cargo ? PIVOT_LEFT_WEAK_PWM_CARGO   : PIVOT_LEFT_WEAK_PWM;
    unsigned long turnDelay = cargo ? PIVOT_LEFT_DELAY_MS_CARGO   : PIVOT_LEFT_DELAY_MS;
    RampTurn(BACKWARD, FORWARD, weakPwm, strongPwm, turnDelay);
    if (Serial) Serial.println("Leave Pivot turn Left");
}

void Controller::PivotTurnRight()
{
    if (Serial) Serial.println("Enter Pivot turn Right");
    bool cargo = _hasPayload;
    // 왼쪽 강, 오른쪽 약 → 우회전 (왼쪽 전진/오른쪽 후진). 회전각은 holdMs 로 결정.
    int strongPwm           = cargo ? PIVOT_RIGHT_STRONG_PWM_CARGO : PIVOT_RIGHT_STRONG_PWM;
    int weakPwm             = cargo ? PIVOT_RIGHT_WEAK_PWM_CARGO   : PIVOT_RIGHT_WEAK_PWM;
    unsigned long turnDelay = cargo ? PIVOT_RIGHT_DELAY_MS_CARGO   : PIVOT_RIGHT_DELAY_MS;
    RampTurn(FORWARD, BACKWARD, strongPwm, weakPwm, turnDelay);
    if (Serial) Serial.println("Leave Pivot turn Right");
}

int16_t Controller::GetLeft()
{
    return analogRead(SensorBottomLeft);
}

int16_t Controller::GetRight()
{
    return analogRead(SensorBottomRight);
}

// EEPROM 캘리브레이션 이력 (2026-05-24)
// 정규화 함수 도입 이후 R_n이 항상 0으로 클램프되어 P 제어가 한쪽으로만 꺾이는 증상.
// 원인: 흑/백 값이 뒤집힌 상태(_black < _white)로 EEPROM에 옛값이 남아 있었음.
//
//                 | leftWhite | leftBlack | rightWhite | rightBlack | motorCalibL | motorCalibR
//   Before (옛)   |    397    |    256    |    402     |    268     |    0.980    |    1.000
//   After  (정상) |    320    |    912    |    423     |    928     |    0.980    |    1.000
//
// After 기준 흰 바닥 raw ≈ L 322 / R 424 로 white 캘리브 값과 일치.
// 검은선에서는 raw가 lB/rB(~900대)에 도달하여 정규화 0~1000 풀 스윙 확보.
// 모터 calib은 원래 정상이라 미변경.
void Controller::readData() {
    int address = START_ADDRESS;

    EEPROM.get(address, _rightWhite);
    address += 2;
    EEPROM.get(address, _leftWhite);
    address += 2;
    EEPROM.get(address, _rightBlack);
    address += 2;
    EEPROM.get(address, _leftBlack);
    address += 2;
    EEPROM.get(address, _motorCalibR);
    address += 4;
    EEPROM.get(address, _motorCalibL);
    address += 4;
}

// 출발 멜로디 — "도-파-라" (C5·F5·A5) 상승 아르페지오.
// 미션에 따라 아래 음계 표를 참고해 tone() 주파수를 바꿔 쓰면 됨 (C5 옥타브 기준):
//   도 C5 = 523   레 D5 = 587   미 E5 = 659   파 F5 = 698
//   솔 G5 = 784   라 A5 = 880   시 B5 = 988   도 C6 = 1047
// (한 옥타브 ↑ 는 ×2, ↓ 는 ÷2. 예: 도 C4 = 262)
void Controller::PlayMelody() {
    tone(pinBuzzer, 523);   // 도 C5
    delay(500);
    tone(pinBuzzer, 698);   // 파 F5
    delay(500);
    tone(pinBuzzer, 880);   // 라 A5
    delay(500);
    noTone(pinBuzzer);
}

// === 좌표 네비게이션 ===
// 매 교차로마다 (dx, dy) 와 현재 교차로의 연결성을 보고 다음 진행 방향을 결정.
// PivotTurn + DoLineTrace(1) 단위로 진행하므로 장애물 우회는 DoLineTrace 내부 로직이
// 자연히 처리하고, 좌표는 매 step 마다 갱신되어 우회 후에도 일관됨.

void Controller::rotateToHeading(Heading target) {
    int8_t delta = ((int8_t)target - (int8_t)currentPose.heading + 4) % 4;
    switch (delta) {
        case 0: /* 직진 그대로 */          break;
        case 1: PivotTurnRight();           break;
        case 2: TurnHalf();                 break;
        case 3: PivotTurnLeft();            break;
    }
    currentPose.heading = target;
}

// 직속 부모(스택 직전 칸) 방향만 conn 에서 제거.
// 즉시 되돌아가는 것만 막고, 더 깊은 경로 칸 마스킹은 maskOverVisited 가 담당.
// 우회 시 같은 칸을 한 번 더 지나갈 수 있어 짧은 우회 경로가 자연스럽게 선택됨.
// (VISIT_LIMIT 회 초과 방문 시 그 칸은 자동 차단되어 무한 루프 방지)
uint8_t Controller::maskCellsOnPath(int8_t x, int8_t y, uint8_t conn) {
    if (_pathLen < 2) return conn;
    int8_t ddx = _pathX[_pathLen - 2] - x;
    int8_t ddy = _pathY[_pathLen - 2] - y;
    if      (ddy == -1 && ddx == 0) conn &= ~CONN_N;
    else if (ddy == 1  && ddx == 0) conn &= ~CONN_S;
    else if (ddx == 1  && ddy == 0) conn &= ~CONN_E;
    else if (ddx == -1 && ddy == 0) conn &= ~CONN_W;
    return conn;
}

// 좌표 네비게이션 (DFS + 백트래킹).
//
// 각 교차로에서 부모/스택 칸/차단 칸을 뺀 남은 방향(fwdConn) 중,
//   - 목표 방향(desiredHeading) 우선, 없으면 남은 아무 방향으로 탐색하여 push.
//   - fwdConn == 0 이면 막다른 길 → 그 칸을 영구 차단 등록 후 부모로 물리 후진(pop).
//   - 스택이 시작칸까지 비면 STUCK → false.
bool Controller::navigateTo(int8_t tx, int8_t ty) {
    if (Serial) {
        Serial.print(F("Nav: ("));
        Serial.print(currentPose.x); Serial.print(F(","));
        Serial.print(currentPose.y); Serial.print(F(") -> ("));
        Serial.print(tx); Serial.print(F(",")); Serial.print(ty); Serial.println(F(")"));
    }

    // 임시 차단 정보만 클리어 (동적 차단 리스트 g_dynBlocked 는 유지 — 옵션 2)
    _blockedAtX = -128;
    _blockedAtY = -128;
    _blockedDirBit = 0;

    // 경로 스택 초기화 — 시작 칸 push
    _pathLen = 0;
    _pathX[_pathLen] = currentPose.x;
    _pathY[_pathLen] = currentPose.y;
    _pathLen++;

    // 사이클 방지용 방문 카운터 초기화 + 시작 칸 카운트
    resetVisit();
    incrementVisit(currentPose.x, currentPose.y);

    while (currentPose.x != tx || currentPose.y != ty) {
        int8_t dx = tx - currentPose.x;
        int8_t dy = ty - currentPose.y;

        // forward 후보: 격자연결 ∩ 차단셀 제외 ∩ VISIT_LIMIT 초과 칸 제외 ∩ 직속 부모 제외
        uint8_t conn0   = lookupConn(currentPose.x, currentPose.y);
        uint8_t conn    = maskBlockedNeighbors(currentPose.x, currentPose.y, conn0);
        conn            = maskOverVisited(currentPose.x, currentPose.y, conn);
        uint8_t fwdConn = maskCellsOnPath(currentPose.x, currentPose.y, conn);

        // 맵 격리: _navMinX 미만 열로 가는 서쪽 이동 차단.
        // (로봇2 격리용. 로봇1 은 _navMinX=-128 이라 절대 안 걸림.)
        if ((fwdConn & CONN_W) && (currentPose.x - 1 < _navMinX)) fwdConn &= ~CONN_W;

        // 직전 장애물 임시 마스킹 (동적 리스트 가득 찼을 때의 안전망)
        if (_blockedAtX == currentPose.x && _blockedAtY == currentPose.y) {
            fwdConn &= ~_blockedDirBit;
        }

        if (Serial) {
            Serial.print(F("Eval ("));
            Serial.print(currentPose.x); Serial.print(F(","));
            Serial.print(currentPose.y); Serial.print(F(") hd="));
            Serial.print((uint8_t)currentPose.heading);
            Serial.print(F(" conn0=0b")); Serial.print(conn0, BIN);
            Serial.print(F(" afterBlk=0b")); Serial.print(conn, BIN);
            Serial.print(F(" fwd=0b")); Serial.print(fwdConn, BIN);
            Serial.print(F(" pathLen=")); Serial.println(_pathLen);
        }
        navlogPush(NAVLOG_TAG_EVAL,
                   (uint8_t)currentPose.x, (uint8_t)currentPose.y,
                   (uint8_t)currentPose.heading,
                   conn0, conn, fwdConn, _pathLen);

        if (fwdConn == 0) {
            // === 막다른 길 — 부모 칸 방향으로 180° 회전 후 한 칸 전진해서 복귀 ===
            // 한 칸 이동 후 바깥 루프가 새 칸에서 fwdConn 재평가 (봇의 새 heading 기준 정면 판단).
            //   - 탈출구 있음 → desiredHeading 이 선택해서 진행
            //   - 여전히 데드엔드 → 다음 iteration 에서 같은 패턴 반복
            //   - 스택이 비거나 부모가 그리드 밖이면 STUCK
            if (Serial) {
                Serial.print(F("Dead-end at ("));
                Serial.print(currentPose.x); Serial.print(F(","));
                Serial.print(currentPose.y); Serial.println(F(") — turn + step backtracking"));
            }
            navlogPush(NAVLOG_TAG_DEADEND,
                       (uint8_t)currentPose.x, (uint8_t)currentPose.y,
                       (uint8_t)currentPose.heading,
                       0, 0, 0, _pathLen);

            if (_pathLen <= 1) {
                if (Serial) Serial.println(F("Nav STUCK: start cell is dead-end."));
                Stop();
                return false;
            }
            int8_t px = _pathX[_pathLen - 2];
            int8_t py = _pathY[_pathLen - 2];
            if (px < 0) {
                if (Serial) Serial.println(F("Nav STUCK: parent is off-grid start pad."));
                Stop();
                return false;
            }

            // 부모 방향 계산 후 회전 → 한 칸 전진
            int8_t bdx = px - currentPose.x;
            int8_t bdy = py - currentPose.y;
            Heading back;
            if      (bdy == -1) back = HD_NORTH;
            else if (bdy == 1)  back = HD_SOUTH;
            else if (bdx == 1)  back = HD_EAST;
            else                back = HD_WEST;
            rotateToHeading(back);
            // 정밀 정렬(reverse-then-forward dance) 조건:
            //   1) 최종 목표 좌표 도착, 또는
            //   2) 수직 이동(N/S) 으로 row 0 (창고행) / row 7 (도시행) 도착
            //      — 가로 라인 위 정렬 필요 (회전/RFID 정확도 확보)
            bool arriveTarget = (px == tx && py == ty);
            bool vertArrival  = (headingDy(back) != 0) && (py == 0 || py == 7);
            bool bprec = arriveTarget || vertArrival;
            if (!DoLineTrace(1, bprec)) {
                // 방금 지나온 길에 장애물 — 드문 케이스(움직이는 장애물). 일단 포기.
                if (Serial) Serial.println(F("Obstacle on backtrack — STUCK."));
                Stop();
                return false;
            }

            // 성공: 현재 칸을 dead-end 로 영구 차단 등록 + 스택 pop + pose 갱신
            addDynBlockedCell(currentPose.x, currentPose.y);
            _pathLen--;
            currentPose.x = px;
            currentPose.y = py;
            incrementVisit(currentPose.x, currentPose.y);
            _blockedAtX = -128;
            continue;
        }

        // === 전진 방향 결정 ===
        Heading desired = desiredHeading(dx, dy, fwdConn, currentPose.heading);
        if ((uint8_t)desired == 0xFF) {
            // desiredHeading 은 목표 반대축으론 안 가지만, 우회를 위해 남은 비트 중 아무거나 선택.
            if      (fwdConn & CONN_N) desired = HD_NORTH;
            else if (fwdConn & CONN_S) desired = HD_SOUTH;
            else if (fwdConn & CONN_E) desired = HD_EAST;
            else                       desired = HD_WEST;
        }

        rotateToHeading(desired);

        int8_t newX = currentPose.x + headingDx(desired);
        int8_t newY = currentPose.y + headingDy(desired);
        // 정밀 정렬 조건:
        //   1) 최종 목표 좌표 도착, 또는
        //   2) 수직 이동(N/S) 으로 row 0 (창고행) / row 7 (도시행) 도착
        //      — 가로 라인 위 정렬 필요 (회전/RFID 정확도 확보)
        bool arriveTarget = (newX == tx && newY == ty);
        bool vertArrival  = (headingDy(desired) != 0) && (newY == 0 || newY == 7);
        bool precise = arriveTarget || vertArrival;

        if (DoLineTrace(1, precise)) {
            // 성공 — pose 갱신, 새 칸 push, 방문 카운트 증가, 임시 마스킹 해제
            _blockedAtX = -128;
            currentPose.x = newX;
            currentPose.y = newY;
            incrementVisit(currentPose.x, currentPose.y);
            if (_pathLen < NAV_PATH_MAX) {
                _pathX[_pathLen] = currentPose.x;
                _pathY[_pathLen] = currentPose.y;
                _pathLen++;
            } else {
                if (Serial) Serial.println(F("Path stack full — push skipped."));
            }
        } else {
            // 장애물 — 앞 칸을 영구 차단, 임시 마스킹 기록. pose 유지, 다음 iteration 이 재계산.
            addDynBlockedCell(newX, newY);
            _blockedAtX = currentPose.x;
            _blockedAtY = currentPose.y;
            switch (desired) {
                case HD_NORTH: _blockedDirBit = CONN_N; break;
                case HD_EAST:  _blockedDirBit = CONN_E; break;
                case HD_SOUTH: _blockedDirBit = CONN_S; break;
                case HD_WEST:  _blockedDirBit = CONN_W; break;
                default: break;
            }
        }
    }
    return true;
}
