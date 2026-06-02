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

// === BFS 최단경로 스크래치 ===
// navigateTo 는 매 교차로에서 currentPose→목표 의 최단경로(BFS)를 구해 그 경로대로 주행한다.
// 장애물을 만나면 그 칸을 동적 차단(g_dynBlocked)에 넣고 현재 위치에서 다시 BFS → 즉시 우회.
// (구 greedy+DFS 백트래킹은 목표 직전 외길이 막히면 맵을 헤매는 한계가 있어 BFS 로 교체.)
//   g_bfsParent[idx] : 0=미방문, 0xFF=출발칸, 그 외=부모로 가는 CONN_* 방향(역추적용)
//   g_bfsQueue[]     : BFS 큐 (셀 인덱스 = y*GRID_COLS + x)
static uint8_t g_bfsParent[GRID_COLS * GRID_ROWS];
static uint8_t g_bfsQueue[GRID_COLS * GRID_ROWS];

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
    // 시리얼 명령 폴링. 주행 중엔 navigateTo 가 블로킹하므로 이 루프가 돌지 않고,
    // 창고에 도착해 다음 RFID 를 기다리는 유휴 상태에서만 입력이 처리된다 —
    // 즉 "창고 도착 후 상태 확인" 타이밍과 정확히 일치.
    HandleSerialCommand();

    if (state == STATE_NONE) {
    } else if (state == STATE_RFIDREAD) {
        ProcessRFIDRead();
    }
}

// 시리얼로 한 글자 명령을 받아 처리. 버퍼는 모두 비운다.
//   m / p : 장애물 맵 출력
//   그 외  : 사용법 안내
void Controller::HandleSerialCommand()
{
    if (!Serial || Serial.available() <= 0) return;

    char cmd = (char)Serial.read();
    while (Serial.available() > 0) Serial.read();  // 나머지(개행 등) 비우기

    switch (cmd) {
    case 'm': case 'M':
    case 'p': case 'P':
        PrintStatusMap();
        break;
    case '\r': case '\n': case ' ':
        break;  // 무시
    default:
        Serial.println(F("cmd: m=map/status"));
        break;
    }
}

// 현재 그리드를 ASCII 맵으로 출력.
//   @ = 현재 위치   # = 창고   ! = 동적 장애물(IR 감지)   x = 정적 장애물
//   C = 도시        . = 빈 교차로   : = 격리 경계 서쪽(진입 금지)
void Controller::PrintStatusMap()
{
    Serial.println();
    Serial.print(F("=== STATUS  pose=("));
    Serial.print(currentPose.x); Serial.print(F(","));
    Serial.print(currentPose.y); Serial.print(F(") hd="));
    const char* hd = (currentPose.heading == HD_NORTH) ? "N" :
                     (currentPose.heading == HD_EAST)  ? "E" :
                     (currentPose.heading == HD_SOUTH) ? "S" : "W";
    Serial.print(hd);
    Serial.print(F(" cargo=")); Serial.print(_hasPayload ? F("Y") : F("N"));
    Serial.print(F(" dynBlocked=")); Serial.println(g_dynBlockedCount);

    for (int8_t y = 0; y < GRID_ROWS; y++) {
        Serial.print(F("y")); Serial.print(y); Serial.print(F(" "));
        for (int8_t x = 0; x < GRID_COLS; x++) {
            char c;
            // 동적 장애물(런타임 IR 감지) 판정
            bool dyn = false;
            for (uint8_t i = 0; i < g_dynBlockedCount; i++) {
                if (g_dynBlocked[i].x == x && g_dynBlocked[i].y == y) { dyn = true; break; }
            }
            // 정적 장애물 판정
            bool stat = false;
            for (uint8_t i = 0; i < BLOCKED_CELL_COUNT; i++) {
                if (BLOCKED_CELLS[i].x == x && BLOCKED_CELLS[i].y == y) { stat = true; break; }
            }
            // 도시 판정
            bool city = false;
            for (uint8_t i = 0; i < CITY_COORD_COUNT; i++) {
                if (CITY_COORDS[i].x == x && CITY_COORDS[i].y == y) { city = true; break; }
            }

            if (currentPose.x == x && currentPose.y == y)        c = '@';
            else if (x == (int8_t)(WAREHOUSE_X) && y == (int8_t)(WAREHOUSE_Y)) c = '#';
            else if (dyn)                                        c = '!';
            else if (stat)                                       c = 'x';
            else if (city)                                       c = 'C';
            else if (_navMinX > -128 && x < _navMinX)            c = ':';
            else                                                 c = '.';
            Serial.print(c);
            Serial.print(' ');
        }
        Serial.println();
    }
    Serial.println(F("@=pos #=WH !=dyn x=static C=city :=off-limits"));
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
    // ⚠ 이 출력은 DoLineTrace 폴링 루프에서 매 PD 틱마다 실행된다.
    // 9600baud 블로킹으로 제어 루프를 ~28Hz 로 throttle → 고속에서 커브 이탈/교차점 누락.
    // 실주행(DEBUG_TRACE=0)에서는 컴파일에서 제외해 루프를 최대 속도로 돌린다.
#if DEBUG_TRACE
    Serial.print("sensor front C/L/R : ");
    Serial.print(center); Serial.print(" / ");
    Serial.print(left);   Serial.print(" / ");
    Serial.println(right);
#endif

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
    delay(REALIGN_CREEP_MS);  // 라인 올라탄 뒤 정렬 크리프. LineTracer 정렬과 동일 형식.

    Stop();
    delay(200); // 차체 안정화
}

bool Controller::DoLineTrace(uint16_t targetCount, bool precise)
{
    _preciseRealign = precise;  // LineTracer 가 도달 시점에 읽음
    _runTargetCount = targetCount;  // LineTrace 가 마지막 칸(=감속 구간) 판정에 사용
    _drivePwm    = DRIVE_START_PWM; // 런 시작 PWM 부터 가속 램프
    _lastDriveMs = millis();        // 슬루 dt 기준 시작

    // 출발 라인 위에서 시작하면 그 라인은 카운트하지 않도록 래치 프라이밍.
    // (cold-start 시 봇을 시작 RFID/교차로 위에 올려놓아도 첫 교차로를 오인하지 않음.
    //  운행 중에도 정밀 정렬이 라인 위에서 끝나므로 동일하게 재카운트 방지.)
    _bSignalHigh = onLine(GetLeft(), GetRight()) ? 1 : 0;
    while (!LineTracer(targetCount)) {
        if (enableObstacleAvoidance && CheckObstacle()) {
            Stop();
            delay(500);
            if (Serial) Serial.println("Obstacle! Backing to prev node.");
            _crossingsDone = nLineCounter;  // 장애물 전까지 통과한 교차로 수 (ReverseToPreviousNode 가 이 칸으로 복귀)
            ReverseToPreviousNode();
            ResetLineCounter();
            return false;  // 네비게이터가 막힌 방향 기록 후 재계산
        }
    }
    _crossingsDone = targetCount;  // 전부 통과
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
            tone(pinBuzzer, 1047);
            delay(200);
            noTone(pinBuzzer);
            delay(50);
            tone(pinBuzzer, 1047);
            delay(200);
            noTone(pinBuzzer);
            delay(1500); 
            
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
        if (_preciseRealign && PRECISE_REALIGN_ENABLE) {
            // 정확한 turn 이 필요한 위치 (y=0 창고 / y=7 도시) — 후진/전진 dance.
            // PRECISE_REALIGN_ENABLE=0 이면 이 dance 를 건너뛰고 아래 else(잠깐 정지)로 처리.
            if (Serial) Serial.println("LineCount Finished. Precise realign...");

            Stop();
            delay(150);

            // 후진으로 선을 완전히 클리어 — 거리 = Power(=MOTOR_POWER) × REALIGN_BACKUP_MS.
            // (240ms 는 옛 400×0.6 튜닝값. MOTOR_POWER 를 바꾸면 이 거리도 변하니 재튜닝.)
            drive(BACKWARD, Power, BACKWARD, Power);
            delay(REALIGN_BACKUP_MS);

            Stop();
            delay(100); // 기어 방향 전환 전 잠깐 대기

            drive(FORWARD, Power - 40, FORWARD, Power - 40);
            while (true) {
                if (onLine(GetLeft(), GetRight())) break;
            }
            delay(REALIGN_CREEP_MS);  // 라인 올라탄 뒤 정렬 크리프.

            Stop();
            delay(200); // 차체 안정화
        } else {
            // 중간 교차로 통과(또는 PRECISE_REALIGN_ENABLE=0) — 정밀 정렬 없이 잠시 정지만
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
#if DEBUG_TRACE
            // 교차로마다 1회. String 힙 할당 + 시리얼이라 실주행에선 컴파일 제외.
            if (Serial) {
                Serial.println(String("LINE!!! :") + String(nLineCounter));
            }
#endif
            _bSignalHigh = 1;
            _prevError = 0;             // 교차로 진입 — D 항 spike 방지
        }
        tone(pinBuzzer, 1047);   // 도
        Forward(CrossingPassPower);   // 교차로 통과 시 감속 — overshoot 방지
        delay(CROSSING_PASS_MS); // 🌟 선을 완전히 넘어가도록 약간의 전진.
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

        // 직진 모션 프로파일 — base PWM 을 목표로 가감속률 제한 슬루(사다리꼴/삼각형).
        //   cruise = 정속 상한(긴 직선에서 도달) / brake = 노드 직전 감속 목표.
        //   감속 시작 = 런 끝 DRIVE_BRAKE_CELLS 칸 전 교차점 통과 시점(=마지막 노드 직전).
        //   런은 같은 heading 연속이라 마지막 칸 = 회전/도착 직전. 중간 칸은 cruise 로 통과.
        //   runLen ≤ BRAKE_CELLS 면 출발부터 brake 목표 → 낮은 피크 삼각형(영상 1칸 패턴).
        bool cargo  = _hasPayload;
        int  cruise = cargo ? MOTOR_POWER_CARGO : Power;
        int  brake  = CrossingPassPower;
        bool braking = (nLineCounter + DRIVE_BRAKE_CELLS >= _runTargetCount);
        int  target  = braking ? brake : cruise;

        unsigned long now = millis();
        unsigned long dt  = now - _lastDriveMs;
        if (dt > 50) dt = 50;          // 루프 지연/첫 진입 시 과도 점프 방지
        _lastDriveMs = now;

        // 슬로프(PWM/ms). 감속 시간을 짧게 두면 급제동(영상 −64 : +30 ≈ ½).
        float accelStep = (float)(cruise - DRIVE_START_PWM) / (float)DRIVE_ACCEL_MS * dt;
        float decelStep = (float)(cruise - brake)           / (float)DRIVE_DECEL_MS * dt;
        if (_drivePwm < target)      { _drivePwm += accelStep; if (_drivePwm > target) _drivePwm = target; }
        else if (_drivePwm > target) { _drivePwm -= decelStep; if (_drivePwm < target) _drivePwm = target; }

#if DEBUG_APPROACH_TONE
        // 감속 전환(가속/정속 → 감속) 순간 1회 부저음 — 비블로킹.
        if (braking && !_inApproachPrev) tone(pinBuzzer, DEBUG_APPROACH_TONE_HZ, 60);
        _inApproachPrev = braking;
#endif
        int basePower = (int)_drivePwm;

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
    digitalWrite(LeftWheelDir, dirHighLow1);
    analogWrite(LeftWheelPWM, power1 * _motorCalibL);

    digitalWrite(RightWheelDir, dirHighLow2);
    analogWrite(RightWheelPWM, power2 * _motorCalibR);
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

    unsigned long stepDelay = (unsigned long)stepMs;
    unsigned long holdDelay = (unsigned long)holdMs;

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

// BFS 로 currentPose → (tx,ty) 최단경로를 _pathX/_pathY 에 채운다 (출발칸 제외, 목표 포함).
// 도달 불가면 false. currentPose 는 그리드 안(x>=0) 가정 — 시작 패드는 navigateTo 가 선처리.
//   g_bfsParent[idx]: 0=미방문 / 0xFF=출발칸 / 그 외=자식→부모 방향(CONN_*) 으로 역추적.
bool Controller::computeBfsPath(int8_t tx, int8_t ty) {
    const uint8_t CELLS = (uint8_t)(GRID_COLS * GRID_ROWS);
    for (uint8_t i = 0; i < CELLS; i++) g_bfsParent[i] = 0;

    int8_t sx = currentPose.x, sy = currentPose.y;
    if (sx == tx && sy == ty) { _pathLen = 0; return true; }

    static const uint8_t DIR[4]  = { CONN_N, CONN_S, CONN_E, CONN_W };
    static const int8_t  DDX[4]  = { 0, 0, 1, -1 };
    static const int8_t  DDY[4]  = { -1, 1, 0, 0 };
    static const uint8_t BACK[4] = { CONN_S, CONN_N, CONN_W, CONN_E };  // 자식→부모 방향(이동의 반대)

    uint8_t head = 0, tail = 0;
    g_bfsParent[(uint8_t)sy * GRID_COLS + (uint8_t)sx] = 0xFF;
    g_bfsQueue[tail++] = (uint8_t)sy * GRID_COLS + (uint8_t)sx;
    bool found = false;

    while (head < tail && !found) {
        uint8_t idx = g_bfsQueue[head++];
        int8_t x = (int8_t)(idx % GRID_COLS);
        int8_t y = (int8_t)(idx / GRID_COLS);

        uint8_t conn = maskBlockedNeighbors(x, y, lookupConn(x, y));
        if ((conn & CONN_W) && (x - 1 < _navMinX)) conn &= ~CONN_W;     // 맵 격리
        // 동적 차단 리스트가 가득 찼을 때의 안전망: 직전 막힌 한 방향 임시 제외.
        if (_blockedAtX == x && _blockedAtY == y) conn &= ~_blockedDirBit;

        for (uint8_t k = 0; k < 4; k++) {
            if (!(conn & DIR[k])) continue;
            int8_t nx = x + DDX[k], ny = y + DDY[k];
            uint8_t nidx = (uint8_t)ny * GRID_COLS + (uint8_t)nx;
            if (g_bfsParent[nidx] != 0) continue;       // 이미 방문(출발칸 0xFF 포함)
            g_bfsParent[nidx] = BACK[k];
            if (nx == tx && ny == ty) { found = true; break; }
            g_bfsQueue[tail++] = nidx;
        }
    }
    if (!found) return false;

    // 목표 → 출발 역추적 후 뒤집어 _pathX/_pathY 채움 (출발칸 제외).
    _pathLen = 0;
    int8_t cx = tx, cy = ty;
    while (!(cx == sx && cy == sy)) {
        _pathX[_pathLen] = cx; _pathY[_pathLen] = cy; _pathLen++;
        uint8_t b = g_bfsParent[(uint8_t)cy * GRID_COLS + (uint8_t)cx];
        if      (b == CONN_N) cy--;
        else if (b == CONN_S) cy++;
        else if (b == CONN_E) cx++;
        else                  cx--;
    }
    for (uint8_t i = 0; i < _pathLen / 2; i++) {
        int8_t a = _pathX[i]; _pathX[i] = _pathX[_pathLen - 1 - i]; _pathX[_pathLen - 1 - i] = a;
        a = _pathY[i];        _pathY[i] = _pathY[_pathLen - 1 - i]; _pathY[_pathLen - 1 - i] = a;
    }
    return true;
}

// 좌표 네비게이션 (BFS 최단경로).
//
// 매 교차로에서 currentPose → 목표 최단경로를 computeBfsPath 로 구하고 그 경로대로 주행한다.
// 주행 중 장애물(DoLineTrace==false)을 만나면 그 칸을 동적 차단에 넣고 break →
// 바깥 루프가 현재 위치에서 다시 BFS → 즉시 최단 우회로 갈아탄다.
//   - 경로가 없으면 STUCK → false.
//   - 시작 패드(그리드 밖, x<0)는 동쪽으로만 진입 가능하므로 BFS 전에 한 칸 선처리.
bool Controller::navigateTo(int8_t tx, int8_t ty) {
    if (Serial) {
        Serial.print(F("Nav: ("));
        Serial.print(currentPose.x); Serial.print(F(","));
        Serial.print(currentPose.y); Serial.print(F(") -> ("));
        Serial.print(tx); Serial.print(F(",")); Serial.print(ty); Serial.println(F(")"));
    }

    _blockedAtX = -128;
    _blockedAtY = -128;
    _blockedDirBit = 0;

    // 무한루프 안전망 — 재계획/선처리 반복 횟수 상한.
    uint16_t guard = 0;
    const uint16_t GUARD_MAX = (uint16_t)(GRID_COLS * GRID_ROWS) * 4 + 8;

    while (currentPose.x != tx || currentPose.y != ty) {
        if (++guard > GUARD_MAX) {
            if (Serial) Serial.println(F("Nav STUCK: guard exceeded."));
            Stop();
            return false;
        }

        // 시작 패드: 그리드 밖이라 인덱싱 불가 → 동쪽으로 한 칸 진입 후 BFS.
        if (currentPose.x < 0) {
            rotateToHeading(HD_EAST);
            if (!DoLineTrace(1, false)) {
                if (Serial) Serial.println(F("Nav STUCK: obstacle at start-pad exit."));
                Stop();
                return false;
            }
            currentPose.x += 1;
            continue;
        }

        if (!computeBfsPath(tx, ty)) {
            if (Serial) Serial.println(F("Nav STUCK: no path (BFS)."));
            Stop();
            return false;
        }
        navlogPush(NAVLOG_TAG_EVAL,
                   (uint8_t)currentPose.x, (uint8_t)currentPose.y,
                   (uint8_t)currentPose.heading, 0, 0, 0, _pathLen);

        // 계획 경로 주행 — 같은 heading 으로 이어지는 연속 직선 구간을 한 번에 통과한다
        // (직진 중 노드마다 멈추지 않음). 회전 노드 / 정밀정렬 지점(최종 목표 또는
        // 수직 이동으로 y0·y7 도착)에서만 그 런의 마지막 칸이 멈추고 정렬한다.
        uint8_t i = 0;
        while (i < _pathLen) {
            // 이번 런의 heading: 현재 위치 → i 번째 칸 방향. (인접 셀이라 한 축 ±1)
            int8_t dx0 = _pathX[i] - currentPose.x, dy0 = _pathY[i] - currentPose.y;
            Heading runHd = (dy0 == -1) ? HD_NORTH : (dy0 == 1) ? HD_SOUTH :
                            (dx0 == 1)  ? HD_EAST  : HD_WEST;

            // 같은 heading 으로 이어지는 칸을 [i, j) 한 런으로 묶는다 (방향 바뀌면 끊김).
            uint8_t j = i + 1;
            while (j < _pathLen) {
                int8_t ddx = _pathX[j] - _pathX[j - 1], ddy = _pathY[j] - _pathY[j - 1];
                Heading hd = (ddy == -1) ? HD_NORTH : (ddy == 1) ? HD_SOUTH :
                             (ddx == 1)  ? HD_EAST  : HD_WEST;
                if (hd != runHd) break;
                j++;
            }
            uint8_t runLen = j - i;
            uint8_t last   = j - 1;

            // 런의 마지막 칸에서만 정밀 정렬 (최종 목표, 또는 수직 이동으로 y0/y7 도착).
            // y0/y7 은 격자 경계라 수직 런은 항상 그 칸에서 끝남 → 통과 중 누락 없음.
            bool arriveTarget = (_pathX[last] == tx && _pathY[last] == ty);
            bool vertArrival  = (runHd == HD_NORTH || runHd == HD_SOUTH) &&
                                (_pathY[last] == 0 || _pathY[last] == 7);
            bool precise = arriveTarget || vertArrival;

            rotateToHeading(runHd);   // 런 시작에 한 번만 회전 (직진 중엔 회전 없음)

            bool ok = DoLineTrace(runLen, precise);
            uint8_t done = _crossingsDone;   // 이번 런에서 실제 통과한 칸 수

            // 통과한 만큼 pose 전진 (런은 한 축이라 마지막 통과 칸으로 갱신).
            if (done > 0) {
                currentPose.x = _pathX[i + done - 1];
                currentPose.y = _pathY[i + done - 1];
            }

            if (ok) {
                _blockedAtX = -128;
                i = j;                  // 다음 런으로
            } else {
                // 장애물 — 막 진입하려던 칸(i+done) 차단 등록, break 후 현재 위치에서 BFS 재계획.
                int8_t bx = _pathX[i + done], by = _pathY[i + done];
                addDynBlockedCell(bx, by);
                _blockedAtX = currentPose.x;
                _blockedAtY = currentPose.y;
                _blockedDirBit = (runHd == HD_NORTH) ? CONN_N : (runHd == HD_EAST) ? CONN_E :
                                 (runHd == HD_SOUTH) ? CONN_S : CONN_W;
                break;
            }
        }
    }
    return true;
}
