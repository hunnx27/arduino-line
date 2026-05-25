#include "Controller.h"

// === 좌표 맵 (4열 × 8행 풀 그리드) ===
// 창고: (2, 0), 도시 행: y=7. 모든 셀에 4방향(N/E/S/W) 라인이 있다고 가정 —
// 가장자리 셀은 격자 밖 방향만 자동 제외. 장애물 우회 시 좌/우로 빠질 라인 보장.
// 만약 layout 에 누락 구간이 생기면 lookupConn 안에서 예외 처리.
#define GRID_COLS 4
#define GRID_ROWS 8

//        x=0         x=1           x=2           x=3
//        ┌──────┐    ┌──────┐      ┌──────────┐  ┌──────┐
//  y=0   │(0,0) │────│ (1,0)│──────│■(2,0)    │──│(3,0) │   ■ 창고
//        └──┬───┘    └──┬───┘      └────┬─────┘  └──┬───┘   WAREHOUSE
//           │           │                │            │
//  y=1     (0,1)───────(1,1)────────────(2,1)────────(3,1)
//           │           │                │            │
//  y=2     (0,2)───────(1,2)────────────(2,2)────────(3,2)
//           │           │                │            │
// ◉(-1,3)─(0,3)───────(1,3)────────────(2,3)────────(3,3)   ◉ 시작 RFID — y=3, 그리드 한 칸 뒤. heading=EAST.
//           │           │                │            │
//  y=4     (0,4)───────(1,4)────────────(2,4)────────(3,4)
//           │           │                │            │
//  y=5     (0,5)───────(1,5)────────────(2,5)────────(3,5)
//           │           │                │            │
//  y=6     (0,6)───────(1,6)────────────(2,6)────────(3,6)
//           │           │                │            │
//  y=7   ┌──┴───┐    ┌──┴───┐      ┌────┴─────┐  ┌──┴───┐
//        │Seoul │────│Inchn │──────│Sejong ★ │──│Daejn │   ← 도시 라인
//        │ (0,7)│    │ (1,7)│      │ (2,7)    │  │ (3,7)│
//        │ UID? │    │ UID? │      │ 148EC573 │  │ UID? │
//        └──────┘    └──────┘      └──────────┘  └──────┘
// === 도시 RFID UID → 좌표 매핑 ===
// 빈 UID 는 RFID 미등록 (실제 태그 부착 후 채워 넣기)
static const CityCoord CITY_COORDS[] = {
    {"",         0, 7},  // Seoul    (col 0)
    {"",         1, 7},  // Incheon  (col 1)
    {"", 2, 7},  // Sejong   (col 2 = 메인 라인. 창고 바로 아래, 현재 유일한 활성 UID)
    {"148EC573",         3, 7},  // Daejeon  (col 3)
};
static const uint8_t CITY_COORD_COUNT = sizeof(CITY_COORDS) / sizeof(CityCoord);

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

bool Controller::CheckObstacle() {
    Serial.print("sensor front center :");
    Serial.println(analogRead(SensorFrontCenter));
    if (analogRead(SensorFrontCenter) < OBSTACLE_THRESHOLD) {
        delay(2);
        if (analogRead(SensorFrontCenter) < OBSTACLE_THRESHOLD) {
            return true;
        }
    }
    return false;
}

void Controller::ReverseToPreviousNode() {
    if (Serial) Serial.println("Obstacle! Reversing...");
    drive(BACKWARD, Power, BACKWARD, Power);
    delay((unsigned long)(500 / SPEED_SCALE));

    while (true) {
        int right = GetRight();
        int left = GetLeft();
        if (right > LINEDETECT_THRESHOLD_MIN && left > LINEDETECT_THRESHOLD_MIN) {
            break;
        }
    }
    Stop();
    delay(400);
}

bool Controller::DoLineTrace(uint16_t targetCount, bool precise)
{
    _preciseRealign = precise;  // LineTracer 가 도달 시점에 읽음
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
        case eInitialPosition:
            if (strRFID.compareTo(s_strRFIDUidForStart) == 0) {
                // 시작 RFID 위치에서 좌표계 시작 → 네비게이터로 창고까지 자동 이동.
                // 장애물 우회도 동일 메커니즘으로 처리됨.
                currentPose = {INIT_START_X, INIT_START_Y, INIT_START_HEADING};
                navigateTo(WAREHOUSE_X, WAREHOUSE_Y);
                rotateToHeading(HD_SOUTH);  // 창고에서 도시 방향(+y, row 7 쪽) 으로 정렬
                LifterUp();
                currentPosition = eWareHousePosition;
            }
            mfrc522.PCD_AntennaOn();
            break;

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

void  Controller::LifterUp()
{
    servo.attach(LiftServo);
    delay(10);
    servo.write(SERVO_UP);
    delay(300);
    servo.detach();
    delay(10);
}

void  Controller::LifterDown()
{
    servo.attach(LiftServo);
    delay(10);
    servo.write(SERVO_DOWN);
    delay(300);
    servo.detach();
    delay(10);
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

            // forward overshoot ∝ v² ∝ PWM² ∝ SPEED_SCALE² 이므로 후진 거리도 SPEED_SCALE² 만큼 줄임.
            // drive() PWM × SPEED_SCALE  +  delay × SPEED_SCALE  =  거리 × SPEED_SCALE².
            drive(BACKWARD, Power, BACKWARD, Power);
            delay(400 * SPEED_SCALE);

            Stop();
            delay(100); // 기어 방향 전환 전 잠깐 대기

            drive(FORWARD, Power - 40, FORWARD, Power - 40);
            while (true) {
                int left = GetLeft();
                int right = GetRight();
                if (left > LINEDETECT_THRESHOLD_MIN && right > LINEDETECT_THRESHOLD_MIN) break;
            }
            delay(300 - (300*SPEED_SCALE));

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
    static uint8_t bSignalHigh = 0;
    static unsigned long lastSensorLog = 0;

    int leftRaw = GetLeft();
    int rightRaw = GetRight();

    // 디버그: 200ms마다 좌/우 raw 값과 정규화 값 출력
    if (millis() - lastSensorLog > 200) {
        Serial.print("L_raw=");  Serial.print(leftRaw);
        Serial.print(" R_raw="); Serial.print(rightRaw);
        Serial.print(" | L_n="); Serial.print(normalizeLeft(leftRaw));
        Serial.print(" R_n=");   Serial.println(normalizeRight(rightRaw));
        lastSensorLog = millis();
    }

    // 교차로(검은선 2개 동시) 판단
    if (rightRaw > LINEDETECT_THRESHOLD_MIN && leftRaw > LINEDETECT_THRESHOLD_MIN) {
        if (bSignalHigh == 0) {
            nLineCounter++;
            if (Serial) {
                Serial.println(String("LINE!!! :") + String(nLineCounter));
            }
            bSignalHigh = 1;
        }

        Forward(Power);
        delay((unsigned long)(50 / SPEED_SCALE)); // 🌟 선을 완전히 넘어가도록 약간의 전진 딜레이 유지 (그래야 후진할 때 선을 확실히 찾습니다)
    }
    else {
        if (bSignalHigh) {
            bSignalHigh = 0;
        }

        // 부드러운 P-제어
        int leftNorm = normalizeLeft(leftRaw);
        int rightNorm = normalizeRight(rightRaw);

        int error = rightNorm - leftNorm;
        float correction = Kp * error;

        if (correction > maxCorrection) correction = maxCorrection;
        if (correction < -maxCorrection) correction = -maxCorrection;

        int basePower = (currentPosition == eWareHousePosition) ? Power - 20 : Power;

        float leftPower = basePower + correction;
        float rightPower = basePower - correction;

        drive(FORWARD, (int)leftPower, FORWARD, (int)rightPower);
    }
}


void Controller::ResetLineCounter()
{
    nLineCounter = 0;
}

void Controller::Move()
{
    Stop();
    delay(10);
    analogWrite(LeftWheelPWM, (int)(140 * SPEED_SCALE));
    analogWrite(RightWheelPWM, (int)(140 * SPEED_SCALE));
    delay((unsigned long)(100 / SPEED_SCALE));
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

void  Controller::Backward(int power)
{
    drive(BACKWARD, power, BACKWARD, power);
}

void  Controller::TurnLeft(int power)
{
    drive(BACKWARD, power, FORWARD, power);
}

void  Controller::TurnRight(int power)
{
    drive(FORWARD, power, BACKWARD, power);
}

void Controller::Stop()
{
    analogWrite(LeftWheelPWM, 0);
    analogWrite(RightWheelPWM, 0);
}

void Controller::TurnHalf() {
    drive(BACKWARD, 80, FORWARD, 80);
    delay((unsigned long)(50 / SPEED_SCALE));
    drive(BACKWARD, 170, FORWARD, 170);
    delay((unsigned long)(450 / SPEED_SCALE));
    Stop();
}

void Controller::PivotTurnLeft()
{
    if (Serial) Serial.println("Enter Pivot turn Left");
    analogWrite(LeftWheelPWM, 0);
    analogWrite(RightWheelPWM, 0);
    delay(10);
    digitalWrite(LeftWheelDir, 0);
    digitalWrite(RightWheelDir, 0);

    Move();
    delay(10);
    // [1] 킥스타트: 정지 마찰 극복용 초기 부스트 (양쪽 동일 PWM, 50ms)
    analogWrite(LeftWheelPWM, (int)(170 * SPEED_SCALE));
    analogWrite(RightWheelPWM, (int)(170 * SPEED_SCALE));
    delay((unsigned long)(50 / SPEED_SCALE));
    // [2] 회전 구간: 왼쪽 약, 오른쪽 강 → 좌회전 (시간으로 회전각 결정)
    analogWrite(LeftWheelPWM, (int)(90 * _motorCalibL * SPEED_SCALE));
    analogWrite(RightWheelPWM, (int)(180 * _motorCalibR * SPEED_SCALE));
    delay((unsigned long)((currentPosition == eWareHousePosition ? 140 : 110) / SPEED_SCALE));

    // [3] 정지
    analogWrite(LeftWheelPWM, 0);
    analogWrite(RightWheelPWM, 0);

    if (Serial) Serial.println("Leave Pivot turn Left");
}

void Controller::PivotTurnRight()
{
    if (Serial) Serial.println("Enter Pivot turn Right");
    analogWrite(LeftWheelPWM, 0);
    analogWrite(RightWheelPWM, 0);
    delay(10);
    digitalWrite(LeftWheelDir, 1);
    digitalWrite(RightWheelDir, 1);

    Move();
    delay(10);
    // [1] 킥스타트: 정지 마찰 극복용 초기 부스트 (양쪽 동일 PWM, 50ms)
    analogWrite(LeftWheelPWM, (int)(170 * _motorCalibL * SPEED_SCALE));
    analogWrite(RightWheelPWM, (int)(170 * _motorCalibR * SPEED_SCALE));
    delay((unsigned long)(50 / SPEED_SCALE));
    // [2] 회전 구간: 왼쪽 강, 오른쪽 약 → 우회전 (시간으로 회전각 결정)
    analogWrite(LeftWheelPWM, (int)(170 * _motorCalibL * SPEED_SCALE));
    analogWrite(RightWheelPWM, (int)(90 * _motorCalibR * SPEED_SCALE));
    delay((unsigned long)((currentPosition == eWareHousePosition ? 140 : 110) / SPEED_SCALE));

    // [3] 정지
    analogWrite(LeftWheelPWM, 0);
    analogWrite(RightWheelPWM, 0);
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

void Controller::PlayMelody() {
    tone(pinBuzzer, 440);
    delay(300);
    tone(pinBuzzer, 587);
    delay(300);
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

bool Controller::navigateTo(int8_t tx, int8_t ty) {
    if (Serial) {
        Serial.print(F("Nav: ("));
        Serial.print(currentPose.x); Serial.print(F(","));
        Serial.print(currentPose.y); Serial.print(F(") -> ("));
        Serial.print(tx); Serial.print(F(",")); Serial.print(ty); Serial.println(F(")"));
    }

    // 직전 호출의 잔여 차단 정보 클리어
    _blockedAtX = -128;
    _blockedAtY = -128;
    _blockedDirBit = 0;

    while (currentPose.x != tx || currentPose.y != ty) {
        int8_t  dx   = tx - currentPose.x;
        int8_t  dy   = ty - currentPose.y;
        uint8_t conn = lookupConn(currentPose.x, currentPose.y);

        // 직전 시도에서 막힌 방향은 일시 마스킹
        if (_blockedAtX == currentPose.x && _blockedAtY == currentPose.y) {
            conn &= ~_blockedDirBit;
        }

        Heading desired = desiredHeading(dx, dy, conn, currentPose.heading);
        if ((uint8_t)desired == 0xFF) {
            if (Serial) {
                Serial.print(F("Nav STUCK at ("));
                Serial.print(currentPose.x); Serial.print(F(","));
                Serial.print(currentPose.y); Serial.print(F(") target ("));
                Serial.print(tx); Serial.print(F(",")); Serial.print(ty);
                Serial.println(F(") — no alt path. Check CROSSINGS[] or expand layout."));
            }
            Stop();
            return false;
        }

        rotateToHeading(desired);

        // 도착할 위치가 창고 행(y=0) 또는 도시 행(y=7) 이면 정밀 정렬 사용
        int8_t newY = currentPose.y + headingDy(desired);
        bool precise = (newY == 0 || newY == 7);

        if (DoLineTrace(1, precise)) {
            // 성공 — 차단 해제 + pose 갱신
            _blockedAtX = -128;
            currentPose.x += headingDx(desired);
            currentPose.y += headingDy(desired);
        } else {
            // 장애물로 중단 — 막힌 방향 기록 (heading은 DoLineTrace 안에서 이미 갱신됨)
            _blockedAtX = currentPose.x;
            _blockedAtY = currentPose.y;
            switch (desired) {
                case HD_NORTH: _blockedDirBit = CONN_N; break;
                case HD_EAST:  _blockedDirBit = CONN_E; break;
                case HD_SOUTH: _blockedDirBit = CONN_S; break;
                case HD_WEST:  _blockedDirBit = CONN_W; break;
                default: break;
            }
            // pose 좌표는 그대로 — 다음 iteration 이 마스킹된 conn 으로 재계산
        }
    }
    return true;
}
