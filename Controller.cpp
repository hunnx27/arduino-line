#include "Controller.h"

// === 좌표 맵 (4열 × 8행 그리드) ===
// 창고: (1, 0), 도시 행: y=7, 메인 라인: x=1
// TODO: 실제 라인 레이아웃과 대조 후 보정 — 분기점 누락/추가 시 여기만 수정.
static const Crossing CROSSINGS[] = {
    // 메인 라인 (창고 → 도시 행, x=1)
    {1, 0, CONN_N},                                  // 창고 (북쪽으로만)
    {1, 1, CONN_N|CONN_S},
    {1, 2, CONN_N|CONN_S},
    {1, 3, CONN_N|CONN_S},
    {1, 4, CONN_N|CONN_S},
    {1, 5, CONN_N|CONN_S},
    {1, 6, CONN_N|CONN_S},
    // 도시 행 (y = 7) — 좌우로 연결
    {0, 7, CONN_E},                                  // col 0 도시 (Daejeon)
    {1, 7, CONN_S|CONN_E|CONN_W},                    // col 1 도시 (Sejong, 메인 라인 종점)
    {2, 7, CONN_E|CONN_W},                           // col 2 도시 (Incheon)
    {3, 7, CONN_W},                                  // col 3 도시 (Seoul)
};
static const uint8_t CROSSING_COUNT = sizeof(CROSSINGS) / sizeof(Crossing);

// === 도시 RFID UID → 좌표 매핑 ===
// 빈 UID 는 RFID 미등록 (실제 태그 부착 후 채워 넣기)
static const CityCoord CITY_COORDS[] = {
    {"148EC573", 0, 7},  // Daejeon — 현재 유일한 활성 UID
    {"",         1, 7},  // Sejong  — UID 미입력
    {"",         2, 7},  // Incheon — UID 미입력
    {"",         3, 7},  // Seoul   — UID 미입력
};
static const uint8_t CITY_COORD_COUNT = sizeof(CITY_COORDS) / sizeof(CityCoord);

// === 네비게이션 헬퍼 ===
static Heading opposite(Heading h) { return (Heading)((h + 2) % 4); }

static int8_t headingDx(Heading h) {
    switch (h) { case HD_EAST: return 1; case HD_WEST: return -1; default: return 0; }
}
static int8_t headingDy(Heading h) {
    switch (h) { case HD_NORTH: return 1; case HD_SOUTH: return -1; default: return 0; }
}

static uint8_t lookupConn(int8_t x, int8_t y) {
    for (uint8_t i = 0; i < CROSSING_COUNT; i++) {
        if (CROSSINGS[i].x == x && CROSSINGS[i].y == y) return CROSSINGS[i].conn;
    }
    return 0;
}

// Y(세로) 우선 → X(가로) 순. 메인 라인이 세로라 이 우선순위로 트리 구조 layout 에 항상 최적해.
static Heading desiredHeading(int8_t dx, int8_t dy, uint8_t conn) {
    if (dy > 0 && (conn & CONN_N)) return HD_NORTH;
    if (dy < 0 && (conn & CONN_S)) return HD_SOUTH;
    if (dx > 0 && (conn & CONN_E)) return HD_EAST;
    if (dx < 0 && (conn & CONN_W)) return HD_WEST;
    return (Heading)0xFF;  // 갈 곳 없음
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

void Controller::DoLineTrace(uint16_t targetCount)
{
    while (!LineTracer(targetCount)) {
        if (enableObstacleAvoidance) {
            if (CheckObstacle()) {
                Stop();
                delay(500);

                if (Serial) Serial.println("Obstacle Detected! Auto Bypassing...");
                enableObstacleAvoidance = false;
                uint16_t savedCounter = nLineCounter;

                ReverseToPreviousNode();

                PivotTurnLeft();
                DoLineTrace(1);

                PivotTurnRight();
                DoLineTrace(2);

                PivotTurnRight();
                DoLineTrace(1);

                PivotTurnLeft();

                if (targetCount > 0) {
                    targetCount--;
                }

                nLineCounter = savedCounter;
                delay(300);
                enableObstacleAvoidance = true;
            }
        }
    }
}

void Controller::ProcessRFIDRead()
{
    if (RFIDRead()) {
        isBusy = true;
        mfrc522.PCD_AntennaOff();
        switch (currentPosition) {
        case eInitialPosition:
            if (strRFID.compareTo(s_strRFIDUidForStart) == 0) {
                DoLineTrace(3);
                PivotTurnLeft();
                DoLineTrace(3);
            }
            Stop();
            TurnHalf();
            LifterUp();
            currentPosition = eWareHousePosition;
            // 좌표계 초기화 — init 시퀀스 종료 직후 로봇이 창고에서 도시 방향(N)을 보고 있다고 선언.
            // 실제 물리적 heading 과 다르면 navigateTo 가 첫 호출 때 TurnHalf 로 자동 보정함.
            currentPose = {1, 0, HD_NORTH};
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
            navigateTo(tx, ty);

            // ── 2) 도착 후 행위 (화물 내리고 180° 회전) ──
            LifterDown();
            Stop();
            delay(700);
            TurnHalf();
            currentPose.heading = opposite(currentPose.heading);

            // ── 3) 물류창고로 복귀 ──
            delay(1000);
            navigateTo(1, 0);  // 창고 좌표

            // 다음 화물 픽업 자세 (180° 회전하여 도시 방향으로 다시 face)
            Stop();
            TurnHalf();
            currentPose.heading = opposite(currentPose.heading);
            LifterUp();

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
        if (Serial) {
            Serial.println("LineCount Finished. Reversing then Forward aligning...");
        }

        // 1. 관성으로 밀려간 상태에서 일단 정지
        Stop();
        delay(150);

        // 🌟 2. 뒤로 충분히 이동하여 선을 완전히 벗어남 (이때는 센서 인식 아예 안 함!)
        // forward overshoot ∝ v² ∝ PWM² ∝ SPEED_SCALE² 이므로 후진 거리도 SPEED_SCALE² 만큼 줄여야 함.
        // drive() PWM × SPEED_SCALE  +  delay × SPEED_SCALE  =  거리 × SPEED_SCALE².
        drive(BACKWARD, Power, BACKWARD, Power);
        delay(400 * SPEED_SCALE);

        Stop();
        delay(100); // 기어 방향 전환 전 잠깐 대기 (전기적 대기, 속도 무관)

        // 🌟 3. 다시 앞으로 천천히 이동하면서 선을 찾음 (앞으로 갈 때 인식!)
        drive(FORWARD, Power - 40, FORWARD, Power - 40);
        while (true) {
            int left = GetLeft();
            int right = GetRight();

            // 앞으로 오다가 양쪽 센서가 교차로(검은 선)를 밟으면 칼정지!
            if (left > LINEDETECT_THRESHOLD_MIN && right > LINEDETECT_THRESHOLD_MIN) {
                break;
            }
        }

        // 4. 완벽한 전진 정렬 완료!
        Stop();
        delay(200); // 차체 안정화 대기 (관성 잔량 처리, 속도 무관)

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
    analogWrite(LeftWheelPWM, (int)(170 * SPEED_SCALE));
    analogWrite(RightWheelPWM, (int)(170 * SPEED_SCALE));
    delay((unsigned long)(50 / SPEED_SCALE));
    analogWrite(LeftWheelPWM, (int)(90 * _motorCalibL * SPEED_SCALE));
    analogWrite(RightWheelPWM, (int)(180 * _motorCalibR * SPEED_SCALE));
    delay((unsigned long)((currentPosition == eWareHousePosition ? 140 : 110) / SPEED_SCALE));

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
    analogWrite(LeftWheelPWM, (int)(170 * _motorCalibL * SPEED_SCALE));
    analogWrite(RightWheelPWM, (int)(170 * _motorCalibR * SPEED_SCALE));
    delay((unsigned long)(50 / SPEED_SCALE));
    analogWrite(LeftWheelPWM, (int)(170 * _motorCalibL * SPEED_SCALE));
    analogWrite(RightWheelPWM, (int)(90 * _motorCalibR * SPEED_SCALE));
    delay((unsigned long)((currentPosition == eWareHousePosition ? 140 : 110) / SPEED_SCALE));

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

void Controller::navigateTo(int8_t tx, int8_t ty) {
    if (Serial) {
        Serial.print(F("Nav: ("));
        Serial.print(currentPose.x); Serial.print(F(","));
        Serial.print(currentPose.y); Serial.print(F(") -> ("));
        Serial.print(tx); Serial.print(F(",")); Serial.print(ty); Serial.println(F(")"));
    }

    while (currentPose.x != tx || currentPose.y != ty) {
        int8_t  dx   = tx - currentPose.x;
        int8_t  dy   = ty - currentPose.y;
        uint8_t conn = lookupConn(currentPose.x, currentPose.y);
        Heading desired = desiredHeading(dx, dy, conn);

        if ((uint8_t)desired == 0xFF) {
            // 갈 수 있는 방향이 없음 — 맵 누락이거나 잘못된 위치. 안전 정지.
            if (Serial) {
                Serial.print(F("Nav STUCK at ("));
                Serial.print(currentPose.x); Serial.print(F(","));
                Serial.print(currentPose.y); Serial.print(F(") target ("));
                Serial.print(tx); Serial.print(F(",")); Serial.print(ty);
                Serial.println(F(") — check CROSSINGS[] map"));
            }
            Stop();
            return;
        }

        rotateToHeading(desired);
        DoLineTrace(1);                         // 한 교차로만 전진 (장애물 우회는 내부에서)
        currentPose.x += headingDx(desired);
        currentPose.y += headingDy(desired);
    }
}
