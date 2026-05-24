#include "Controller.h"

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
    delay(500);

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
            mfrc522.PCD_AntennaOn();
            break;

        case eWareHousePosition:
            if (strRFID.compareTo(s_strRFIDUidForSeoul) == 0) {
                delay(500);
                DoLineTrace(7);
                PivotTurnRight();
                DoLineTrace(2);
                PivotTurnLeft();
            } else if (strRFID.compareTo(s_strRFIDUidForIncheon) == 0) {
                delay(500);
                DoLineTrace(7);
                PivotTurnRight();
                DoLineTrace(1);
                PivotTurnLeft();
            } else if (strRFID.compareTo(s_strRFIDUidForSejong) == 0) {
                delay(500);
                DoLineTrace(7);
                PivotTurnRight();
                DoLineTrace(1);
                PivotTurnLeft();
                DoLineTrace(1);
            } else if (strRFID.compareTo(s_strRFIDUidForDaejeon) == 0) {
                delay(500);
                DoLineTrace(7);
                PivotTurnLeft();
                DoLineTrace(1);
                PivotTurnRight();
            } else if (strRFID.compareTo(s_strRFIDUidForDaegu) == 0) {
                delay(500);
                DoLineTrace(8);
                PivotTurnLeft();
                DoLineTrace(1);
                PivotTurnRight();
                DoLineTrace(1);
            } else if (strRFID.compareTo(s_strRFIDUidForGwangju) == 0) {
                delay(500);
                DoLineTrace(8);
                PivotTurnLeft();
                DoLineTrace(1);
                PivotTurnRight();
            } else if (strRFID.compareTo(s_strRFIDUidForChuncheon) == 0) {
                delay(500);
                DoLineTrace(8);
                PivotTurnLeft();
                DoLineTrace(3);
                PivotTurnRight();
                DoLineTrace(1);
            } else if (strRFID.compareTo(s_strRFIDUidForJeju) == 0) {
                delay(500);
                DoLineTrace(8);
                PivotTurnLeft();
                DoLineTrace(4);
                PivotTurnRight();
                DoLineTrace(1);
            }
            mfrc522.PCD_AntennaOn();
            state = STATE_RFIDREAD;
            LifterDown();
            Stop();
            delay(700);
            TurnHalf();
            currentPosition = eTargetPosition;
            break;

        case 2:
            if (strRFID.compareTo(s_strRFIDUidForSeoul) == 0) {
                delay(1000);
                DoLineTrace(7);
                PivotTurnRight();
                DoLineTrace(2);
                PivotTurnLeft();
            } else if (strRFID.compareTo(s_strRFIDUidForIncheon) == 0) {
                delay(1000);
                DoLineTrace(7);
                PivotTurnRight();
                DoLineTrace(1);
                PivotTurnLeft();
            } else if (strRFID.compareTo(s_strRFIDUidForSejong) == 0) {
                delay(1000);
                DoLineTrace(7);
                PivotTurnRight();
                DoLineTrace(2);
                PivotTurnLeft();
            } else if (strRFID.compareTo(s_strRFIDUidForDaejeon) == 0) {
                delay(700);
                DoLineTrace(7);
                PivotTurnLeft();
                DoLineTrace(1);
                PivotTurnRight();
            } else if (strRFID.compareTo(s_strRFIDUidForDaegu) == 0) {
                delay(1000);
                DoLineTrace(8);
                PivotTurnLeft();
                DoLineTrace(1);
                PivotTurnRight();
                DoLineTrace(1);
            } else if (strRFID.compareTo(s_strRFIDUidForGwangju) == 0) {
                delay(1000);
                DoLineTrace(8);
                PivotTurnLeft();
                DoLineTrace(2);
                PivotTurnRight();
                DoLineTrace(1);
            } else if (strRFID.compareTo(s_strRFIDUidForChuncheon) == 0) {
                delay(1000);
                DoLineTrace(8);
                PivotTurnLeft();
                DoLineTrace(3);
                PivotTurnRight();
                DoLineTrace(1);
            } else if (strRFID.compareTo(s_strRFIDUidForJeju) == 0) {
                delay(1000);
                DoLineTrace(8);
                PivotTurnLeft();
                DoLineTrace(4);
                PivotTurnRight();
                DoLineTrace(1);
            }
            Stop();
            TurnHalf();
            LifterUp();
            currentPosition = eWareHousePosition;
            mfrc522.PCD_AntennaOn();
            break;
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
        drive(BACKWARD, Power, BACKWARD, Power);
        // 400ms 동안 뒤로 갑니다. 만약 선을 덜 벗어나면 500으로 늘리고, 너무 많이 가면 300으로 줄이세요.
        delay(400);

        Stop();
        delay(100); // 기어 방향 전환 전 잠깐 대기

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
        delay(200); // 차체가 완전히 안정화될 때까지 대기한 뒤 턴을 수행

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
        delay(50); // 🌟 선을 완전히 넘어가도록 약간의 전진 딜레이 유지 (그래야 후진할 때 선을 확실히 찾습니다)
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
    analogWrite(LeftWheelPWM, 140);
    analogWrite(RightWheelPWM, 140);
    delay(100);
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
    delay(50);
    drive(BACKWARD, 170, FORWARD, 170);
    delay(450);
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
    analogWrite(LeftWheelPWM, 170);
    analogWrite(RightWheelPWM, 170);
    delay(50);
    analogWrite(LeftWheelPWM, 90 * _motorCalibL);
    analogWrite(RightWheelPWM, 180 * _motorCalibR);
    delay(currentPosition == eWareHousePosition ? 140 : 110);

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
    analogWrite(LeftWheelPWM, 170 * _motorCalibL);
    analogWrite(RightWheelPWM, 170 * _motorCalibR);
    delay(50);
    analogWrite(LeftWheelPWM, 170 * _motorCalibL);
    analogWrite(RightWheelPWM, 90 * _motorCalibR);
    delay(currentPosition == eWareHousePosition ? 140 : 110);

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
