#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>
#include <SPI.h>
#include <deprecated.h>
#include <MFRC522.h>
#include <MFRC522Extended.h>
#include <require_cpp11.h>
#include <Servo.h>
#include <EEPROM.h>

#define START_ADDRESS 240

#define SERVO_DOWN  90-70
#define SERVO_UP    180-80
#define SERVO_DEF   SERVO_DOWN

#define LINEDETECT_THRESHOLD_MIN 730  // 교차로 인식용 블랙 임계값
#define BLANKDETECT_THERSHOLD_MAX 500

#define FORWARD   0
#define BACKWARD  1

// 장애물 인식 기준 거리값
#define OBSTACLE_THRESHOLD 900

enum POSITION {
    eInitialPosition = 0,
    eWareHousePosition,
    eTargetPosition
};
extern POSITION currentPosition;

enum APP_STATE {
    STATE_NONE = 0,
    STATE_RFIDREAD,
    STATE_CMDLIST,
    STATE_LINECOUNTER,
    STATE_LINETRACER,
    STATE_TURNRIGHT,
    STATE_TURNLEFT,
    STATE_TURNHALF
};
extern APP_STATE state;

class Controller {
private:
    uint8_t RFIDReaderSlaveSelect = 2;
    uint8_t pinBuzzer = 3;
    uint8_t RFIDReaderReset = 4;
    uint8_t RightWheelPWM = 6;
    uint8_t LeftWheelPWM = 5;
    uint8_t RightWheelDir = 8;
    uint8_t LeftWheelDir = 7;
    uint8_t LiftServo = 9;
    uint8_t UserButton = A3;
    uint8_t SensorFrontRight = A2;
    uint8_t SensorFrontLeft = A1;
    uint8_t SensorFrontCenter = A0;
    uint8_t SensorBottomRight = A7;
    uint8_t SensorBottomLeft = A6;

    uint16_t nLineCounter = 0;
    uint16_t targetLineCount = 0;

    int16_t _rightWhite;
    int16_t _leftWhite;
    int16_t _rightBlack;
    int16_t _leftBlack;
    float _motorCalibR;
    float _motorCalibL;

    int Power = 110;
    unsigned long lastRFIDTime = 0;

    // 💡 첫 번째 코드에서 가져온 P제어용 변수
    float Kp = 0.05;              // 비례 상수 (필요시 수정)
    float maxCorrection = 35.0;   // 급격한 꺾임 방지

    String s_strRFIDUidForStart = String("44303B74");
    String s_strRFIDUidForSeoul = String("84CA4874");
    String s_strRFIDUidForIncheon = String("A4263674");
    String s_strRFIDUidForSejong = String("446CF4BB");
    String s_strRFIDUidForDaejeon = String("74200C74");
    String s_strRFIDUidForDaegu = String("");
    String s_strRFIDUidForGwangju = String("");
    String s_strRFIDUidForChuncheon = String("");
    String s_strRFIDUidForJeju = String("5468E6BB");

    MFRC522 mfrc522;
    Servo servo;
    String  strRFID;
    bool isBusy = false;

public:
    bool enableObstacleAvoidance = true;

    Controller() : mfrc522(RFIDReaderSlaveSelect, RFIDReaderReset)
    {}

    void init();
    void RunOnce();
    void ProcessRFIDRead();

    //Lift Controller
    void  LifterUp();
    void  LifterDown();

    // RFID Read
    bool RFIDRead();

    // Functions for Drive
    bool LineTracer(uint16_t nTargetLineCounter);
    void DoLineTrace(uint16_t targetCount);
    void LineTrace();
    void ResetLineCounter();
    void Move();
    void drive(int dir1, int power1, int dir2, int power2);
    void Forward(int power);
    void Backward(int power);
    void TurnLeft(int power);
    void TurnRight(int power);
    void Stop();
    void TurnHalf();
    void PivotTurnLeft();
    void PivotTurnRight();

    // IR Sensor Value
    int16_t GetLeft();
    int16_t GetRight();
    void readData();
    void PlayMelody();

    // 장애물 관련 함수
    bool CheckObstacle();
    void ReverseToPreviousNode();

    // 💡 첫 번째 코드에서 가져온 정규화 함수 선언
    int normalizeLeft(int rawValue);
    int normalizeRight(int rawValue);
};

#endif // CONTROLLER_H
