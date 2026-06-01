#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Servo.h>
#include <EEPROM.h>

// EEPROM 캘리브레이션 영역 시작 주소 (하드웨어 layout — 변경 비추).
#define START_ADDRESS 240

// 모터 방향 비트 (하드웨어 wiring 종속 — 변경 비추).
#define FORWARD   0
#define BACKWARD  1

enum POSITION {
    eInitialPosition = 0,
    eWareHousePosition,
    eTargetPosition
};

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

// === 좌표 기반 네비게이션 타입 ===
enum Heading : uint8_t {
    HD_NORTH = 0, HD_EAST = 1, HD_SOUTH = 2, HD_WEST = 3
};

// 4방향 비트마스크 (Crossing.conn 용)
#define CONN_N  0x01
#define CONN_E  0x02
#define CONN_S  0x04
#define CONN_W  0x08

struct Pose {
    int8_t  x;
    int8_t  y;
    Heading heading;
};

struct Crossing {
    int8_t  x;
    int8_t  y;
    uint8_t conn;       // 이 교차로에서 갈 수 있는 방향 비트마스크
};

struct CityCoord {
    const char* uid;
    int8_t x;
    int8_t y;
};

struct BlockedCell {
    int8_t x;
    int8_t y;
};

// 튜닝 가능 상수는 Settings.h 한 곳에 모음.
// (BLOCKED_CELLS / CITY_COORDS / INIT_START_HEADING 가 위의 타입을 참조하므로
//  반드시 타입 정의 뒤, class Controller 정의 앞 에서 include.)
#include "Settings.h"

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

    // === 튜닝 값 기본치는 Settings.h 매크로에서 가져옴 ===
    int Power                          = MOTOR_POWER;
    int CrossingPassPower              = CROSSING_PASS_POWER;
    int CrossingApproachPower          = CROSSING_APPROACH_POWER;
    unsigned long CrossingApproachMs       = CROSSING_APPROACH_MS;
    unsigned long CrossingApproachMsCargo  = CROSSING_APPROACH_MS_CARGO;
    // 직전 교차로 검출 시각 — LineTrace 안에서 갱신. 사전 감속 타이머 기준.
    unsigned long _lastCrossingTime = 0;
    unsigned long lastRFIDTime = 0;

    // PD 제어 상수 (기본값 Settings.h 에서)
    float Kp            = PID_KP;
    float Kd            = PID_KD;
    float maxCorrection = PID_MAX_CORRECTION;

    // D 항 계산용 — 직전 error 저장. 교차로 진입/이탈 시 0 으로 리셋해서 spike 방지.
    int _prevError = 0;

    // 시작용 RFID UID. 도시 UID 는 Settings.h 의 CITY_COORDS 테이블 참조.
    String s_strRFIDUidForStart = String(START_RFID_UID);

    MFRC522 mfrc522;
    Servo servo;
    int _servoAngle = SERVO_DEF;   // 현재 리프터 각도 추적 (부드러운 슬루 기준점)
    bool _hasPayload = false;      // 팔레트 적재 여부 — LifterUp/Down 이 토글. cargo(느린/부드러운) 거동 판정 기준.
    bool _inApproachPrev = false;  // 사전 감속 전환 edge 감지용 (DEBUG_APPROACH_TONE).
    String  strRFID;
    bool isBusy = false;

    POSITION currentPosition = eInitialPosition;
    APP_STATE state = STATE_NONE;

    // 현재 좌표/방향. 부팅 직후엔 시작 RFID 위치에서 시작 (eInitialPosition 핸들러에서 재설정).
    Pose currentPose = {INIT_START_X, INIT_START_Y, INIT_START_HEADING};

    // 교차로 카운트 디바운스 래치. 1 = 현재 라인 위(이미 카운트됨).
    // DoLineTrace 진입 시 현재 센서값으로 프라이밍 → 출발 라인 위에서 시작해도 재카운트 방지.
    uint8_t _bSignalHigh = 0;

    // 네비게이션 보조 상태
    bool _preciseRealign = true;            // LineTracer 정렬 dance 수행 여부 (y=0/7 에서만 true)
    int8_t _blockedAtX = -128;              // 직전에 장애물로 막힌 좌표 (-128 = 없음)
    int8_t _blockedAtY = -128;
    uint8_t _blockedDirBit = 0;             // 막힌 방향의 CONN_* 비트
    // 맵 격리 경계: 이 열 미만으로 서쪽 이동 금지. 부팅 시 무제한(-128),
    // eInitialPosition 최초 창고 도착 후 NAV_MIN_X 로 설정 (로봇2=4, 로봇1=-128 무효).
    int8_t _navMinX = -128;

    // DFS 백트래킹용 경로 스택. navigateTo 시작 시 currentPose 를 push 하고,
    // 한 칸 전진할 때마다 push, 막다른 길에서 pop 하며 부모 칸으로 물리 후진.
    // 동시에 사이클 방지 — 이미 스택에 있는 칸 방향은 forward 후보에서 제외.
    // 단순 경로 최대 GRID_COLS*GRID_ROWS + 그리드 밖 시작 1 + 여유 1.
    // 로봇1(4×8)=34, 로봇2(8×8)=66. uint8_t 범위 내.
    static const uint8_t NAV_PATH_MAX = GRID_COLS * GRID_ROWS + 2;  // 로봇1:34, 로봇2:66
    int8_t _pathX[NAV_PATH_MAX];
    int8_t _pathY[NAV_PATH_MAX];
    uint8_t _pathLen = 0;

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
    void  LifterMove(int targetAngle);   // 목표각까지 단계적으로 슬루(부드러운 이동)

    // RFID Read
    bool RFIDRead();

    // Functions for Drive
    bool LineTracer(uint16_t nTargetLineCounter);
    // precise=true 시 라인 도달 후 후진-전진 정렬 dance 수행 (정확한 turn 위치 확보).
    // 좌표 네비게이션은 y=0/y=7 에서만 precise 사용. 반환값: false 면 장애물로 인해 중단.
    bool DoLineTrace(uint16_t targetCount, bool precise = true);
    void LineTrace();
    void ResetLineCounter();
    void drive(int dir1, int power1, int dir2, int power2);
    void Forward(int power);
    void Stop();
    void TurnHalf();
    void PivotTurnLeft();
    void PivotTurnRight();
    // 사다리꼴(가속→정속→감속) 회전 프리미티브 — 위 3개 회전이 공통 사용.
    void RampTurn(int dirL, int dirR, int cruiseL, int cruiseR, unsigned long holdMs);

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

    // 양 바닥 센서가 검은선(교차로) 위인지 — 캘리브 정규화 기준(폴백: raw).
    bool onLine(int rawLeft, int rawRight);

    // 좌표 네비게이션 — 목적지 도달 시 true, 막혀서 중단되면 false 반환
    bool navigateTo(int8_t tx, int8_t ty);
    void rotateToHeading(Heading target);

    // 시리얼 명령 폴링 — 창고 유휴(다음 RFID 대기) 중에만 RunOnce 에서 호출됨.
    void HandleSerialCommand();
    // 현재 그리드 상태(장애물/창고/도시/현재 위치)를 ASCII 맵으로 시리얼 출력.
    void PrintStatusMap();

    // BFS 최단경로를 _pathX/_pathY 에 채움 (출발칸 제외, 목표 포함). 도달 불가 시 false.
    bool computeBfsPath(int8_t tx, int8_t ty);
};

#endif // CONTROLLER_H
