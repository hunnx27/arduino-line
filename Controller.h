#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Servo.h>
#include <EEPROM.h>

#define START_ADDRESS 240

#define SERVO_DOWN  90-70
#define SERVO_UP    180-100
#define SERVO_DEF   SERVO_DOWN

#define LINEDETECT_THRESHOLD_MIN 730  // 교차로 인식용 블랙 임계값
#define BLANKDETECT_THERSHOLD_MAX 500

#define FORWARD   0
#define BACKWARD  1

// 장애물 인식 기준 거리값 (analog read 가 이 값 미만이면 장애물 있음으로 판단)
// 측면 센서는 진행 경로보다 옆을 보므로 보통 더 가깝게(=더 낮은 값) 트리거하도록
// 별도 상수로 분리. 초기값은 중앙과 동일하게 두고 실측 후 튜닝.
#define OBSTACLE_THRESHOLD       500
#define OBSTACLE_THRESHOLD_SIDE  500

// 전체 속도 스케일 (1.0 = 원래 속도, 낮을수록 느림)
// PWM은 이 비율만큼 줄고, 각도/거리를 유지해야 하는 delay는 1/SPEED_SCALE 배로 늘어남.
// 권장 범위 0.5 ~ 1.0. 0.5 미만은 정지마찰을 못 이겨 모터가 안 돌 수 있음.
#define SPEED_SCALE 0.6f

// 시작 RFID 가 놓인 그리드 좌표 (col=x, row=y) + 부팅 직후 로봇 heading.
// 시작 위치는 그리드 진입선 한 칸 뒤 (x = -1). 첫 forward 가 (0, 3) 으로 진입.
// lookupConn 에 (-1, 3) → CONN_E 특수 케이스 정의되어 있음.
#define INIT_START_X        -1
#define INIT_START_Y        3
#define INIT_START_HEADING  HD_EAST

// 창고 좌표 (col=x, row=y). 메인 라인 = col 2 layout.
// 사용자 prose: warehouse at (row 0, col 2) = 내 코드 (2, 0).
#define WAREHOUSE_X         2
#define WAREHOUSE_Y         0

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

// 진입 금지 교차점. navigateTo() 가 진입 직전에 그 방향을 마스킹해 우회한다.
// 장애물 회피(런타임 감지)와 달리 사전 정의된 정적 차단이라 후진 없이 perpendicular fallback 으로 빠진다.
// 추가 예: { {2, 3}, {3, 5} }
static const BlockedCell BLOCKED_CELLS[] = {
    // 진입 금지 셀을 여기에 추가. 아래는 빈 배열 회피용 sentinel — 그리드 밖이라 매칭되지 않음.
    {2, 3}, {3,5}
};
static const uint8_t BLOCKED_CELL_COUNT = sizeof(BLOCKED_CELLS) / sizeof(BlockedCell);

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

    // 시작용 RFID UID. 도시 UID 는 Controller.cpp 의 CITY_COORDS 테이블 참조.
    String s_strRFIDUidForStart = String("647AB573");

    MFRC522 mfrc522;
    Servo servo;
    String  strRFID;
    bool isBusy = false;

    POSITION currentPosition = eInitialPosition;
    APP_STATE state = STATE_NONE;

    // 현재 좌표/방향. 부팅 직후엔 시작 RFID 위치에서 시작 (eInitialPosition 핸들러에서 재설정).
    Pose currentPose = {INIT_START_X, INIT_START_Y, INIT_START_HEADING};

    // 네비게이션 보조 상태
    bool _preciseRealign = true;            // LineTracer 정렬 dance 수행 여부 (y=0/7 에서만 true)
    int8_t _blockedAtX = -128;              // 직전에 장애물로 막힌 좌표 (-128 = 없음)
    int8_t _blockedAtY = -128;
    uint8_t _blockedDirBit = 0;             // 막힌 방향의 CONN_* 비트

    // DFS 백트래킹용 경로 스택. navigateTo 시작 시 currentPose 를 push 하고,
    // 한 칸 전진할 때마다 push, 막다른 길에서 pop 하며 부모 칸으로 물리 후진.
    // 동시에 사이클 방지 — 이미 스택에 있는 칸 방향은 forward 후보에서 제외.
    // 4×8 그리드 단순 경로 최대 32 + 그리드 밖 시작 1 = 33. 여유로 34.
    static const uint8_t NAV_PATH_MAX = 34;
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

    // RFID Read
    bool RFIDRead();

    // Functions for Drive
    bool LineTracer(uint16_t nTargetLineCounter);
    // precise=true 시 라인 도달 후 후진-전진 정렬 dance 수행 (정확한 turn 위치 확보).
    // 좌표 네비게이션은 y=0/y=7 에서만 precise 사용. 반환값: false 면 장애물로 인해 중단.
    bool DoLineTrace(uint16_t targetCount, bool precise = true);
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

    // 좌표 네비게이션 — 목적지 도달 시 true, 막혀서 중단되면 false 반환
    bool navigateTo(int8_t tx, int8_t ty);
    void rotateToHeading(Heading target);

    // 스택에 있는 칸으로 향하는 방향 비트를 conn 에서 제거 (사이클 방지 + 부모 자동 제외).
    uint8_t maskCellsOnPath(int8_t x, int8_t y, uint8_t conn);
};

#endif // CONTROLLER_H
