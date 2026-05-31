#ifndef SETTINGS_H
#define SETTINGS_H

// =====================================================================
// Settings.h — 모든 튜닝 가능 상수 모음.
// 거동 조정은 여기서. 코드 로직 수정 없이 값만 바꾸고 재컴파일/업로드.
//
// 포함 순서 (Controller.h 안):
//   1) Arduino + 라이브러리 헤더
//   2) Heading 열거형, BlockedCell / CityCoord 구조체 정의
//   3) #include "Settings.h"  ← 이 헤더. 위 타입을 참조함.
//   4) class Controller (Settings 매크로를 기본값으로 사용)
// 이 순서가 깨지면 Settings.h 안의 array literal 및 INIT_START_HEADING 가 컴파일 안 됨.
// =====================================================================


// -------------------- 전체 속도 스케일 --------------------
// 1.0 = 원래 속도. PWM 은 이 비율만큼 줄고, 각도/거리 유지 delay 는 1/SPEED_SCALE 배.
// 권장 0.5 ~ 1.0. 0.5 미만은 정지마찰로 모터 안 돌 수 있음.
#define SPEED_SCALE 1.0f


// -------------------- 그리드 크기 --------------------
// 격자 col(x) / row(y) 수. 변경 시 CITY_COORDS, BLOCKED_CELLS 좌표 검토 필요.
#define GRID_COLS 4
#define GRID_ROWS 8


// -------------------- 출발지 (시작 RFID) 위치 --------------------
// 시작 RFID 패드. 격자 외부 한 칸 뒤 (x = -1). 첫 forward 가 (0, 3) 진입.
// lookupConn 안에 (-1, 3) → CONN_E 특수 케이스 정의돼 있음.
#define INIT_START_X        -1
#define INIT_START_Y         3
#define INIT_START_HEADING  HD_EAST   // ← Heading enum 이 먼저 정의돼 있어야 함

// 시작 RFID UID 카드 식별. 도시 UID 는 CITY_COORDS 참조.
#define START_RFID_UID "647AB573"


// -------------------- 물류창고 위치 --------------------
// col=x, row=y. 메인 라인은 col 2 layout.
#define WAREHOUSE_X 2
#define WAREHOUSE_Y 0


// -------------------- 도시 위치 (RFID UID → 좌표 매핑) --------------------
// 도시는 모두 row 7. UID 빈 문자열인 항목은 미등록 (실 태그 부착 후 채우기).
// 새 도시 추가 시 lookupCityCoord() 가 자동 매핑.
static const CityCoord CITY_COORDS[] = {
    {"",         0, 7},  // Seoul   (col 0)
    {"",         1, 7},  // Incheon (col 1)
    {"",         2, 7},  // Sejong  (col 2 — 메인 라인. 창고 바로 아래)
    {"148EC573", 3, 7},  // Daejeon (col 3)
};
static const uint8_t CITY_COORD_COUNT = sizeof(CITY_COORDS) / sizeof(CityCoord);


// -------------------- 접근 제어 (정적 차단) 셀 --------------------
// 격자 layout 변경 시 같이 수정. 추가는 { {x1,y1}, {x2,y2}, ... } 형식.
// 런타임 동적 차단(g_dynBlocked)과는 별개 — 컴파일 시 결정되는 영구 차단.
static const BlockedCell BLOCKED_CELLS[] = {
    {2, 3}, {3, 5}, {1, 5}
};
static const uint8_t BLOCKED_CELL_COUNT = sizeof(BLOCKED_CELLS) / sizeof(BlockedCell);


// -------------------- 모터 PWM (직진 / 감속) --------------------
// 일반 주행 base PWM (직진 P/D 제어용)
#define MOTOR_POWER           110
// 화물(팔레트) 적재 시 base PWM — 무거우니 약간 낮춤
#define MOTOR_POWER_CARGO      90

// 교차로 통과 시 (양 센서 검출 상태) 감속 PWM — overshoot 방지.
// 50 미만 비추 (정지마찰로 멈출 위험).
#define CROSSING_PASS_POWER    110

// 교차로 도착 전 사전 감속 PWM — 직전 교차로 이후 일정 시간 지나면 base 를 이 값으로 낮춤.
#define CROSSING_APPROACH_POWER         70
#define CROSSING_APPROACH_POWER_CARGO   70

// 사전 감속 임계값 (ms) — 직전 교차로 이후 이 시간 지나면 감속 시작.
// 한 칸 평균 이동 시간의 ~70% 적당. 화물 적재 시 더 느리므로 따로.
#define CROSSING_APPROACH_MS          400
#define CROSSING_APPROACH_MS_CARGO    700


// -------------------- PD 제어 (라인 트레이서) --------------------
// 진동 시: Kd ↓ 또는 Kp ↓. 곡선 lag 시: Kp ↑.
// 일반적으로 Kd 는 Kp 의 5~30 배 사이에서 시작.
#define PID_KP               0.03f
#define PID_KD               0.3f
// 보정량 saturation. 합산 후 절대값이 이 값 넘으면 클램프.
#define PID_MAX_CORRECTION   35.0f


// -------------------- 라인 / 장애물 센서 임계값 --------------------
// 양 바닥 센서가 이 값 이상이면 교차로 검출
#define LINEDETECT_THRESHOLD_MIN     730
#define BLANKDETECT_THERSHOLD_MAX    500

// IR 거리 센서 — analog read 가 이 값 미만이면 장애물 있음으로 판단.
// 측면용은 따로 (벽/측면 라인 오감지 줄이려면 더 낮게 설정 가능).
#define OBSTACLE_THRESHOLD       600
#define OBSTACLE_THRESHOLD_SIDE  600


// -------------------- 리프터 서보 각도 --------------------
#define SERVO_DOWN   (90  - 70)
#define SERVO_UP     (180 - 100)
#define SERVO_DEF    SERVO_DOWN


// -------------------- 회전 PWM / 타이밍 --------------------
// TurnHalf (180° 제자리 회전) — 양 바퀴 역방향 같은 PWM.
// 화물 적재 시 각속도 ↓ + delay ↑ 로 팔레트 슬라이드 방지.
#define TURNHALF_PWM             170
#define TURNHALF_DELAY_MS        450
#define TURNHALF_PWM_CARGO       150
#define TURNHALF_DELAY_MS_CARGO  520

// Pivot turn 킥스타트 [1] — 정지마찰 극복용 초기 부스트 (양쪽 동일 PWM).
// 정지 상태에서 모터가 안 도는 문제 있을 때 ↑. 너무 강하면 jerky.
#define PIVOT_KICK_PWM    170
#define PIVOT_KICK_MS     50

// Pivot turn 회전 구간 [2] — 강/약 PWM 차이로 시간 회전. 회전각은 delay 로 결정.
// 좌/우 비대칭 — 좌회전이 약간 더 빨라 강측 180 (우회전 강측 170).
// 화물 적재 시 PWM ↓ (각속도 ↓ → 팔레트 슬라이드 방지). 각도 유지 위해 delay ↑.
#define PIVOT_LEFT_STRONG_PWM         170   // 좌회전 시 오른쪽 바퀴
#define PIVOT_LEFT_WEAK_PWM           90    // 좌회전 시 왼쪽 바퀴
#define PIVOT_LEFT_STRONG_PWM_CARGO   150
#define PIVOT_LEFT_WEAK_PWM_CARGO     80

#define PIVOT_RIGHT_STRONG_PWM        170   // 우회전 시 왼쪽 바퀴
#define PIVOT_RIGHT_WEAK_PWM          90    // 우회전 시 오른쪽 바퀴
#define PIVOT_RIGHT_STRONG_PWM_CARGO  150
#define PIVOT_RIGHT_WEAK_PWM_CARGO    80

#define PIVOT_DELAY_MS                140
#define PIVOT_DELAY_MS_CARGO          170


// -------------------- 네비게이션 보조 --------------------
// 한 navigateTo 안에서 각 칸 최대 방문 횟수 (사이클 방지).
// 초과 시 그 칸 진입 차단 → 자연스럽게 데드엔드 처리.
#define VISIT_LIMIT 2

// 동적 차단 셀(g_dynBlocked) 최대 저장 개수. 부팅마다 RAM 리셋.
#define MAX_DYN_BLOCKED 8


// -------------------- EEPROM NavLog 디버그 버퍼 --------------------
// 매 Eval / DeadEnd / DynBlock 이벤트 기록. 부팅 시 자동 dump → clear.
// 엔트리당 8 바이트. 엔트리 수 변경 시 EEPROM 영역 [0, ENTRIES*8 + 1] 차지.
// 캘리브 영역 240~255 와 안 겹치게 (ENTRIES * 8 ≤ 238 권장).
#define NAVLOG_ENTRIES 25


#endif // SETTINGS_H
