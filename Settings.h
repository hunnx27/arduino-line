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
// 첫 출발 시 RFID 검증 방식.
//   1 → START_RFID_UID 와 일치하는 시작 카드여야만 출발.
//   0 → 아무 RFID 나 인식되면 바로 출발 (디버깅/시연용).
#define REQUIRE_START_RFID_MATCH  0


// -------------------- 물류창고 위치 --------------------
// col=x, row=y. 메인 라인은 col 2 layout.
#define WAREHOUSE_X 2
#define WAREHOUSE_Y 0


// -------------------- 도시 위치 (RFID UID → 좌표 매핑) --------------------
// 도시는 모두 row 7. UID 빈 문자열인 항목은 미등록 (실 태그 부착 후 채우기).
// 새 도시 추가 시 lookupCityCoord() 가 자동 매핑.
static const CityCoord CITY_COORDS[] = {
    {"647AB573",         0, 7},  // Seoul   (col 0)
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
#define CROSSING_PASS_POWER    70

// 교차로 도착 전 사전 감속 PWM — 직전 교차로 이후 일정 시간 지나면 base 를 이 값으로 낮춤.
#define CROSSING_APPROACH_POWER         90
#define CROSSING_APPROACH_POWER_CARGO   80

// 사전 감속 임계값 (ms) — 직전 교차로 이후 이 시간 지나면 감속 시작.
// 한 칸 평균 이동 시간의 ~70% 적당. 화물 적재 시 더 느리므로 따로.
#define CROSSING_APPROACH_MS          500
#define CROSSING_APPROACH_MS_CARGO    600


// -------------------- PD 제어 (라인 트레이서) --------------------
// 진동 시: Kd ↓ 또는 Kp ↓. 곡선 lag 시: Kp ↑.
// 일반적으로 Kd 는 Kp 의 5~30 배 사이에서 시작.
#define PID_KP               0.04f
#define PID_KD               0.3f
// 조향 보정량 saturation. leftPWM = base+correction, rightPWM = base-correction
// 이므로 좌우 바퀴 PWM 차이는 최대 2×이 값. 라인에서 아무리 벗어나도(또는 D항이
// 순간 튀어도) 이 폭 이상은 안 꺾는다 → 주 역할은 D항 스파이크 억제.
//   ↑ 올림(예 50): 곡선/급이탈에서 라인 못 따라가고 바깥으로 흘러나갈 때.
//                  (Kp 올려도 여기 걸리면 소용없음 — 이 상한부터 풀어야 함)
//   ↓ 내림(예 20): 직선에서 좌우 지그재그/진동(overshoot) 심할 때.
// 튜닝 순서: Kp/Kd 를 먼저 맞추고, saturation 이 실제로 걸릴 때만 마지막에 조정.
#define PID_MAX_CORRECTION   30.0f


// -------------------- 디버그 출력 --------------------
// LineTrace 의 센서 raw/정규화 값 주기 출력 토글.
//   0 → OFF (실주행/대회용). 핫 루프에서 9600baud ~80바이트 출력은 TX 버퍼(64B)를
//        넘겨 매 200ms 마다 수십 ms 동안 루프를 멈춤 → 그 "장님 구간"에 교차로가
//        겹치면 미인식. OFF 면 이 구간이 사라져 교차로 인식이 안정됨.
//   1 → ON  (센서 튜닝/캘리브 확인용). 주행 정확도는 떨어질 수 있음.
#define DEBUG_TRACE 0


// -------------------- 라인 / 장애물 센서 임계값 --------------------
// 양 바닥 센서가 이 값 이상이면 교차로 검출
#define LINEDETECT_THRESHOLD_MIN     730
#define BLANKDETECT_THERSHOLD_MAX    500

// IR 거리 센서 — analog read 가 이 값 미만이면 장애물 있음으로 판단.
// 측면용은 따로 (벽/측면 라인 오감지 줄이려면 더 낮게 설정 가능).
#define OBSTACLE_THRESHOLD       600
#define OBSTACLE_THRESHOLD_SIDE  600


// -------------------- 리프터 서보 --------------------
// 리프터 높이 = 서보 각도. 값은 기계 조립 기준의 오프셋 식으로 표기(실제 각도는 주석).
//   화물 충돌/안 닿으면 SERVO_UP ↑, 바닥 긁히면 SERVO_DOWN ↓ 식으로 조정.
#define SERVO_DOWN   (90  - 70)    // 내림 위치 = 20°
#define SERVO_UP     (180 - 140)   // 올림 위치 = 80°
#define SERVO_DEF    SERVO_DOWN     // 기본(부팅) 위치
// 부드러운 이동: 목표각으로 한 번에 쏘지 않고 단계적으로 슬루.
//   SERVO_STEP_DEG = 스텝당 각도 (작을수록 더 부드럽고 느림)
//   SERVO_STEP_MS  = 스텝 간 대기(ms) (클수록 더 천천히)
//   이동 총시간 ≈ (각도차 / STEP_DEG) × STEP_MS.
//     예) 20°→80° = 60/2 × 15 ≈ 450ms.
#define SERVO_STEP_DEG   2
#define SERVO_STEP_MS    15
// 목표 도달 후 detach 전 안착 대기(ms).
#define SERVO_SETTLE_MS  120


// -------------------- 회전 가감속 (부드러운 회전) --------------------
// [핵심 개념 — 먼저 읽기]
//   • PWM = 모터 속도/힘 (0~255). 클수록 바퀴가 빠르고 세게 돈다.
//   • holdMs(delay) = 그 속도를 유지하는 시간 → 사실상 "얼마나 도느냐(회전각)"를 정함.
//   • 회전각 ≈ PWM × 시간 (속도 × 시간 = 돈 양).
//       → 덜 돌게 하려면 delay ↓ (또는 PWM ↓).   더 돌게 하려면 delay ↑ (또는 PWM ↑).
//
// 모든 회전(TurnHalf 180° / PivotTurnLeft·Right 90°)은 사다리꼴 속도 프로파일:
//   가속(TURN_START_PWM→cruise) → 정속(cruise, holdMs) → 감속(cruise→TURN_START_PWM) → 정지.
//   예전처럼 정지→곧장 풀파워로 튀거나 풀파워→급정지하지 않아 부드럽다.
//   cruise PWM·holdMs 는 아래 기존 상수(TURNHALF_PWM, PIVOT_*_PWM, *_DELAY_MS)를 그대로 사용.
//
//   TURN_RAMP_STEPS   : 가속/감속을 몇 단계로 쪼개나.
//       ↑ 올림(예 8→14): 더 잘게 → 더 부드럽게. "회전 시작/끝이 거칠고 덜컥인다" 싶을 때.
//       ↓ 내림(예 8→4) : 빠릿하게(덜 부드럽). "회전이 굼떠서 답답하다" 싶을 때.
//   TURN_RAMP_STEP_MS : 한 단계 유지 시간(ms).
//       ↑ 올림(예 12→20): 가감속을 더 완만하게. "정지할 때 끼익/휘청거린다" 싶을 때.
//       ↓ 내림(예 12→6) : 가감속을 빨리 끝냄. "회전 한 번이 너무 오래 걸린다" 싶을 때.
//   TURN_START_PWM    : 가감속이 시작/끝나는 PWM(= 바퀴가 겨우 도는 최소 속도).
//       ↑ 올림(예 90→110): "출발 때 바퀴가 안 돌고 멈칫한다"(저PWM에서 정지마찰에 묶임).
//       ↓ 내림(예 90→70) : "출발이 홱 튀듯 거칠다"(시작 속도가 너무 높음).
//
//   가감속 편도 시간 ≈ TURN_RAMP_STEPS × TURN_RAMP_STEP_MS (기본 8×12 ≈ 96ms, 양끝 합 ≈ 192ms).
// ⚠ 가감속 구간도 회전을 "보탠다". STEPS/STEP_MS 를 바꾸면 회전각이 같이 변함.
//   → 이 세 값을 손대면 아래 *_DELAY_MS 로 90°/180° 를 다시 맞춰야 함.
//   (실제로 가감속 도입 후 TurnHalf 가 더 돌아서 TURNHALF_DELAY_MS 를 450→330 으로 낮춘 상태.)
#define TURN_RAMP_STEPS     8
#define TURN_RAMP_STEP_MS   12
#define TURN_START_PWM      90
// 회전 정지 후 차체 흔들림이 가라앉을 때까지 대기(ms). 모든 회전(Pivot/TurnHalf)에 일괄 적용.
//   ↑ 올림: "회전 직후 라인/RFID 인식이 흔들린다, 헤딩이 들쭉날쭉하다" 싶을 때.
//   ↓ 내림: "회전 끝나고 너무 뜸 들인다(굼뜨다)" 싶을 때.
#define TURN_SETTLE_MS      150


// -------------------- 회전 속도(cruise PWM) / 회전각(holdMs) --------------------
// 위 가감속 프로파일의 "정속 구간" 값들. PWM=속도, DELAY_MS=각도(시간) 임을 기억.
//
// TurnHalf — 제자리 180° 회전 (왼쪽 후진 + 오른쪽 전진, 양 바퀴 같은 PWM).
//   TURNHALF_PWM      : 회전 속도.
//       ↑ 올림: 빨리 돎(시간 ↓) 단 관성으로 지나칠(overshoot) 위험 ↑.
//       ↓ 내림: 천천히·정확. "180° 지점을 자꾸 지나친다" 싶으면 PWM 을 낮춰보라.
//   TURNHALF_DELAY_MS : 회전각(정속 유지 시간). "180° 다 못 돈다"→ ↑ / "더 돈다(현 증상)"→ ↓.
//       예) 200° 처럼 더 돌면 330→300 식으로 내려 정확히 반바퀴 될 때까지 미세조정.
// 화물(_CARGO): PWM 을 낮춰(원심력으로 팔레트가 밀려나는 것 방지) 각속도 ↓ →
//   느려진 만큼 DELAY 를 늘려(↑) 180° 를 다시 채운다.
#define TURNHALF_PWM             170
#define TURNHALF_DELAY_MS        330
#define TURNHALF_PWM_CARGO       150
#define TURNHALF_DELAY_MS_CARGO  390

// (구 PIVOT_KICK_* 킥스타트는 RampTurn 의 TURN_START_PWM 가감속으로 대체됨)

// Pivot turn — 90° 방향 전환 (한 칸에서 좌/우로 꺾기). 양 바퀴를 반대로 돌리되
// 강/약 PWM 차이를 둔다 → 회전 중심이 약한 바퀴 쪽으로 치우친 제자리 회전.
//   STRONG_PWM : 빠른(바깥) 바퀴 속도.   WEAK_PWM : 느린(안쪽) 바퀴 속도.
//   두 값 차이가 클수록  → 돌면서 앞으로 더 쏠림(완만한 호를 그림).
//   두 값 차이가 작을수록 → 더 제자리에 가깝게 돎(둘이 같으면 순수 제자리 회전).
//   좌/우는 기구 편차 보정용으로 살짝 비대칭. 한쪽만 덜/더 꺾이면 그쪽 STRONG 만 미세조정.
// 화물(_CARGO): PWM ↓(슬라이드 방지) → 느려진 만큼 PIVOT_DELAY_MS_CARGO 로 각도 보충.
#define PIVOT_LEFT_STRONG_PWM         170   // 좌회전: 오른쪽(바깥) 바퀴
#define PIVOT_LEFT_WEAK_PWM           90    // 좌회전: 왼쪽(안쪽) 바퀴
#define PIVOT_LEFT_STRONG_PWM_CARGO   150
#define PIVOT_LEFT_WEAK_PWM_CARGO     80

#define PIVOT_RIGHT_STRONG_PWM        170   // 우회전: 왼쪽(바깥) 바퀴
#define PIVOT_RIGHT_WEAK_PWM          90    // 우회전: 오른쪽(안쪽) 바퀴
#define PIVOT_RIGHT_STRONG_PWM_CARGO  150
#define PIVOT_RIGHT_WEAK_PWM_CARGO    80

// 회전각(정속 유지 시간). "90° 못 돈다"→ ↑ / "90° 넘게 돈다"→ ↓.
// 피벗은 정속 구간이 짧아 가감속 비중이 커서, STEPS/STEP_MS 를 바꾸면 특히 민감 —
// 그 경우 벤치에서 90° 다시 맞출 것.
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
