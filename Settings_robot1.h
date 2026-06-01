#ifndef SETTINGS_ROBOT1_H
#define SETTINGS_ROBOT1_H

// =====================================================================
// Settings_robot1.h — 로봇1 튜닝 가능 상수 모음 (맵1, col 0~3).
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

// -------------------- 맵 격리 경계 --------------------
// 횡단(최초 진입) 이후 이 열 미만으로 서쪽 이동 금지 → 자기 맵에 격리.
// 로봇1 은 단일 맵이므로 -128(무제한) = 격리 안 함.
#define NAV_MIN_X -128


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
#define CROSSING_APPROACH_MS          300
#define CROSSING_APPROACH_MS_CARGO    400

// [디버그] 사전 감속이 시작되는 순간(전속→감속 전환)에 짧은 부저음.
// CROSSING_APPROACH_MS 경과 시점을 귀로 확인용. 교차로 사이 1회 울림(비블로킹).
//   0 = 끔(실주행/대회).  1 = 켬.
#define DEBUG_APPROACH_TONE      1
#define DEBUG_APPROACH_TONE_HZ   784   // 솔(G5)


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
// 라인/교차로 검출은 흑/백 캘리브(EEPROM) 기반 정규화 값으로 판정 → 로봇·바닥 독립.
//   normalizeLeft/Right() 가 raw 를 흰=0 ~ 검=1000 으로 매핑. 양 바닥 센서가
//   LINEDETECT_NORM_MIN 이상이면 "검은선(교차로) 위"로 인정.
//   ↑ 올림: 옅은 선/반사 노이즈에 덜 민감(놓칠 위험 ↑). ↓ 내림: 더 민감(오검출 ↑).
//   DEBUG_TRACE=1 로 라인 위에서 찍히는 L_n/R_n 값을 보고 그 사이로 맞추면 됨.
#define LINEDETECT_NORM_MIN          700
// 안전장치: 흑-백 raw 격차가 이 값보다 작으면 캘리브 무효로 보고 아래 raw 폴백 사용
// (캘리브 안 된/EEPROM 초기화된 보드에서 정규화가 항상 1000 되는 오작동 방지).
#define LINEDETECT_CALIB_MIN_SPAN    100
// 캘리브 무효 시 폴백 raw 임계값 (구 LINEDETECT_THRESHOLD_MIN). 로봇 종속이므로 비상용.
#define LINEDETECT_RAW_FALLBACK      730

// IR 거리 센서 — analog read 가 이 값 미만이면 장애물 있음으로 판단.
// 측면용은 따로 (벽/측면 라인 오감지 줄이려면 더 낮게 설정 가능).
#define OBSTACLE_THRESHOLD       700
#define OBSTACLE_THRESHOLD_SIDE  700


// -------------------- 리프터 서보 --------------------
// 리프터 높이 = 서보 각도. 값은 기계 조립 기준의 오프셋 식으로 표기(실제 각도는 주석).
//   화물 충돌/안 닿으면 SERVO_UP ↑, 바닥 긁히면 SERVO_DOWN ↓ 식으로 조정.
#define SERVO_DOWN   (90  - 70)    // 내림 위치 = 20°
#define SERVO_UP     (180 - 150)   // 올림 위치 = 80°
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
// 화물 적재 시 가감속(_CARGO).
//   ★ 부드러움 = STEPS(스텝을 잘게 쪼갬, 증분 작아짐) 에서 나오고,
//     과회전 = STEP_MS(총 가감속 시간이 길수록 회전을 더 보탬) 에서 나온다. 둘을 분리해 생각.
//   부드럽게 유지하며 과회전만 줄이려면: STEPS 는 크게 두고 STEP_MS 만 ↓.
//     (편도 가감속 시간 ≈ STEPS × STEP_MS. 14×10 ≈ 140ms, 양끝 ≈ 280ms.)
//   화물 회전이 90° 넘게 돌면 STEP_MS_CARGO ↓ (또는 아래 *_DELAY_MS_CARGO ↓).
// START_PWM_CARGO: 화물이 무거워 출발이 멈칫하면 ↑, 시작이 거칠면 ↓.
#define TURN_RAMP_STEPS_CARGO     14
#define TURN_RAMP_STEP_MS_CARGO   10
#define TURN_START_PWM_CARGO      90
// 회전 정지 후 차체 흔들림이 가라앉을 때까지 대기(ms). 모든 회전(Pivot/TurnHalf)에 일괄 적용.
//   ↑ 올림: "회전 직후 라인/RFID 인식이 흔들린다, 헤딩이 들쭉날쭉하다" 싶을 때.
//   ↓ 내림: "회전 끝나고 너무 뜸 들인다(굼뜨다)" 싶을 때.
#define TURN_SETTLE_MS      150

// [디버그] 회전각 측정용 — 매 회전(안정화 후) 추가로 이만큼 더 멈춤(ms).
// 회전→정지 상태로 N초 멈추므로 각도기/눈으로 90°·180° 가 맞는지 측정하기 쉬움.
// ⚠ 실주행 시 반드시 0 으로 되돌릴 것 (안 그러면 회전마다 멈춤).
#define DEBUG_TURN_PAUSE_MS  0


// -------------------- 회전별 속도(PWM) / 각도(DELAY_MS) --------------------
// 회전 단위로 묶음: 각 회전의 [일반]/[화물] · 속도/각도가 한 자리에 모여 있다.
// 기억:  PWM = 속도 (천천히 돌리려면 ↓) ,  DELAY_MS = 각도 (더 돌리려면 ↑).
//   "같은 각도로 더 천천히" = PWM ↓ + DELAY ↑ 를 한 쌍으로 함께 조정.
//   화물(_CARGO)은 무게 때문에 보통 PWM 을 더 낮추고(슬라이드 방지) DELAY 로 각도 보충.
// (가감속/안정화/디버그 정지는 위 "회전 가감속(공통)" 섹션 — 모든 회전에 공통 적용.)


// ===== TurnHalf : 제자리 180° (왼쪽 후진 + 오른쪽 전진, 양 바퀴 같은 PWM) =====
//   PWM  ↑ 빨리(관성 overshoot 위험) / ↓ 천천히·정확.
//   DELAY  "180° 못 채움"→↑ / "넘게 돎"→↓.
#define TURNHALF_PWM             170   // [일반] 속도
#define TURNHALF_DELAY_MS        330   // [일반] 각도
#define TURNHALF_PWM_CARGO       120   // [화물] 속도
#define TURNHALF_DELAY_MS_CARGO  410   // [화물] 각도


// ===== PivotTurnLeft : 90° 좌회전 (왼쪽 후진 + 오른쪽 전진) =====
//   STRONG=바깥(빠른) 바퀴, WEAK=안쪽(느린) 바퀴.
//     두 값 차 ↑ → 돌며 앞으로 쏠림(완만한 호) / 차 ↓ → 제자리에 가깝게(같으면 순수 제자리).
//   DELAY = 90° 각도 ("못 돎"→↑ / "넘게 돎"→↓).
#define PIVOT_LEFT_STRONG_PWM         170   // [일반] 오른쪽(바깥) 바퀴 속도
#define PIVOT_LEFT_WEAK_PWM           120   // [일반] 왼쪽(안쪽) 바퀴 속도
#define PIVOT_LEFT_DELAY_MS           145   // [일반] 각도
#define PIVOT_LEFT_STRONG_PWM_CARGO   120   // [화물] 오른쪽(바깥) 바퀴 속도
#define PIVOT_LEFT_WEAK_PWM_CARGO      70   // [화물] 왼쪽(안쪽) 바퀴 속도
#define PIVOT_LEFT_DELAY_MS_CARGO     225   // [화물] 각도


// ===== PivotTurnRight : 90° 우회전 (왼쪽 전진 + 오른쪽 후진) =====
//   구조는 좌회전과 동일. 좌/우는 기구·모터 편차로 값이 살짝 다를 수 있음(한쪽만 미세조정).
#define PIVOT_RIGHT_STRONG_PWM        170   // [일반] 왼쪽(바깥) 바퀴 속도
#define PIVOT_RIGHT_WEAK_PWM          120   // [일반] 오른쪽(안쪽) 바퀴 속도
#define PIVOT_RIGHT_DELAY_MS          155   // [일반] 각도
#define PIVOT_RIGHT_STRONG_PWM_CARGO  120   // [화물] 왼쪽(바깥) 바퀴 속도
#define PIVOT_RIGHT_WEAK_PWM_CARGO     70   // [화물] 오른쪽(안쪽) 바퀴 속도
#define PIVOT_RIGHT_DELAY_MS_CARGO    230   // [화물] 각도


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


#endif // SETTINGS_ROBOT1_H
