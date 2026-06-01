#ifndef MOTORCALIB_SETTINGS_ROBOT2_H
#define MOTORCALIB_SETTINGS_ROBOT2_H

// 로봇2 모터 캘리브값.
//
// 튜닝 규칙:
// - 왼쪽으로 휨  → 오른쪽 바퀴가 빠름 → CALIB_R 줄이기 (예: 1.0 → 0.95 → 0.90)
// - 오른쪽으로 휨 → 왼쪽 바퀴가 빠름  → CALIB_L 줄이기
// - 한쪽만 줄여서 보정 (양쪽 다 1.0 미만이면 최대 속도 손해)
// - 값이 0이거나 NaN이면 메인 스케치 drive()에서 PWM이 0이 되므로 주의
#define CALIB_R 1.00f  // TODO: 로봇2 현장 캘리브값 — 직진 테스트로 측정 후 채울 것
#define CALIB_L 1.00f  // TODO: 로봇2 현장 캘리브값 — 직진 테스트로 측정 후 채울 것

#endif // MOTORCALIB_SETTINGS_ROBOT2_H
