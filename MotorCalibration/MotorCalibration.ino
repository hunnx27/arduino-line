// 모터 캘리브레이션 EEPROM 기록 스케치
//
// 메인 스케치(sketch_apr10c.ino)의 drive()는 PWM 값에 _motorCalibR / _motorCalibL을
// 곱해 좌/우 바퀴 속도 편향을 보정한다. 값은 EEPROM 248(R) / 252(L)에 float로 저장.
//
// EEPROM 레이아웃 (메인 스케치 readData()와 동일):
//   248 | 4 | _motorCalibR (float)
//   252 | 4 | _motorCalibL (float)
//
// 사용법:
// 1) 아래 CALIB_R / CALIB_L 값 수정
// 2) 이 스케치 업로드 (setup()에서 한 번 기록하고 멈춤)
// 3) 시리얼 모니터(9600)에서 Before/After 결과 확인
// 4) 메인 스케치(sketch_apr10c.ino) 업로드 → 직진 테스트
// 5) 여전히 편향 있으면 값 조정 후 2)부터 반복
//
// 튜닝 규칙:
// - 왼쪽으로 휨  → 오른쪽 바퀴가 빠름 → CALIB_R 줄이기 (예: 1.0 → 0.95 → 0.90)
// - 오른쪽으로 휨 → 왼쪽 바퀴가 빠름  → CALIB_L 줄이기
// - 한쪽만 줄여서 보정 (양쪽 다 1.0 미만이면 최대 속도 손해)
// - 값이 0이거나 NaN이면 메인 스케치 drive()에서 PWM이 0이 되므로 주의

#include <Arduino.h>
#include <EEPROM.h>

// ↓↓↓ 이 두 값을 조정해서 업로드 ↓↓↓
#define CALIB_R 1.00f
#define CALIB_L 0.98f
// ↑↑↑

// 메인 스케치 START_ADDRESS = 240 기준 레이아웃
#define EEPROM_MOTOR_CALIB_R 248  // float, 4 bytes
#define EEPROM_MOTOR_CALIB_L 252  // float, 4 bytes

void setup() {
  Serial.begin(9600);
  while (!Serial) { ; }
  delay(200);

  Serial.println(F("=== Motor Calibration Writer ==="));

  float prevR = 0, prevL = 0;
  EEPROM.get(EEPROM_MOTOR_CALIB_R, prevR);
  EEPROM.get(EEPROM_MOTOR_CALIB_L, prevL);
  Serial.print(F("Before: R="));
  Serial.print(prevR, 4);
  Serial.print(F(" L="));
  Serial.println(prevL, 4);

  float newR = CALIB_R;
  float newL = CALIB_L;
  EEPROM.put(EEPROM_MOTOR_CALIB_R, newR);
  EEPROM.put(EEPROM_MOTOR_CALIB_L, newL);

  float verifyR = 0, verifyL = 0;
  EEPROM.get(EEPROM_MOTOR_CALIB_R, verifyR); 
  EEPROM.get(EEPROM_MOTOR_CALIB_L, verifyL);
  Serial.print(F("After:  R="));
  Serial.print(verifyR, 4);
  Serial.print(F(" L="));
  Serial.println(verifyL, 4);

  if (verifyR == newR && verifyL == newL) {
    Serial.println(F("OK: 기록 완료. 메인 스케치 업로드 후 테스트하세요."));
  } else {
    Serial.println(F("FAIL: 검증 불일치 — EEPROM 쓰기 실패"));
  }
}

void loop() {
}
