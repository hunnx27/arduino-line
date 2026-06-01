#ifndef SETTINGS_H
#define SETTINGS_H

// =====================================================================
// Settings.h — 빌드 대상 로봇 선택 셀렉터.
// 실제 튜닝/맵 값은 Settings_robot1.h / Settings_robot2.h 에 있다.
//
// 전환 방법:
//   1) 아래 ROBOT_ID 를 1 또는 2 로 바꿔 업로드, 또는
//   2) 파일 수정 없이 빌드플래그: -DROBOT_ID=2
//      (예) arduino-cli compile --fqbn arduino:avr:nano \
//             --build-property "build.extra_flags=-DROBOT_ID=2" .
// =====================================================================

#ifndef ROBOT_ID
  #define ROBOT_ID 1
#endif

#if   ROBOT_ID == 1
  #include "Settings_robot1.h"
#elif ROBOT_ID == 2
  #include "Settings_robot2.h"
#else
  #error "ROBOT_ID must be 1 or 2"
#endif

#endif // SETTINGS_H
