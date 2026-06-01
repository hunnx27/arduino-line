#ifndef MOTORCALIB_SETTINGS_H
#define MOTORCALIB_SETTINGS_H
// 빌드 대상 선택. 이 파일의 ROBOT_ID 를 바꾸거나 -DROBOT_ID=2 빌드플래그로 덮어쓴다.
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
#endif // MOTORCALIB_SETTINGS_H
