#ifndef PLATFORM_TURN_H
#define PLATFORM_TURN_H

#include <Arduino.h>
#include "init.h"

// 状态定义
enum PlatformNavState {
  PN_INIT = 0,
  PN_MOVE_FORWARD,
  PN_DETECT_PLATFORM,
  PN_PLATFORM_LEVEL,
  PN_ROTATE_90,
  PN_CONFIRM_LEVEL,
  PN_COMPLETE
};

// === 修改1: 所有float改为int16_t/uint16_t ===
extern int16_t platformTurnYaw;          // float → int16_t (×100)
extern uint32_t platformTurnStartTime;   // long → uint32_t
extern uint32_t platformLevelStartTime;  // long → uint32_t

// === 修改2: 数组大小优化 ===
// 原: float wallDistanceReadings[10]
extern uint16_t wallDistanceReadings[5];  // 10→5, float→uint16_t (节省30B)
extern uint8_t wallDistanceIndex;         // uint16_t → uint8_t (节省1B)

// 函数声明
void handlePlatformTurn();

#endif
