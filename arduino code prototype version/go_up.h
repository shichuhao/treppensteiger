#ifndef GO_UP_H
#define GO_UP_H

#include <Arduino.h>
#include "init.h"

// 状态定义
enum GoUpState {
  GU_INIT = 0,
  GU_MOVE_FORWARD,
  GU_CHECK_STAIR,
  GU_CLIMB,
  GU_STABILIZE,
  GU_COMPLETE
};

// === 修改1: 所有float改为int16_t/uint16_t ===
extern uint16_t currentDistance;   // float → uint16_t (距离×10)
extern int16_t currentPitch;       // float → int16_t (角度×100)
extern int16_t currentRoll;        // float → int16_t (角度×100)
extern uint32_t lastSensorUpdate;  // long → uint32_t

// 函数声明
void handleGoUp();
uint16_t measureDistanceFiltered();  // === 修改2: 返回值优化 ===

#endif
