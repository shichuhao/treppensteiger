#ifndef DOWNHILL_H
#define DOWNHILL_H

#include <Arduino.h>
#include "init.h"

// 状态定义
enum DownhillState {
  DH_INIT = 0,
  DH_APPROACH,
  DH_DESCEND,
  DH_STABILIZE,
  DH_COMPLETE
};

// === 修改1: 所有float改为int16_t ===
extern int16_t maxDownhillPitch;   // float → int16_t (×100)
extern uint32_t downhillStartTime; // long → uint32_t

// 函数声明
void handleDownhill();

#endif
