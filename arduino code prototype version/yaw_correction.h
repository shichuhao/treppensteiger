#ifndef YAW_CORRECTION_H
#define YAW_CORRECTION_H

#include <Arduino.h>
#include "init.h"

// === 修改1: 所有float改为int16_t (角度×100) ===
extern int16_t referenceYaw;      // float → int16_t (节省2B)
extern int16_t currentYaw;        // float → int16_t (节省2B)
extern int16_t yawDeviation;      // float → int16_t (节省2B)
extern uint32_t lastYawUpdate;    // long → uint32_t

// 函数声明
void initYawCorrection();
void updateYaw();
int16_t getYawDeviation();  // === 修改2: 返回值 float → int16_t ===

#endif
