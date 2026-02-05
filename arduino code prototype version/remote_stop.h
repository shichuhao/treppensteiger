#ifndef REMOTE_STOP_H
#define REMOTE_STOP_H

#include <Arduino.h>

// === 修改1: 删除SoftwareSerial，改用硬件Serial ===
// 原: #include <SoftwareSerial.h>
// 原: extern SoftwareSerial espSerial;

extern uint32_t lastCmdTime;  // === 修改2: long → uint32_t ===

// 函数声明
void setupESP8266();  // === 修改3: 函数名更清晰 ===
bool checkRemoteStop();

#endif
