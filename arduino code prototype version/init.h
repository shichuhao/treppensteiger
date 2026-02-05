#ifndef INIT_H
#define INIT_H

#include <Arduino.h>
#include "MPU6050.h"

// === 修改1: float常量改为宏定义 (节省RAM) ===
#define PLATFORM_PITCH_MIN -500  // -5.0度×100
#define PLATFORM_PITCH_MAX 500   // 5.0度×100

// 超声波引脚
#define TRIG_PIN 12
#define ECHO_PIN 11

// 电机引脚
#define MOTOR_FL_IN1 2
#define MOTOR_FL_IN2 3
#define MOTOR_FL_PWM 5
#define MOTOR_FR_IN1 4
#define MOTOR_FR_IN2 7
#define MOTOR_FR_PWM 6
#define MOTOR_BL_IN1 8
#define MOTOR_BL_IN2 A0
#define MOTOR_BL_PWM 9
#define MOTOR_BR_IN1 A1
#define MOTOR_BR_IN2 A2
#define MOTOR_BR_PWM 10

// === 修改2: 全局变量类型优化 ===
extern MPU6050 mpu;
extern uint32_t initStartTime;  // long → uint32_t

// 函数声明
void setupPins();
void initializeMPU6050();
void calibrateMPU6050();
bool checkPlatformLevel();
void stopAllMotors();
uint16_t measureDistance();  // === 修改3: 返回值改为uint16_t (距离×10) ===
void motorsForward(uint8_t speed);
void motorsBackward(uint8_t speed);
void motorsRotateLeft(uint8_t speed);
void motorsRotateRight(uint8_t speed);

#endif
