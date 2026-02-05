#ifndef INIT_H
#define INIT_H

#include <Arduino.h>

#include <SoftwareSerial.h>
#include "MPU6050.h"
#include <avr/wdt.h>
#include "go_up.h"

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
// HC-SR04 Ultrasonic Sensor
#define HC_TRIG_PIN 3
#define HC_ECHO_PIN 2

// MPU6050 IMU Sensor
#define MPU6050_ADDRESS 0x68

// L298N Motor Driver 1 (Left)
#define L298N_1_IN1 4
#define L298N_1_IN2 5
#define L298N_1_ENA 9
#define L298N_1_IN3 6
#define L298N_1_IN4 7
#define L298N_1_ENB 10

// L298N Motor Driver 2 (Right)
#define L298N_2_IN1 11
#define L298N_2_IN2 12
#define L298N_2_ENA 3
#define L298N_2_IN3 13
#define L298N_2_IN4 8
#define L298N_2_ENB 6

// ESP8266 WiFi Module
#define ESP_RX_PIN 10
#define ESP_TX_PIN 11
#define ESP_BAUD 115200


//Watching dog
#define WATCHDOG_TIMEOUT WDTO_2S

// ============================================================================
// GLOBAL VARIABLES (EXTERN)
// ============================================================================
extern MPU6050 mpu;
extern SoftwareSerial espSerial;

extern volatile bool mpuReady;
extern volatile bool hcReady;
extern volatile bool espReady;
extern volatile bool motorsReady;

extern const int PLATFORM_PITCH_MIN;
extern const int PLATFORM_PITCH_MAX;

extern unsigned long initStartTime;
extern uint8_t initErrorCode;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================
void initializeGPIO();
void initializePWM();
void initializeMPU6050();
void initializeHCSR04();
void initializeL298N();
void initializeESP8266();
void setupInterrupts();
void setupWatchdog();
void printInitStatus();

#endif // INIT_H
