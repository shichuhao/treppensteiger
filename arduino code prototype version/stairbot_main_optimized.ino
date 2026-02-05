/*
 * ============================================================================
 * Stair-Climbing Robot - Main Control Program
 * Arduino UNO R3 with MPU6050, HC-SR04, ESP8266, 2x L298N Motor Drivers
 * 
 * Integrated Modules:
 * - init: System Initialization
 * - Go_up: Uphill climbing control
 * - Downhill: Downhill descent control
 * - Platform_turn: Platform navigation and turning
 * - Remote_stop: Remote control via ESP8266
 * - Yaw_correction: Gyroscope-based yaw correction
 * 
 * Author: Gruppe 2
 * Date: 2026-02-05
 * Version: 2.0.0
 * ============================================================================
 */

#include "init.h"
#include "go_up.h"
#include "downhill.h"
#include "platform_turn.h"
#include "yaw_correction.h"
#include "remote_stop.h"


// ===========================
// State machine definition — overall high-level modes
// ===========================
enum SystemState {
  STATE_INIT = 0,        // Initialization (only defined here; actual logic is done in setup -> switches to IDLE)
  STATE_IDLE,            // Idle, waiting for command
  STATE_GO_UP,           // Stair climbing mode
  STATE_PLATFORM_TURN,   // Turning on platform mode
  STATE_DOWNHILL,        // Descending stairs/slope mode
  STATE_EMERGENCY_STOP   // Emergency stop mode
};


// Current system state
SystemState currentState = STATE_INIT;
// Whether the system is ready (platform level, etc.)
bool systemReady = false;
// Last timestamp (ms) printed for system status
uint32_t lastStatusPrint = 0;  // use uint32_t for compatibility with millis()


void setup() {

  // Initialize serial port for debug output and command input
  Serial.begin(115200);  // Higher baud rate = faster transfer

  // Startup message (F() macro stores string in Flash to save RAM)
  Serial.println(F("\n=== Stairbot Starting ==="));
  Serial.println(F("Optimized for UNO 2KB RAM"));

  // Initialize motor and sensor IO pins
  setupPins();
  delay(100);

  // Initialize MPU6050 (IMU)
  Serial.println(F("Init MPU6050..."));
  initializeMPU6050();
  delay(500);

  // Calibrate MPU6050 (offsets, noise, etc.)
  Serial.println(F("Calibrating MPU..."));
  calibrateMPU6050();
  delay(500);

  // Initialize ESP8266 module for remote control / stop command
  Serial.println(F("Init ESP8266..."));
  setupESP8266();
  delay(500);

  // Initialize yaw correction module (record initial heading, etc.)
  Serial.println(F("Init yaw correction..."));
  initYawCorrection();
  delay(200);

  // Check if the platform is level to ensure safe start
  Serial.println(F("Checking level..."));
  if (checkPlatformLevel()) {
    Serial.println(F("✓ Platform level"));
    systemReady = true;
    // If level, switch to idle state, waiting for commands
    currentState = STATE_IDLE;
  } else {
    Serial.println(F("✗ Platform not level!"));
    // Not level, enter emergency stop state
    currentState = STATE_EMERGENCY_STOP;
  }

  Serial.println(F("\n=== System Ready ==="));
  // Print initial system status (attitude, distance, state, etc.)
  printSystemStatus();
}


void loop() {
  // ========= 1. Check remote emergency stop first =========
  // e.g., stop command received via ESP8266 or serial
  if (checkRemoteStop()) {
    currentState = STATE_EMERGENCY_STOP;
    Serial.println(F("Emergency stop activated!"));
  }

  // ========= 2. Execute logic for current system state =========
  switch (currentState) {

    case STATE_IDLE:
      // Idle: listen for commands via Serial, no movement
      if (Serial.available()) {
        char cmd = Serial.read();

        if (cmd == 'U' || cmd == 'u') {
          // Stair climbing mode
          Serial.println(F("\n>>> GO UP MODE"));
          currentState = STATE_GO_UP;

        } else if (cmd == 'T' || cmd == 't') {
          // Platform turning mode
          Serial.println(F("\n>>> PLATFORM TURN MODE"));
          currentState = STATE_PLATFORM_TURN;

        } else if (cmd == 'D' || cmd == 'd') {
          // Descending stairs/slope mode
          Serial.println(F("\n>>> DOWNHILL MODE"));
          currentState = STATE_DOWNHILL;

        } else if (cmd == 'S' || cmd == 's') {
          // Manual emergency stop
          Serial.println(F("\n>>> STOP"));
          currentState = STATE_EMERGENCY_STOP;
        }
      }
      break;


    case STATE_GO_UP:
      // Stair climbing main logic (implemented in go_up.cpp)
      handleGoUp();
      break;


    case STATE_PLATFORM_TURN:
      // Platform turning logic (implemented in platform_turn.cpp)
      handlePlatformTurn();
      break;


    case STATE_DOWNHILL:
      // Descending stairs/slope main logic (implemented in downhill.cpp)
      handleDownhill();
      break;


    case STATE_EMERGENCY_STOP:
      // Emergency stop: immediately halt all motors
      stopAllMotors();
      Serial.println(F("System stopped. Send 'R' to resume."));

      // In STOP state, only 'R' / 'r' resumes to IDLE
      if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 'R' || cmd == 'r') {
          Serial.println(F("Resuming..."));
          currentState = STATE_IDLE;
        }
      }
      // Add delay to reduce serial flooding
      delay(500);
      break;
  }

  // ========= 3. Periodically print system status =========
  // Print every 5 seconds (instead of 1) to reduce serial usage
  if (millis() - lastStatusPrint > 5000) {
    printSystemStatus();
    lastStatusPrint = millis();
  }

  // Small delay to reduce CPU load and avoid excessive polling
  delay(10);
}


// Print current system status (attitude angles, distance, state, etc.)
void printSystemStatus() {
  Serial.println(F("\n--- Status ---"));

  // Import pitch, roll, and distance variables from other files
  extern int16_t currentPitch, currentRoll;
  extern uint16_t currentDistance;

  // Print pitch and roll (stored as ×100; divide by 100 to convert to degrees)
  Serial.print(F("Pitch: "));
  Serial.print(currentPitch / 100);
  Serial.print(F("° Roll: "));
  Serial.print(currentRoll / 100);

  // Print distance (stored as ×10; divide by 10 to show in cm)
  Serial.print(F("° Dist: "));
  Serial.print(currentDistance / 10);
  Serial.println(F(" cm"));

  // Print current state machine mode
  Serial.print(F("State: "));
  switch (currentState) {
    case STATE_IDLE:           Serial.println(F("IDLE")); break;
    case STATE_GO_UP:          Serial.println(F("GO_UP")); break;
    case STATE_PLATFORM_TURN:  Serial.println(F("PLATFORM_TURN")); break;
    case STATE_DOWNHILL:       Serial.println(F("DOWNHILL")); break;
    case STATE_EMERGENCY_STOP: Serial.println(F("EMERGENCY_STOP")); break;
  }

  // Optional: print remaining RAM for debugging memory usage
  // extern int __heap_start, *__brkval;
  // int freeRam = (int) &freeRam - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
  // Serial.print(F("Free RAM: "));
  // Serial.print(freeRam);
  // Serial.println(F(" bytes"));
}

