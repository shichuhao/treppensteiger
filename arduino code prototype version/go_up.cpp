#include "go_up.h"
#include "yaw_correction.h"

/**
 * @brief Global state variables for climbing navigation
 * @details Values are scaled to avoid floating-point math for better MCU performance.
 */
static GoUpState goUpState = GU_INIT;
uint16_t currentDistance = 0;   ///< Distance to obstacle (mm or cm * 10)
int16_t currentPitch = 0;        ///< Upward/Downward tilt (degrees * 100)
int16_t currentRoll = 0;         ///< Side-to-side tilt (degrees * 100)
uint32_t lastSensorUpdate = 0;   ///< Timestamp of last IMU read (ms)

/** @brief Internal counters for state timing and retry logic */
static uint32_t stateStartTime = 0;
static uint8_t climbAttempts = 0;

/** 
 * @brief Threshold constants (Scaled by 10 or 100)
 */
#define STAIR_DETECT_DISTANCE 150   // 15.0cm: Distance to start stair-climbing sequence
#define STAIR_CLIMB_DISTANCE 50     // 5.0cm: Minimum safety gap to avoid collision
#define CLIMB_PITCH_THRESHOLD 1500  // 15.0°: Pitch required to confirm climbing posture
#define STABILIZE_PITCH_MAX 500     // 5.0°: Pitch threshold to confirm flat ground
#define MAX_CLIMB_ATTEMPTS 3        // Maximum retries before aborting climb
#define SENSOR_UPDATE_INTERVAL 50   // 50ms (20Hz) sensor polling rate

/**
 * @brief Updates IMU and distance sensors using fixed-point math
 * @details Calculates Pitch and Roll without 'float' variables to save CPU cycles.
 * 5729.58 is the conversion factor for Radians to Degrees * 100.
 */
void updateSensors() {
  uint32_t currentTime = millis();

  if (currentTime - lastSensorUpdate >= SENSOR_UPDATE_INTERVAL) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Calculate Pitch (Tilt forward/backward) and Roll (Tilt left/right)
    // Scaled by 100: 1500 equals 15.00 degrees
    currentPitch = (int16_t)(atan2(ay, sqrt((long)ax*ax + (long)az*az)) * 5729.58);
    currentRoll = (int16_t)(atan2(-ax, az) * 5729.58);

    currentDistance = measureDistanceFiltered();
    lastSensorUpdate = currentTime;
  }
}

/**
 * @brief Performs 10 ultrasonic samples and returns the average
 * @return uint16_t Filtered distance or 9999 if no valid target
 */
uint16_t measureDistanceFiltered() {
  uint32_t sum = 0;
  uint8_t validCount = 0;

  for (uint8_t i = 0; i < 10; i++) {
    uint16_t dist = measureDistance();
    if (dist < 9999) {
      sum += dist;
      validCount++;
    }
    delay(10); // Small delay to prevent ultrasonic echo interference
  }

  return validCount > 0 ? (uint16_t)(sum / validCount) : 9999;
}

/**
 * @brief Main Finite State Machine (FSM) for climbing stairs/obstacles
 * 
 * @details Implementation Strategy:
 * 1. MOVE_FORWARD: Approach the stair at medium speed.
 * 2. CHECK_STAIR: Stop and verify distance/alignment.
 * 3. CLIMB: Maximum power to overcome the step, with active yaw correction.
 * 4. STABILIZE: Confirm the robot is level on the next step.
 */
void handleGoUp() {
  updateSensors();

  switch (goUpState) {
    case GU_INIT:
      Serial.println(F("GU_INIT: System Reset"));
      stateStartTime = millis();
      climbAttempts = 0;
      goUpState = GU_MOVE_FORWARD;
      break;

    case GU_MOVE_FORWARD:
      /** @section GU_MOVE_FORWARD Approach phase */
      motorsForward(180); 

      if (currentDistance < STAIR_DETECT_DISTANCE) {
        Serial.println(F("Stair detected: Preparing for climb"));
        goUpState = GU_CHECK_STAIR;
        stateStartTime = millis();
      }
      break;

    case GU_CHECK_STAIR:
      /** @section GU_CHECK_STAIR Pre-climb adjustment */
      stopAllMotors();
      delay(200); // Allow physical vibration to settle

      // If too close to the stair, back up slightly to gain momentum
      if (currentDistance < STAIR_CLIMB_DISTANCE) {
        Serial.println(F("Safety: Too close, backing up"));
        motorsBackward(150);
        delay(300);
      }

      goUpState = GU_CLIMB;
      stateStartTime = millis();
      break;

    case GU_CLIMB:
      /** @section GU_CLIMB Active climbing with yaw correction */
      // If we haven't reached the climb angle yet, use full power
      if (currentPitch < CLIMB_PITCH_THRESHOLD) {
        motorsForward(255); // Maximum torque

        // Correction to stay straight during the high-torque phase
        int16_t yawDev = getYawDeviation();
        if (yawDev > 300) {        // Drifted > 3.0° Left
          motorsRotateLeft(100);
          delay(50);
        } else if (yawDev < -300) { // Drifted > 3.0° Right
          motorsRotateRight(100);
          delay(50);
        }
      } else {
        // Reduce power slightly once the front wheels are up
        motorsForward(200);
      }

      // Exit condition: Robot is level (low pitch) and clear of the step
      if (currentPitch < STABILIZE_PITCH_MAX && currentDistance > STAIR_DETECT_DISTANCE) {
        goUpState = GU_STABILIZE;
        stateStartTime = millis();
        Serial.println(F("Climb sequence successful"));
      }

      // Safety timeout: If climb takes >10s, attempt retry
      if (millis() - stateStartTime > 10000) {
        climbAttempts++;
        if (climbAttempts >= MAX_CLIMB_ATTEMPTS) {
          Serial.println(F("CRITICAL: Climb failed, aborting"));
          stopAllMotors();
          goUpState = GU_INIT;
        } else {
          Serial.println(F("Timeout: Retrying climb sequence"));
          goUpState = GU_CHECK_STAIR;
        }
      }
      break;

    case GU_STABILIZE:
      /** @section GU_STABILIZE Settling on level ground */
      motorsForward(150);

      // Move forward for 1 second to ensure full clearance of the edge
      if (millis() - stateStartTime > 1000) {
        stopAllMotors();
        goUpState = GU_COMPLETE;
        Serial.println(F("GU_COMPLETE: Level ground confirmed"));
      }
      break;

    case GU_COMPLETE:
      /** @section GU_COMPLETE Idle state */
      stopAllMotors();
      break;
  }
}
