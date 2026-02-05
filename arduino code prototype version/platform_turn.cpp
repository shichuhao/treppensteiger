#include "platform_turn.h"
#include "yaw_correction.h"

/**
 * @file platform_turn.cpp
 * @brief Logic for platform detection and 90-degree rotation.
 * 
 * @details Uses a rolling average for wall detection and a state machine 
 * to ensure the robot is stable and level before and after turning.
 */

// === State & Navigation Variables ===
static PlatformNavState platformNavState = PN_INIT;
int16_t platformTurnYaw = 0;                 ///< Reference yaw angle at start of turn (deg * 100)
uint32_t platformTurnStartTime = 0;          ///< Timer for rotation phase (ms)
uint32_t platformLevelStartTime = 0;         ///< Timer for stability checks (ms)

// === Memory Optimized Buffer (uint16_t[5] instead of float[10]) ===
uint16_t wallDistanceReadings[5] = {9999, 9999, 9999, 9999, 9999};
uint8_t wallDistanceIndex = 0;               ///< Cyclic index for the rolling buffer

// Internal Logic Counters
static uint8_t platformLevelConfirmCounter = 0; ///< Consecutive level readings required
static uint8_t turnAttempts = 0;                ///< Retry counter for failed rotations

// === Threshold Constants (Scaled by 10 or 100) ===
#define PLATFORM_DISTANCE_MAX 300       // 30.0cm: Gap detected (edge of platform)
#define PLATFORM_PITCH_THRESHOLD 500    // 5.0°: Max tilt allowed for "level" state
#define WALL_DISTANCE_THRESHOLD 200     // 20.0cm: Distance used to distinguish wall vs. platform
#define YAW_TARGET_90 9000              // 90.0°: Target turn angle
#define YAW_TOLERANCE 300               // 3.0°: Acceptable error for rotation
#define MAX_TURN_ATTEMPTS 3             // Max retries for rotation
#define LEVEL_CONFIRM_COUNT 5           // Required stable samples for confirmation

/**
 * @brief Adds a new distance reading to the rolling average buffer.
 * @param distance Distance in 0.1cm units.
 */
void addWallDistance(uint16_t distance) {
  wallDistanceReadings[wallDistanceIndex] = distance;
  wallDistanceIndex = (wallDistanceIndex + 1) % 5; 
}

/**
 * @brief Calculates the average distance from the rolling buffer.
 * @return uint16_t Average distance (0.1cm) or 9999 if no valid data.
 */
uint16_t getAverageWallDistance() {
  uint32_t sum = 0;
  uint8_t count = 0;

  for (uint8_t i = 0; i < 5; i++) {
    if (wallDistanceReadings[i] < 9999) {
      sum += wallDistanceReadings[i];
      count++;
    }
  }
  return count > 0 ? (uint16_t)(sum / count) : 9999;
}

/**
 * @brief Checks if both Pitch and Roll are within level thresholds.
 * @return bool True if platform is level.
 */
bool isPlatformLevel() {
  extern int16_t currentPitch;
  extern int16_t currentRoll;

  // Check absolute values against 5.0° threshold
  return (abs(currentPitch) < PLATFORM_PITCH_THRESHOLD && 
          abs(currentRoll) < PLATFORM_PITCH_THRESHOLD);
}

/**
 * @brief Main FSM for platform navigation.
 * 
 * Logic flow:
 * 1. PN_MOVE_FORWARD: Drive until sensors see a gap (>30cm).
 * 2. PN_DETECT_PLATFORM: Verify if the gap is a platform or just a far wall.
 * 3. PN_PLATFORM_LEVEL: Wait for the robot to stop vibrating and sit level.
 * 4. PN_ROTATE_90: Execute 90-degree turn using IMU feedback.
 * 5. PN_CONFIRM_LEVEL: Final check before completing navigation.
 */
void handlePlatformTurn() {
  extern void updateSensors();
  updateSensors();

  extern uint16_t currentDistance;
  extern int16_t currentPitch;
  extern int16_t currentYaw;

  switch (platformNavState) {
    case PN_INIT:
      /** @section PN_INIT Reset system for platform navigation */
      Serial.println(F("PN_INIT: Clearing buffers"));
      platformTurnStartTime = millis();
      turnAttempts = 0;

      for (uint8_t i = 0; i < 5; i++) wallDistanceReadings[i] = 9999;
      wallDistanceIndex = 0;

      platformNavState = PN_MOVE_FORWARD;
      break;

    case PN_MOVE_FORWARD:
      /** @section PN_MOVE_FORWARD Tracking walls while moving */
      motorsForward(150);
      addWallDistance(currentDistance);

      // Trigger if distance suddenly increases (platform edge)
      if (currentDistance > PLATFORM_DISTANCE_MAX) {
        Serial.println(F("Platform edge detected"));
        platformNavState = PN_DETECT_PLATFORM;
        platformTurnStartTime = millis();
      }
      break;

    case PN_DETECT_PLATFORM:
      /** @section PN_DETECT_PLATFORM Validation phase */
      stopAllMotors();
      delay(300);

      // Check if previous readings were actually a wall
      uint16_t avgDist = getAverageWallDistance();
      if (avgDist < WALL_DISTANCE_THRESHOLD) {
        Serial.println(F("False alarm: detected wall, not edge"));
        platformNavState = PN_MOVE_FORWARD;
        break;
      }

      platformNavState = PN_PLATFORM_LEVEL;
      platformLevelStartTime = millis();
      platformLevelConfirmCounter = 0;
      break;

    case PN_PLATFORM_LEVEL:
      /** @section PN_PLATFORM_LEVEL Stability check before rotation */
      stopAllMotors();

      if (isPlatformLevel()) {
        platformLevelConfirmCounter++;
        if (platformLevelConfirmCounter >= LEVEL_CONFIRM_COUNT) {
          Serial.println(F("Stability confirmed: Preparing rotation"));
          platformNavState = PN_ROTATE_90;
          platformTurnStartTime = millis();
          platformTurnYaw = currentYaw; // Save starting heading
        }
      } else {
        platformLevelConfirmCounter = 0;
      }
      delay(200);

      if (millis() - platformLevelStartTime > 5000) {
        Serial.println(F("Level timeout: Resuming search"));
        platformNavState = PN_MOVE_FORWARD;
      }
      break;

    case PN_ROTATE_90:
      /** @section PN_ROTATE_90 IMU-based 90 degree turn */
      {
        // Calculate relative turn progress
        int16_t yawDiff = currentYaw - platformTurnYaw;

        // Normalize angle to [-180, 180] range
        while (yawDiff > 18000) yawDiff -= 36000;
        while (yawDiff < -18000) yawDiff += 36000;

        int16_t yawError = YAW_TARGET_90 - abs(yawDiff);

        // Feedback logs
        Serial.print(F("Current Turn: ")); Serial.print(yawDiff / 100);
        Serial.println(F(" deg"));

        if (abs(yawError) < YAW_TOLERANCE) {
          Serial.println(F("Target 90.0 deg reached"));
          stopAllMotors();
          platformNavState = PN_CONFIRM_LEVEL;
          platformLevelStartTime = millis();
          platformLevelConfirmCounter = 0;
        } else {
          // Adaptive rotation direction
          if (yawError > 0) motorsRotateLeft(120);
          else motorsRotateRight(120);
        }

        // Handle rotation timeout and retries
        if (millis() - platformTurnStartTime > 10000) {
          turnAttempts++;
          if (turnAttempts >= MAX_TURN_ATTEMPTS) {
            Serial.println(F("CRITICAL: Rotation failed"));
            stopAllMotors();
            platformNavState = PN_INIT;
          } else {
            Serial.println(F("Retrying rotation..."));
            platformNavState = PN_ROTATE_90;
            platformTurnStartTime = millis();
          }
        }
      }
      break;

    case PN_CONFIRM_LEVEL:
      /** @section PN_CONFIRM_LEVEL Final check after turn */
      stopAllMotors();

      if (isPlatformLevel()) {
        platformLevelConfirmCounter++;
        if (platformLevelConfirmCounter >= LEVEL_CONFIRM_COUNT) {
          Serial.println(F("Turn sequence successful"));
          platformNavState = PN_COMPLETE;
        }
      } else {
        platformLevelConfirmCounter = 0;
      }
      delay(200);

      if (millis() - platformLevelStartTime > 3000) {
        Serial.println(F("Confirm timeout: Ending sequence"));
        platformNavState = PN_COMPLETE;
      }
      break;

    case PN_COMPLETE:
      /** @section PN_COMPLETE Idle */
      stopAllMotors();
      break;
  }
}
