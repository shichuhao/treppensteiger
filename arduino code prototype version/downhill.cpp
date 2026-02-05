#include "downhill.h"
#include "yaw_correction.h"

/**
 * @brief Global state variables for downhill navigation
 * @details Optimized for memory using static storage and fixed-point arithmetic
 */
static DownhillState downhillState = DH_INIT;
int16_t maxDownhillPitch = 0;      ///< Maximum detected pitch angle (degrees * 100)
uint32_t downhillStartTime = 0;    ///< Timestamp of state entry (ms)

/** @brief Internal counter for recovery logic */
static uint8_t descendAttempts = 0;

/** 
 * @brief Downhill control constants
 * @note Values scaled by 10 or 100 to avoid floating point overhead
 */
#define DOWNHILL_PITCH_THRESHOLD -1000  // -10.0° threshold to trigger descent mode
#define DOWNHILL_DISTANCE_MIN 100       // 10.0cm minimum safety distance
#define STABILIZE_PITCH_MAX 500         // 5.0° stability tolerance
#define MAX_DESCEND_ATTEMPTS 3          // Maximum retries before safety stop

/**
 * @brief Main Finite State Machine (FSM) for downhill stair/slope handling
 *
 * @details Manages the transition from detection to descent and stabilization.
 * Implements yaw correction during descent to maintain a straight path.
 * Physical Strategy:
 * 1. Approach: Move forward until a steep negative pitch is detected.
 * 2. Descend: Controlled movement with active yaw correction.
 * 3. Stabilize: Slow down and verify level ground after pitch recovery.
 *
 * @return void
 */
void handleDownhill() {
  // Update sensor data from external IMU/Odometer modules
  extern void updateSensors();
  updateSensors();

  extern uint16_t currentDistance;
  extern int16_t currentPitch;

  switch (downhillState) {
    case DH_INIT:
      /** @section DH_INIT Initialization of downhill parameters */
      Serial.println(F("DH_INIT: Resetting sensors and timers"));
      downhillStartTime = millis();
      descendAttempts = 0;
      maxDownhillPitch = 0;
      downhillState = DH_APPROACH;
      break;

    case DH_APPROACH:
      /** @section DH_APPROACH Move towards the edge */
      // Check if pitch drops below threshold (e.g., -10.0°)
      if (currentPitch < DOWNHILL_PITCH_THRESHOLD) {
        Serial.println(F("Edge detected: Transitioning to DH_DESCEND"));
        downhillState = DH_DESCEND;
        downhillStartTime = millis();
      } else {
        // Safe approach speed
        motorsForward(150);
      }
      break;

    case DH_DESCEND:
      /** @section DH_DESCEND Controlled descent logic */
      // Record the steepest pitch encountered for exit condition calculation
      if (currentPitch < maxDownhillPitch) {
        maxDownhillPitch = currentPitch;
      }

      // Constant slow drive to maintain momentum against stair friction
      motorsForward(180);

      // --- Active Yaw Correction ---
      // Correct left/right drift during descent based on IMU data
      int16_t yawDev = getYawDeviation();
      if (yawDev > 300) {  // Drifted more than 3.0° Left
        motorsRotateLeft(80);
        delay(50);
      } else if (yawDev < -300) { // Drifted more than 3.0° Right
        motorsRotateRight(80);
        delay(50);
      }

      // --- Exit Condition ---
      // Check if robot has leveled out (pitch recovered by 10.0°)
      if (currentPitch > maxDownhillPitch + 1000) {
        Serial.println(F("Pitch leveled: Entering stabilization"));
        downhillState = DH_STABILIZE;
        downhillStartTime = millis();
      }

      // --- Safety Timeout ---
      // If descent takes >15s, attempt recovery or emergency stop
      if (millis() - downhillStartTime > 15000) {
        descendAttempts++;
        if (descendAttempts >= MAX_DESCEND_ATTEMPTS) {
          Serial.println(F("CRITICAL: Descend failed, emergency stop"));
          stopAllMotors();
          downhillState = DH_INIT;
        } else {
          Serial.println(F("Timeout: Retrying approach"));
          downhillState = DH_APPROACH;
        }
      }
      break;

    case DH_STABILIZE:
      /** @section DH_STABILIZE Buffering motion after descent */
      motorsForward(120);

      // Drive for 1 second to ensure all wheels are on flat ground
      if (millis() - downhillStartTime > 1000) {
        stopAllMotors();
        downhillState = DH_COMPLETE;
        Serial.println(F("DH_COMPLETE: Robot stabilized"));
      }
      break;

    case DH_COMPLETE:
      /** @section DH_COMPLETE Final idle state */
      stopAllMotors();
      break;
  }
}
