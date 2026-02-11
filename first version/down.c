// ============================================================================
// SIMPLIFIED DOWNHILL MODULE - COMPLETE CODE
// ============================================================================
// NEW CONSTANTS FOR DOWNHILL MODULE - Add to existing constants section
// ============================================================================

// Downhill detection thresholds
#define DOWNHILL_PITCH_THRESHOLD -10     // degrees - negative pitch indicates downhill
#define DOWNHILL_ANGLE_CONFIRM -8        // degrees - sustained angle to confirm downhill

// Simplified motor control (3 levels only)
#define DOWNHILL_NO_BRAKE_SPEED 60       // PWM for no brake (normal descent)
#define DOWNHILL_LIGHT_BRAKE_SPEED 80    // PWM for light brake
#define DOWNHILL_MEDIUM_BRAKE_SPEED 120  // PWM for medium brake (reverse)

// Stair bottom detection (using pitch only, no distance sensor)
#define PLATFORM_PITCH_MIN -3.0          // degrees - lower bound for level surface
#define PLATFORM_PITCH_MAX 5.0           // degrees - upper bound for level surface
#define PLATFORM_CONFIRM_TIME 500        // milliseconds - sustained level pitch to confirm platform

// Brake level thresholds
#define LIGHT_BRAKE_PITCH -12.0          // degrees - pitch threshold for light brake
#define MEDIUM_BRAKE_PITCH -15.0         // degrees - pitch threshold for medium brake

// ============================================================================
// NEW GLOBAL VARIABLES FOR DOWNHILL - Add to existing variables section
// ============================================================================

// Downhill state variables
bool isDownhillMode = false;             // Flag for downhill detection
bool isDescending = false;               // Flag for active descent (can be removed)
float maxDownhillPitch = 0.0;            // Track steepest pitch during descent
unsigned long downhillStartTime = 0;     // Time when downhill started
//int descentStepCount = 0;                // Count stairs descended
// Downhill exit confirmation
int downhillExitConfirmCount = 0;
// Platform detection variables
unsigned long platformLevelStartTime = 0; // Time when pitch first entered level range
bool platformLevelDetecting = false;     // Flag indicating we're timing level pitch

// ============================================================================
// NEW FUNCTION PROTOTYPES FOR DOWNHILL - Add to existing prototypes
// ============================================================================

// Downhill module
bool detectDownhill_Downhill();
void controlDownhillMotors_Downhill();
void applyBraking(int brakeLevel);
bool detectStairBottom();

// ============================================================================
// MODIFIED MAIN LOOP - Replace or modify existing loop()
// ============================================================================

void loop() {
  wdt_reset();

  // Check for remote commands
  processRemoteCommands_Stop();

  unsigned long currentTime = millis();

  if (currentTime - lastSensorUpdate >= SENSOR_UPDATE_INTERVAL) {
    lastSensorUpdate = currentTime;

    // Priority check: Downhill mode (highest priority for safety)
    if (detectDownhill_Downhill()) {
      controlDownhillMotors_Downhill();
    }

    // Check if currently turning
    else if (isTurning) {
      if (checkTurnCompletion_Turn()) {
        isTurning = false;
        Serial.println(" Turn completed successfully");
      }
    }

    // Normal climbing/flat surface control
    else {
      controlMotors();

      // Check if should turn (only for uphill mode)
      if (detectLevelEnd_Turn() && !remoteStopRequested) {
        Serial.println(" Level end detected - Initiating turn");
        performStairLevelTurn();
      }
    }
  }

  delay(10);
}

// ============================================================================
// DOWNHILL MODULE - UC01: DOWNHILL DETECTION
// ============================================================================

/**
 * @brief Detects downhill stair condition using pitch angle
 *
 * @details This function implements Downhill_UC01: Stair Distance Recognition
 * - Monitors pitch angle from MPU6050 for negative values (nose down)
 * - Distinguishes between slight decline and actual stair descent
 *
 * Detection criteria:
 * - Pitch < -10°: Initial downhill detection threshold
 * - Pitch < -8° sustained: Confirms active descent
 *
 * Safety features:
 * - Requires sustained negative pitch (not just noise)
 *
 * Libraries used:
 * - None directly (uses global sensor data from updateAttitude_UC02)
 *
 * @return bool true if downhill condition detected, false otherwise
 */
bool detectDownhill_Downhill() {
  // Update sensor data
  currentDistance = measureDistance();
  updateAttitude();

  // Check for negative pitch (nose down = downhill)
  if (currentPitch < DOWNHILL_PITCH_THRESHOLD) {
    // Confirm it's sustained downhill (not just momentary tilt)
    static int downhillConfirmCount = 0;

    if (currentPitch < DOWNHILL_ANGLE_CONFIRM) {
      downhillConfirmCount++;
    } else {
      downhillConfirmCount = 0;
    }

    // Require 3 consecutive readings to confirm downhill
    if (downhillConfirmCount >= 3) {
      if (!isDownhillMode) {
        // First detection of downhill
        isDownhillMode = true;
        isDescending = true;
        downhillStartTime = millis();
        //descentStepCount = 0;
        maxDownhillPitch = currentPitch;

        // Reset platform detection
        platformLevelDetecting = false;
        platformLevelStartTime = 0;

        Serial.println("\n╔════════════════════════════════════╗");
        Serial.println("║ DOWNHILL MODE ACTIVATED            ║");
        Serial.println("╚════════════════════════════════════╝");
        Serial.print("Initial pitch: ");
        Serial.print(currentPitch, 1);
        Serial.println("°");
      }

      // Track maximum pitch during descent
      if (currentPitch < maxDownhillPitch) {
        maxDownhillPitch = currentPitch;
      }

      return true;
    }

  } else {
      // Pitch is positive or near zero
      if (isDownhillMode && currentPitch > -3.0) {
          // NEW: Increment exit confirmation counter
          downhillExitConfirmCount++;

          Serial.print("Exit confirmation count: ");
          Serial.print(downhillExitConfirmCount);
          Serial.print(" / ");
          Serial.println(DOWNHILL_EXIT_CONFIRM_COUNT);

          //  Only exit after 3 consecutive level readings
          if (downhillExitConfirmCount >= DOWNHILL_EXIT_CONFIRM_COUNT) {
              // Exiting downhill - reached bottom
              Serial.println("\n╔════════════════════════════════════╗");
              Serial.println("║ DOWNHILL MODE COMPLETED            ║");
              Serial.println("╚════════════════════════════════════╝");
              Serial.print("Steepest pitch reached: ");
              Serial.print(maxDownhillPitch, 1);
              Serial.println("°");
              Serial.print("Descent duration: ");
              Serial.print((millis() - downhillStartTime) / 1000.0, 1);
              Serial.println(" seconds");

              isDownhillMode = false;
              isDescending = false;
              downhillExitConfirmCount = 0;  // Reset counter
              maxDownhillPitch = 0.0;        // Reset max pitch
              stopMotors();
          }
      } else {
          // Pitch went back to negative, reset exit counter (NEW)
          if (downhillExitConfirmCount > 0) {
              Serial.println("Pitch negative again - Resetting exit counter");
              downhillExitConfirmCount = 0;
          }
      }
  }

    return false;
}

// ============================================================================
// DOWNHILL MODULE - UC02: ATTITUDE MONITORING
// ============================================================================

/**
 * @brief Enhanced attitude detection for downhill scenario
 *
 * @details This extends existing updateAttitude() with downhill awareness
 * - Existing function already provides pitch and roll data
 * - This comment explains downhill-specific interpretation
 *
 * Downhill pitch interpretation:
 * - Pitch > +15°: Uphill climbing (existing logic)
 * - Pitch ≈ 0°: Level surface (existing logic)
 * - Pitch < -10°: Downhill descent (NEW - triggers downhill mode)
 *
 * Roll angle is equally important for downhill:
 * - Roll > 20°: Risk of tipping sideways on stairs
 * - More critical during descent due to gravity
 *
 * @note No code changes needed to updateAttitude()
 * @note Just use existing currentPitch value with negative thresholds
 */

// ============================================================================
// DOWNHILL MODULE - UC03: DOWNHILL MOTOR CONTROL (SIMPLIFIED)
// ============================================================================

/**
 * @brief Simplified downhill motor control with 3-level braking
 *
 * @details This function implements Downhill_UC03: Downhill Motor Control
 * - Manages safe descent using simplified braking strategy
 * - Prevents uncontrolled acceleration down stairs
 *
 * Simplified control strategy:
 * 1. Mild pitch (> -12°): No brake (normal forward motion)
 * 2. Moderate pitch (-12° to -15°): Light brake (slow forward)
 * 3. Steep pitch (< -15°): Medium brake (reverse direction)
 *
 * Safety features:
 * - Roll angle monitoring (prevent sideways tip)
 * - Obstacle detection (emergency stop)
 *
 * Libraries used:
 * - Arduino.h (motor control via L298N functions)
 *
 * @return void (directly controls motors)
 */
void controlDownhillMotors_Downhill() {
  Serial.println("=====================================");
  Serial.print(" DOWNHILL MODE | Pitch: ");
  Serial.print(currentPitch, 1);
  Serial.print("° | Roll: ");
  Serial.print(currentRoll, 1);
  Serial.println("°");
  Serial.print("Distance ahead: ");
  Serial.print(currentDistance, 1);
  Serial.println(" cm");

  // SAFETY CHECK 1: Excessive roll angle - EMERGENCY STOP
  if (abs(currentRoll) > ROLL_ANGLE_LIMIT) {
    Serial.println(" EMERGENCY: Excessive roll during descent!");
    stopMotors();
    return;
  }

  // SAFETY CHECK 2: Obstacle too close - STOP
  if (currentDistance < EMERGENCY_STOP_DISTANCE) {
    Serial.println(" EMERGENCY: Obstacle ahead during descent!");
    stopMotors();
    return;
  }

  // Simplified 3-level braking based on pitch
  if (currentPitch < MEDIUM_BRAKE_PITCH) {
    // Steep descent (< -15°): Medium brake (reverse)
    Serial.print(" STEEP DESCENT (");
    Serial.print(currentPitch, 1);
    Serial.println("°) - Medium brake");
    applyBraking(2);
  }
  else if (currentPitch < LIGHT_BRAKE_PITCH) {
    // Moderate descent (-12° to -15°): Light brake (slow forward)
    Serial.print(" MODERATE DESCENT (");
    Serial.print(currentPitch, 1);
    Serial.println("°) - Light brake");
    applyBraking(1);
  }
  else {
    // Mild descent (> -12°): No brake (normal forward)
    Serial.print(" MILD DESCENT (");
    Serial.print(currentPitch, 1);
    Serial.println("°) - No brake");
    applyBraking(0);
  }

  // Check if reached bottom of stairs (using pitch only)
  if (detectStairBottom()) {
    Serial.println(" PLATFORM REACHED - Stopping");
    stopMotors();
    isDownhillMode = false;
    isDescending = false;
  }

  Serial.println("=====================================\n");
}

// ============================================================================
// DOWNHILL MODULE - HELPER: APPLY BRAKING (SIMPLIFIED)
// ============================================================================

/**
 * @brief Simplified 3-level braking system
 *
 * @details Implements graduated braking strategy using L298N motor control
 * - Level 0: No brake (normal forward motion at 60 PWM)
 * - Level 1: Light brake (slow forward at 80 PWM)
 * - Level 2: Medium brake (reverse direction at 120 PWM)
 *
 * Braking mechanism:
 * - Forward direction + low speed = controlled descent
 * - Reverse direction = motors resist motion (active brake)
 *
 * Physical principle:
 * - Gravity pulls robot downward on stairs
 * - Motors provide resistance to control speed
 * - PWM value controls braking intensity
 *
 * @param brakeLevel Integer 0-2 indicating brake intensity
 * @return void
 */
void applyBraking(int brakeLevel) {
  switch (brakeLevel) {
    case 0:
      // No brake: Normal forward motion (60 PWM)
      setMotorDirection(true); // Forward
      setMotorSpeed(DOWNHILL_NO_BRAKE_SPEED, DOWNHILL_NO_BRAKE_SPEED);
      Serial.println("   Brake: NONE (normal forward)");
      break;

    case 1:
      // Light brake: Slow Reverse (60 PWM)
      setMotorDirection(false); //Reverse
      setMotorSpeed(DOWNHILL_LIGHT_BRAKE_SPEED, DOWNHILL_LIGHT_BRAKE_SPEED);
      Serial.println("   Brake: LIGHT ");
      break;

    case 2:
      // Medium brake: Reverse direction (120 PWM)
      setMotorDirection(false); // Reverse
      setMotorSpeed(DOWNHILL_MEDIUM_BRAKE_SPEED, DOWNHILL_MEDIUM_BRAKE_SPEED);
      Serial.println("   Brake: MEDIUM ");
      break;

    default:
      // Safety default: Stop motors completely
      stopMotors();
      Serial.println("   Brake: EMERGENCY STOP");
      break;
  }
}

// ============================================================================
// DOWNHILL MODULE - HELPER: DETECT STAIR BOTTOM (PITCH ONLY, NO DISTANCE)
// ============================================================================

/**
 * @brief Detects when robot has reached the bottom of stairs using PITCH ONLY
 *
 * @details Uses sustained level pitch to confirm platform arrival
 * - REMOVED: Distance sensor check (no longer used)
 * - NEW: Pitch must remain in level range (-2° to +5°) for PLATFORM_CONFIRM_TIME ms
 * - Prevents false positives from momentary pitch changes
 *
 * Detection logic:
 * 1. Robot is in downhill mode (isDownhillMode = true)
 * 2. Pitch enters level range (PLATFORM_PITCH_MIN to PLATFORM_PITCH_MAX)
 * 3. Pitch stays in range for sustained duration (PLATFORM_CONFIRM_TIME)
 * 4. → Confirms platform/flat surface reached
 *
 * Timing mechanism:
 * - First entry to level range: Record timestamp
 * - Continuous level pitch: Check elapsed time
 * - Exit level range before timeout: Reset timer
 * - Timeout reached: Confirm platform detection
 *
 * @return bool true if stair bottom (platform) detected, false otherwise
 */
bool detectStairBottom() {
  // Must be in downhill mode
  if (!isDownhillMode) {
    return false;
  }

  // Check if pitch is in level range
  bool pitchInLevelRange = (currentPitch > PLATFORM_PITCH_MIN &&
                            currentPitch < PLATFORM_PITCH_MAX);

  if (pitchInLevelRange) {
    // Pitch is level - start or continue timing
    if (!platformLevelDetecting) {
      // First time entering level range - start timer
      platformLevelDetecting = true;
      platformLevelStartTime = millis();

      Serial.println("\n Level pitch detected - Starting platform confirmation timer");
      Serial.print("   Pitch: ");
      Serial.print(currentPitch, 1);
      Serial.println("°");
    } else {
      // Already timing - check if sustained long enough
      unsigned long elapsedTime = millis() - platformLevelStartTime;

      Serial.print("   Platform timer: ");
      Serial.print(elapsedTime);
      Serial.print(" / ");
      Serial.print(PLATFORM_CONFIRM_TIME);
      Serial.println(" ms");

      if (elapsedTime >= PLATFORM_CONFIRM_TIME) {
        // Sustained level pitch confirmed - platform reached!
        Serial.println("\n Platform Detection Confirmed:");
        Serial.print("   Pitch: ");
        Serial.print(currentPitch, 1);
        Serial.println("° (Level)");
        Serial.print("   Duration: ");
        Serial.print(elapsedTime);
        Serial.println(" ms (Sustained)");

        // Reset detection state
        platformLevelDetecting = false;
        platformLevelStartTime = 0;

        return true;
      }
    }
  } else {
    // Pitch exited level range - reset timer
    if (platformLevelDetecting) {
      Serial.println("   ️ Pitch left level range - Resetting platform timer");
      platformLevelDetecting = false;
      platformLevelStartTime = 0;
    }
  }

  return false;
}

// ============================================================================
// MODIFIED CONTROL FUNCTION - Update existing controlMotors()
// ============================================================================

/**
 * @brief Enhanced control function with downhill awareness
 *
 * @details Modify existing controlMotors() to avoid conflicts:
 * - Only execute if NOT in downhill mode
 * - Downhill control takes priority in main loop
 *
 * Add this check at the beginning of existing controlMotors():
 */
void controlMotors() {
  // Check remote stop first
  if (remoteStopRequested) {
    if (robotState != 0) {
      executeStopCommand_Stop();
    }
    return;
  }

  // NEW: Skip if in downhill mode (handled separately)
  if (isDownhillMode) {
    return; // Downhill control is handled by controlDownhillMotors_Downhill()
  }

  // ... (rest of existing uphill climbing logic continues unchanged)
  currentDistance = measureDistance();
  updateAttitude();

  // ... existing code ...
}
