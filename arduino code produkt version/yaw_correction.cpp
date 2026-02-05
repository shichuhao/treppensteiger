// ============================================================================
// YAW CORRECTION MODULE - UPHILL + DOWNHILL FULL SUPPORT
// ============================================================================
#include "init.h"
#include "yaw_correction.h"
// ============================================================================
// CONSTANTS - Full-scene yaw correction parameters
// ============================================================================
/*
#define YAW_CORRECTION_THRESHOLD 2.5f   // degrees - Reduced threshold for sensitivity
#define YAW_RATE_THRESHOLD 4.0f         // deg/s - Noise threshold
#define YAW_CORRECTION_GAIN_UP 10       // Uphill gain (aggressive correction)
#define YAW_CORRECTION_GAIN_DOWN 8      // Downhill gain (conservative correction)
#define MAX_YAW_CORRECTION 45           // Maximum PWM differential

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

float referenceYaw = 0.0f;          // Yaw baseline (reset at climb start)
float currentYaw = 0.0f;            // Current integrated yaw angle
float yawDeviation = 0.0f;          // Deviation from reference (+=right drift)
bool yawCorrectionActive = false;   // Correction status flag
unsigned long lastYawUpdate = 0;    // Last integration timestamp
*/
// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================
/*
void initializeYawReference();      // Reset yaw baseline
float getYawDeviation();            // Calculate yaw error by gyro integration
void applyYawCorrection(int baseSpeedOverride = -1);  // Apply correction
int calculateCorrectionPWM(float deviation, bool isDownhill);  // PWM calculation
bool shouldApplyYawCorrection();    // Scene judgment: when to correct
*/
// ============================================================================
// INITIALIZE YAW REFERENCE
// ============================================================================

/**
 * @brief Resets yaw reference when entering climb/downhill mode
 *
 * @details Called when starting straight-line motion (climb or descent)
 * - Sets current yaw as baseline (0 deviation)
 * - Clears accumulated error and status flags
 *
 * @return void
 */
void initializeYawReference() {
    referenceYaw = 0.0f;
    currentYaw = 0.0f;
    yawDeviation = 0.0f;
    yawCorrectionActive = false;
    lastYawUpdate = millis();

    Serial.println(" Yaw reference reset");
}

// ============================================================================
// GET YAW DEVIATION - Gyroscope Integration
// ============================================================================

/**
 * @brief Calculates yaw deviation using gyroscope Z-axis integration
 *
 * @details Integrates gyro Z-axis (gz) over time:
 * - yaw_rate = gz / 65.5 (°/s for MPU6050 ±500°/s range)
 * - deviation = integrated_yaw - reference_yaw
 * - Noise filtering: ignore rates < YAW_RATE_THRESHOLD
 *
 * @return float Yaw deviation in degrees (+ = right drift, - = left drift)
 */
float getYawDeviation() {
    // Read gyroscope Z-axis (assume updated in main loop)
    float yawRate = gz / 65.5f;  // Convert raw to °/s

    // Noise filtering
    if (fabs(yawRate) < YAW_RATE_THRESHOLD) {
        yawRate = 0.0f;
    }

    unsigned long now = millis();
    float dt = (now - lastYawUpdate) / 1000.0f;  // Delta time in seconds
    lastYawUpdate = now;

    currentYaw += yawRate * dt;           // Time integration
    yawDeviation = currentYaw - referenceYaw;  // Calculate error

    return yawDeviation;
}

// ============================================================================
// SCENE JUDGMENT: WHEN TO APPLY CORRECTION
// ============================================================================

/**
 * @brief Determines if yaw correction should be applied
 *
 * @details Only applies during active climbing/descending:
 * - Uphill: isClimbingMode = true
 * - Downhill: isDownhillMode = true
 * - Skip: flat ground, platform navigation, emergency stop
 *
 * @return bool true = apply correction, false = skip
 */
bool shouldApplyYawCorrection() {
    // Skip during remote stop or emergency
    if (remoteStopRequested || emergencyStop) {
        return false;
    }

    // Only during uphill/downhill climbing
    return (isClimbingMode || isDownhillMode);
}

// ============================================================================
// CALCULATE PWM CORRECTION
// ============================================================================

/**
 * @brief Calculates PWM differential based on deviation and scene
 *
 * @details Proportional control with scene-adaptive gain:
 * - Uphill: Higher gain (12) for aggressive correction
 * - Downhill: Lower gain (8) for stability during braking
 * - Output clamped to prevent excessive correction
 *
 * @param deviation Yaw error in degrees
 * @param isDownhill Downhill scene flag
 * @return int PWM correction (+ = speed up right, - = speed up left)
 */
int calculateCorrectionPWM(float deviation, bool isDownhill) {
    int gain = isDownhill ? YAW_CORRECTION_GAIN_DOWN : YAW_CORRECTION_GAIN_UP;
    int correction = (int)(deviation * gain);
    correction = constrain(correction, -MAX_YAW_CORRECTION, MAX_YAW_CORRECTION);
    return correction;
}

// ============================================================================
// APPLY YAW CORRECTION (Core Function)
// ============================================================================

/**
 * @brief Applies differential drive correction for yaw drift
 *
 * @details Full correction pipeline:
 * 1. Get deviation from gyro integration
 * 2. Scene-adaptive base speed selection
 * 3. Calculate PWM differential (corrected physics)
 * 4. Apply to motors: right drift → left turn (right fast)
 * 5. Logging and status management
 *
 * Physics (corrected):
 * - Right drift (+dev): right fast → left turn ✓
 * - Left drift (-dev): left fast → right turn ✓
 *
 * @param baseSpeedOverride Manual base speed (-1 = auto select)
 * @return void (directly sets motor speeds)
 */
void applyYawCorrection(int baseSpeedOverride=-1) {
    float deviation = getYawDeviation();

    // Skip small deviations
    if (fabs(deviation) < YAW_CORRECTION_THRESHOLD) {
        if (yawCorrectionActive) {
            Serial.println(" Yaw within tolerance");
            yawCorrectionActive = false;
        }
        return;
    }

    // Correction activation
    if (!yawCorrectionActive) {
        Serial.print(" Yaw correction activated: ");
        Serial.print(deviation, 1);
        Serial.println("°");
        yawCorrectionActive = true;
    }

    // Select base speed by scene
    int baseSpeed;
    if (baseSpeedOverride >= 0) {
        baseSpeed = baseSpeedOverride;
    } else {
        if (isDownhillMode) {
            baseSpeed = -DOWNHILL_LIGHT_BRAKE_SPEED;  // Downhill speed
        } else if (isClimbingMode) {
            baseSpeed = CLIMB_SPEED;                 // Uphill speed
        } else {
            baseSpeed = NORMAL_SPEED;                // Normal speed
        }
    }

    // Calculate correction (scene-adaptive)
    bool isDownhill = isDownhillMode;
    int correctionPWM = calculateCorrectionPWM(deviation, isDownhill);

        // Differential application
        int leftSpeed  = baseSpeed + correctionPWM;   // Right drift → left slow
        int rightSpeed = baseSpeed - correctionPWM;   // Right drift → right fast → LEFT TURN

    // Constrain and apply
    leftSpeed  = constrain(leftSpeed,  0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);
    setMotorSpeed(leftSpeed, rightSpeed);

    // Logging
    Serial.print(" dev=");
    Serial.print(deviation, 1);
    Serial.print("° base=");
    Serial.print(baseSpeed);
    Serial.print(" corr=");
    Serial.print(correctionPWM);
    Serial.print(" L/R=");
    Serial.print(leftSpeed);
    Serial.print("/");
    Serial.println(rightSpeed);
}


