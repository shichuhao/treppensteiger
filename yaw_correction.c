//This is the first version of the yaw correction code, which can only be used in upstair conditions.

// ============================================================================
// YAW CORRECTION MODULE - Using Gyroscope Only (No Magnetometer)
// ============================================================================

// ============================================================================
// NEW CONSTANTS FOR YAW CORRECTION - Add to existing constants
// ============================================================================

// Yaw correction thresholds
#define YAW_CORRECTION_THRESHOLD 3.0    // degrees - trigger correction if yaw > 3°
#define YAW_RATE_THRESHOLD 5.0          // deg/s - ignore noise below this rate
#define YAW_CORRECTION_GAIN 10          // PWM adjustment per degree (tunable)
#define MAX_YAW_CORRECTION 50           // Maximum PWM differential for correction

// ============================================================================
// NEW GLOBAL VARIABLES FOR YAW CORRECTION
// ============================================================================

// Yaw tracking variables
float referenceYaw = 0.0;               // Yaw angle when climbing started (baseline)
float currentYaw = 0.0;                 // Current accumulated yaw angle
float yawDeviation = 0.0;               // Deviation from reference (positive = right drift)
bool yawCorrectionActive = false;       // Flag indicating correction in progress
unsigned long lastYawUpdate = 0;        // Timestamp for yaw integration

// ============================================================================
// NEW FUNCTION PROTOTYPES FOR YAW CORRECTION
// ============================================================================

void initializeYawReference();
float getYawDeviation();
void applyYawCorrection();
int calculateCorrectionPWM(float deviation);

// ============================================================================
// YAW CORRECTION - INITIALIZE REFERENCE
// ============================================================================

/**
 * @brief Initializes yaw reference when starting to climb stairs
 * 
 * @details Call this when climbing mode begins to set baseline yaw angle
 * - Resets accumulated yaw to zero
 * - Sets reference yaw as current heading
 * - Should be called when robot starts climbing straight ahead
 * 
 * This creates a "straight line reference" - any deviation from this
 * indicates the robot is drifting left or right during climb
 * 
 * @return void
 */
void initializeYawReference() {
    referenceYaw = 0.0;      // Reset reference to zero
    currentYaw = 0.0;        // Reset current yaw
    yawDeviation = 0.0;      // Clear any previous deviation
    lastYawUpdate = millis();
    
    Serial.println("📐 Yaw reference initialized for straight climbing");
}

// ============================================================================
// YAW CORRECTION - GET YAW DEVIATION
// ============================================================================

/**
 * @brief Calculates yaw deviation from straight-line reference
 * 
 * @details Integrates gyroscope Z-axis to track accumulated yaw angle
 * - Reads gz (yaw rate) from MPU6050/MPU9250
 * - Integrates over time: yaw += yaw_rate * dt
 * - Compares to reference yaw (should be ~0 for straight climb)
 * - Returns deviation (positive = drifting right, negative = drifting left)
 * 
 * Gyroscope integration:
 * - MPU6050/9250 at ±500°/s range: sensitivity = 65.5 LSB/(°/s)
 * - Formula: yaw_rate (°/s) = raw_gz / 65.5
 * - Time integration: current_yaw += yaw_rate * (current_time - last_time)
 * 
 * Noise filtering:
 * - Ignores yaw rates below YAW_RATE_THRESHOLD (5°/s)
 * - Prevents integration of sensor noise
 * 
 * Libraries used:
 * - MPU6050.h or MPU9250.h (gyroscope data)
 * - Arduino.h (millis for timing)
 * 
 * @return float Yaw deviation in degrees (+ = right, - = left)
 */
float getYawDeviation() {
    // Get current sensor data (already called in main loop)
    // Using global gz (gyroscope Z-axis raw value)
    
    // Calculate yaw rate from gyroscope Z-axis
    // For ±500°/s range: sensitivity = 65.5 LSB/(°/s)
    float yawRate = gz / 65.5;  // Convert to degrees/second
    
    // Noise filter: ignore very small rates
    if (abs(yawRate) < YAW_RATE_THRESHOLD) {
        yawRate = 0.0;  // Treat as noise
    }
    
    // Calculate time step
    unsigned long currentTime = millis();
    float dt = (currentTime - lastYawUpdate) / 1000.0;  // Convert to seconds
    lastYawUpdate = currentTime;
    
    // Integrate yaw angle
    currentYaw += yawRate * dt;
    
    // Calculate deviation from reference (should be ~0 for straight climb)
    yawDeviation = currentYaw - referenceYaw;
    
    return yawDeviation;
}

// ============================================================================
// YAW CORRECTION - APPLY CORRECTION
// ============================================================================

/**
 * @brief Applies differential motor speed to correct yaw drift
 * 
 * @details Adjusts left/right motor speeds to compensate for yaw deviation
 * - Positive deviation (drifting right) → increase left speed, decrease right
 * - Negative deviation (drifting left) → increase right speed, decrease left
 * - Correction magnitude proportional to deviation angle
 * 
 * Control strategy:
 * - Small deviation (< 3°): No correction (within tolerance)
 * - Medium deviation (3° - 10°): Proportional correction
 * - Large deviation (> 10°): Maximum correction applied
 * 
 * PWM adjustment formula:
 * - correction_pwm = deviation * YAW_CORRECTION_GAIN
 * - left_motor = base_speed + correction_pwm
 * - right_motor = base_speed - correction_pwm
 * 
 * Example:
 * - Base speed: 180 PWM (climbing)
 * - Deviation: +5° (drifting right)
 * - Correction: 5 * 10 = 50 PWM
 * - Left motor: 180 + 50 = 230 (faster)
 * - Right motor: 180 - 50 = 130 (slower)
 * - Result: Robot turns left to correct rightward drift
 * 
 * Libraries used:
 * - Arduino.h (analogWrite for motor control)
 * 
 * @return void (directly adjusts motor speeds)
 */
void applyYawCorrection() {
    // Get current yaw deviation
    float deviation = getYawDeviation();
    
    // Check if correction is needed
    if (abs(deviation) < YAW_CORRECTION_THRESHOLD) {
        // Within tolerance - no correction needed
        if (yawCorrectionActive) {
            Serial.println("✓ Yaw correction complete (within tolerance)");
            yawCorrectionActive = false;
        }
        return;
    }
    
    // Correction needed
    if (!yawCorrectionActive) {
        Serial.println("🔧 Yaw correction activated");
        yawCorrectionActive = true;
    }
    
    // Calculate PWM correction amount
    int correctionPWM = calculateCorrectionPWM(deviation);
    
    // Get current base speed (from climbing state)
    int baseSpeed = CLIMB_SPEED;  // Default to climb speed
    if (robotState == 1) baseSpeed = NORMAL_SPEED;
    if (robotState == 2) baseSpeed = APPROACH_SPEED;
    
    // Apply differential correction
    // Positive deviation (right drift) → increase left, decrease right
    // Negative deviation (left drift) → increase right, decrease left
    int leftSpeed = baseSpeed - correctionPWM;
    int rightSpeed = baseSpeed + correctionPWM;
    
    // Constrain to valid PWM range
    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);
    
    // Apply corrected speeds
    setMotorSpeed(leftSpeed, rightSpeed);
    
    // Debug output
    Serial.print("🔧 Yaw correction: deviation = ");
    Serial.print(deviation, 2);
    Serial.print("°, L_PWM = ");
    Serial.print(leftSpeed);
    Serial.print(", R_PWM = ");
    Serial.println(rightSpeed);
}

// ============================================================================
// YAW CORRECTION - CALCULATE CORRECTION PWM
// ============================================================================

/**
 * @brief Calculates PWM differential needed to correct yaw deviation
 * 
 * @details Converts angular deviation to motor speed adjustment
 * - Uses proportional gain (YAW_CORRECTION_GAIN)
 * - Limits maximum correction to prevent overcorrection
 * - Returns signed value (positive = speed up left side)
 * 
 * Formula:
 * - correction = deviation * gain
 * - Clamped to ±MAX_YAW_CORRECTION
 * 
 * Gain tuning guide:
 * - Too low (< 5): Slow correction, continues drifting
 * - Optimal (10-15): Smooth correction
 * - Too high (> 20): Oscillation, overcorrection
 * 
 * @param deviation Yaw deviation in degrees (+ = right, - = left)
 * @return int PWM adjustment value (+ = increase left, - = decrease left)
 */
int calculateCorrectionPWM(float deviation) {
    // Proportional correction
    int correction = (int)(deviation * YAW_CORRECTION_GAIN);
    
    // Limit maximum correction
    correction = constrain(correction, -MAX_YAW_CORRECTION, MAX_YAW_CORRECTION);
    
    return correction;
}

// ============================================================================
// INTEGRATION INTO EXISTING CONTROL FUNCTIONS
// ============================================================================

/**
 * @brief Modified controlMotors_UC03 with yaw correction
 * 
 * @details Add yaw correction to climbing control logic
 * 
 * INSERT THIS CODE in controlMotors_UC03() after detecting climbing mode:
 */

void controlMotors_UC03() {
    // ... existing remote stop check ...
    
    if (remoteStopRequested) {
        if (robotState != 0) {
            executeStopCommand_Stop_UC02();
        }
        return;
    }
    
    // ... existing downhill check ...
    
    if (isDownhillMode) {
        return;
    }
    
    // Update sensor data
    currentDistance = measureDistance_UC01();
    updateAttitude_UC02();
    
    // ... existing emergency stop logic ...
    
    // Detect climbing mode transition (initialize yaw reference)
    static bool wasClimbing = false;
    if (isClimbingMode && !wasClimbing) {
        // Just started climbing - initialize yaw reference
        initializeYawReference();
        wasClimbing = true;
    } else if (!isClimbingMode && wasClimbing) {
        // Stopped climbing - reset flag
        wasClimbing = false;
        yawCorrectionActive = false;
    }
    
    // Get sensor data (already done above)
    bool stairDetected = (currentDistance < STAIR_HEIGHT_THRESHOLD);
    
    // ===== CLIMBING MODE WITH YAW CORRECTION =====
    if (isClimbingMode && stairDetected) {
        // Apply yaw correction during climbing
        applyYawCorrection();  // This adjusts motor speeds internally
        
        robotState = 3;
        Serial.println("🔺 CLIMBING MODE with yaw correction");
    }
    else if (stairDetected && !isClimbingMode) {
        setMotorSpeed(APPROACH_SPEED, APPROACH_SPEED);
        setMotorDirection(true);
        robotState = 2;
        Serial.println("⚠️  STAIR DETECTED - Approaching");
    }
    else if (isClimbingMode && !stairDetected) {
        // Still on incline - apply correction
        applyYawCorrection();
        robotState = 3;
        Serial.println("🔺 ON INCLINE with yaw correction");
    }
    else {
        setMotorSpeed(NORMAL_SPEED, NORMAL_SPEED);
        setMotorDirection(true);
        robotState = 1;
        Serial.println("✅ NORMAL MODE");
    }
    
    Serial.println("=====================================\n");
}

