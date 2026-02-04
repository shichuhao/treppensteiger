//!!!NEED TO TUNE
// ============================================================================
// CONSTANTS - Platform Navigation with Wall Distance Control
// ============================================================================

// Stage 1: No stair detection conditions
#define NO_STAIR_DISTANCE 100              // cm - confirms no stairs ahead  !!!need to tune!!!!
#define NO_STAIR_CONFIRM_COUNT 3           // confirm 3 consecutive readings
#define NO_STAIR_DWELL_TIME 500            // ms - pause after detecting no stair

// Stage 2: Platform level detection conditions
#define PLATFORM_PITCH_MIN -5.0            // degrees - level platform range
#define PLATFORM_PITCH_MAX 5.0             // degrees
#define PLATFORM_PITCH_CONFIRM_COUNT 5     // confirm 5 consecutive readings  ~100*5ms for confirm

// Stage 3: Wall distance management !!!!!
#define MIN_WALL_DISTANCE 15               // cm - minimum safety distance
#define TARGET_WALL_DISTANCE 50            // cm - target approach distance   !!!NEED TO TUNE
#define WALL_DISTANCE_TOLERANCE 5          // cm - acceptable error ±5cm
#define FIRST_STRAIGHT_DISTANCE 40         // cm - first straight segment
#define FIRST_STRAIGHT_SPEED 80            // PWM - forward movement speed
#define APPROACH_WALL_SPEED 60             // PWM - slower speed for precision

// Stage 4: Turn parameters
#define PLATFORM_TURN_SPEED_OUTER 150      // PWM - outer wheel during turn
#define PLATFORM_TURN_SPEED_INNER 50       // PWM - inner wheel during turn
#define PLATFORM_TURN_ANGLE 90             // degrees - each turn is 90°
#define PLATFORM_TURN_TOLERANCE 5          // degrees - ±5° tolerance

// Stage 5: Next stair detection
#define NEXT_STAIR_DISTANCE 20             // cm - stair detected threshold

// ============================================================================
// NEW GLOBAL VARIABLES - Platform Navigation State Machine
// ============================================================================

// Platform navigation state machine
enum PlatformNavState {
    NAV_IDLE = 0,
    NAV_NO_STAIR_DETECTED = 1,
    NAV_PLATFORM_LEVEL_CHECK = 2,
    NAV_FIRST_STRAIGHT = 3,
    NAV_FIRST_TURN = 4,
    NAV_APPROACH_WALL = 5,
    NAV_SECOND_TURN = 6,
    NAV_NEXT_STAIR_CHECK = 7,
    NAV_COMPLETE = 8
};

PlatformNavState platformNavState = NAV_IDLE;
unsigned long stateStartTime = 0;
unsigned long distanceMeasurementTime = 0;

// Counters for confirmation
int noStairConfirmCounter = 0;
int platformLevelConfirmCounter = 0;

// Distance tracking
float wallDistanceReadings[10];  // Store last 10 wall distance readings
int wallDistanceIndex = 0;

// Turn tracking
int platformTurnCount = 0;  // Track which 90° turn we're on (1 or 2)
float platformTurnYaw = 0.0;  // Accumulated yaw for platform turn
unsigned long platformTurnStartTime = 0;

// ============================================================================
// NEW FUNCTION PROTOTYPES
// ============================================================================

void initiatePlatformNavigation();
void executePlatformNavigation();
void checkNoStairCondition();
void checkPlatformLevelCondition();
void executeFirstStraightMovement();
void executeFirstTurn();
void approachWallDistance();
void executeSecondTurn();
void checkNextStairCondition();
float getWallDistance();
float getAverageWallDistance();
void logNavigationState();

// ============================================================================
// MODIFIED MAIN LOOP - Add Platform Navigation
// ============================================================================

void loop() {
    wdt_reset();

    // Check for remote commands
    processRemoteCommands_Stop();

    unsigned long currentTime = millis();
    if (currentTime - lastSensorUpdate >= SENSOR_UPDATE_INTERVAL) {
        lastSensorUpdate = currentTime;

        // Priority 1: Downhill mode
        if (detectDownhill_Downhill()) {
            controlDownhillMotors_Downhill();
            platformNavState = NAV_IDLE;  // Reset platform nav
        }
            // Priority 2: Platform navigation (complex state machine)
        else if (platformNavState != NAV_IDLE) {
            executePlatformNavigation();
        }
            // Normal climbing control
        else {
            controlMotors();

            // Check if should enter platform navigation
            checkNoStairCondition();
        }
    }

    delay(10);
}

// ============================================================================
// STAGE 1: NO STAIR DETECTION
// ============================================================================

/**
 * @brief Detects when HC-SR04 shows no more stairs ahead
 *
 * @details Condition 1 for platform navigation:
 * - Distance reading > 100cm (no stairs detected)
 * - Requires 3 consecutive confirmations
 * - Must be in climbing mode (confirms transition from stairs to platform)
 *
 * @return void
 */
void checkNoStairCondition() {
    currentDistance = measureDistance();

    // Check if distance indicates no stairs
    if (currentDistance > NO_STAIR_DISTANCE) {
        noStairConfirmCounter++;

        if (noStairConfirmCounter >= NO_STAIR_CONFIRM_COUNT) {
            Serial.println("\n╔════════════════════════════════════╗");
            Serial.println("║   CONDITION 1: NO STAIR DETECTED   ║");
            Serial.println("╚════════════════════════════════════╝");
            Serial.print("Distance ahead: ");
            Serial.print(currentDistance, 1);
            Serial.println(" cm");

            // Move to next state: check if platform is level
            platformNavState = NAV_PLATFORM_LEVEL_CHECK;
            platformLevelConfirmCounter = 0;
            noStairConfirmCounter = 0;
            stateStartTime = millis();
        }
    } else {
        noStairConfirmCounter = 0;  // Reset counter
    }
}

// ============================================================================
// STAGE 2: PLATFORM LEVEL DETECTION
// ============================================================================

/**
 * @brief Detects when robot reaches level platform
 *
 * @details Condition 2 for platform navigation:
 * - Pitch angle within -5° to +5° (level surface)
 * - Requires 5 consecutive confirmations
 * - Ensures platform is stable before proceeding
 *
 * @return void (updates platformNavState)
 */
void checkPlatformLevelCondition() {
    updateAttitude();

    // Check if pitch indicates level surface
    bool isPitchLevel = (currentPitch >= PLATFORM_PITCH_MIN &&
                         currentPitch <= PLATFORM_PITCH_MAX);

    if (isPitchLevel) {
        platformLevelConfirmCounter++;

        if (platformLevelConfirmCounter >= PLATFORM_PITCH_CONFIRM_COUNT) {
            Serial.println("\n╔════════════════════════════════════╗");
            Serial.println("║   CONDITION 2: PLATFORM LEVEL      ║");
            Serial.println("╚════════════════════════════════════╝");
            Serial.print("Pitch angle: ");
            Serial.print(currentPitch, 1);
            Serial.println("° (Level)");
            Serial.println("\n  Proceeding to Stage 3: First Straight Movement");

            // Move to Stage 3: first straight movement
            platformNavState = NAV_FIRST_STRAIGHT;
            stateStartTime = millis();
            platformLevelConfirmCounter = 0;
            wallDistanceIndex = 0;  // Reset distance tracking
        }
    } else {
        platformLevelConfirmCounter = 0;  // Reset if not level
    }
}

// ============================================================================
// STAGE 3: FIRST STRAIGHT MOVEMENT WITH WALL DISTANCE MONITORING
// ============================================================================

/**
 * @brief Moves forward while monitoring wall distance
 *
 * @details Stage 3: Move ~40cm forward while continuously monitoring side wall
 * - Maintains forward speed (80 PWM)
 * - Measures wall distance every 100ms
 * - Stores readings for analysis
 * - Stops if too close to wall (< 15cm)
 * - Proceeds to turn when distance data accumulated
 *
 * Distance monitoring:
 * - Store 10 consecutive readings
 * - Average to smooth noise
 * - Check minimum distance (safety)
 * - Record for wall approach phase
 *
 * @return void (updates state when complete)
 */
void executeFirstStraightMovement() {
    // Measure wall distance
    float wallDist = getWallDistance();
    wallDistanceReadings[wallDistanceIndex] = wallDist;
    wallDistanceIndex = (wallDistanceIndex + 1) % 10;//Keep the array to store only the most recent 10 readings.

    // Safety check: emergency stop if too close
    if (wallDist < MIN_WALL_DISTANCE) {
        Serial.print("️  WALL TOO CLOSE: ");
        Serial.print(wallDist, 1);
        Serial.println(" cm - STOPPING");
        stopMotors();
        platformNavState = NAV_IDLE;
        return;
    }

    // Continue moving forward
    setMotorDirection(true);
    setMotorSpeed(FIRST_STRAIGHT_SPEED, FIRST_STRAIGHT_SPEED);

    // Check elapsed time/distance
    unsigned long elapsedTime = millis() - stateStartTime;
    int readingCount = wallDistanceIndex + 1;

    Serial.print(" First straight movement: ");
    Serial.print(elapsedTime / 100);  // Approximate distance in cm !!!NEED TO TUNE
    Serial.print("cm traveled | Wall distance: ");
    Serial.print(wallDist, 1);
    Serial.println(" cm");

    // After ~40cm (400ms) and distance data collected   !!!NEED TO TUNE
    if (elapsedTime > 400 && readingCount >= 5) {
        Serial.println("\n First straight movement complete");
        Serial.println("  Proceeding to Stage 4: First 90° Turn");

        platformNavState = NAV_FIRST_TURN;
        platformTurnCount = 1;
        platformTurnStartTime = millis();
        platformTurnYaw = 0.0;
        stateStartTime = millis();
    }
}

// ============================================================================
// STAGE 4: FIRST 90° TURN
// ============================================================================

/**
 * @brief Executes first 90° turn on platform
 *
 * @details Stage 4: Rotate 90° (right or left based on wall position)
 * - Uses gyroscope integration for angle tracking
 * - Differential speed control (outer 150, inner 50 PWM)
 * - Tolerance ±5° for platform turns
 *
 * Turn direction logic:
 * - Typically right turn (clockwise) for standard stairs
 * - Can be reversed if left turn needed
 *
 * @return void (updates state when turn complete)
 */
void executeFirstTurn() {
    // Update sensor data for angle tracking
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Calculate yaw rate
    float yawRate = gz / 65.5;  // Convert to °/s

    // Integrate yaw
    platformTurnYaw += yawRate * 0.1;  // dt = 0.1s

    float turnedAngle = abs(platformTurnYaw);

    // Debug output
    Serial.print(" Platform Turn 1: ");
    Serial.print(turnedAngle, 1);
    Serial.print("° / ");
    Serial.print(PLATFORM_TURN_ANGLE);
    Serial.println("°");

    // Execute differential turn   (DEFALUT: TURN RIGHT) WHEN TURN LEFT, !!!NEED TO TUNE!!! ALSO NEED TO TUNE THE PWM
    setMotorDirection(true);
    analogWrite(L298N_1_ENA, PLATFORM_TURN_SPEED_OUTER);
    analogWrite(L298N_1_ENB, PLATFORM_TURN_SPEED_OUTER);
    analogWrite(L298N_2_ENA, PLATFORM_TURN_SPEED_INNER);
    analogWrite(L298N_2_ENB, PLATFORM_TURN_SPEED_INNER);

    // Check if turn complete
    if (turnedAngle >= (PLATFORM_TURN_ANGLE - PLATFORM_TURN_TOLERANCE)) {
        stopMotors();
        Serial.println(" First 90° turn complete");
        Serial.println("  Proceeding to Stage 5: Approach Wall Distance");

        platformNavState = NAV_APPROACH_WALL;
        stateStartTime = millis();
    }

    // Timeout safety   !!!NEED TO TUNE
    if ((millis() - platformTurnStartTime) > 5000) {
        stopMotors();
        Serial.println(" Turn timeout");
        platformNavState = NAV_IDLE;
    }
}

// ============================================================================
// STAGE 5: APPROACH TARGET WALL DISTANCE
// ============================================================================

/**
 * @brief Moves forward to achieve target wall distance (~50cm)
 *
 * @details Stage 5: Approach until distance_to_wall ≈ 50cm
 * - Slower speed (60 PWM) for precision
 * - Continuous distance monitoring
 * - Adjusts movement based on distance:
 *   * If distance > 55cm: continue forward
 *   * If distance 45-55cm: at target, proceed to turn
 *   * If distance < 45cm: stop and potentially back up
 *
 * Precision control:
 * - Tolerance: ±5cm (45-55cm acceptable)
 * - Stop when within range
 *
 * @return void
 */
void approachWallDistance() {
    // Measure wall distance
    float wallDist = getWallDistance();

    Serial.print(" Approaching wall: ");
    Serial.print(wallDist, 1);
    Serial.print(" cm / Target: ");
    Serial.print(TARGET_WALL_DISTANCE);
    Serial.println(" cm");

    // Safety check
    if (wallDist < MIN_WALL_DISTANCE) {
        Serial.print("  CRITICAL: IMMEDIATE STOP ");
        Serial.print(wallDist, 1);
        Serial.println(" cm - STOPPING");
        stopMotors();
        platformNavState = NAV_IDLE;
        return;
    }

    // Check if at target distance
    float distanceFromTarget = abs(wallDist - TARGET_WALL_DISTANCE);

    if (distanceFromTarget <= WALL_DISTANCE_TOLERANCE) {  //reach to the turning point
        // Within tolerance range (45-55cm)
        Serial.print(" Target distance reached: ");
        Serial.print(wallDist, 1);
        Serial.println(" cm");

        stopMotors();
        delay(300);  // Brief pause for stability

        Serial.println("  Proceeding to Stage 6: Second 90° Turn");
        platformNavState = NAV_SECOND_TURN;
        platformTurnCount = 2;
        platformTurnStartTime = millis();
        platformTurnYaw = 0.0;
        return;
    }

    // Not at target yet
    if (wallDist > (TARGET_WALL_DISTANCE + WALL_DISTANCE_TOLERANCE)) {
        // Too far from wall - move forward
        setMotorDirection(true);
        setMotorSpeed(APPROACH_WALL_SPEED, APPROACH_WALL_SPEED);
    } else if (wallDist < (TARGET_WALL_DISTANCE - WALL_DISTANCE_TOLERANCE)) {
        // Too close to wall - back up slightly
        setMotorDirection(false);  // Reverse
        setMotorSpeed(60, 60);  // Slow reverse
    }
}

// ============================================================================
// STAGE 6: SECOND 90° TURN
// ============================================================================

/**
 * @brief Executes second 90° turn to face next stair
 *
 * @details Stage 6: Rotate another 90° (opposite direction to first turn)
 * - Total rotation from start: 180°
 * - Results in facing next flight of stairs
 * - Same angle integration method as first turn
 *
 * @return void
 */
void executeSecondTurn() {
    // Update sensor data
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Calculate yaw rate (note: may need to reverse sign)
    float yawRate = gz / 65.5;

    // Integrate yaw
    platformTurnYaw += yawRate * 0.1;
    float turnedAngle = abs(platformTurnYaw);

    // Debug output
    Serial.print("🔄 Platform Turn 2: ");
    Serial.print(turnedAngle, 1);
    Serial.print("° / ");
    Serial.print(PLATFORM_TURN_ANGLE);
    Serial.println("°");

    // Execute differential turn (opposite direction to first turn)
    setMotorDirection(true);
    analogWrite(L298N_2_ENA, PLATFORM_TURN_SPEED_OUTER);  // Swap L and R
    analogWrite(L298N_2_ENB, PLATFORM_TURN_SPEED_OUTER);
    analogWrite(L298N_1_ENA, PLATFORM_TURN_SPEED_INNER);
    analogWrite(L298N_1_ENB, PLATFORM_TURN_SPEED_INNER);

    // Check if turn complete
    if (turnedAngle >= (PLATFORM_TURN_ANGLE - PLATFORM_TURN_TOLERANCE)) {
        stopMotors();
        Serial.println("✓ Second 90° turn complete");
        Serial.println("➡️  Proceeding to Stage 7: Next Stair Detection");

        platformNavState = NAV_NEXT_STAIR_CHECK;
        stateStartTime = millis();
    }

    // Timeout safety
    if ((millis() - platformTurnStartTime) > 5000) {
        stopMotors();
        Serial.println("⚠️  Turn timeout");
        platformNavState = NAV_IDLE;
    }
}

// ============================================================================
// STAGE 7: NEXT STAIR DETECTION
// ============================================================================

/**
 * @brief Detects next stair and initiates climbing
 *
 * @details Stage 7: Verify stairs detected and resume climbing
 * - Distance < 20cm indicates stair detected
 * - Pitch angle will change as robot encounters stairs
 * - Automatically transitions to climbing mode
 *
 * @return void
 */
void checkNextStairCondition() {
    currentDistance = measureDistance();
    updateAttitude();

    Serial.print("🔍 Looking for next stair: ");
    Serial.print(currentDistance, 1);
    Serial.println(" cm");

    // Check if stairs detected
    if (currentDistance < NEXT_STAIR_DISTANCE) {
        Serial.println("\n╔════════════════════════════════════╗");
        Serial.println("║   NEXT STAIR DETECTED!             ║");
        Serial.println("║   Resuming climbing mode           ║");
        Serial.println("╚════════════════════════════════════╝");

        // Reset platform navigation
        platformNavState = NAV_IDLE;

        // Resume normal climbing control
        // Next loop will execute controlMotors() normally
    }
}

// ============================================================================
// MAIN PLATFORM NAVIGATION EXECUTOR
// ============================================================================

/**
 * @brief State machine executor for platform navigation
 *
 * @details Routes to correct stage based on platformNavState
 * - Manages complete platform transition
 * - Handles all 7 stages sequentially
 *
 * @return void
 */
void executePlatformNavigation() {
    switch (platformNavState) {
        case NAV_PLATFORM_LEVEL_CHECK:
            checkPlatformLevelCondition();
            break;

        case NAV_FIRST_STRAIGHT:
            executeFirstStraightMovement();
            break;

        case NAV_FIRST_TURN:
            executeFirstTurn();
            break;

        case NAV_APPROACH_WALL:
            approachWallDistance();
            break;

        case NAV_SECOND_TURN:
            executeSecondTurn();
            break;

        case NAV_NEXT_STAIR_CHECK:
            checkNextStairCondition();
            break;

        default:
            platformNavState = NAV_IDLE;
            break;
    }
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

/**
 * @brief Gets wall distance measurement (likely from side sensor)
 *
 * @details For now, use front HC-SR04 (TODO: add side sensor for better results)
 *
 * @return float Distance in cm
 */
float getWallDistance() {
    // TODO: In real implementation, add side-facing HC-SR04 sensor
    // For now, use front sensor as proxy
    return measureDistance();
}

/**
 * @brief Calculates average of recent wall distance readings
 *
 * @details Smooths noisy sensor data
 *
 * @return float Average distance in cm
 */
float getAverageWallDistance() {
    float sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += wallDistanceReadings[i];
    }
    return sum / 10.0;
}
// ============================================================================
// PLATFORM NAVIGATION - INITIATE FUNCTION (IMPLEMENTATION)
// ============================================================================

/**
 * @brief Initiates platform navigation state machine
 *
 * @details This function is called when both conditions are met:
 * - Condition 1: No stairs detected (HC-SR04 distance > 100cm)
 * - Condition 2: Platform level confirmed (MPU6050 pitch -5° ~ +5°)
 *
 * Initialization steps:
 * 1. Stop current motion for safety
 * 2. Reset all navigation state variables
 * 3. Set initial state to FIRST_STRAIGHT (Stage 3)
 * 4. Initialize wall distance tracking array
 * 5. Record start time and reset counters
 * 6. Provide user feedback via Serial
 *
 * This serves as the entry point to the 7-stage platform navigation sequence
 *
 * @return void (sets platformNavState to start navigation)
 */
void initiatePlatformNavigation() {
    // 1. Stop all current motion
    stopMotors();
    delay(300);  // Brief pause for stability

    // 2. Reset all navigation state variables
    platformNavState = NAV_FIRST_STRAIGHT;  // Start at Stage 3
    stateStartTime = millis();

    // Reset counters
    noStairConfirmCounter = 0;
    platformLevelConfirmCounter = 0;

    // Initialize wall distance tracking
    wallDistanceIndex = 0;
    for (int i = 0; i < 10; i++) {
        wallDistanceReadings[i] = 999.0;  // Initialize with max distance
    }

    // Reset turn tracking
    platformTurnCount = 0;
    platformTurnYaw = 0.0;

    // 3. Provide clear feedback
    Serial.println("\n╔══════════════════════════════════════════════════════╗");
    Serial.println("║                 PLATFORM NAVIGATION INITIATED        ║");
    Serial.println("╠══════════════════════════════════════════════════════╣");
    Serial.println("║   Stage 1   No stairs detected                       ║");
    Serial.println("║   Stage 2   Platform level confirmed                 ║");
    Serial.println("║                                                        ║");
    Serial.println("║       Entering Stage 3: First Straight Movement       ║");
    Serial.println("║                                                        ║");
    Serial.println("║   Next: Monitor wall distance → First 90° turn        ║");
    Serial.println("╚══════════════════════════════════════════════════════╝");

    Serial.print(" Platform Navigation Started - ");
    Serial.print(FIRST_STRAIGHT_DISTANCE);
    Serial.print("cm straight → Target wall distance: ");
    Serial.print(TARGET_WALL_DISTANCE);
    Serial.print("±");
    Serial.print(WALL_DISTANCE_TOLERANCE);
    Serial.println("cm");

    Serial.println("=====================================");
}
// ============================================================================
// PLATFORM NAVIGATION - STATE LOGGING FUNCTION (IMPLEMENTATION)
// ============================================================================

/**
 * @brief Logs current platform navigation state and key parameters
 *
 * @details Provides detailed debugging information for platform navigation
 * - Shows current stage number and name
 * - Displays state duration (elapsed time)
 * - Shows key sensor readings (pitch, distance)
 * - Logs wall distance tracking progress
 * - Displays next stage preview
 *
 * This function is called periodically during navigation to help
 * debugging and monitoring of the 7-stage process
 *
 * Output format:
 * [Stage X] State Name | Time: XXs | Pitch: XX.X° | Wall: XX.Xcm
 *
 * @return void
 */
void logNavigationState() {
    unsigned long elapsedTime = (millis() - stateStartTime) / 1000;

    Serial.print("[Stage ");
    Serial.print(platformNavState);
    Serial.print("] ");

    switch (platformNavState) {
        case NAV_IDLE:
            Serial.print("IDLE");
            break;
        case NAV_NO_STAIR_DETECTED:
            Serial.print("No Stair Detection");
            break;
        case NAV_PLATFORM_LEVEL_CHECK:
            Serial.print("Platform Level Check");
            break;
        case NAV_FIRST_STRAIGHT:
            Serial.print("First Straight Movement");
            break;
        case NAV_FIRST_TURN:
            Serial.print("First 90° Turn");
            break;
        case NAV_APPROACH_WALL:
            Serial.print("Approach Wall Distance");
            break;
        case NAV_SECOND_TURN:
            Serial.print("Second 90° Turn");
            break;
        case NAV_NEXT_STAIR_CHECK:
            Serial.print("Next Stair Detection");
            break;
        case NAV_COMPLETE:
            Serial.print("Navigation Complete");
            break;
        default:
            Serial.print("UNKNOWN");
            break;
    }

    Serial.print(" | Time: ");
    Serial.print(elapsedTime);
    Serial.print("s");

    // Add sensor data
    updateAttitude();
    currentDistance = measureDistance();

    Serial.print(" | Pitch: ");
    Serial.print(currentPitch, 1);
    Serial.print("°");

    Serial.print(" | Dist: ");
    Serial.print(currentDistance, 1);
    Serial.print("cm");

    // Wall distance specific info
    if (platformNavState == NAV_FIRST_STRAIGHT ||
        platformNavState == NAV_APPROACH_WALL) {
        float avgWallDist = getAverageWallDistance();
        Serial.print(" | WallAvg: ");
        Serial.print(avgWallDist, 1);
        Serial.print("cm");
    }

    // Turn progress
    if (platformNavState == NAV_FIRST_TURN ||
        platformNavState == NAV_SECOND_TURN) {
        Serial.print(" | Turn: ");
        Serial.print(abs(platformTurnYaw), 1);
        Serial.print("°/");
        Serial.print(PLATFORM_TURN_ANGLE);
        Serial.print("°");
    }

    Serial.println();

    // Preview next stage
    Serial.print("➡️  Next: ");
    switch (platformNavState) {
        case NAV_IDLE:
            Serial.println("Waiting for stair end detection");
            break;
        case NAV_NO_STAIR_DETECTED:
            Serial.println("Platform level confirmation");
            break;
        case NAV_PLATFORM_LEVEL_CHECK:
            Serial.println("First straight movement (40cm)");
            break;
        case NAV_FIRST_STRAIGHT:
            Serial.println("First 90° turn");
            break;
        case NAV_FIRST_TURN:
            Serial.println("Approach target wall distance (50cm)");
            break;
        case NAV_APPROACH_WALL:
            Serial.println("Second 90° turn");
            break;
        case NAV_SECOND_TURN:
            Serial.println("Detect next stair");
            break;
        case NAV_NEXT_STAIR_CHECK:
            Serial.println("Resume climbing");
            break;
        default:
            Serial.println("Complete navigation sequence");
            break;
    }
}

//Method to be tested
/**
 * @brief Advanced: Analyze echo signal characteristics
 *
 * @details Different materials have different acoustic properties:
 * -return 1 Stairs (wood/concrete steps): Multiple echoes (soft), scattered reflection
 * -return 0 Wall (smooth concrete): Strong single echo, direct reflection
 *
 * Note: Arduino pulseIn() doesn't directly measure signal strength,
 * but we can infer from echo pulse width consistency:
 */
bool isStairsNotWall() {
    // Multiple measurements were taken to analyze the stability of the echo pulse width.
    long echoPulses[5];
    for (int i = 0; i < 5; i++) {
        digitalWrite(HC_TRIG_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(HC_TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(HC_TRIG_PIN, LOW);

        echoPulses[i] = pulseIn(HC_ECHO_PIN, HIGH, 30000);
        delay(50);
    }

    // Calculate pulse width variance
    float pulseVariance = 0;
    float avgPulse = 0;
    for (int i = 0; i < 5; i++) {
        avgPulse += echoPulses[i];
    }
    avgPulse /= 5;

    for (int i = 0; i < 5; i++) {
        pulseVariance += abs(echoPulses[i] - avgPulse);
    }
    pulseVariance /= 5;

    // High variance = stairs (irregular reflection)
    // Low variance = walls (regular reflection)
    bool isStair = (pulseVariance > avgPulse * 0.05);  // 方差>5%

    return isStair;
}


bool isStairsNotWall2() {
    static float previousDistance = 100.0f;
    static int stairCnt = 0;
    static int wallCnt  = 0;

    currentDistance = measureDistance();


    // too far away
    if (currentDistance > 60.0f) {
        previousDistance = currentDistance;
        stairCnt = wallCnt = 0;
        return false;
    }

    float d = fabs(currentDistance - previousDistance);
    previousDistance = currentDistance;

    bool closeEnough = (currentDistance < 40.0f);//30-40
   // bool stairPitch  = (currentPitch < -3.0f && currentPitch > -25.0f);
  //  bool wallPitch   = (fabs(currentPitch) < 3.0f);

    // Distance variation characteristics: Staircases have many reflection points and exhibit significant variations; walls are flat and show little variation.
    bool stairPattern = (d > 1.5f);
    bool wallPattern  = (d < 0.5f);

    Serial.print(" Stair vs Wall: Dist=");
    Serial.print(currentDistance, 1);
    Serial.print("cm | d=");
    Serial.print(d, 2);
    Serial.print("cm | Pitch=");
    Serial.print(currentPitch, 1);
    Serial.print("° -> ");

    if (closeEnough && stairPattern ) {
        stairCnt++;
        wallCnt = 0;
        if (stairCnt >= 3) {   //3 times
            Serial.println("STAIR ");
            stairCnt = wallCnt = 0;
            return true;
        } else {
            Serial.print("STAIR?( ");
            Serial.print(stairCnt);
            Serial.println("/3 )");
            return false;
        }
    } else if (closeEnough && wallPattern) {
        wallCnt++;
        stairCnt = 0;
        if (wallCnt >= 3) {
            Serial.println("WALL ️");
            stairCnt = wallCnt = 0;
            return false;
        } else {
            Serial.print("WALL?( ");
            Serial.print(wallCnt);
            Serial.println("/3 )");
            return false;
        }
    } else {
        Serial.println("UNKNOWN");
        stairCnt = wallCnt = 0;
        return false;
    }
}