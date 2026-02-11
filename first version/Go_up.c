/*
* ============================================================================
* Stair Climbing Robot - Climbing Control System
* Platform: Arduino Uno R3
* 
* USE CASES:
* UC01: Stair Distance Recognition (HC-SR04) and Data Processing
* UC02: Robot Attitude Detection (MPU6050) and Data Processing  
* UC03: Go up Motor Control
*
* Author: Gruppe 2
* Date: 2026-01-30
* Version: 1.0.0
* ============================================================================
*/
#include "Arduino.h"
#include <Wire.h>
#include "MPU6050.h"
#include "SoftwareSerial.h"
#include "avr/wdt.h"


// ============================================================================
// PIN DEFINITIONS (from UC00)
// ============================================================================

// ===== HC-SR04 Ultrasonic Sensor =====
#define HC_TRIG_PIN 3              // Trigger pin
#define HC_ECHO_PIN 2              // Echo pin

// ===== MPU6050 IMU Sensor =====
#define MPU6050_ADDRESS 0x68       // I2C address (AD0 = GND)

// ===== L298N Motor Driver 1 (Left Side - Motors A & B) =====
#define L298N_1_IN1 4              // Motor A direction 1
#define L298N_1_IN2 5              // Motor A direction 2
#define L298N_1_ENA 9              // Motor A speed (PWM, Timer1)
#define L298N_1_IN3 6              // Motor B direction 1
#define L298N_1_IN4 7              // Motor B direction 2
#define L298N_1_ENB 10             // Motor B speed (PWM, Timer1)

// ===== L298N Motor Driver 2 (Right Side - Motors C & D) =====
#define L298N_2_IN1 11             // Motor C direction 1
#define L298N_2_IN2 12             // Motor C direction 2
#define L298N_2_ENA 3              // Motor C speed (PWM, Timer2)
#define L298N_2_IN3 13             // Motor D direction 1
#define L298N_2_IN4 8              // Motor D direction 2
#define L298N_2_ENB 6              // Motor D speed (PWM, Timer0)

// ===== ESP8266 WiFi Module =====
#define ESP_RX_PIN 10              // SoftwareSerial RX
#define ESP_TX_PIN 11              // SoftwareSerial TX
#define ESP_BAUD 115200            // ESP8266 baud rate******************************************************

// ============================================================================
// GLOBAL CONSTANTS
//Should be changed while testing
// ============================================================================

// Distance thresholds (UC01)
#define STAIR_DISTANCE_THRESHOLD 20  // cm - minimum distance to detect stair
#define EMERGENCY_STOP_DISTANCE 5  // cm - emergency stop threshold///////////////
#define MAX_DISTANCE 400           // cm - maximum measurable distance
#define SAMPLE_COUNT 5             // Number of samples for distance averaging

// Attitude thresholds (UC02)
#define CLIMB_ANGLE_THRESHOLD 15  // degrees - pitch angle for climbing mode , maybe should less than 15
#define ROLL_ANGLE_LIMIT 20        // degrees - maximum allowable roll angle

// Motor control parameters (UC03)
#define NORMAL_SPEED 150           // PWM value for flat surface (0-255)
#define CLIMB_SPEED 200           // PWM value for climbing stairs (0-255)
#define APPROACH_SPEED 100         // PWM value when approaching stair

// System timing
#define SENSOR_UPDATE_INTERVAL 100 // ms - main loop update rate
#define WATCHDOG_TIMEOUT WDTO_2S   // 2 second watchdog timeout

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// Sensor objects
MPU6050 mpu(MPU6050_ADDRESS);
SoftwareSerial espSerial(ESP_RX_PIN, ESP_TX_PIN);

// System state flags (from UC00)
volatile bool mpuReady = false;
volatile bool hcReady = false;
volatile bool espReady = false;
volatile bool motorsReady = false;

// Sensor data (UC01 & UC02)
float currentDistance = 0.0;       // Current measured distance in cm
float currentPitch = 0.0;          // Current pitch angle in degrees
float currentRoll = 0.0;           // Current roll angle in degrees
int16_t ax, ay, az;                // Raw accelerometer data
int16_t gx, gy, gz;                // Raw gyroscope data

// Control state (UC03)
bool isClimbingMode = false;       // Flag for climbing mode status
bool emergencyStop = false;        // Emergency stop flag
uint8_t robotState = 0;            // 0=stopped, 1=normal, 2=approaching, 3=climbing

// Timing variables
unsigned long initStartTime = 0;
unsigned long lastSensorUpdate = 0;

// Error tracking
uint8_t initErrorCode = 0;         // Bit field: 0x01=MPU, 0x02=HC-SR04, 0x04=ESP8266

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

// Use case functions
float measureDistance();      // UC01: Distance measurement
void updateAttitude();        // UC02: Attitude detection
void controlMotors();         // UC03: Motor control (integrates UC01 & UC02)
void setMotorDirectionDifferential(bool leftForward, bool rightForward);
// Motor control helper functions
void setMotorSpeed(int leftSpeed, int rightSpeed);
void stopMotors();
void setMotorDirection(bool forward);


// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Reset watchdog timer to prevent system reset
  wdt_reset();
  
  // Check if enough time has passed for sensor update
  unsigned long currentTime = millis();
  if (currentTime - lastSensorUpdate >= SENSOR_UPDATE_INTERVAL) {
    lastSensorUpdate = currentTime;
    
    // Main control function - integrates all use cases
    // This function calls UC01 (distance) and UC02 (attitude) internally
    controlMotors();
  }
  
  // Small delay to prevent CPU overload
  delay(10);
}

// ============================================================================
// UC01: STAIR DISTANCE RECOGNITION AND DATA PROCESSING
// ============================================================================

/**
 * @brief Measures distance to obstacle/stair using HC-SR04 ultrasonic sensor
 * 
 * @details This function implements UC01: Stair Distance Recognition
 * - Sends 10μs trigger pulse to HC-SR04 sensor
 * - Measures echo pulse duration to calculate distance
 * - Takes multiple samples and returns median value for noise reduction
 * - Filters out invalid readings (0 or > MAX_DISTANCE)
 * - Uses median filter instead of average for better outlier rejection
 * 
 * Libraries Used:
 * - Arduino.h (digitalWrite, digitalRead, pulseIn, delayMicroseconds)
 * 
 * Technical Notes:
 * - Speed of sound: 343 m/s = 0.0343 cm/μs
 * - Distance = (echo_duration * 0.0343) / 2 (round trip)
 * - Simplified: distance = echo_duration / 58.0 (or * 0.01715)
 * - Timeout set to 30ms for max range ~500cm
 * 
 * @return float Distance in centimeters (5-400 cm valid range)
 */
float measureDistance() {
  float samples[SAMPLE_COUNT];
  
  // Take multiple samples for statistical filtering
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    // Clear the trigger pin
    digitalWrite(HC_TRIG_PIN, LOW);
    delayMicroseconds(2);
    
    // Send 10μs HIGH pulse to trigger sensor
    digitalWrite(HC_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(HC_TRIG_PIN, LOW);
    
    // Measure echo pulse duration (timeout after 30ms = ~500cm)
    long duration = pulseIn(HC_ECHO_PIN, HIGH, 30000);//Arduino.h  sd-rcv
    
    // Calculate distance in centimeters
    // Formula: distance = (duration * speed_of_sound) / 2
    // With speed_of_sound = 343 m/s = 0.0343 cm/μs
    float distance = duration * 0.01715;//cm
    
    // Store valid readings only, replace invalid with MAX_DISTANCE
    if (distance > 0 && distance <= MAX_DISTANCE) {
      samples[i] = distance;
    } else {
      samples[i] = MAX_DISTANCE;  // Treat invalid as maximum distance ~22ms
    }
    
    delay(10);  // Small delay between samples (10ms * 5 = 50ms total)
  }
  
  // Bubble sort to find median value  S-----L
  // Median is more robust than average for noisy sensor data
  for (int i = 0; i < SAMPLE_COUNT - 1; i++) {
    for (int j = i + 1; j < SAMPLE_COUNT; j++) {
      if (samples[i] > samples[j]) {
        float temp = samples[i];
        samples[i] = samples[j];
        samples[j] = temp;
      }
    }
  }
  
  // Return median value (middle element of sorted array)
  return samples[SAMPLE_COUNT / 2];
}

// ============================================================================
// UC02: ROBOT ATTITUDE DETECTION AND DATA PROCESSING
// ============================================================================

/**
 * @brief Reads and processes attitude data from MPU6050 IMU sensor
 * 
 * @details This function implements UC02: Robot Attitude Detection
 * - Reads raw accelerometer and gyroscope data via I2C
 * - Calculates pitch (forward/backward tilt) and roll (left/right tilt) angles
 * - Uses accelerometer-based angle calculation for low-frequency stability
 * - Determines if robot is on an incline (climbing mode detection)
 * - Detects excessive roll angle for safety
 * 
 * Libraries Used:
 * - Wire.h (I2C communication with MPU6050)
 * - MPU6050.h (sensor interface library)
 * 
 * Angle Calculation:
 * - Pitch = atan2(-ax, sqrt(ay² + az²)) * 180/π
 * - Roll = atan2(ay, az) * 180/π
 * - Uses accelerometer only (gyro integration would require complementary filter)
 * 
 * Global Variables Modified:
 * - currentPitch: Pitch angle in degrees (positive = nose up)
 * - currentRoll: Roll angle in degrees (positive = right side up)
 * - isClimbingMode: Set to true if pitch exceeds CLIMB_ANGLE_THRESHOLD
 * 
 * @return void (updates global variables)
 */
void updateAttitude() {
  // Read raw sensor data from MPU6050
  // ax, ay, az: Accelerometer readings (includes gravity + linear acceleration)
  // gx, gy, gz: Gyroscope readings (angular velocity)
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Calculate pitch angle (rotation around Y-axis, forward/backward tilt)
  // When climbing stairs, pitch angle increases (nose up = positive)
  // Formula: pitch = atan2(-ax, sqrt(ay² + az²)) * 180/π
  // Note: -ax because accelerometer measures opposite to tilt direction
  currentPitch = atan2(-ax, sqrt((long)ay*ay + (long)az*az)) * 180.0 / PI;
  
  // Calculate roll angle (rotation around X-axis, left/right tilt)
  // Used to detect if robot is tilting sideways (stability check)
  // Formula: roll = atan2(ay, az) * 180/π
  currentRoll = atan2(ay, az) * 180.0 / PI;//Prevent rollover
  
  // Determine if robot is in climbing mode based on pitch angle
  // Using absolute value to handle both uphill and downhill
  if (abs(currentPitch) > CLIMB_ANGLE_THRESHOLD) {
    isClimbingMode = true;
  } else {
    isClimbingMode = false;
  }
  
  // Check for excessive roll angle (stability warning)
  if (abs(currentRoll) > ROLL_ANGLE_LIMIT) {
    Serial.println(" WARNING: Excessive roll angle detected!");
  }
  
  // Debug output to Serial Monitor
  Serial.print("Pitch: ");
  Serial.print(currentPitch, 1);  // 1 decimal place
  Serial.print("° | Roll: ");
  Serial.print(currentRoll, 1);
  Serial.print("° | Climbing: ");
  Serial.println(isClimbingMode ? "YES" : "NO");
}

// ============================================================================
// UC03: UPHILL MOTOR CONTROL (INTEGRATES UC01 & UC02)
// ============================================================================

/**
 * @brief Integrated motor control based on distance and attitude data
 * 
 * @details This function implements UC03: Uphill Motor Control
 * - Combines distance sensor data (UC01) and attitude data (UC02)
 * - Implements decision logic for motor speed and direction control
 * - Manages multiple operating states: stopped, normal, approaching, climbing
 * - Provides emergency stop if obstacle too close
 * - Adjusts motor speed based on terrain (flat vs incline)
 * 
 * Libraries Used:
 * - None directly (calls UC01 and UC02 functions, uses motor control helpers)
 * 
 * Control States:
 * - State 0 (Stopped): Robot is stationary, emergency stop active
 * - State 1 (Normal): Flat surface, standard speed, no obstacles
 * - State 2 (Approaching): Stair detected ahead, reduce speed to prepare
 * - State 3 (Climbing): On incline, high power mode for climbing
 * 
 * Decision Logic:
 * 1. Emergency stop: Distance < 5cm → STOP immediately
 * 2. Stair approach: Distance < threshold + climbing → High power
 * 3. Stair detected: Distance < threshold → Prepare to climb
 * 4. On incline: Pitch > threshold → Maintain climb power
 * 5. Normal operation: Flat surface, no obstacles → Standard speed
 * 
 * @return void (controls motors via helper functions)
 */
void controlMotors() {
  // ===== STEP 1: Acquire sensor data from UC01 and UC02 =====
  
  // Call UC01 to measure distance to obstacle/stair
  currentDistance = measureDistance();
    Serial.println("=====================================");
    Serial.print("Distance: ");
    Serial.print(currentDistance, 1);//decimal 1
    Serial.println(" cm");
  // Call UC02 to update robot attitude (pitch and roll)
  updateAttitude();
  
  // ===== STEP 2: Implement decision logic for motor control =====
  
  // PRIORITY 1: Emergency stop if obstacle very close
  // This prevents collision with walls or sudden obstacles
  if (currentDistance < EMERGENCY_STOP_DISTANCE) {          // Abstand<5cm,stop maybe 2cm 3cm
    stopMotors();
    emergencyStop = true;
    robotState = 0;  // Stopped state
    Serial.println(" EMERGENCY STOP - Obstacle too close!");
    Serial.println("=====================================\n");
    return;  // Exit function immediately
  } else {
    emergencyStop = false;  // Clear emergency stop flag
  }
  
  // Detect if stair is ahead (distance less than threshold)
  bool stairDetected = (currentDistance < STAIR_DISTANCE_THRESHOLD);
  
  // PRIORITY 2: Active climbing mode (on incline with stair detected)
  // Robot is physically tilted and stair is detected → Maximum power needed
  if (isClimbingMode && stairDetected) {                        //Pitch>15 Degree && Abstand <20cm
    setMotorSpeed(CLIMB_SPEED, CLIMB_SPEED);
    setMotorDirection(true);  // Forward
    robotState = 3;  // Climbing state
    Serial.println("CLIMBING MODE - High power engaged");
  }
  
  // PRIORITY 3: Stair detected but not yet climbing
  // Prepare to climb by increasing power before reaching stair
  else if (stairDetected && !isClimbingMode) {              //Abstand <20cm && ! Pitch>15
    setMotorSpeed(APPROACH_SPEED, APPROACH_SPEED);
    setMotorDirection(true);  // Forward
    robotState = 2;  // Approaching state
    Serial.println(" STAIR DETECTED - Approaching");
  }
  
  // PRIORITY 4: On incline but no immediate obstacle
  // Maintain climbing power to continue up slope
  else if (isClimbingMode && !stairDetected) {
    setMotorSpeed(CLIMB_SPEED, CLIMB_SPEED);
    setMotorDirection(true);  // Forward
    robotState = 3;  // Climbing state
    Serial.println("ON INCLINE - Maintaining climb power");
  }
  
  // PRIORITY 5: Normal operation on flat surface
  // Standard speed for flat terrain with clear path ahead
  else {
    setMotorSpeed(NORMAL_SPEED, NORMAL_SPEED);
    setMotorDirection(true);  // Forward
    robotState = 1;  // Normal state
    Serial.println("NORMAL MODE - Standard speed");
  }
  
  Serial.println("=====================================\n");
}

// ============================================================================
// MOTOR CONTROL HELPER FUNCTIONS
// ============================================================================

/**
 * @brief Sets individual motor speeds for left and right sides
 * 
 * @details Controls all 4 motors (2 left + 2 right) using PWM
 * - Left side: Motors A and B (L298N_1)
 * - Right side: Motors C and D (L298N_2)
 * - Allows differential drive for turning (not used in basic climbing)
 * 
 * @param leftSpeed PWM value for left motors (0-255)
 * @param rightSpeed PWM value for right motors (0-255)
 * @return void
 */
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  // Constrain values to valid PWM range (0-255)
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
  // Set speed for left side motors (A and B)
  analogWrite(L298N_1_ENA, leftSpeed);   // Motor A speed
  analogWrite(L298N_1_ENB, leftSpeed);   // Motor B speed
  
  // Set speed for right side motors (C and D)
  analogWrite(L298N_2_ENA, rightSpeed);  // Motor C speed
  analogWrite(L298N_2_ENB, rightSpeed);  // Motor D speed
}

/**
 * @brief Sets motor direction for all 4 motors
 * 
 * @details Controls H-bridge direction pins for forward/reverse
 * - L298N logic: IN1=HIGH, IN2=LOW → Forward
 * - L298N logic: IN1=LOW, IN2=HIGH → Reverse
 * 
 * @param forward true = forward, false = reverse
 * @return void
 */
void setMotorDirection(bool forward) {
  if (forward) {
    // Left side motors - forward
    digitalWrite(L298N_1_IN1, HIGH);
    digitalWrite(L298N_1_IN2, LOW);
    digitalWrite(L298N_1_IN3, HIGH);
    digitalWrite(L298N_1_IN4, LOW);
    
    // Right side motors - forward
    digitalWrite(L298N_2_IN1, HIGH);
    digitalWrite(L298N_2_IN2, LOW);
    digitalWrite(L298N_2_IN3, HIGH);
    digitalWrite(L298N_2_IN4, LOW);
  } else {
    // Left side motors - reverse
    digitalWrite(L298N_1_IN1, LOW);
    digitalWrite(L298N_1_IN2, HIGH);
    digitalWrite(L298N_1_IN3, LOW);
    digitalWrite(L298N_1_IN4, HIGH);
    
    // Right side motors - reverse
    digitalWrite(L298N_2_IN1, LOW);
    digitalWrite(L298N_2_IN2, HIGH);
    digitalWrite(L298N_2_IN3, LOW);
    digitalWrite(L298N_2_IN4, HIGH);
  }
}

void setMotorDirectionDifferential(bool leftForward, bool rightForward) {
    // l
    if (leftForward) {
        digitalWrite(L298N_1_IN1, HIGH);
        digitalWrite(L298N_1_IN3, HIGH);
    } else {
        digitalWrite(L298N_1_IN1, LOW);
        digitalWrite(L298N_1_IN3, LOW);
    }

    // r
    if (rightForward) {
        digitalWrite(L298N_2_IN1, HIGH);
        digitalWrite(L298N_2_IN3, HIGH);
    } else {
        digitalWrite(L298N_2_IN1, LOW);
        digitalWrite(L298N_2_IN3, LOW);
    }
}


//setMotorDirectionDifferential(true, false);
//setMotorDirectionDifferential(false, true);
//setMotorDirectionDifferential(true, true);

/**
 * @brief Emergency stop - immediately halts all motors
 * 
 * @details Two methods for stopping:
 * - Brake mode: Both direction pins LOW (active braking)
 * - Coast mode: PWM to 0 (free-wheeling stop)
 * Current implementation uses brake mode for faster stop
 * 
 * @return void
 */
void stopMotors() {
  // Method 1: Brake mode (both direction pins LOW)
  digitalWrite(L298N_1_IN1, LOW);
  digitalWrite(L298N_1_IN2, LOW);
  digitalWrite(L298N_1_IN3, LOW);
  digitalWrite(L298N_1_IN4, LOW);
  digitalWrite(L298N_2_IN1, LOW);
  digitalWrite(L298N_2_IN2, LOW);
  digitalWrite(L298N_2_IN3, LOW);
  digitalWrite(L298N_2_IN4, LOW);
  
  // Set all speed controls to 0
  analogWrite(L298N_1_ENA, 0);
  analogWrite(L298N_1_ENB, 0);
  analogWrite(L298N_2_ENA, 0);
  analogWrite(L298N_2_ENB, 0);
}
