#include "init.h"
#include <Wire.h>

/**
 * @file init.cpp
 * @brief Hardware initialization and low-level motor/sensor control.
 * 
 * @details This module handles:
 * - Pin configuration for L298N motor drivers and Ultrasonic sensors.
 * - MPU6050 IMU setup, filtering, and calibration.
 * - Basic motion primitives (Forward, Backward, Rotation).
 * - Fixed-point pitch calculation for platform leveling.
 */

// === Global Objects & State Variables ===
MPU6050 mpu;                ///< Global IMU object for reading orientation data
uint32_t initStartTime = 0; ///< Global timestamp for timing-sensitive operations

/**
 * @brief Configures I/O pins for motors and sensors.
 * @details Initializes all motor driver pins as OUTPUT and sets up 
 * the ultrasonic TRIG/ECHO interface. Ensures motors are stopped initially.
 */
void setupPins() {
  /* --- Motor FL (Front Left) --- */
  pinMode(MOTOR_FL_IN1, OUTPUT);
  pinMode(MOTOR_FL_IN2, OUTPUT);
  pinMode(MOTOR_FL_PWM, OUTPUT);

  /* --- Motor FR (Front Right) --- */
  pinMode(MOTOR_FR_IN1, OUTPUT);
  pinMode(MOTOR_FR_IN2, OUTPUT);
  pinMode(MOTOR_FR_PWM, OUTPUT);

  /* --- Motor BL (Back Left) --- */
  pinMode(MOTOR_BL_IN1, OUTPUT);
  pinMode(MOTOR_BL_IN2, OUTPUT);
  pinMode(MOTOR_BL_PWM, OUTPUT);

  /* --- Motor BR (Back Right) --- */
  pinMode(MOTOR_BR_IN1, OUTPUT);
  pinMode(MOTOR_BR_IN2, OUTPUT);
  pinMode(MOTOR_BR_PWM, OUTPUT);

  /* --- Ultrasonic Sensor --- */
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initial safety state
  stopAllMotors();
}

/**
 * @brief Initializes the MPU6050 IMU via I2C.
 * @details Sets I2C clock to 400kHz (Fast Mode), configures 
 * gyro/accel ranges and enables the Digital Low Pass Filter (DLPF).
 */
void initializeMPU6050() {
  Wire.begin();               // Initialize I2C Communication
  Wire.setClock(400000L);     // Set I2C frequency to 400 kHz

  mpu.initialize();           // Set default MPU6050 configuration

  // Connection check: halt if hardware is not responding
  if (!mpu.testConnection()) {
    Serial.println(F("CRITICAL: MPU6050 connection failed"));
    while(1);                 // Hang indefinitely for safety
  }

  // Range Configuration:
  // Gyro: +/- 250 degrees/sec
  // Accel: +/- 2g sensitivity
  // DLPF: 20Hz bandwidth to reduce vibration noise from motors
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setDLPFMode(MPU6050_DLPF_BW_20);

  Serial.println(F("MPU6050 Initialized OK"));
}

/**
 * @brief Computes sensor offsets for stationary calibration.
 * @details Samples 200 raw data points to find zero-error offsets.
 * Gravity (1g) is accounted for on the Z-axis (16384 LSB at 2g range).
 */
void calibrateMPU6050() {
  Serial.println(F("Calibrating IMU... Hold robot still."));

  int32_t axSum = 0, aySum = 0, azSum = 0;
  int32_t gxSum = 0, gySum = 0, gzSum = 0;
  const uint8_t samples = 200;    

  for (uint8_t i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    axSum += ax; aySum += ay; azSum += az;
    gxSum += gx; gySum += gy; gzSum += gz;
    delay(5); 
  }

  // Calculate average error and invert for offset
  int16_t axOffset = -(axSum / samples);
  int16_t ayOffset = -(aySum / samples);
  int16_t azOffset = 16384 - (azSum / samples); // Standard gravity at 2g range
  int16_t gxOffset = -(gxSum / samples);
  int16_t gyOffset = -(gySum / samples);
  int16_t gzOffset = -(gzSum / samples);

  // Apply hardware offsets to MPU internal registers
  mpu.setXAccelOffset(axOffset);
  mpu.setYAccelOffset(ayOffset);
  mpu.setZAccelOffset(azOffset);
  mpu.setXGyroOffset(gxOffset);
  mpu.setYGyroOffset(gyOffset);
  mpu.setZGyroOffset(gzOffset);

  Serial.println(F("Calibration Complete"));
}

/**
 * @brief Checks if the robot platform is currently level.
 * @return bool True if pitch is within specified min/max thresholds.
 */
bool checkPlatformLevel() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Fixed-point pitch calculation (degrees * 100)
  // atan2 result in rad * 180/pi * 100 -> degrees * 100
  int16_t pitch = (int16_t)(
      atan2(ay, sqrt((long)ax*ax + (long)az*az)) * 5729.58
  ); 

  return (pitch >= PLATFORM_PITCH_MIN && pitch <= PLATFORM_PITCH_MAX);
}

/**
 * @brief Emergency stop for all 4 motors.
 * @details Sets IN1/IN2 to LOW for all bridges and sets PWM to 0.
 */
void stopAllMotors() {
  digitalWrite(MOTOR_FL_IN1, LOW); digitalWrite(MOTOR_FL_IN2, LOW);
  digitalWrite(MOTOR_FR_IN1, LOW); digitalWrite(MOTOR_FR_IN2, LOW);
  digitalWrite(MOTOR_BL_IN1, LOW); digitalWrite(MOTOR_BL_IN2, LOW);
  digitalWrite(MOTOR_BR_IN1, LOW); digitalWrite(MOTOR_BR_IN2, LOW);

  analogWrite(MOTOR_FL_PWM, 0);
  analogWrite(MOTOR_FR_PWM, 0);
  analogWrite(MOTOR_BL_PWM, 0);
  analogWrite(MOTOR_BR_PWM, 0);
}

/**
 * @brief Triggers and reads the HC-SR04 ultrasonic sensor.
 * @return uint16_t Distance in units of 0.1cm (10.0cm = 100).
 * @note Timeout is set to 30ms (~5m range).
 */
uint16_t measureDistance() {
  // Trigger a 10us pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Capture echo pulse duration in microseconds
  uint32_t duration = pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration == 0) return 9999; // Out of range indicator

  // Calculation: (duration / 2) * speed of sound (0.0343 cm/us)
  // Simplified for integer math: (duration * 17) / 1000 -> cm
  // We multiply result to get cm * 10 (0.1cm resolution)
  return (uint16_t)((duration * 17) / 100);
}

/**
 * @brief Drives all motors in the forward direction.
 * @param speed PWM intensity (0-255).
 */
void motorsForward(uint8_t speed) {
  digitalWrite(MOTOR_FL_IN1, HIGH); digitalWrite(MOTOR_FL_IN2, LOW);
  digitalWrite(MOTOR_FR_IN1, HIGH); digitalWrite(MOTOR_FR_IN2, LOW);
  digitalWrite(MOTOR_BL_IN1, HIGH); digitalWrite(MOTOR_BL_IN2, LOW);
  digitalWrite(MOTOR_BR_IN1, HIGH); digitalWrite(MOTOR_BR_IN2, LOW);

  analogWrite(MOTOR_FL_PWM, speed);
  analogWrite(MOTOR_FR_PWM, speed);
  analogWrite(MOTOR_BL_PWM, speed);
  analogWrite(MOTOR_BR_PWM, speed);
}

/**
 * @brief Drives all motors in the backward direction.
 * @param speed PWM intensity (0-255).
 */
void motorsBackward(uint8_t speed) {
  digitalWrite(MOTOR_FL_IN1, LOW); digitalWrite(MOTOR_FL_IN2, HIGH);
  digitalWrite(MOTOR_FR_IN1, LOW); digitalWrite(MOTOR_FR_IN2, HIGH);
  digitalWrite(MOTOR_BL_IN1, LOW); digitalWrite(MOTOR_BL_IN2, HIGH);
  digitalWrite(MOTOR_BR_IN1, LOW); digitalWrite(MOTOR_BR_IN2, HIGH);

  analogWrite(MOTOR_FL_PWM, speed);
  analogWrite(MOTOR_FR_PWM, speed);
  analogWrite(MOTOR_BL_PWM, speed);
  analogWrite(MOTOR_BR_PWM, speed);
}

/**
 * @brief Executes a zero-point left turn (tank turn).
 * @param speed PWM intensity (0-255).
 */
void motorsRotateLeft(uint8_t speed) {
  // Left side backward, right side forward
  digitalWrite(MOTOR_FL_IN1, LOW);  digitalWrite(MOTOR_FL_IN2, HIGH);
  digitalWrite(MOTOR_FR_IN1, HIGH); digitalWrite(MOTOR_FR_IN2, LOW);
  digitalWrite(MOTOR_BL_IN1, LOW);  digitalWrite(MOTOR_BL_IN2, HIGH);
  digitalWrite(MOTOR_BR_IN1, HIGH); digitalWrite(MOTOR_BR_IN2, LOW);

  analogWrite(MOTOR_FL_PWM, speed);
  analogWrite(MOTOR_FR_PWM, speed);
  analogWrite(MOTOR_BL_PWM, speed);
  analogWrite(MOTOR_BR_PWM, speed);
}

/**
 * @brief Executes a zero-point right turn (tank turn).
 * @param speed PWM intensity (0-255).
 */
void motorsRotateRight(uint8_t speed) {
  // Left side forward, right side backward
  digitalWrite(MOTOR_FL_IN1, HIGH); 
  digitalWrite(MOTOR_FL_IN2, LOW);
  digitalWrite(MOTOR_FR_IN1, LOW); 
  digitalWrite(MOTOR_FR_IN2, HIGH);
  digitalWrite(MOTOR_BL_IN1, HIGH); 
  digitalWrite(MOTOR_BL_IN2, LOW);
  digitalWrite(MOTOR_BR_IN1, LOW); 
  digitalWrite(MOTOR_BR_IN2, HIGH);

  analogWrite(MOTOR_FL_PWM, speed);
  analogWrite(MOTOR_FR_PWM, speed);
  analogWrite(MOTOR_BL_PWM, speed);
  analogWrite(MOTOR_BR_PWM, speed);
}
