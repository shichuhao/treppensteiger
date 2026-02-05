#include "yaw_correction.h"

/**
 * @file yaw_correction.cpp
 * @brief Yaw estimation and drift correction using MPU6050 gyroscope.
 * 
 * @details This module integrates the Z-axis gyroscope data to track the 
 * robot's heading. It uses 32-bit integer math to avoid the performance 
 * penalty of floating-point operations while maintaining precision.
 */

// === Global Heading Variables ===
int16_t referenceYaw = 0;     ///< Target heading (degrees * 100)
int16_t currentYaw = 0;        ///< Current estimated heading (degrees * 100)
int16_t yawDeviation = 0;      ///< Difference between current and reference (degrees * 100)
uint32_t lastYawUpdate = 0;    ///< Timestamp of last integration step (ms)

// Internal Accumulator
static int32_t yawIntegral = 0;  ///< High-precision accumulator to prevent overflow during integration

/** @brief Constants for Gyroscope processing */
#define YAW_UPDATE_INTERVAL 20   ///< 20ms update rate (50Hz) for stable integration

/**
 * @brief Resets the heading reference.
 * @details Sets the current orientation as the new zero-point (North).
 */
void initYawCorrection() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  referenceYaw = 0;
  yawIntegral = 0;
  lastYawUpdate = millis();

  Serial.println(F("Yaw System: Reset to Reference"));
}

/**
 * @brief Integrates angular velocity to track heading.
 * 
 * @details Physical Logic:
 * 1. Read raw Z-axis gyroscope value (gz).
 * 2. Sensitivity at 250 deg/s is 131 LSB per deg/s.
 * 3. Delta Angle = (gz / 131) * (dt / 1000).
 * 4. To maintain deg * 100 precision without float:
 *    Delta = (gz * dt * 100) / (131 * 1000)
 */
void updateYaw() {
  uint32_t now = millis();
  uint16_t dt = now - lastYawUpdate; 

  if (dt >= YAW_UPDATE_INTERVAL) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    /** 
     * @section Fixed-Point Integration
     * Formula: (gz * dt * 100) / 131000
     * result is degrees * 100 
     */
    int32_t deltaYaw = ((int32_t)gz * dt * 100) / 131000;
    yawIntegral += deltaYaw;

    // Convert internal long-precision integral to standard degree * 100 format
    currentYaw = (int16_t)(yawIntegral);

    /** 
     * @section Normalization
     * Keeps the angle within the range of [-180.00°, 180.00°]
     */
    while (currentYaw > 18000)  currentYaw -= 36000;
    while (currentYaw < -18000) currentYaw += 36000;

    // Calculate deviation from the intended path
    yawDeviation = currentYaw - referenceYaw;

    lastYawUpdate = now;
  }
}

/**
 * @brief Returns the current heading error.
 * @return int16_t Deviation in degrees * 100 (e.g., 300 = 3.0° drift).
 */
int16_t getYawDeviation() {
  updateYaw(); // Ensure we have the latest data
  return yawDeviation; 
}
