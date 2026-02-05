/*
 * ============================================================================
 * Stair-Climbing Robot - Main Control Program
 * Arduino UNO R3 with MPU6050, HC-SR04, ESP8266, 2x L298N Motor Drivers
 * 
 * Integrated Modules:
 * - UC00: System Initialization
 * - Go_up: Uphill climbing control
 * - Downhill: Downhill descent control
 * - Platform_turn: Platform navigation and turning
 * - Remote_stop: Remote control via ESP8266
 * - Yaw_correction: Gyroscope-based yaw correction
 * 
 * Author: Gruppe 2
 * Date: 2026-02-05
 * Version: 2.0.0
 * ============================================================================
 */
 #include "init.h"

//#include <Wire.h>
//#include <SoftwareSerial.h>
//#include "MPU6050.h"

//#include "go_up.h"
//#include "downhill.h"
#include "platform_turn.h"
//#include "remote_stop.h"
#include "yaw_correction.h"


#define SENSOR_UPDATE_INTERVAL 100 // ms - main loop update rate

// ============================================================================
// GLOBAL VARIABLE DEFINITIONS (from init.h)
// ============================================================================
MPU6050 mpu(MPU6050_ADDRESS);
SoftwareSerial espSerial(ESP_RX_PIN, ESP_TX_PIN);

volatile bool mpuReady = false;
volatile bool hcReady = false;
volatile bool espReady = false;
volatile bool motorsReady = false;

unsigned long initStartTime = 0;
uint8_t initErrorCode = 0;

// ============================================================================
// GLOBAL VARIABLE DEFINITIONS (from go_up.h)
// ============================================================================
float currentDistance = 0.0;
float currentPitch = 0.0;
float currentRoll = 0.0;
int16_t ax, ay, az;
int16_t gx, gy, gz;

bool isClimbingMode = false;
bool emergencyStop = false;
uint8_t robotState = 0;

unsigned long lastSensorUpdate = 0;

// ============================================================================
// GLOBAL VARIABLE DEFINITIONS (from downhill.h)
// ============================================================================
bool isDownhillMode = false;
bool isDescending = false;
float maxDownhillPitch = 0.0;
unsigned long downhillStartTime = 0;

int downhillExitConfirmCount = 0;
unsigned long platformLevelStartTime = 0;
bool platformLevelDetecting = false;

// ============================================================================
// GLOBAL VARIABLE DEFINITIONS (from platform_turn.h)
// ============================================================================
PlatformNavState platformNavState = NAV_IDLE;
unsigned long stateStartTime = 0;
unsigned long distanceMeasurementTime = 0;

int noStairConfirmCounter = 0;
int platformLevelConfirmCounter = 0;

float wallDistanceReadings[5];
int wallDistanceIndex = 0;

int platformTurnCount = 0;
float platformTurnYaw = 0.0;
unsigned long platformTurnStartTime = 0;

// ============================================================================
// GLOBAL VARIABLE DEFINITIONS (from remote_stop.h)
// ============================================================================
bool remoteStopRequested = false;
// emergencyStop already defined in go_up section
unsigned long lastCmdTime = 0;
char cmdBuffer[CMD_BUFFER_SIZE];
int cmdIndex = 0;

// ============================================================================
// GLOBAL VARIABLE DEFINITIONS (from yaw_correction.h)
// ============================================================================
float referenceYaw = 0.0f;
float currentYaw = 0.0f;
float yawDeviation = 0.0f;
bool yawCorrectionActive = false;
unsigned long lastYawUpdate = 0;

// ============================================================================
// ARDUINO SETUP FUNCTION
// ============================================================================
void setup() {
  initStartTime = millis();
  
  // Initialize Serial for debugging
  Serial.begin(9600);
  delay(500);
  
  Serial.println("\n╔════════════════════════════════════╗");
  Serial.println("║  STAIRBOT MAIN CONTROL v2.0       ║");
  Serial.println("╚════════════════════════════════════╝");
  Serial.print("Startup Time: ");
  Serial.println(millis());
  Serial.println();
  
  // Step 1: Initialize GPIO pins
  Serial.println("[1/9] Initializing GPIO pins...");
  initializeGPIO();
  delay(100);
  
  // Step 2: Configure PWM frequency
  Serial.println("[2/9] Configuring PWM frequency...");
  initializePWM();
  delay(100);
  
  // Step 3: Initialize MPU6050
  Serial.println("[3/9] Initializing MPU6050 IMU sensor...");
  initializeMPU6050();
  delay(500);
  
  // Step 4: Initialize HC-SR04
  Serial.println("[4/9] Initializing HC-SR04 ultrasonic sensor...");
  initializeHCSR04();
  delay(100);
  
  // Step 5: Initialize L298N Motor Drivers
  Serial.println("[5/9] Initializing L298N motor drivers...");
  initializeL298N();
  delay(100);
  
  // Step 6: Initialize ESP8266 WiFi Module
  Serial.println("[6/9] Initializing ESP8266 WiFi module...");
  initializeESP8266();
  delay(500);
  
  // Step 7: Setup interrupts
  Serial.println("[7/9] Setting up interrupts...");
  setupInterrupts();
  delay(100);
  
  // Step 8: Setup Watchdog Timer
  Serial.println("[8/9] Enabling watchdog timer...");
  setupWatchdog();
  delay(100);
  
  // Step 9: Initialize Yaw Correction Module
  Serial.println("[9/9] Initializing yaw correction...");
  initializeYawReference();
  delay(100);
  
  // Print final status
  Serial.println("\n═══════════════════════════════════");
  Serial.println("   INITIALIZATION STATUS");
  Serial.println("═══════════════════════════════════");
  printInitStatus();
  
  unsigned long initTime = millis() - initStartTime;
  Serial.print("Total Initialization Time: ");
  Serial.print(initTime);
  Serial.println(" ms");
  
  Serial.println("\n╔════════════════════════════════════╗");
  Serial.println("║      SYSTEM READY - STARTING       ║");
  Serial.println("╚════════════════════════════════════╝\n");
  
  delay(1000);
}

// ============================================================================
// ARDUINO MAIN LOOP
// ============================================================================
void loop() {
  // Reset watchdog timer to prevent system reset
  wdt_reset();
  
  // Priority 0: Check for remote commands (highest priority for safety)
  processRemoteCommands_Stop();
  
  // If remote stop is active, don't execute other control logic
  if (remoteStopRequested || emergencyStop) {
    stopMotors();
    delay(100);
    return;
  }
  
  // Update sensors at regular intervals
  unsigned long currentTime = millis();
  if (currentTime - lastSensorUpdate >= SENSOR_UPDATE_INTERVAL) {
    lastSensorUpdate = currentTime;
    
    // Priority 1: Downhill mode (safety critical)
    if (detectDownhill_Downhill()) {
      controlDownhillMotors_Downhill();
      platformNavState = NAV_IDLE; // Reset platform navigation
    }
    
    // Priority 2: Platform navigation (complex state machine)
    else if (platformNavState != NAV_IDLE) {
      executePlatformNavigation();
    }
    
    // Priority 3: Normal climbing control with yaw correction
    else {
      controlMotors();
      
      // Apply yaw correction during climbing if needed
      if (shouldApplyYawCorrection()) {
        applyYawCorrection();
      }
    }
  }
  
  // Small delay to prevent CPU overload
  delay(10);
}

// ============================================================================
// END OF MAIN.C
// ============================================================================
