#include "remote_stop.h"
#include "init.h"

/**
 * @file remote_stop.cpp
 * @brief Remote emergency stop handling via ESP8266.
 * 
 * @details Optimized to use Hardware Serial instead of SoftwareSerial to 
 * save approximately 95 bytes of SRAM and reduce CPU overhead. 
 * This module monitors incoming serial characters for a stop command.
 */

// === Global Timing Variables ===
uint32_t lastCmdTime = 0;  ///< Timestamp of the last received command (ms)

/** 
 * @brief Communication Constants
 */
#define CMD_TIMEOUT 5000   ///< Timeout threshold for command validity (ms)

/**
 * @brief Initializes the ESP8266 communication interface.
 * 
 * @details Assumes the Hardware Serial port has already been initialized 
 * in the main setup (e.g., Serial.begin(9600)). 
 * Using the F() macro to store strings in Flash memory instead of SRAM.
 */
void setupESP8266() {
  /* 
   * Note: Hardware Serial is shared with the Debug console.
   * Ensure the ESP8266 and the PC Serial Monitor use the same baud rate.
   */
  lastCmdTime = millis();
  Serial.println(F("ESP8266 interface: READY")); 
}

/**
 * @brief Polls the serial buffer for an emergency stop command.
 * 
 * @details Implementation Strategy:
 * - Checks if data is available on the hardware serial port.
 * - Listens for the 'S' or 's' character (Stop).
 * - Immediately invokes stopAllMotors() upon detection.
 * 
 * @return bool True if a stop command was detected and executed.
 */
bool checkRemoteStop() {
  // Check if bytes are waiting in the Hardware Serial RX buffer
  if (Serial.available()) {
    char cmd = Serial.read();
    lastCmdTime = millis(); // Update heartbeat timer

    // Detect Stop Command ('S' for Stop)
    if (cmd == 'S' || cmd == 's') {
      Serial.println(F("EVENT: Remote STOP signal received"));
      
      // Execute immediate hardware shutdown
      stopAllMotors();
      return true;
    }
  }

  return false;
}
