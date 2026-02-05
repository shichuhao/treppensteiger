// ============================================================================
// REMOTE_STOP.c - Remote Stop Control Module via ESP8266
// ============================================================================
// Use cases:
// UC01: Send/Receive stop commands (ESP8266 communication)
// UC02: Process stop signal (L298N motor control)
// ============================================================================

#include "init.h"
#include "remote_stop.h"




// ============================================================================
// CONSTANTS - Remote Stop Configuration
// ============================================================================
/*
// ESP8266 Serial Configuration
#define ESP8266_SERIAL Serial1      // Use Serial1 for ESP8266 communication
#define ESP8266_BAUD_RATE 115200    // ESP8266 baud rate

// Command Definitions
#define CMD_STOP "STOP\n"           // Stop command
#define CMD_START "START\n"         // Start command
#define CMD_EMERGENCY "EMERGENCY\n" // Emergency stop command

// Buffer Settings
#define CMD_BUFFER_SIZE 32          // Command buffer size
#define CMD_TIMEOUT 10000           // Command timeout (ms)

// Motor Pin Definitions (L298N)
#define L298N_1_IN1 2               // Left motor IN1
#define L298N_1_IN2 3               // Left motor IN2
#define L298N_1_ENA 4               // Left motor ENA (PWM)
#define L298N_1_ENB 5               // Left motor ENB (PWM)

#define L298N_2_IN1 6               // Right motor IN1
#define L298N_2_IN2 7               // Right motor IN2
#define L298N_2_ENA 8               // Right motor ENA (PWM)
#define L298N_2_ENB 9               // Right motor ENB (PWM)

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// Remote Stop State
bool remoteStopRequested = false;   // Remote stop request flag
bool emergencyStop = false;         // Emergency stop flag
unsigned long lastCmdTime = 0;      // Last command timestamp

// Command Buffer
char cmdBuffer[CMD_BUFFER_SIZE];    // Command buffer
int cmdIndex = 0;                   // Buffer index
*/
// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================
/*
// UC01: ESP8266 Communication
void processRemoteCommands_Stop();
void checkESP8266Buffer();
void sendCommandToESP8266(const char* cmd);

// UC02: Motor Control
void executeStopCommand_Stop();
void stopMotors();
void resumeMotors();
void initRemoteStopModule();
*/
// ============================================================================
// UC01: PROCESS REMOTE COMMANDS
// ============================================================================

/**
 * @brief Process remote stop commands (main entry function)
 *
 * @details Receive commands from ESP8266 and perform corresponding actions:
 * - STOP: normal stop (can be resumed)
 * - START: resume operation
 * - EMERGENCY: emergency stop (requires manual reset)
 *
 * Invocation timing:
 * - Called in main loop() on every cycle
 * - Highest priority (before other control logic)
 *
 * @return void
 */
void processRemoteCommands_Stop() {
    // 1. Check ESP8266 buffer
    checkESP8266Buffer();

    // 2. Check if a complete command was received (ends with \n)
    if (cmdIndex > 0 && cmdBuffer[cmdIndex - 1] == '\n') {
        // Remove newline character
        cmdBuffer[cmdIndex - 1] = '\0';

        Serial.print(" Remote command received: ");
        Serial.println(cmdBuffer);

        // 3. Parse and execute command
        if (strcmp(cmdBuffer, "STOP") == 0) {
            // Normal stop
            remoteStopRequested = true;
            executeStopCommand_Stop();
            Serial.println("  Remote STOP command executed");

        } else if (strcmp(cmdBuffer, "START") == 0) {
            // Resume
            if (!emergencyStop) {
                remoteStopRequested = false;
                resumeMotors();
                Serial.println("  Remote START command - resuming");
            } else {
                Serial.println("  Cannot start - Emergency stop active");
            }

        } else if (strcmp(cmdBuffer, "EMERGENCY") == 0) {
            // Emergency stop
            remoteStopRequested = true;
            emergencyStop = true;
            executeStopCommand_Stop();
            Serial.println(" EMERGENCY STOP from remote!");

        } else {
            Serial.print(" Unknown command: ");
            Serial.println(cmdBuffer);
        }

        // 4. Reset buffer
        cmdIndex = 0;
        memset(cmdBuffer, 0, CMD_BUFFER_SIZE);
        lastCmdTime = millis();
    }

    // 5. Timeout check (auto resume after 10s inactivity)
    if (remoteStopRequested && !emergencyStop) {
        if ((millis() - lastCmdTime) > CMD_TIMEOUT) {
            Serial.println("  Remote command timeout - auto-resuming");
            remoteStopRequested = false;
            resumeMotors();
        }
    }
}

// ============================================================================
// UC01: CHECK ESP8266 BUFFER
// ============================================================================

/**
 * @brief Check ESP8266 serial buffer
 *
 * @details Read command bytes one by one and store them into buffer
 * - Prevent buffer overflow
 * - Support multi-byte commands
 *
 * @return void
 */
void checkESP8266Buffer() {
    while (ESP8266_SERIAL.available() && cmdIndex < (CMD_BUFFER_SIZE - 1)) {
        char c = ESP8266_SERIAL.read();
        cmdBuffer[cmdIndex++] = c;

        // Debug output (optional)
        // Serial.print(c);
    }

    // Prevent buffer overflow
    if (cmdIndex >= (CMD_BUFFER_SIZE - 1)) {
        Serial.println(" Command buffer overflow - resetting");
        cmdIndex = 0;
        memset(cmdBuffer, 0, CMD_BUFFER_SIZE);
    }
}

// ============================================================================
// UC01: SEND COMMAND TO ESP8266
// ============================================================================

/**
 * @brief Send command to ESP8266 (optional feature)
 *
 * @details Used for main controller to send status feedback
 * - Example: send robot state
 * - Supports bidirectional communication
 *
 * @param cmd Command string to send
 * @return void
 */
void sendCommandToESP8266(const char* cmd) {
    ESP8266_SERIAL.print(cmd);
    Serial.print(" Sent to ESP8266: ");
    Serial.println(cmd);
}

// ============================================================================
// UC02: EXECUTE STOP COMMAND
// ============================================================================

/**
 * @brief Execute stop command (UC02 core)
 *
 * @details Stop all motor activity
 * - Brake mode (short-circuit braking)
 * - Fully stops within 0.3 seconds
 * - Maintains stop until START command received
 *
 * Invocation scenarios:
 * - Remote STOP command
 * - EMERGENCY command
 * - Safety stop from other modules
 *
 * @return void
 */
void executeStopCommand_Stop() {
    // Call lower-layer stop function
    stopMotors();

    // Send acknowledgment to ESP8266 (optional)
    sendCommandToESP8266("ACK:STOP\n");

    // Record stop time
    lastCmdTime = millis();

    Serial.println(" All motors stopped");
}


// ============================================================================
// UC02: RESUME MOTORS
// ============================================================================

/**
 * @brief Resume motor operation (optional implementation)
 *
 * @details Clear stop status and allow normal operation
 * - Clears stop flag
 * - Actual motor restart handled by main control logic
 *
 * Notes:
 * - Does not start motors directly
 * - Only clears stop flag
 * - Main loop will handle resumption logic
 *
 * @return void
 */
void resumeMotors() {
    remoteStopRequested = false;

    // Optionally send acknowledgment to ESP8266
    sendCommandToESP8266("ACK:START\n");

    Serial.println(" Remote stop cleared - motors can resume");

    // Note: Actual restart handled by controlMotors() in main loop
}

// ============================================================================
// INITIALIZATION FUNCTION (Call in setup())
// ============================================================================

/**
 * @brief Initialize remote stop module (called in setup())
 *
 * @details Full initialization procedure:
 * 1. Initialize motor control pins (L298N)
 * 2. Initialize ESP8266 communication
 * 3. Reset all status flags
 *
 * @return void
 */
void initRemoteStopModule() {

    // 2. Initial stop state
    stopMotors();

    // 3. Initialize ESP8266
    initializeESP8266();


    // 4. Reset flags
    remoteStopRequested = false;
    emergencyStop = false;
    lastCmdTime = millis();

    Serial.println(" Remote Stop Module initialized\n");
}

// ============================================================================
// HELPER: GET STOP STATUS (For other modules)
// ============================================================================

/**
 * @brief Check if currently in stop state (for use by other modules)
 *
 * @return bool true = stopped, false = running
 */
bool isRemoteStopped() {
    return remoteStopRequested;
}

/**
 * @brief Check emergency stop status
 *
 * @return bool true = emergency stop active, false = normal
 */
bool isEmergencyStopped() {
    return emergencyStop;
}

/**
 * @brief Clear emergency stop (manual reset)
 *
 * @details After an emergency stop, manual reset is required
 * - Press button to reset
 * - Or send remote reset command
 *
 * @return void
 */
void clearEmergencyStop() {
    emergencyStop = false;
    remoteStopRequested = false;
    Serial.println(" Emergency stop cleared manually");
}

// ============================================================================
// END OF REMOTE_STOP.c
// ============================================================================
