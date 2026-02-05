#ifndef REMOTE_STOP_H
#define REMOTE_STOP_H



// ============================================================================
// CONSTANTS
// ============================================================================
#define ESP8266_SERIAL espSerial  
#define ESP8266_BAUD_RATE 115200    // ESP8266 baud rate

// Command Definitions
#define CMD_STOP "H\n"           // Stop command
#define CMD_START "A\n"         // Start command
#define CMD_EMERGENCY "N\n" // Emergency stop command
#define CMD_BUFFER_SIZE 2
#define CMD_TIMEOUT 10000


// ============================================================================
// GLOBAL VARIABLES (EXTERN)
// ============================================================================
extern bool remoteStopRequested;
extern bool emergencyStop;
extern unsigned long lastCmdTime;
extern char cmdBuffer[CMD_BUFFER_SIZE];
extern int cmdIndex;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================
void initRemoteStopModule();
void processRemoteCommands_Stop();
void checkESP8266Buffer();
void sendCommandToESP8266(const char* cmd);
void executeStopCommand_Stop();
void resumeMotors();
bool isRemoteStopped();
bool isEmergencyStopped();
void clearEmergencyStop();

#endif // REMOTE_STOP_H
