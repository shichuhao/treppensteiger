#ifndef GO_UP_H
#define GO_UP_H


#include "init.h"

// ============================================================================
// CONSTANTS
// ============================================================================
#define STAIR_DISTANCE_THRESHOLD 20
#define EMERGENCY_STOP_DISTANCE 5
#define MAX_DISTANCE 400
#define SAMPLE_COUNT 5

#define CLIMB_ANGLE_THRESHOLD 15
#define ROLL_ANGLE_LIMIT 20

#define NORMAL_SPEED 150
#define CLIMB_SPEED 200
#define APPROACH_SPEED 100




// ============================================================================
// GLOBAL VARIABLES (EXTERN)
// ============================================================================
extern float currentDistance;
extern float currentPitch;
extern float currentRoll;
extern int16_t ax, ay, az;
extern int16_t gx, gy, gz;

extern bool isClimbingMode;
extern bool emergencyStop;
extern uint8_t robotState;

//extern unsigned long lastSensorUpdate;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================
float measureDistance();
void updateAttitude();
void controlMotors();
void setMotorSpeed(int leftSpeed, int rightSpeed);
void stopMotors();
void setMotorDirection(bool forward);
void setMotorDirectionDifferential(bool leftForward, bool rightForward);

#endif // GO_UP_H
