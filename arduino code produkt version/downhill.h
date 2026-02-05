#ifndef DOWNHILL_H
#define DOWNHILL_H


#include "init.h"

// ============================================================================
// CONSTANTS
// ============================================================================
#define DOWNHILL_PITCH_THRESHOLD -10
#define DOWNHILL_ANGLE_CONFIRM -8

#define DOWNHILL_NO_BRAKE_SPEED 60
#define DOWNHILL_LIGHT_BRAKE_SPEED 80
#define DOWNHILL_MEDIUM_BRAKE_SPEED 120

#define PLATFORM_PITCH_MIN_DOWN -3.0
#define PLATFORM_PITCH_MAX_DOWN 5.0
#define PLATFORM_CONFIRM_TIME 500

#define LIGHT_BRAKE_PITCH -12.0
#define MEDIUM_BRAKE_PITCH -15.0

#define DOWNHILL_EXIT_CONFIRM_COUNT 3

// ============================================================================
// GLOBAL VARIABLES (EXTERN)
// ============================================================================
extern bool isDownhillMode;
extern bool isDescending;
extern float maxDownhillPitch;
extern unsigned long downhillStartTime;

extern int downhillExitConfirmCount;
extern unsigned long platformLevelStartTime;
extern bool platformLevelDetecting;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================
bool detectDownhill_Downhill();
void controlDownhillMotors_Downhill();
void applyBraking(int brakeLevel);
bool detectStairBottom();

#endif // DOWNHILL_H
