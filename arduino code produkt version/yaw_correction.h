#ifndef YAW_CORRECTION_H
#define YAW_CORRECTION_H


#include "init.h"
#include "downhill.h"
#include "remote_stop.h"
// ============================================================================
// CONSTANTS
// ============================================================================
#define YAW_CORRECTION_THRESHOLD 2.5f
#define YAW_RATE_THRESHOLD 4.0f
#define YAW_CORRECTION_GAIN_UP 10
#define YAW_CORRECTION_GAIN_DOWN 8
#define MAX_YAW_CORRECTION 45

// ============================================================================
// GLOBAL VARIABLES (EXTERN)
// ============================================================================
extern float referenceYaw;
extern float currentYaw;
extern float yawDeviation;
extern bool yawCorrectionActive;
extern unsigned long lastYawUpdate;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================
void initializeYawReference();
float getYawDeviation();
void applyYawCorrection(int baseSpeedOverride=-1);
int calculateCorrectionPWM(float deviation, bool isDownhill);
bool shouldApplyYawCorrection();

#endif // YAW_CORRECTION_H
