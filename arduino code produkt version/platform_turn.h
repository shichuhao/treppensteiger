#ifndef PLATFORM_TURN_H
#define PLATFORM_TURN_H


#include "init.h"

// ============================================================================
// CONSTANTS
// ============================================================================
#define NO_STAIR_DISTANCE 100
#define NO_STAIR_CONFIRM_COUNT 3
#define NO_STAIR_DWELL_TIME 500

#define PLATFORM_PITCH_MIN_TURN -5
#define PLATFORM_PITCH_MAX_TURN 5
#define PLATFORM_PITCH_CONFIRM_COUNT 5

#define MIN_WALL_DISTANCE 15
#define TARGET_WALL_DISTANCE 50
#define WALL_DISTANCE_TOLERANCE 5
#define FIRST_STRAIGHT_DISTANCE 40
#define FIRST_STRAIGHT_SPEED 80
#define APPROACH_WALL_SPEED 60

#define PLATFORM_TURN_SPEED_OUTER 150
#define PLATFORM_TURN_SPEED_INNER 50
#define PLATFORM_TURN_ANGLE 90
#define PLATFORM_TURN_TOLERANCE 5

#define NEXT_STAIR_DISTANCE 20

// ============================================================================
// ENUMS
// ============================================================================
enum PlatformNavState {
  NAV_IDLE = 0,
  NAV_NO_STAIR_DETECTED = 1,
  NAV_PLATFORM_LEVEL_CHECK = 2,
  NAV_FIRST_STRAIGHT = 3,
  NAV_FIRST_TURN = 4,
  NAV_STAIR_OR_WALL_CHECK = 5,
  NAV_APPROACH_WALL = 6,
  NAV_SECOND_TURN = 7
};

// ============================================================================
// GLOBAL VARIABLES (EXTERN)
// ============================================================================
extern PlatformNavState platformNavState;
extern unsigned long stateStartTime;
extern unsigned long distanceMeasurementTime;

extern int noStairConfirmCounter;
extern int platformLevelConfirmCounter;

extern float wallDistanceReadings[5];
extern int wallDistanceIndex;

extern int platformTurnCount;
extern float platformTurnYaw;
extern unsigned long platformTurnStartTime;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================
void initiatePlatformNavigation();
void executePlatformNavigation();
void checkNoStairCondition();
void checkPlatformLevelCondition();
void executeFirstStraightMovement();
void executeFirstTurn();
void approachWallDistance();
void executeSecondTurn();
void checkNextStairCondition();
float getWallDistance();
float getAverageWallDistance();
void logNavigationState();
void checkStairOrWall();
bool isStairsNotWall();
bool isStairsNotWall2();

#endif // PLATFORM_TURN_H
