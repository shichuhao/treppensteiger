# Stairbot Prototype – Code Overview

This document briefly describes the structure and purpose of the prototype code for the Stairbot, an Arduino‑based stair‑climbing robot.

## 1. Hardware Overview

- **Controller**: Arduino Uno  
- **IMU**: MPU6050 (3‑axis accelerometer + 3‑axis gyroscope)  
- **Distance sensor**: Ultrasonic module (e.g., HC‑SR04‑type)  
- **Actuators**: Four DC motors with H‑bridge motor drivers  
- **Wireless module**: ESP8266 for simple remote commands (e.g., stop, mode selection)

The robot can climb stairs, turn on intermediate platforms, and go downhill while maintaining balance and heading.

## 2. Numeric and Memory Design

To fit within the 2 KB SRAM of the Arduino Uno, the code uses:

- **Fixed‑point representation**  
  - Angles (pitch, roll, yaw) stored as `int16_t` in degrees ×100  
  - Distances stored as `uint16_t` in centimeters ×10  
- **No dynamic allocation** (no `new`/`malloc` in user code)  
- **Flash‑stored strings** using the `F()` macro for debug prints

This design reduces RAM usage and avoids the overhead and instability of floats where possible.

## 3. Module Overview

### 3.1 `init.h` / `init.cpp`

Responsibilities:

- Define pin mappings for motors and sensors.  
- Initialize all I/O pins and stop the motors at startup.  
- Configure and initialize the MPU6050 over I2C (`Wire`).  
- Calibrate the IMU by averaging multiple samples and writing bias offsets.  
- Provide helper functions:
  - `setupPins()`
  - `initializeMPU6050()`
  - `calibrateMPU6050()`
  - `checkPlatformLevel()` – checks if the robot is standing on a level surface using pitch.
  - `measureDistance()` – reads the ultrasonic sensor and returns distance in 0.1 cm units.
  - Motor helpers: `motorsForward()`, `motorsBackward()`, `motorsRotateLeft()`, `motorsRotateRight()`, `stopAllMotors()`.

### 3.2 `go_up.h` / `go_up.cpp`

Implements the **stair‑climbing behavior** as a small state machine:

- States include initialization, moving forward, detecting the first stair, climbing, stabilizing, and completion.  
- Periodically reads:
  - IMU data → pitch and roll (fixed‑point integers)  
  - Ultrasonic distance → distance to the stair edge  
- Uses filtered distance and pitch thresholds to:
  - Detect when the robot is close enough to a stair.  
  - Command the motors to climb at appropriate speed.  
  - Decide when the robot has reached the top and needs to stabilize.

### 3.3 `platform_turn.h` / `platform_turn.cpp`

Handles **turning on an intermediate platform**:

- Maintains a small history buffer of wall distances to distinguish a real platform edge from a nearby wall.  
- Uses the IMU to confirm that the robot is approximately level before starting a turn.  
- Performs a ~90° rotation using yaw feedback:
  - Integrates gyro data from the IMU to estimate heading.  
  - Rotates until the yaw difference reaches the desired angle within a tolerance.

### 3.4 `downhill.h` / `downhill.cpp`

Implements **downhill behavior**:

- Detects a negative pitch beyond a configured threshold to recognize a descending slope.  
- Records the most negative pitch reached to evaluate progress.  
- Commands the motors to move cautiously downhill while:
  - Correcting yaw to maintain a straight path.  
  - Detecting when the robot has reached level ground again.  
- Handles timeouts and retries if the downhill sequence does not complete in time.

### 3.5 `yaw_correction.h` / `yaw_correction.cpp`

Provides **heading estimation and yaw correction**:

- Reads gyro data from the MPU6050 and performs a simple integration to estimate yaw.  
- Stores yaw as an `int16_t` in degrees ×100 and keeps it normalized to a useful range (e.g., −180° to 180°).  
- Exposes functions to:
  - Initialize yaw reference.  
  - Update yaw regularly.  
  - Return yaw deviation for other modules to use for steering corrections.

### 3.6 `remote_stop.h` / `remote_stop.cpp`

Implements **remote stop and basic remote control** via the ESP8266:

- Uses the hardware serial interface instead of a software serial port.  
- Listens for simple one‑character commands from the ESP8266, for example:
  - Stop command that immediately halts all motors (emergency stop).  
- Stores timestamps of the last valid command to support possible timeouts or watchdog logic.

### 3.7 `stairbot_main.ino`

Top‑level **application and system state machine**:

- Defines global system states such as:
  - `IDLE`  
  - `GO_UP` (climbing stairs)  
  - `PLATFORM_TURN`  
  - `DOWNHILL`  
  - `EMERGENCY_STOP`
- In `setup()`:
  - Initializes serial, pins, IMU, ESP8266 communication, yaw correction, and level check.  
  - Prints basic status information for debugging.  
- In `loop()`:
  - Checks for remote stop commands.  
  - Dispatches to the appropriate behavior module based on the current state.  
  - Periodically prints a compact status summary (pitch, roll, distance, and current state).

## 4. Typical Control Flow

1. System powers on and runs `setup()`.  
2. IMU is initialized and calibrated; level is checked.  
3. The robot waits in `IDLE` until a command (serial or ESP8266) selects a mode.  
4. Depending on the mode:
   - `GO_UP`: approach staircase and climb.  
   - `PLATFORM_TURN`: detect platform and perform a 90° turn.  
   - `DOWNHILL`: detect and descend a slope.  
5. At any time, a remote stop command can trigger the emergency stop state, which shuts down all motors.

## 5. Intended Use

This prototype is intended as a research and teaching platform for:

- Studying stair‑climbing and multi‑terrain locomotion.  
- Experimenting with low‑cost sensing (IMU + ultrasonic).  
- Exploring fixed‑point control algorithms on constrained microcontrollers.

