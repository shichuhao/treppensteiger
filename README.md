<img src="https://r2cdn.perplexity.ai/pplx-full-logo-primary-dark%402x.png" style="height:64px;margin-right:32px"/>

## Stairbot Prototype – Short Overview

This prototype is an Arduino‑based stair‑climbing robot (“Stairbot”) designed to climb stairs, turn on platforms, and go downhill autonomously, using low‑cost sensors and DC motors.

### Hardware

- Arduino Uno as main controller
- MPU6050 IMU for pitch/roll/yaw estimation
- Ultrasonic sensor (HC‑SR04‑type) for distance to stairs/walls
- Four DC motors with H‑bridge drivers
- ESP8266 module for simple remote stop and mode commands


### Core Software Modules

- `init.*`
    - Configures pins, motors, ultrasonic sensor, and I2C.
    - Initializes and calibrates the MPU6050.
    - Provides basic motor helpers (`motorsForward/Backward/RotateLeft/Right`) and `measureDistance()`.
- `go_up.*`
    - State machine for climbing stairs (approach, detect stair, climb, stabilize).
    - Uses filtered ultrasonic distance and pitch/roll (stored as integers) to decide when to climb and when to stop.
- `platform_turn.*`
    - Detects a platform edge and performs a 90° turn.
    - Uses a small distance history buffer and IMU tilt checks to avoid confusing walls with platforms.
- `downhill.*`
    - Detects descending slope and controls safe downhill motion.
    - Tracks maximum pitch and uses yaw correction to keep a straight path.
- `yaw_correction.*`
    - Integrates gyro data to estimate yaw (heading) using integer math.
    - Provides yaw deviation used by other modules to correct heading.
- `remote_stop.*`
    - Uses hardware serial with ESP8266 to receive simple commands (e.g., stop).
    - Implements a remote emergency stop that immediately halts motors.
- `stairbot_main.ino`
    - Top‑level system state machine: idle, go up, platform turn, downhill, emergency stop.
    - Handles serial commands, periodic status printing, and calls the behavior modules.


### Memory and Numeric Design

- Uses `int16_t` for angles (pitch/roll/yaw ×100) and `uint16_t` for distances (cm ×10) to save RAM and avoid float overhead.
- Strings are stored in flash using `F()` macros to keep dynamic memory usage low.

If you describe your target audience (e.g., classmates, supervisor), the document can be adjusted in depth and tone.
<span style="display:none">[^1][^10][^11][^12][^13][^14][^15][^16][^17][^18][^19][^2][^20][^21][^22][^23][^24][^25][^26][^27][^28][^29][^3][^30][^31][^32][^33][^34][^35][^36][^37][^38][^39][^4][^40][^41][^42][^43][^44][^5][^6][^7][^8][^9]</span>

<div align="center">⁂</div>

[^1]: down.c

[^2]: UC00_Init.c

[^3]: REMOTE_STOP.c

[^4]: Go_up.c

[^5]: turnv2_1.c

[^6]: yaw_correction_up_down.c

[^7]: yaw_correction.c

[^8]: downhill.c

[^9]: platform_turn.c

[^10]: remote_stop.c

[^11]: init.c

[^12]: go_up.c

[^13]: paste.txt

[^14]: paste.txt

[^15]: paste.txt

[^16]: paste.txt

[^17]: downhill.cpp

[^18]: paste.txt

[^19]: remote_stop.cpp

[^20]: remote_stop.h

[^21]: stairbot_main.ino

[^22]: yaw_correction.cpp

[^23]: downhill.cpp

[^24]: downhill.h

[^25]: yaw_correction.h

[^26]: go_up.cpp

[^27]: init.cpp

[^28]: init.h

[^29]: platform_turn.cpp

[^30]: go_up.h

[^31]: platform_turn.h

[^32]: yaw_correction.cpp

[^33]: downhill.cpp

[^34]: yaw_correction.h

[^35]: downhill.h

[^36]: init.cpp

[^37]: init.h

[^38]: go_up.cpp

[^39]: go_up.h

[^40]: platform_turn.cpp

[^41]: platform_turn.h

[^42]: remote_stop.cpp

[^43]: remote_stop.h

[^44]: stairbot_main.ino

