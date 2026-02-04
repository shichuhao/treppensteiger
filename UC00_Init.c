/*
 * ============================================================================
 * Stair-Climbing Robot - System Initialization (UC00)
 * Arduino UNO R3 with MPU6050, HC-SR04, ESP8266, 2x L298N Motor Drivers
 * 
 * USE CASE UC00: System Initialization
 * 11 Core Functional Modules:***************************************
 * - GPIO pin initialization (13 digital pins + 3 analog pins)
 * - PWM frequency configuration (Timer0/1/2 set to ~1kHz)
 * - MPU6050 IMU initialization (I2C @ 400kHz)
 * - HC-SR04 ultrasonic sensor initialization
 * - L298N motor driver initialization (2 drivers, 4 motors)
 * - ESP8266 WiFi module initialization (SoftwareSerial @ 115200)
 * - Interrupt configuration
 * - Watchdog timer (2 second timeout)
 * 
 * Author: Guppe 2
 * Date: 2026-01-29
 * Version: 1.0.0
 *
 *  In this version, the Arduino should be connected to the computer via serial communication.
 *
 *  In the next version, this will be changed to ESP8266 WLAN communication.
 *
 * ============================================================================
 */
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <SoftwareSerial.h>
#include <avr/wdt.h>
#include <avr/io.h>

// ============================================================================
// 1. PIN DEFINITIONS
// ============================================================================

// ===== HC-SR04 Ultrasonic Sensor =====
#define HC_TRIG_PIN     3      // Trigger pin
#define HC_ECHO_PIN     2      // Echo pin

// ===== MPU6050 IMU Sensor =====
#define MPU6050_ADDRESS 0x68   // I2C address (AD0 = LOW) GND

// ===== L298N Motor Driver 1 (links) (Motor A & B) =====
#define L298N_1_IN1     4      // Motor A direction 1
#define L298N_1_IN2     5      // Motor A direction 2 (PWM)
#define L298N_1_ENA     9      // Motor A speed (PWM, Timer1)
#define L298N_1_IN3     6      // Motor B direction 1 (PWM)
#define L298N_1_IN4     7      // Motor B direction 2
#define L298N_1_ENB     10     // Motor B speed (PWM, Timer1)

// ===== L298N Motor Driver 2 (rechts) (Motor C & D) =====
#define L298N_2_IN1     11     // Motor C direction 1 (PWM)
#define L298N_2_IN2     12     // Motor C direction 2
#define L298N_2_ENA     3      // Motor C speed (PWM)
#define L298N_2_IN3     13     // Motor D direction 1
#define L298N_2_IN4     8      // Motor D direction 2
#define L298N_2_ENB     6      // Motor D speed (PWM)

// ===== ESP8266 WiFi Module =====
#define ESP_RX_PIN      10     // SoftwareSerial RX
#define ESP_TX_PIN      11     // SoftwareSerial TX
#define ESP_BAUD        115200 // ESP8266 baud rate

// ============================================================================
// 2. GLOBAL VARIABLES
// ============================================================================

MPU6050 mpu(MPU6050_ADDRESS);
SoftwareSerial espSerial(ESP_RX_PIN, ESP_TX_PIN);

// System state flags
volatile bool mpuReady = false;
volatile bool hcReady = false;
volatile bool espReady = false;
volatile bool motorsReady = false;

// Timing
unsigned long initStartTime = 0;

// Error tracking
uint8_t initErrorCode = 0;

// ============================================================================
// 3. FUNCTION DECLARATIONS
// ============================================================================

void initializeGPIO();
void initializePWM();
void initializeMPU6050();
void initializeHCSR04();
void initializeL298N();
void initializeESP8266();
void setupInterrupts();
void setupWatchdog();
void printInitStatus();

// ============================================================================
// 4. MAIN INITIALIZATION ROUTINE
// ============================================================================

void setup() {
    initStartTime = millis();
    
    // Initialize Serial for debugging
    Serial.begin(9600);
    delay(500);
    
    Serial.println("\n===== STAIRBOT INITIALIZATION =====");
    Serial.print("Startup Time: ");
    Serial.println(millis());
    Serial.println();
    
    // Step 1: Initialize GPIO pins
    Serial.println("[1/8] Initializing GPIO pins...");
    initializeGPIO();
    delay(100);
    
    // Step 2: Configure PWM frequency
    Serial.println("[2/8] Configuring PWM frequency...");
    initializePWM();
    delay(100);
    
    // Step 3: Initialize MPU6050
    Serial.println("[3/8] Initializing MPU6050 IMU sensor...");
    initializeMPU6050();
    delay(500);
    
    // Step 4: Initialize HC-SR04
    Serial.println("[4/8] Initializing HC-SR04 ultrasonic sensor...");
    initializeHCSR04();
    delay(100);
    
    // Step 5: Initialize L298N Motor Drivers
    Serial.println("[5/8] Initializing L298N motor drivers...");
    initializeL298N();
    delay(100);
    
    // Step 6: Initialize ESP8266 WiFi Module
    Serial.println("[6/8] Initializing ESP8266 WiFi module...");
    initializeESP8266();
    delay(500);
    
    // Step 7: Setup interrupts
    Serial.println("[7/8] Setting up interrupts...");
    setupInterrupts();
    delay(100);
    
    // Step 8: Setup Watchdog Timer
    Serial.println("[8/8] Enabling watchdog timer...");
    setupWatchdog();
    delay(100);
    
    // Print final status
    Serial.println("\n===== INITIALIZATION STATUS =====");
    printInitStatus();
    
    unsigned long initTime = millis() - initStartTime;
    Serial.print("Total Initialization Time: ");
    Serial.print(initTime);
    Serial.println(" ms");
    
    Serial.println("===== SYSTEM READY =====\n");
}

// ============================================================================
// 5. GPIO INITIALIZATION
 /**

 * @brief Initialize GPIO pin configuration

 *
 * @details Configure the mode and initial state of all digital input/output pins.

 * - Set HC-SR04 pins to output mode

 * - Set 2 L298N pins to input mode and enable the internal pull-up resistor


 * - Initialize all output pins to the safe state (LOW)

 *
 * Libraries called:

 * - Arduino.h (pinMode, digitalWrite)

 *
 * Pins involved:

 * - LED_PIN (D13): Status indicator

 * - BUTTON_PIN (D2): User control button

 * - ENABLE_SENSORS_PIN (D7): Sensor always enabled

 */

// ============================================================================

void initializeGPIO() {
    // HC-SR04 pins
    pinMode(HC_TRIG_PIN, OUTPUT);
    digitalWrite(HC_TRIG_PIN, LOW);
    pinMode(HC_ECHO_PIN, INPUT);

    
    Serial.println(" All GPIO pins initialized successfully");
}

// ============================================================================
// 6. PWM FREQUENCY CONFIGURATION
/**

* @brief Initializes PWM timer configuration

*
* @details Configures the Arduino UNO's timer registers to generate high-frequency PWM signals.

* - Configures the prescalers for Timer1 and Timer2

* - Sets the PWM frequency to ~1 kHz (D9/D10)

* - Sets the PWM frequency to ~1 kHz (D5/D6)

*
* @warning Modifying the timer configuration will affect the accuracy of millis() and delay()

*
* Libraries called:

* - Arduino.h (analogWrite)

* - avr/io.h (direct register manipulation)

*
* Technical details:

* - Timer1 (16-bit): TCCR1B |= (1 << CS10);

* - Timer2 (8-bit): TCCR2B = (TCCR2B & 0xF8) | 0x01;

*/
// ============================================================================
// it originally wrote in form of Register.  to understand please serch ATmega Data Sheet
void initializePWM() {
    // ===== Configure Timer1 (16-bit, D9 & D10) for 1kHz =====
    //TCCR1A = 0;
    //TCCR1B = 0;
    //TCCR1A |= (1 << COM1A1) | (1 << COM1B1);  //COM1A1-OCR1A-D9  COM1B1-OCR1B-D10
    //TCCR1A |= (1 << WGM11);
    //TCCR1B |= (1 << WGM13);
    //TCCR1B |= (1 << CS11);
    //ICR1 = 1999;

    // ===== Configure Timer0 (8-bit, D5 & D6) for ~1kHz =====
    //TCCR0A = 0;
    //TCCR0B = 0;
    //TCCR0A |= (1 << WGM01) | (1 << WGM00);
    //TCCR0A |= (1 << COM0A1) | (1 << COM0B1);  //COM0A1-OCR0A-D6 COM1B1-OCR0B-D5
    //TCCR0B |= (1 << CS01) | (1 << CS00);

  //DO NOT NEED   // ===== Configure Timer2 (8-bit, D3 & D11) for ~1kHz =====
   // TCCR2A = 0;
    //TCCR2B = 0;
    //TCCR2A |= (1 << WGM21) | (1 << WGM20);
    //TCCR2A |= (1 << COM2A1) | (1 << COM2B1);
   // TCCR2B |= (1 << CS22);
    analogWrite(L298N_1_ENA, 0);   // Motor A speed (D9)
    analogWrite(L298N_1_ENB, 0);   // Motor B speed (D10)
    analogWrite(L298N_2_ENA, 0);   // Motor C speed (D3)
    analogWrite(L298N_2_ENB, 0);   // Motor D speed ()



}

// ============================================================================
// 7. MPU6050 INITIALIZATION
 /**

 * @brief Initializes the MPU6050 six-axis sensor

 *
 * @details Initializes and configures the MPU6050 IMU sensor for attitude detection.

 * - Initialize I2C communication (400 kHz fast mode)

 * - Configure gyroscope range to ±500°/s (suitable for stair-climbing environments)

 * - Configure accelerometer range to ±8g (shock-resistant)

 * - Configure digital low-pass filter to 21 Hz

 * - Set sampling rate to 200 Hz

 *
 * @note Stair-climbing machine dedicated configuration (high-shock, high-vibration environment)

 *
 * Libraries called:

 * - Wire.h (I2C communication)
 * - MPU6050.h (sensor driver library)
 * - SoftwareSerial.h

 *
 * I2C configuration:

 * - Device address: 0x68 (AD0 connected to GND)

 * - Clock frequency: 400 kHz

 * - Pin connections: SDA → A4, SCL → A5

 *
 * @return void No return value, sets initErrorCode |= 0x01 on failure

 */

// ============================================================================


void initializeMPU6050() {
    Wire.begin();
    Wire.setClock(400000);//     Faster reading and writing
    
    delay(100);
    
    // Test connection
    uint8_t devAddr = MPU6050_ADDRESS;  //device address
    uint8_t regAddr = 0x75;     //Register address
    uint8_t data = 0;

    Wire.beginTransmission(devAddr);
    Wire.write(regAddr);
    if (Wire.endTransmission() != 0) {
        Serial.println("  ✗ MPU6050: I2C transmission failed");    //Test:USB-> Computer   In Use: Esp8266->  WebSerial.println
        initErrorCode |= 0x01;
        return;
    }
    //  Check if the communication target is indeed MPU6050 when use MPU6500,disable this!!!!!!!
    Wire.requestFrom(devAddr, 1);
    if (Wire.available()) {
        data = Wire.read();
        if (data != 0x68) {
            Serial.print("  ✗ MPU6050: WHO_AM_I returned 0x");
            Serial.println(data, HEX);
            initErrorCode |= 0x01;
            return;
        }
    } else {
        Serial.println("  ✗ MPU6050: No I2C response");
        initErrorCode |= 0x01;
        return;
    }
    
    mpu.initialize();
    
    if (!mpu.testConnection()) {
        Serial.println("  ✗ MPU6050: testConnection() failed");
        initErrorCode |= 0x01;
        return;
    }
    
    // Configure
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);     // Gyroscope range-->Sensitivity je klein desto empfindlich, If set to 500: Physical value = Original value / 65.5 , If set to 1000: Physical value = Original value / 32.8
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);     // Accelerometer range           je klein desto empfindlich
    mpu.setDLPFMode(MPU6050_DLPF_BW_21);           //     low-pass filter
    mpu.setRate(4);                                //  Sampling rate divider       Samplerate=InternalOutputRate/(1+Rate)  The sensor generates 200 new data points per second.
    mpu.setIntDataReadyEnabled(true);              //      Enable "Data Ready" interrupt
    
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);   // (ax ay az) Gravity + Inertia     (gx gy gz)angular velocity

    Serial.print("  > Test Read: AX="); Serial.print(ax);
    Serial.print(" | GZ="); Serial.println(gz);            //Test-Read
    Serial.println("  ✓ MPU6050 initialized successfully");
    mpuReady = true;
}

// ============================================================================
// 8. HC-SR04 INITIALIZATION
/**

* @brief Initialize the HC-SR04 ultrasonic ranging sensor

*
* @details Configure the ultrasonic sensor for stair height and obstacle detection.

* - Configure the Trigger pin to output mode

* - Configure the Echo pin to input mode

* - Perform the first ranging measurement to verify that the sensor is working properly

*
* @note Ranging range: 2cm - 400cm (theoretical), actual stable range 5cm - 300cm

*
* Libraries called:

* - Arduino.h (pinMode, digitalWrite, pulseIn)
  - SoftwareSerial.h
*
* Pin connections:

* - HCSR04_TRIG_PIN (D8): Trigger pin

* - HCSR04_ECHO_PIN (D7): Echo pin

*
* @return void No return value, sets initErrorCode |= 0x02 on failure

*/

// ============================================================================

void initializeHCSR04() {
    if (digitalRead(HC_TRIG_PIN) != LOW) {
        digitalWrite(HC_TRIG_PIN, LOW);      // Ensure the Trig pin is at a low level.
    }
    
    unsigned long startTime = millis();
    int attempts = 0;
    const int MAX_ATTEMPTS = 3;
    
    while (attempts < MAX_ATTEMPTS && (millis() - startTime) < 2000) {
        digitalWrite(HC_TRIG_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(HC_TRIG_PIN, HIGH);  //Sending a 10μs high-level pulse is the operating command of the HC-SR04.
        delayMicroseconds(10);
        digitalWrite(HC_TRIG_PIN, LOW);


        //The first `while` loop waits for the sound to be emitted;
        // the second `while` loop measures the number of microseconds it takes for the sound to travel back.
        unsigned long echoStart = micros();    //timer in Microseconds
        while (digitalRead(HC_ECHO_PIN) == LOW && (micros() - echoStart) < 30000) {
            // Wait                     // 30000 Microseconds-> 5 meter
        }
        
        echoStart = micros();
        while (digitalRead(HC_ECHO_PIN) == HIGH && (micros() - echoStart) < 30000) {
            // Wait
        }
        
        unsigned long echoDuration = micros() - echoStart;
        
        if (echoDuration > 10 && echoDuration < 30000) {
            uint16_t distance_cm = echoDuration / 58;  //        It takes approximately 29 microseconds for a sound wave to travel 1 centimeter.
            Serial.print("  ✓ HC-SR04 initialized successfully");
            Serial.print(" (Distance: ");
            Serial.print(distance_cm);
            Serial.println(" cm)");
            hcReady = true;
            return;
        }
        
        attempts++;
        delay(100);
    }
    
    Serial.println("  ✗ HC-SR04: Sensor not responding");
    initErrorCode |= 0x02;
}

// ============================================================================
// 9. L298N MOTOR DRIVER INITIALIZATION
/**

* @brief Initialize the L298N dual H-bridge motor driver

*
* @details Configure two L298N modules to control 4 DC motors.

* - Configure the motor direction control pins (IN1-IN4) to output mode

* - Configure the motor speed control pins (ENA/ENB) to output mode

* - Initialize all motors to stop state

*
* @note Use two L298Ns to control 4 motors (2 on the left + 2 on the right)

* @warning The motor driver power supply must be independent (cannot use Arduino 5V)

*
* Libraries called:

* - Arduino.h (pinMode, digitalWrite, analogWrite)
  - SoftwareSerial.h
*
* Hardware connections:

* - L298N_1: Left motors A, B (6 pins)

* - L298N_2: Right motors C, D (6 pins)

*/

// ============================================================================

void initializeL298N() {
   // L298N_1 Motor A (links)
   pinMode(L298N_1_IN1, OUTPUT);
   digitalWrite(L298N_1_IN1, LOW);
   pinMode(L298N_1_IN2, OUTPUT);
   digitalWrite(L298N_1_IN2, LOW);
   pinMode(L298N_1_ENA, OUTPUT);
   analogWrite(L298N_1_ENA, 0);//Forced stop of motor
                               // analogWrite(pin, value)   value=Duty cycle  0-255
                               //0: The pin is always low, the motor stops.
                               //127: Power is on half the time and off the other half, the motor runs at approximately half power.
                               //255: Power is always on, the motor runs at full speed.

   // L298N_1 Motor B
   pinMode(L298N_1_IN3, OUTPUT);
   digitalWrite(L298N_1_IN3, LOW);
   pinMode(L298N_1_IN4, OUTPUT);
   digitalWrite(L298N_1_IN4, LOW);
   pinMode(L298N_1_ENB, OUTPUT);
   analogWrite(L298N_1_ENB, 0);//Forced stop of motor

   // L298N_2 Motor C (rechts)
   pinMode(L298N_2_IN1, OUTPUT);
   digitalWrite(L298N_2_IN1, LOW);
   pinMode(L298N_2_IN2, OUTPUT);
   digitalWrite(L298N_2_IN2, LOW);
   pinMode(L298N_2_ENA, OUTPUT);
   analogWrite(L298N_2_ENA, 0);//Forced stop of motor

   // L298N_2 Motor D
   pinMode(L298N_2_IN3, OUTPUT);
   digitalWrite(L298N_2_IN3, LOW);
   pinMode(L298N_2_IN4, OUTPUT);
   digitalWrite(L298N_2_IN4, LOW);
   pinMode(L298N_2_ENB, OUTPUT);
   analogWrite(L298N_2_ENB, 0);//Forced stop of motor
    
    // Test pulse
    Serial.println("  - Testing PWM outputs...");
    
    analogWrite(L298N_1_ENA, 100);
    delay(50);
    analogWrite(L298N_1_ENA, 0);
    
    analogWrite(L298N_1_ENB, 100);
    delay(50);
    analogWrite(L298N_1_ENB, 0);
    
    analogWrite(L298N_2_ENA, 100);
    delay(50);
    analogWrite(L298N_2_ENA, 0);
    
    analogWrite(L298N_2_ENB, 100);
    delay(50);
    analogWrite(L298N_2_ENB, 0);
    
    Serial.println("  ✓ L298N motor drivers initialized successfully");
    
    motorsReady = true;
}

// ============================================================================
// 10. ESP8266 INITIALIZATION
/**
* @brief Initializes the ESP8266 WiFi module
*
* @details Configures the ESP8266 for remote control via software serial port.
* - Initialize software serial port (baud rate 115200)
* - Verify AT command response
* - Configure to Station mode
* - Connect to specified WiFi network
* - Start TCP server (port 8080)
*
* @warning The software serial port may be unstable at high baud rates; 115200 is recommended.
*
* Libraries called:
* - SoftwareSerial.h (software serial port)
* - String.h (string processing)
*
* AT command sequence:
* - AT → Test connection
* - AT+RST → Software reset
* - AT+CWJAP → Connect to WiFi
* - AT+CIPSERVER → Start server
*
* @return void No return value; sets initErrorCode |= 0x04 on failure
*/

// ============================================================================

void initializeESP8266() {
    espSerial.begin(115200); //Establish serial communication between the MCU and ESP8266 at a baud rate of 115200.
    delay(500);

    while (espSerial.available()) {
        espSerial.read();
    }
    
    Serial.println("  - Sending AT command to ESP8266...");
    
    espSerial.println("AT");
    
    unsigned long timeout = millis() + 2000;
    String response = "";
    
    while (millis() < timeout) {
        if (espSerial.available()) {
            char c = espSerial.read();
            response += c;
            Serial.write(c);
            
            if (response.indexOf("OK") != -1) {
                Serial.println("\n  ✓ ESP8266 responded to AT command");
                espReady = true;
                
                Serial.println("  - Querying firmware version...");
                espSerial.println("AT+GMR");
                delay(500);
                
                while (espSerial.available()) {
                    Serial.write(espSerial.read());
                }
                
                Serial.println("\n  ✓ ESP8266 WiFi module initialized");
                return;
            }
        }
    }
    
    Serial.println("\n  ✗ ESP8266: No response from module");
    initErrorCode |= 0x04;
}

// ============================================================================
// 11. INTERRUPT SETUP
/**
* @brief Configure external interrupts and pin change interrupts
*
* @details Set up hardware interrupts for real-time response to critical events.
* - Configure external interrupt INT0 (--) for the emergency stop button
* - Configure external interrupt INT1 (--) for encoder counting
*
* @note Interrupt Service Routine (ISR) must be as short as possible
* @warning Do not use Serial.print() or delay() within an interrupt
*
* Libraries called:
* - Arduino.h (attachInterrupt, digitalPinToInterrupt)
* - SoftwareSerial.h
* Trigger modes:
* - FALLING: Falling edge triggered
* - CHANGE: Level change triggered

*/

// ============================================================================

void setupInterrupts() {
    // External interrupts configuration (if needed)
    // Currently: placeholder for future interrupt handlers
    
    Serial.println("  ✓ Interrupts configured");
}

// ============================================================================
// 12. WATCHDOG TIMER SETUP
/**
 *
* @brief Configure the watchdog timer
 *
*
 *
* @details Enable the ATmega328P hardware watchdog for system self-recovery.
 *
* - Enable the watchdog timer
 *
* - Set the timeout to 4 seconds
 *
* - Prevent system deadlock and infinite loops
 *
*
 *
* @note You must periodically call wdt_reset() in loop() to feed the watchdog.
 *
* @warning If you forget to feed the watchdog, the system will automatically reset.
 *
*
 *
* Libraries called:
 *
* - avr/wdt.h (Watchdog timer control)
* - SoftwareSerial.h
 *
* Timeout options:
 *
* - WDTO_2S: 2 seconds
 *
*/
// ============================================================================

void setupWatchdog() {
    cli();
    wdt_reset();
    wdt_enable(WDTO_2S);
    sei();
    
    Serial.println("  ✓ Watchdog timer enabled (2 second timeout)");
}

// ============================================================================
// 13. STATUS REPORTING
 /**

 * @brief Prints the initialization status of all modules

 *
 * @details Prints a detailed status report after initialization is complete.

 * - Parses the initErrorCode error code

 * - Prints the initialization result for each module (✓ or ✗)

 * - Displays the total initialization time

 *
 * Libraries called:

 * - Arduino.h (Serial.print, millis)
 * - SoftwareSerial.h

 *
 * Error code parsing:

 * - 0x01: MPU6050 failed

 * - 0x02: HC-SR04 failed

 * - 0x04: ESP8266 failed

 */
// ============================================================================

void printInitStatus() {
    Serial.println("\n--- System Component Status ---");
    
    Serial.print("MPU6050 IMU:        ");
    Serial.println(MPUReady ? "✓ READY" : "✗ FAILED");
    
    Serial.print("HC-SR04 Sensor:     ");
    Serial.println(HCReady ? "✓ READY" : "✗ FAILED");
    
    Serial.print("L298N Motors:       ");
    Serial.println(motorsReady ? "✓ READY" : "✗ FAILED");
    
    Serial.print("ESP8266 WiFi:       ");
    Serial.println(ESPReady ? "✓ READY" : "✗ FAILED");
    
    if (initErrorCode != 0) {
        Serial.print("Error Code:         0x");
        Serial.println(initErrorCode, HEX);
        
        if (initErrorCode & 0x01) Serial.println("  - MPU6050 initialization failed");
        if (initErrorCode & 0x02) Serial.println("  - HC-SR04 initialization failed");
        if (initErrorCode & 0x04) Serial.println("  - ESP8266 initialization failed");
    } else {
        Serial.println("Error Code:         0x00 (No errors)");
    }
    
    Serial.println("--- End of Status ---\n");
}












