/*
 * Complete Rover Navigation System with 8 Segments
 * 
 * This version includes a total of 8 segments (4 straight lines with 4 turns),
 * allowing the robot to navigate a more complex path.
 */

#include <Wire.h>
#include <ArduinoJson.h>

// =============== CONFIGURATION ===============

// Pin definitions
#define LEFT_MOTOR_PWM 6      // PWM pin for left motor
#define LEFT_MOTOR_DIR 5      // Direction pin for left motor
#define RIGHT_MOTOR_PWM 10    // PWM pin for right motor
#define RIGHT_MOTOR_DIR 9     // Direction pin for right motor
#define HALL_SENSOR_PIN 3     // Hall sensor for wheel rotation
#define LED_PIN 2             // LED for visual feedback
#define CALIBRATION_SWITCH 12 // Switch to enter calibration mode

// Navigation parameters
#define BASE_SPEED 150        // Base motor speed (0-255)
#define MAX_CORRECTION 50     // Maximum speed adjustment for PID
#define TURN_DIRECTION 1      // 1 for right turn, -1 for left turn
#define TOTAL_SEGMENTS 8      // Total segments in route (now 8 instead of 4)

// Hall sensor debounce settings
#define DEBOUNCE_TIME 50      // Minimum time between pulses (milliseconds)

// PID Controller constants
float Kp = 2.0;               // Proportional gain
float Ki = 0.1;               // Integral gain
float Kd = 0.5;               // Derivative gain

// =============== SENSOR ADDRESSES ===============

// MPU-6050 Settings
#define MPU_ADDR        0x68
#define PWR_MGMT_1      0x6B
#define ACCEL_CONFIG    0x1C
#define GYRO_CONFIG     0x1B
#define CONFIG          0x1A
#define SMPLRT_DIV      0x19
#define INT_ENABLE      0x38
#define ACCEL_XOUT_H    0x3B
#define GYRO_XOUT_H     0x43
#define WHO_AM_I_MPU    0x75  // Should return 0x68 for MPU-6050

// QMC5883L Settings
#define QMC5883L_ADDR          0x0D  // I2C 7-bit address
#define QMC5883L_REG_X_LSB     0x00
#define QMC5883L_REG_X_MSB     0x01
#define QMC5883L_REG_Y_LSB     0x02
#define QMC5883L_REG_Y_MSB     0x03
#define QMC5883L_REG_Z_LSB     0x04
#define QMC5883L_REG_Z_MSB     0x05
#define QMC5883L_REG_STATUS    0x06
#define QMC5883L_REG_CONFIG_1  0x09
#define QMC5883L_REG_CONFIG_2  0x0A
#define QMC5883L_REG_SET_RESET 0x0B

// =============== GLOBAL VARIABLES ===============

// Serial communication with ESP32
const int BUFFER_SIZE = 256;
char inputBuffer[BUFFER_SIZE];
int bufferIndex = 0;
bool newCommandReceived = false;
String lastCommand = "";

// Operation mode
enum Mode {
  CALIBRATION,
  NAVIGATION,
  STOPPED
};
Mode currentMode = STOPPED;

// Sensor data
float accelX, accelY, accelZ;    // Acceleration in g's
float gyroX, gyroY, gyroZ;       // Angular velocity in degrees/s
float magX, magY, magZ;          // Magnetic field (calibrated)
float roll, pitch, yaw;          // Orientation in degrees

// Scaling factors
const float ACCEL_SCALE = 16384.0;  // LSB/g for ±2g range
const float GYRO_SCALE = 131.0;     // LSB/(deg/s) for ±250deg/s range

// Sensor calibration
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;  // Gyro bias correction

// Magnetometer calibration values - pre-calibrated from previous run
float magXoffset = -46.0, magYoffset = -742.5, magZoffset = -207.5;
float magXscale = 1.261, magYscale = 1.000, magZscale = 1.143;
bool magCalibrated = true;  // Set to true to use pre-calibrated values

// Calibration variables
int16_t magXmin, magXmax, magYmin, magYmax, magZmin, magZmax;
unsigned long calibrationStartTime = 0;
const unsigned long CALIBRATION_DURATION = 30000; // 30 seconds for calibration

// Distance tracking
volatile unsigned long pulseCount = 0;
volatile unsigned long lastPulseTime = 0;  // For debouncing
volatile bool shouldBlinkLED = false;

// Configure how many pulses to travel in each segment
// Now with 8 segments: straight, turn, straight, turn, straight, turn, straight, stop
// Expanded from 4 to 8 segments
unsigned long segmentTargets[TOTAL_SEGMENTS] = {
  4, 0,   // First straight segment, then turn
  4, 0,   // Second straight segment, then turn  
  4, 0,   // Third straight segment, then turn
  4, 0    // Fourth straight segment, then stop
};

// PID controller variables
float targetHeading = 0.0;      // Target angle (0 = initial direction)
float integral = 0.0;           // Integral term
float prevError = 0.0;          // Previous error for derivative
unsigned long lastPIDTime = 0;  // Last update time for PID

// Navigation state
int currentSegment = 0;         // Current segment of movement
bool turningInProgress = false; // Flag for when turning
unsigned long turnStartTime = 0; // When turning started
float turnStartHeading = 0.0;   // Initial heading when turning started
float turnTargetHeading = 0.0;  // Target heading for turn completion

// System state tracking
bool sensorInitialized = false;
bool magSensorPresent = false;
unsigned long lastUpdate = 0;

// =============== FUNCTION DECLARATIONS ===============

// Sensor functions
bool initSensors();
bool initMPU6050();
bool initQMC5883L();
void calibrateMagnetometer();
void endCalibration();
void readSensors();
bool readAccelGyro();
bool readMagnetometer();
bool readRawMagnetometer(int16_t* x, int16_t* y, int16_t* z);
float calculateHeading();

// Navigation functions
void updateNavigation();
float computePID();
void executeSegment();
void startMotors(int leftSpeed, int rightSpeed);
void stopMotors();
void turnRobot(int direction, float degrees);
void driveStraight(float correction);
void finishRoute();

// Utility functions
bool writeByte(uint8_t address, uint8_t reg, uint8_t data);
uint8_t readByte(uint8_t address, uint8_t reg);
bool readBytes(uint8_t address, uint8_t reg, uint8_t count, uint8_t* dest);

// Function prototypes for ESP32 communication
void setupSerialComm();
void processSerialComm();
void handleJsonCommand(const JsonDocument& doc);
void sendStatusUpdate();

// =============== MAIN CODE ===============

void setup() {

  // Initialize serial communication
  Serial.begin(115200);
  delay(1000);

 

  // Configure pins
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR_DIR, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR, OUTPUT);
  pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);  // Use internal pull-up resistor
  pinMode(LED_PIN, OUTPUT);
  pinMode(CALIBRATION_SWITCH, INPUT_PULLUP);
  
  // Set motors to stopped state
  stopMotors();
  
  // Configure hall sensor interrupt
  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hallSensorISR, FALLING);
  
  // Initialize I2C communication
  Wire.begin();
  
  Serial.println(F("Rover Navigation System - 8 Segment Version"));
  Serial.println(F("========================================="));
  
  // Initialize sensors
  if (!initSensors()) {
    Serial.println(F("Error initializing sensors. Cannot proceed."));
    while (1) {
      // Blink LED rapidly to indicate error
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }
  
  // Check if we should enter calibration mode
  if (digitalRead(CALIBRATION_SWITCH) == LOW) {
    Serial.println(F("Calibration switch ON - entering calibration mode"));
    currentMode = CALIBRATION;
    calibrateMagnetometer();
  } else {
    // Use pre-calibrated values
    Serial.println(F("Using pre-calibrated magnetometer values:"));
    Serial.print(F("X offset: ")); Serial.print(magXoffset);
    Serial.print(F(", Y offset: ")); Serial.print(magYoffset);
    Serial.print(F(", Z offset: ")); Serial.println(magZoffset);
    
    // Initialize navigation mode
    Serial.println(F("Entering navigation mode"));
    prepareNavigation();
  }

  setupSerialComm(); // Initialize Serial communication with ESP32
}

void loop() {



  // Main state machine
  switch (currentMode) {
    case CALIBRATION:
      updateCalibration();
      break;
      
    case NAVIGATION:
      // Read sensors and update navigation
      readSensors();
      
      // Handle LED blinking for pulse detection
      if (shouldBlinkLED) {
        digitalWrite(LED_PIN, HIGH);
        delay(50);
        digitalWrite(LED_PIN, LOW);
        shouldBlinkLED = false;
      }
      
      // Print pulse count every second for debugging
      static unsigned long lastPulseDisplay = 0;
      if (millis() - lastPulseDisplay > 1000) {
        Serial.print(F("Pulse count: "));
        Serial.print(pulseCount);
        Serial.print(F("/"));
        Serial.print(segmentTargets[currentSegment]);
        Serial.print(F(" | Segment: "));
        Serial.print(currentSegment + 1); // Display 1-based for user
        Serial.print(F("/"));
        Serial.println(TOTAL_SEGMENTS);
        lastPulseDisplay = millis();
      }
      
      // Update navigation logic
      updateNavigation();
      break;
      
    case STOPPED:
      // In stopped mode, just keep LED on
      digitalWrite(LED_PIN, HIGH);
      
      // Check if we should restart
      static unsigned long lastCheckTime = 0;
      if (millis() - lastCheckTime > 1000) {  // Check once per second
        if (digitalRead(CALIBRATION_SWITCH) == LOW) {
          // Recalibrate if switch is ON
          currentMode = CALIBRATION;
          calibrateMagnetometer();
        }
        lastCheckTime = millis();
      }
      break;
  }
  
  processSerialComm(); // Check for commands from ESP32
  // Brief delay for stability
  delay(10);
}


// =============== CALIBRATION CODE ===============

void calibrateMagnetometer() {
  Serial.println(F("\n===== MAGNETOMETER CALIBRATION ====="));
  Serial.println(F("Rotate the rover in all directions for 30 seconds."));
  Serial.println(F("Focus on horizontal rotations (full 360 degrees)."));
  Serial.println(F("The LED will blink during calibration."));
  Serial.println(F("Flip the switch OFF when done."));
  
  // Reset calibration flag during recalibration
  magCalibrated = false;
  
  // Initialize min/max values
  magXmin = magYmin = magZmin = 32767;
  magXmax = magYmax = magZmax = -32768;
  
  // Record start time
  calibrationStartTime = millis();
  
  // Rapid blink LED to signal start of calibration
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}

void updateCalibration() {
  // Check if calibration has been canceled
  if (digitalRead(CALIBRATION_SWITCH) == HIGH) {
    endCalibration();
    return;
  }
  
  // Read raw magnetometer data
  int16_t magRawX, magRawY, magRawZ;
  if (readRawMagnetometer(&magRawX, &magRawY, &magRawZ)) {
    // Update min/max values
    if (magRawX < magXmin) magXmin = magRawX;
    if (magRawX > magXmax) magXmax = magRawX;
    if (magRawY < magYmin) magYmin = magRawY;
    if (magRawY > magYmax) magYmax = magRawY;
    if (magRawZ < magZmin) magZmin = magRawZ;
    if (magRawZ > magZmax) magZmax = magRawZ;
    
    // Print values occasionally
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 500) {
      int secondsLeft = (CALIBRATION_DURATION - (millis() - calibrationStartTime)) / 1000;
      if (secondsLeft < 0) secondsLeft = 0;
      
      Serial.print(F("Cal: "));
      Serial.print(secondsLeft);
      Serial.print(F("s left | X: "));
      Serial.print(magRawX);
      Serial.print(F(", Y: "));
      Serial.print(magRawY);
      Serial.print(F(", Z: "));
      Serial.println(magRawZ);
      
      // Calculate ranges as percentages for XY plane (most important)
      int xRange = magXmax - magXmin;
      int yRange = magYmax - magYmin;
      int xyQuality = constrain(map(xRange + yRange, 0, 10000, 0, 100), 0, 100);
      
      // Create a simple progress bar
      Serial.print(F("XY Quality: ["));
      for (int i = 0; i < 20; i++) {
        Serial.print(i < xyQuality/5 ? '#' : '-');
      }
      Serial.print(F("] "));
      Serial.print(xyQuality);
      Serial.println(F("%"));
      
      lastPrint = millis();
    }
    
    // Flash LED at rate based on data quality
    static unsigned long lastLedToggle = 0;
    int xRange = magXmax - magXmin;
    int yRange = magYmax - magYmin;
    int xyQuality = constrain(map(xRange + yRange, 0, 10000, 0, 100), 0, 100);
    int flashRate = map(xyQuality, 0, 100, 500, 50);  // Faster flashing as quality improves
    
    if (millis() - lastLedToggle > flashRate) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      lastLedToggle = millis();
    }
  }
  
  // Check if calibration time is up
  if (millis() - calibrationStartTime >= CALIBRATION_DURATION) {
    endCalibration();
  }
}

void endCalibration() {
  // Check if we have enough data
  if (magXmin == 32767 || magXmax == -32768 || 
      magYmin == 32767 || magYmax == -32768 || 
      magZmin == 32767 || magZmax == -32768) {
    
    Serial.println(F("Error: Not enough data for calibration."));
    Serial.println(F("Using previous calibration values."));
    
    // Enter navigation mode with previous values
    magCalibrated = true;
    prepareNavigation();
    return;
  }
  
  // Calculate hard iron offsets (bias)
  magXoffset = (magXmax + magXmin) / 2.0;
  magYoffset = (magYmax + magYmin) / 2.0;
  magZoffset = (magZmax + magZmin) / 2.0;
  
  // Calculate soft iron scaling (optional, more advanced)
  float xRange = (magXmax - magXmin) / 2.0;
  float yRange = (magYmax - magYmin) / 2.0;
  float zRange = (magZmax - magZmin) / 2.0;
  
  // Find the maximum range to normalize all axes
  float maxRange = xRange;
  if (yRange > maxRange) maxRange = yRange;
  if (zRange > maxRange) maxRange = zRange;
  
  // Calculate scaling factors if ranges are valid (avoid division by zero)
  if (xRange > 0) magXscale = maxRange / xRange;
  if (yRange > 0) magYscale = maxRange / yRange;
  if (zRange > 0) magZscale = maxRange / zRange;
  
  // Print the calibration values
  Serial.println(F("\n=== Calibration Complete ==="));
  
  Serial.println(F("Hard Iron Offsets:"));
  Serial.print(F("X: ")); Serial.print(magXoffset);
  Serial.print(F("  Y: ")); Serial.print(magYoffset);
  Serial.print(F("  Z: ")); Serial.println(magZoffset);
  
  Serial.println(F("Soft Iron Scales:"));
  Serial.print(F("X: ")); Serial.print(magXscale, 3);
  Serial.print(F("  Y: ")); Serial.print(magYscale, 3);
  Serial.print(F("  Z: ")); Serial.println(magZscale, 3);
  
  magCalibrated = true;
  
  // Switch to navigation mode
  prepareNavigation();
}

// =============== NAVIGATION SETUP AND CONTROL ===============

void prepareNavigation() {
  // Get initial heading as reference
  for (int i = 0; i < 10; i++) {
    readSensors();
    delay(50);
  }
  
  targetHeading = calculateHeading();
  Serial.print(F("Initial heading: "));
  Serial.print(targetHeading);
  Serial.println(F(" degrees"));
  
  // Reset navigation variables
  integral = 0.0;
  prevError = 0.0;
  lastPIDTime = 0;
  
  // Start with segment 0
  currentSegment = 0;
  pulseCount = 0;
  
  // Signal ready to start
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
  
  // Wait before starting
  Serial.println(F("Starting in 3 seconds..."));
  delay(3000);
  
  // Begin navigation
  currentMode = NAVIGATION;
  executeSegment();
}

void updateNavigation() {
  // Get current heading
  float currentHeading = calculateHeading();
  
  // Check if we're in the middle of a turn
  if (turningInProgress) {
    // Calculate how far we've turned
    float headingChange = currentHeading - turnStartHeading;
    
    // Normalize the heading change to -180 to +180
    while (headingChange > 180) headingChange -= 360;
    while (headingChange < -180) headingChange += 360;
    
    // Check if turn is complete (or timeout occurred)
    bool turnComplete = false;
    
    if (TURN_DIRECTION > 0) {  // Right turn
      if (headingChange >= 85) {  // Allow for slight undershoot
        turnComplete = true;
      }
    } else {  // Left turn
      if (headingChange <= -85) {  // Allow for slight undershoot
        turnComplete = true;
      }
    }
    
    // Also add a timeout in case sensors are unreliable
    if (millis() - turnStartTime > 5000) {  // 5 second timeout
      turnComplete = true;
      Serial.println(F("Turn timeout reached."));
    }
    
    if (turnComplete) {
      turningInProgress = false;
      
      // Update target heading to maintain after turn
      targetHeading = currentHeading;
      
      // Move to next segment
      currentSegment++;
      pulseCount = 0;
      
      Serial.print(F("Turn complete. New heading: "));
      Serial.println(targetHeading);
      
      // Start moving for the next segment
      executeSegment();
    } else {
      // Continue turning
      int turnRate = 150; // Adjust this based on your robot's turning speed
      
      if (TURN_DIRECTION > 0) {  // Right turn
        startMotors(turnRate, -turnRate);  // Right turn
      } else {
        startMotors(-turnRate, turnRate);  // Left turn
      }
    }
  } else {
    // We're in a straight-line segment
    
    // Check if we've reached the end of the current segment
    if (pulseCount >= segmentTargets[currentSegment]) {
      Serial.print(F("Segment "));
      Serial.print(currentSegment + 1); // 1-based for display
      Serial.println(F(" complete."));
      
      // Reset pulse count for next segment
      pulseCount = 0;
      
      // Move to next segment
      currentSegment++;
      
      // Execute next segment
      executeSegment();
    } else {
      // Continue with current segment
      // Use PID to maintain straight line
      float correction = computePID();
      
      // Apply correction
      driveStraight(correction);
      
      // Print debug info occasionally
      if (millis() - lastUpdate > 500) {  // Every 500ms
        Serial.print(F("Heading: "));
        Serial.print(currentHeading);
        Serial.print(F("° Target: "));
        Serial.print(targetHeading);
        Serial.print(F("° Pulses: "));
        Serial.print(pulseCount);
        Serial.print(F("/"));
        Serial.print(segmentTargets[currentSegment]);
        Serial.print(F(" Correction: "));
        Serial.println(correction);
        
        lastUpdate = millis();
      }
    }
  }
}

void executeSegment() {
  if (currentSegment >= TOTAL_SEGMENTS) {
    // We've completed all segments
    finishRoute();
    return;
  }
  
  Serial.print(F("Starting segment "));
  Serial.print(currentSegment + 1); // 1-based for display
  Serial.print(F(" of "));
  Serial.println(TOTAL_SEGMENTS);
  
  // Even segments are straight lines, odd segments are turns
  if (currentSegment % 2 == 0) {
    // Straight segment
    Serial.print(F("Moving straight for "));
    Serial.print(segmentTargets[currentSegment]);
    Serial.println(F(" pulses"));
    driveStraight(0);  // Initial correction is zero
  } else {
    // Turn segment
    turnRobot(TURN_DIRECTION, 90);  // 90-degree turn
  }
}

float computePID() {
  unsigned long now = millis();
  float dt = (now - lastPIDTime) / 1000.0; // Convert to seconds
  
  // Use a minimum time step to avoid extreme values
  if (dt < 0.01) {
    dt = 0.01;
  }
  
  // Calculate current heading
  float currentHeading = calculateHeading();
  
  // Calculate error (difference from target heading)
  float error = targetHeading - currentHeading;
  
  // Normalize error to -180 to +180 degrees
  while (error > 180) error -= 360;
  while (error < -180) error += 360;
  
  // Calculate integral term
  integral += error * dt;
  
  // Limit integral to prevent windup
  if (integral > 20) integral = 20;
  if (integral < -20) integral = -20;
  
  // Calculate derivative term
  float derivative = (error - prevError) / dt;
  
  // Calculate PID output
  float output = Kp * error + Ki * integral + Kd * derivative;
  
  // Limit output to maximum correction
  if (output > MAX_CORRECTION) output = MAX_CORRECTION;
  if (output < -MAX_CORRECTION) output = -MAX_CORRECTION;
  
  // Save for next iteration
  prevError = error;
  lastPIDTime = now;
  
  return output;
}

// =============== SENSOR INITIALIZATION AND READING ===============

bool initSensors() {
  bool mpuInitialized = initMPU6050();
  
  // Try to initialize magnetometer, but continue if it fails
  magSensorPresent = initQMC5883L();
  if (!magSensorPresent) {
    Serial.println(F("WARNING: Magnetometer not available. Using gyroscope only."));
  }
  
  return mpuInitialized; // We need at least the MPU-6050 to work
}

bool initMPU6050() {
  Serial.println(F("Initializing MPU-6050..."));
  
  // Check communication
  Wire.beginTransmission(MPU_ADDR);
  byte error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.println(F("Communication error with MPU-6050."));
    return false;
  }
  
  // Check WHO_AM_I register
  uint8_t whoAmI = readByte(MPU_ADDR, WHO_AM_I_MPU);
  Serial.print(F("MPU WHO_AM_I: 0x"));
  Serial.println(whoAmI, HEX);
  
  if (whoAmI != 0x68) {
    Serial.println(F("Unexpected WHO_AM_I value. Continuing anyway..."));
  }
  
  // Reset device
  writeByte(MPU_ADDR, PWR_MGMT_1, 0x80);
  delay(100);
  
  // Wake up device and select best available clock source
  writeByte(MPU_ADDR, PWR_MGMT_1, 0x01);
  delay(100);
  
  // Configure device
  writeByte(MPU_ADDR, CONFIG, 0x03);         // Set DLPF to ~42Hz for gyro
  writeByte(MPU_ADDR, SMPLRT_DIV, 0x04);     // Set sample rate to 200Hz (1kHz / (1 + 4))
  writeByte(MPU_ADDR, GYRO_CONFIG, 0x00);    // Set gyro range to ±250 deg/s
  writeByte(MPU_ADDR, ACCEL_CONFIG, 0x00);   // Set accel range to ±2g
  
  // Perform gyro calibration
  Serial.println(F("Calibrating gyroscope..."));
  Serial.println(F("Keep the robot still."));
  
  int32_t gyroBiasXSum = 0, gyroBiasYSum = 0, gyroBiasZSum = 0;
  int validSamples = 0;
  
  for (int i = 0; i < 100; i++) {
    if (readAccelGyro()) {
      gyroBiasXSum += gyroX * GYRO_SCALE;
      gyroBiasYSum += gyroY * GYRO_SCALE;
      gyroBiasZSum += gyroZ * GYRO_SCALE;
      validSamples++;
    }
    delay(10);
  }
  
  if (validSamples > 0) {
    gyroBiasX = (float)gyroBiasXSum / validSamples / GYRO_SCALE;
    gyroBiasY = (float)gyroBiasYSum / validSamples / GYRO_SCALE;
    gyroBiasZ = (float)gyroBiasZSum / validSamples / GYRO_SCALE;
    
    Serial.println(F("Gyroscope calibrated successfully."));
  }
  
  Serial.println(F("MPU-6050 initialized."));
  sensorInitialized = true;
  return true;
}

bool initQMC5883L() {
  Serial.println(F("Initializing QMC5883L magnetometer..."));
  
  // Check communication
  Wire.beginTransmission(QMC5883L_ADDR);
  byte error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.println(F("Communication error with QMC5883L."));
    return false;
  }
  
  // Reset the device
  writeByte(QMC5883L_ADDR, QMC5883L_REG_SET_RESET, 0x01);
  delay(50);
  
  // Configure the device (8 samples avg, 200Hz data rate, +/-8 Gauss range)
  writeByte(QMC5883L_ADDR, QMC5883L_REG_CONFIG_1, 0x1D);
  
  // Set interrupt pin behavior
  writeByte(QMC5883L_ADDR, QMC5883L_REG_CONFIG_2, 0x01);
  delay(50);
  
  // Check configuration
  uint8_t config1 = readByte(QMC5883L_ADDR, QMC5883L_REG_CONFIG_1);
  if (config1 != 0x1D) {
    Serial.println(F("QMC5883L configuration failed."));
    return false;
  }
  
  Serial.println(F("QMC5883L initialized successfully."));
  return true;
}

void readSensors() {
  readAccelGyro();
  
  if (magSensorPresent) {
    readMagnetometer();
  }
  
  // Calculate roll and pitch from accelerometer
  roll = atan2(accelY, accelZ) * 180.0 / PI;
  pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;
}

bool readAccelGyro() {
  uint8_t rawData[14];
  
  if (!readBytes(MPU_ADDR, ACCEL_XOUT_H, 14, rawData)) {
    return false;
  }
  
  // Extract raw values
  int16_t accelRawX = ((int16_t)rawData[0] << 8) | rawData[1];
  int16_t accelRawY = ((int16_t)rawData[2] << 8) | rawData[3];
  int16_t accelRawZ = ((int16_t)rawData[4] << 8) | rawData[5];
  
  int16_t gyroRawX = ((int16_t)rawData[8] << 8) | rawData[9];
  int16_t gyroRawY = ((int16_t)rawData[10] << 8) | rawData[11];
  int16_t gyroRawZ = ((int16_t)rawData[12] << 8) | rawData[13];
  
  // Convert to physical units
  accelX = accelRawX / ACCEL_SCALE;
  accelY = accelRawY / ACCEL_SCALE;
  accelZ = accelRawZ / ACCEL_SCALE;
  
  // Apply calibration to gyro values
  gyroX = gyroRawX / GYRO_SCALE - gyroBiasX;
  gyroY = gyroRawY / GYRO_SCALE - gyroBiasY;
  gyroZ = gyroRawZ / GYRO_SCALE - gyroBiasZ;
  
  return true;
}

bool readMagnetometer() {
  int16_t magRawX, magRawY, magRawZ;
  
  if (!readRawMagnetometer(&magRawX, &magRawY, &magRawZ)) {
    return false;
  }
  
  // Apply calibration
  magX = (magRawX - magXoffset) * magXscale;
  magY = (magRawY - magYoffset) * magYscale;
  magZ = (magRawZ - magZoffset) * magZscale;
  
  return true;
}

bool readRawMagnetometer(int16_t* x, int16_t* y, int16_t* z) {
  uint8_t rawData[6];
  
  // Check status register first
  uint8_t status = readByte(QMC5883L_ADDR, QMC5883L_REG_STATUS);
  if (!(status & 0x01)) {  // Data not ready
    return false;
  }
  
  if (!readBytes(QMC5883L_ADDR, QMC5883L_REG_X_LSB, 6, rawData)) {
    return false;
  }
  
  // Convert the data (LSB first for QMC5883L)
  *x = ((int16_t)rawData[1] << 8) | rawData[0];
  *y = ((int16_t)rawData[3] << 8) | rawData[2];
  *z = ((int16_t)rawData[5] << 8) | rawData[4];
  
  return true;
}

float calculateHeading() {
  float heading;
  
  if (magSensorPresent && magCalibrated) {
    // Calculate tilt-compensated heading when magnetometer is available
    float magXcomp = magX * cos(pitch * PI/180) + magZ * sin(pitch * PI/180);
    float magYcomp = magX * sin(roll * PI/180) * sin(pitch * PI/180) + 
                     magY * cos(roll * PI/180) - 
                     magZ * sin(roll * PI/180) * cos(pitch * PI/180);
    
    heading = atan2(magYcomp, magXcomp) * 180.0 / PI;
  } else {
    // Use integrated gyro for heading when magnetometer is not available
    static float gyroHeading = 0.0;
    static unsigned long lastGyroUpdate = 0;
    
    unsigned long now = millis();
    float dt = (now - lastGyroUpdate) / 1000.0; // Convert to seconds
    
    if (lastGyroUpdate != 0 && dt > 0.001 && dt < 0.5) {
      gyroHeading += gyroZ * dt; // Integrate gyro Z rate to get heading
    }
    
    lastGyroUpdate = now;
    heading = gyroHeading;
  }
  
  // Normalize to 0-360 degrees
  while (heading < 0) heading += 360;
  while (heading >= 360) heading -= 360;
  
  return heading;
}

// =============== MOTOR CONTROL ===============

void driveStraight(float correction) {
  // Calculate motor speeds based on correction
  int leftSpeed = BASE_SPEED - correction;
  int rightSpeed = BASE_SPEED + correction;
  
  // Ensure speeds are within valid range
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
  // Apply speeds to motors
  startMotors(leftSpeed, rightSpeed);
}

void turnRobot(int direction, float degrees) {
  // Start a turning sequence
  turningInProgress = true;
  turnStartTime = millis();
  turnStartHeading = calculateHeading();
  
  Serial.print(F("Starting "));
  Serial.print(degrees);
  Serial.print(F(" degree turn "));
  Serial.println(direction > 0 ? F("right") : F("left"));
  
  // Calculate target heading
  turnTargetHeading = turnStartHeading + (direction * degrees);
  
  // Normalize to 0-360
  while (turnTargetHeading < 0) turnTargetHeading += 360;
  while (turnTargetHeading >= 360) turnTargetHeading -= 360;
  
  Serial.print(F("Current heading: "));
  Serial.print(turnStartHeading);
  Serial.print(F("° Target heading: "));
  Serial.println(turnTargetHeading);
  
  // Initial turning motor control is applied in updateNavigation()
}

void startMotors(int leftSpeed, int rightSpeed) {
  // Set motor directions based on sign
  if (leftSpeed >= 0) {
    digitalWrite(LEFT_MOTOR_DIR, HIGH);  // Forward
  } else {
    digitalWrite(LEFT_MOTOR_DIR, LOW);   // Reverse
    leftSpeed = -leftSpeed;              // Make positive for PWM
  }
  
  if (rightSpeed >= 0) {
    digitalWrite(RIGHT_MOTOR_DIR, HIGH); // Forward
  } else {
    digitalWrite(RIGHT_MOTOR_DIR, LOW);  // Reverse
    rightSpeed = -rightSpeed;            // Make positive for PWM
  }
  
  // Apply PWM values
  analogWrite(LEFT_MOTOR_PWM, leftSpeed);
  analogWrite(RIGHT_MOTOR_PWM, rightSpeed);
}

void stopMotors() {
  // Set motors to zero power
  analogWrite(LEFT_MOTOR_PWM, 0);
  analogWrite(RIGHT_MOTOR_PWM, 0);
  
  // Force motors to stop
  digitalWrite(LEFT_MOTOR_DIR, LOW);
  digitalWrite(RIGHT_MOTOR_DIR, LOW);
}

void finishRoute() {
  // Stop the robot
  stopMotors();
  currentMode = STOPPED;
  
  Serial.println(F("Route completed! Robot stopped."));
  
  // Blink LED 5 times to indicate completion
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
  
  // LED stays on to show stopped state
  digitalWrite(LED_PIN, HIGH);
}

// =============== INTERRUPT SERVICE ROUTINE ===============

void hallSensorISR() {
  // Only count pulses during navigation, not during turns,
  // and only if enough time has passed since last pulse (debounce)
  unsigned long now = millis();
  if (currentMode == NAVIGATION && !turningInProgress && 
      (now - lastPulseTime > DEBOUNCE_TIME)) {
    
    // Update last pulse time
    lastPulseTime = now;
    
    // Increment pulse count
    pulseCount++;
    
    // Set flag to blink LED in main loop
    shouldBlinkLED = true;
  }
}

// =============== I2C UTILITY FUNCTIONS ===============

bool writeByte(uint8_t address, uint8_t reg, uint8_t data) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(data);
  byte error = Wire.endTransmission();
  
  return (error == 0);
}

uint8_t readByte(uint8_t address, uint8_t reg) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false);
  
  Wire.requestFrom(address, (uint8_t)1);
  if (Wire.available()) {
    return Wire.read();
  }
  
  return 0;
}

bool readBytes(uint8_t address, uint8_t reg, uint8_t count, uint8_t* dest) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  byte error = Wire.endTransmission(false);
  
  if (error != 0) {
    return false;
  }
  
  uint8_t bytesReceived = Wire.requestFrom(address, count);
  
  if (bytesReceived != count) {
    return false;
  }
  
  for (uint8_t i = 0; i < count; i++) {
    dest[i] = Wire.read();
  }
  
  return true;
}


//websocket esp 32 communitaction functions

void setupSerialComm() {
  // Initialize Serial1 for communication with ESP32
  Serial1.begin(115200);
  
  Serial.println(F("Serial communication initialized"));
}



void processSerialComm() {
  // Process incoming serial data
  while (Serial1.available()) {
    char c = Serial1.read();
    
    // Handle JSON message starting with '{'
    if (c == '{') {
      bufferIndex = 0;
      inputBuffer[bufferIndex++] = c;
    }
    // Add to buffer if inside a message
    else if (bufferIndex > 0 && bufferIndex < BUFFER_SIZE - 1) {
      inputBuffer[bufferIndex++] = c;
      
      // End of message
      if (c == '}') {
        inputBuffer[bufferIndex] = '\0'; // Null terminate
        
        // Process the message
        StaticJsonDocument<256> doc;
        DeserializationError error = deserializeJson(doc, inputBuffer);
        
        if (!error) {
          // Process the JSON command
          handleJsonCommand(doc);
        } else {
          Serial.print(F("JSON parsing error: "));
          Serial.println(error.c_str());
        }
        
        // Reset for next message
        bufferIndex = 0;
      }
    }
    // Buffer overflow protection
    else if (bufferIndex >= BUFFER_SIZE - 1) {
      bufferIndex = 0;
    }
  }
}

// Send status update to ESP32
void sendStatusUpdate() {
  StaticJsonDocument<256> status;
  status["type"] = "status";
  status["segment"] = currentSegment + 1; // 1-based for display
  status["pulseCount"] = pulseCount;
  
  // Get target pulses for current segment
  uint16_t targetPulses = currentSegment < TOTAL_SEGMENTS ? segmentTargets[currentSegment] : 0;
  status["targetPulses"] = targetPulses;
  
  // Add mode information
  switch (currentMode) {
    case CALIBRATION:
      status["mode"] = "calibration";
      break;
    case NAVIGATION:
      status["mode"] = "navigation";
      break;
    case STOPPED:
      status["mode"] = "stopped";
      break;
  }
  
  // Add heading information if available
  if (sensorInitialized) {
    status["heading"] = calculateHeading();
  }
  
  // Add turning status
  status["isTurning"] = turningInProgress;
  
  // Send the status JSON
  String statusJson;
  serializeJson(status, statusJson);
  Serial1.println(statusJson);
}

// Handle JSON commands from ESP32
void handleJsonCommand(const JsonDocument& doc) {
  const char* cmd = doc["cmd"];
  
  if (cmd) {
    if (strcmp(cmd, "config") == 0) {
      // Handle configuration command
      Serial.println(F("Received configuration command"));
      
      // Get shape type
      const char* shape = doc["shape"];
      
      // Get pulses array - fix the array access
      JsonArrayConst pulses = doc["pulses"].as<JsonArrayConst>();
      
      // Update configuration if in STOPPED mode
      if (currentMode == STOPPED) {
        // Reset current segment and pulse count
        currentSegment = 0;
        pulseCount = 0;
        
        // Configure segments based on shape
        if (shape && strcmp(shape, "circle") == 0) {
          // For circles, create a special navigation pattern
          float radius = doc["radius"];
          int angle = doc["angle"];
          
          Serial.print(F("Circle: radius="));
          Serial.print(radius);
          Serial.print(F(", angle="));
          Serial.println(angle);
          
          // For simplicity, we'll use the first segment for a circle
          if (!pulses.isNull() && pulses.size() > 0) {
            segmentTargets[0] = pulses[0]; // Total pulses for circle
            
            // Set remaining segments to 0
            for (int i = 1; i < TOTAL_SEGMENTS; i++) {
              segmentTargets[i] = 0;
            }
          }
        } 
        else if (shape && (strcmp(shape, "square") == 0 || strcmp(shape, "rectangle") == 0)) {
          // For square/rectangle, assign pulses to segments with turns in between
          int segmentCount = 0;
          
          if (!pulses.isNull()) {
            for (size_t i = 0; i < pulses.size() && segmentCount < TOTAL_SEGMENTS; i++) {
              // Straight segment
              segmentTargets[segmentCount++] = pulses[i];
              
              // Turn segment (if not the last segment)
              if (i < pulses.size() - 1 && segmentCount < TOTAL_SEGMENTS) {
                segmentTargets[segmentCount++] = 0; // Turn segments use 0 pulses
              }
            }
          }
          
          // Clear any unused segments
          for (int i = segmentCount; i < TOTAL_SEGMENTS; i++) {
            segmentTargets[i] = 0;
          }
          
          if (strcmp(shape, "square") == 0) {
            float side = doc["side"];
            Serial.print(F("Square: side="));
            Serial.println(side);
          } else {
            float length = doc["length"];
            float width = doc["width"];
            Serial.print(F("Rectangle: length="));
            Serial.print(length);
            Serial.print(F(", width="));
            Serial.println(width);
          }
        }
        
        // Print the updated segment targets
        Serial.println(F("Updated segment targets:"));
        for (int i = 0; i < TOTAL_SEGMENTS; i++) {
          Serial.print(segmentTargets[i]);
          Serial.print(F(" "));
        }
        Serial.println();
        
        // Send confirmation
        Serial1.println("{\"type\":\"status\",\"message\":\"Configuration updated\",\"success\":true}");
      } else {
        // Cannot update while navigating
        Serial.println(F("Cannot update configuration while navigating"));
        Serial1.println("{\"type\":\"status\",\"message\":\"Cannot update while navigating\",\"success\":false}");
      }
    }
    else if (strcmp(cmd, "status") == 0) {
      // Send status update
      sendStatusUpdate();
    }
    else if (strcmp(cmd, "start") == 0) {
      // Start navigation if in STOPPED mode
      if (currentMode == STOPPED) {
        prepareNavigation();
        Serial1.println("{\"type\":\"status\",\"message\":\"Navigation started\",\"success\":true}");
      } else {
        Serial1.println("{\"type\":\"status\",\"message\":\"Already navigating\",\"success\":false}");
      }
    }
    else if (strcmp(cmd, "stop") == 0) {
      // Stop navigation if in NAVIGATION mode
      if (currentMode == NAVIGATION) {
        stopMotors();
        currentMode = STOPPED;
        Serial1.println("{\"type\":\"status\",\"message\":\"Navigation stopped\",\"success\":true}");
      } else {
        Serial1.println("{\"type\":\"status\",\"message\":\"Already stopped\",\"success\":false}");
      }
    }
  }
} 