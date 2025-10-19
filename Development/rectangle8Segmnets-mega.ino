// Arduino Mega - Extended Control with Sensors and Rectangle Navigation
#include <Wire.h>

// Motor Pins
#define LEFT_FORWARD 6
#define LEFT_BACKWARD 5
#define RIGHT_FORWARD 10
#define RIGHT_BACKWARD 9

// Status LED
#define STATUS_LED 36

// Hall Effect Sensor
#define HALL_SENSOR_PIN 3

// =============== SENSOR ADDRESSES ===============
// MPU-6050 Settings
#define MPU_ADDR        0x68
#define PWR_MGMT_1      0x6B
#define ACCEL_CONFIG    0x1C
#define GYRO_CONFIG     0x1B
#define CONFIG          0x1A
#define SMPLRT_DIV      0x19
#define ACCEL_XOUT_H    0x3B
#define GYRO_XOUT_H     0x43
#define WHO_AM_I_MPU    0x75

// QMC5883L Settings
#define QMC5883L_ADDR          0x0D
#define QMC5883L_REG_X_LSB     0x00
#define QMC5883L_REG_Y_LSB     0x02
#define QMC5883L_REG_Z_LSB     0x04
#define QMC5883L_REG_STATUS    0x06
#define QMC5883L_REG_CONFIG_1  0x09
#define QMC5883L_REG_CONFIG_2  0x0A
#define QMC5883L_REG_SET_RESET 0x0B

// =============== GLOBAL VARIABLES ===============
// Motor control
int currentLeftSpeed = 0;
int currentRightSpeed = 0;
int baseLeftPWM = 150;
int baseRightPWM = 150;

// Sensor data
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float magX, magY, magZ;
float roll, pitch, yaw;

// Scaling factors
const float ACCEL_SCALE = 16384.0;
const float GYRO_SCALE = 131.0;

// Sensor calibration
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
float magXoffset = -46.0, magYoffset = -742.5, magZoffset = -207.5;
float magXscale = 1.261, magYscale = 1.000, magZscale = 1.143;

// PID Controller
float Kp = 2.0, Ki = 0.1, Kd = 0.5;
float targetHeading = 0.0;
float integral = 0.0;
float prevError = 0.0;
unsigned long lastPIDTime = 0;

// Navigation state
bool straightMode = false;
bool turningMode = false;
bool manualMode = false;
bool rectangleMode = false;
bool sensorsInitialized = false;
bool magSensorPresent = false;
String turnDirection = "";
float turnStartHeading = 0.0;
unsigned long turnStartTime = 0;

// Rectangle navigation variables
int rectLength = 4;
int rectWidth = 2;
String rectDirection = "clockwise"; // "clockwise" or "anticlockwise"
int rectSpeed = 120;
int rectCurrentSegment = 0; // 0=length1, 1=turn1, 2=width1, 3=turn2, 4=length2, 5=turn3, 6=width2, 7=turn4
int rectCurrentPulses[4] = {4, 0, 2, 0}; // length, turn, width, turn pattern
int rectCurrentTarget = 0;

// Hall sensor variables
volatile unsigned long pulseCount = 0;
volatile unsigned long lastPulseTime = 0;
volatile unsigned long consecutivePulses = 0;
volatile unsigned long lastValidPulseTime = 0;
bool hallSensorEnabled = false;
bool magnetInterferenceDetected = false;

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  
  pinMode(LEFT_FORWARD, OUTPUT);
  pinMode(LEFT_BACKWARD, OUTPUT);
  pinMode(RIGHT_FORWARD, OUTPUT);
  pinMode(RIGHT_BACKWARD, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);
  
  // Setup hall sensor interrupt
  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hallSensorISR, FALLING);
  
  // Initialize I2C
  Wire.begin();
  
  // Initialize sensors
  digitalWrite(STATUS_LED, HIGH); // LED on during initialization
  if (initSensors()) {
    sensorsInitialized = true;
    Serial.println("Sensors initialized successfully");
    // Blink LED 3 times to indicate success
    for (int i = 0; i < 3; i++) {
      digitalWrite(STATUS_LED, LOW);
      delay(200);
      digitalWrite(STATUS_LED, HIGH);
      delay(200);
    }
  } else {
    Serial.println("Sensor initialization failed");
    // Rapid blink to indicate error
    for (int i = 0; i < 10; i++) {
      digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
      delay(100);
    }
  }
  
  stopMotors();
  Serial.println("Extended Motor Controller Ready");
  Serial2.println("READY");
}

void loop() {
  // Handle serial commands
  if (Serial2.available()) {
    String cmd = Serial2.readStringUntil('\n');
    cmd.trim();
    Serial.println("Received: " + cmd);
    handleCommand(cmd);
  }
  
  // Update sensor readings
  if (sensorsInitialized) {
    readSensors();
  }
  
  // Handle straight line mode with PID
  if (straightMode && sensorsInitialized) {
    updateStraightMode();
  }
  
  // Handle turning mode
  if (turningMode && sensorsInitialized) {
    updateTurnMode();
  }
  
  // Handle rectangle mode
  if (rectangleMode && sensorsInitialized) {
    updateRectangleMode();
  }
  
  // Update status LED
  if (straightMode || turningMode || rectangleMode) {
    // Slow blink during operation
    static unsigned long lastLedToggle = 0;
    if (millis() - lastLedToggle > 500) {
      digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
      lastLedToggle = millis();
    }
  } else {
    digitalWrite(STATUS_LED, HIGH); // Solid on when idle
  }
  
  delay(20);
}

void handleCommand(String cmd) {
  if (cmd.startsWith("MOVE:")) {
    // Manual motor control
    int commaPos = cmd.indexOf(',');
    if (commaPos > 0) {
      int leftSpeed = cmd.substring(5, commaPos).toInt();
      int rightSpeed = cmd.substring(commaPos + 1).toInt();
      straightMode = false;
      turningMode = false;
      manualMode = false;
      rectangleMode = false;
      hallSensorEnabled = false;
      moveMotors(leftSpeed, rightSpeed);
      Serial2.println("MOVED:" + String(leftSpeed) + "," + String(rightSpeed));
    }
  }
  else if (cmd.startsWith("MANUAL:")) {
    // Manual navigation control - single command execution
    int commaPos = cmd.indexOf(',');
    if (commaPos > 0) {
      String direction = cmd.substring(7, commaPos);
      int speed = cmd.substring(commaPos + 1).toInt();
      straightMode = false;
      turningMode = false;
      manualMode = false;
      rectangleMode = false;
      hallSensorEnabled = false;
      
      handleManualNavigation(direction, speed);
      Serial2.println("MANUAL:" + direction);
    }
  }
  else if (cmd.startsWith("STRAIGHT:")) {
    // Start straight line mode
    int commaPos = cmd.indexOf(',');
    if (commaPos > 0) {
      baseLeftPWM = cmd.substring(9, commaPos).toInt();
      baseRightPWM = cmd.substring(commaPos + 1).toInt();
      manualMode = false;
      rectangleMode = false;
      hallSensorEnabled = false;
      startStraightMode();
      Serial2.println("STRAIGHT_STARTED");
    }
  }
  else if (cmd.startsWith("TURN:")) {
    // Start turn mode
    turnDirection = cmd.substring(5);
    manualMode = false;
    rectangleMode = false;
    hallSensorEnabled = false;
    startTurnMode();
    Serial2.println("TURN_STARTED:" + turnDirection);
  }
  else if (cmd.startsWith("RECTANGLE:")) {
    // Start rectangle mode: RECTANGLE:length,width,direction,speed
    int comma1 = cmd.indexOf(',');
    int comma2 = cmd.indexOf(',', comma1 + 1);
    int comma3 = cmd.indexOf(',', comma2 + 1);
    if (comma1 > 0 && comma2 > 0 && comma3 > 0) {
      rectLength = cmd.substring(10, comma1).toInt();
      rectWidth = cmd.substring(comma1 + 1, comma2).toInt();
      rectDirection = cmd.substring(comma2 + 1, comma3);
      rectSpeed = cmd.substring(comma3 + 1).toInt();
      
      manualMode = false;
      straightMode = false;
      turningMode = false;
      startRectangleMode();
      Serial2.println("RECTANGLE_STARTED");
    }
  }
  else if (cmd.startsWith("PID:")) {
    // Set PID parameters
    int comma1 = cmd.indexOf(',');
    int comma2 = cmd.indexOf(',', comma1 + 1);
    if (comma1 > 0 && comma2 > 0) {
      Kp = cmd.substring(4, comma1).toFloat();
      Ki = cmd.substring(comma1 + 1, comma2).toFloat();
      Kd = cmd.substring(comma2 + 1).toFloat();
      Serial.println("PID updated: Kp=" + String(Kp) + " Ki=" + String(Ki) + " Kd=" + String(Kd));
      Serial2.println("PID_SET");
    }
  }
  else if (cmd == "STOP") {
    straightMode = false;
    turningMode = false;
    manualMode = false;
    rectangleMode = false;
    hallSensorEnabled = false;
    magnetInterferenceDetected = false; // Reset interference flag
    pulseCount = 0; // Reset pulse count on manual stop
    stopMotors();
    Serial2.println("STOPPED");
  }
  else if (cmd == "STATUS") {
    String status = "STATUS:L=" + String(currentLeftSpeed) + ",R=" + String(currentRightSpeed);
    status += ",Pulses=" + String(pulseCount);
    if (magnetInterferenceDetected) {
      status += ",MAGNET_INTERFERENCE=TRUE";
    }
    if (sensorsInitialized) {
      status += ",H=" + String(calculateHeading(), 1);
      status += ",Mode=";
      if (rectangleMode) status += "RECTANGLE(" + String(rectCurrentSegment) + "/8)";
      else if (straightMode) status += "STRAIGHT";
      else if (turningMode) status += "TURNING";
      else if (manualMode) status += "MANUAL_NAV";
      else status += "MANUAL";
    } else {
      status += ",Sensors=FAILED";
    }
    Serial2.println(status);
  }
}

// =============== SENSOR FUNCTIONS ===============
bool initSensors() {
  bool mpuOk = initMPU6050();
  magSensorPresent = initQMC5883L();
  
  if (!magSensorPresent) {
    Serial.println("WARNING: Magnetometer not available. Using gyroscope only.");
  }
  
  return mpuOk;
}

bool initMPU6050() {
  Serial.println("Initializing MPU-6050...");
  
  // Check communication
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("MPU-6050 not found");
    return false;
  }
  
  // Check WHO_AM_I
  uint8_t whoAmI = readByte(MPU_ADDR, WHO_AM_I_MPU);
  Serial.print("MPU WHO_AM_I: 0x");
  Serial.println(whoAmI, HEX);
  
  if (whoAmI != 0x68) {
    Serial.println("Unexpected WHO_AM_I value");
  }
  
  // Reset and configure
  writeByte(MPU_ADDR, PWR_MGMT_1, 0x80);
  delay(100);
  writeByte(MPU_ADDR, PWR_MGMT_1, 0x01);
  delay(100);
  writeByte(MPU_ADDR, CONFIG, 0x03);
  writeByte(MPU_ADDR, SMPLRT_DIV, 0x04);
  writeByte(MPU_ADDR, GYRO_CONFIG, 0x00);
  writeByte(MPU_ADDR, ACCEL_CONFIG, 0x00);
  
  // Calibrate gyroscope
  Serial.println("Calibrating gyroscope... Keep robot still.");
  calibrateGyro();
  
  Serial.println("MPU-6050 initialized");
  return true;
}

bool initQMC5883L() {
  Serial.println("Initializing QMC5883L...");
  
  Wire.beginTransmission(QMC5883L_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("QMC5883L not found");
    return false;
  }
  
  // Reset and configure
  writeByte(QMC5883L_ADDR, QMC5883L_REG_SET_RESET, 0x01);
  delay(50);
  writeByte(QMC5883L_ADDR, QMC5883L_REG_CONFIG_1, 0x1D);
  writeByte(QMC5883L_ADDR, QMC5883L_REG_CONFIG_2, 0x01);
  delay(50);
  
  Serial.println("QMC5883L initialized");
  return true;
}

void calibrateGyro() {
  int32_t sumX = 0, sumY = 0, sumZ = 0;
  int samples = 100;
  
  for (int i = 0; i < samples; i++) {
    if (readAccelGyro()) {
      sumX += gyroX * GYRO_SCALE;
      sumY += gyroY * GYRO_SCALE;
      sumZ += gyroZ * GYRO_SCALE;
    }
    delay(10);
  }
  
  gyroBiasX = (float)sumX / samples / GYRO_SCALE;
  gyroBiasY = (float)sumY / samples / GYRO_SCALE;
  gyroBiasZ = (float)sumZ / samples / GYRO_SCALE;
  
  Serial.println("Gyroscope calibrated");
}

void readSensors() {
  readAccelGyro();
  if (magSensorPresent) {
    readMagnetometer();
  }
  
  // Calculate roll and pitch
  roll = atan2(accelY, accelZ) * 180.0 / PI;
  pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;
}

bool readAccelGyro() {
  uint8_t rawData[14];
  if (!readBytes(MPU_ADDR, ACCEL_XOUT_H, 14, rawData)) {
    return false;
  }
  
  int16_t accelRawX = ((int16_t)rawData[0] << 8) | rawData[1];
  int16_t accelRawY = ((int16_t)rawData[2] << 8) | rawData[3];
  int16_t accelRawZ = ((int16_t)rawData[4] << 8) | rawData[5];
  
  int16_t gyroRawX = ((int16_t)rawData[8] << 8) | rawData[9];
  int16_t gyroRawY = ((int16_t)rawData[10] << 8) | rawData[11];
  int16_t gyroRawZ = ((int16_t)rawData[12] << 8) | rawData[13];
  
  accelX = accelRawX / ACCEL_SCALE;
  accelY = accelRawY / ACCEL_SCALE;
  accelZ = accelRawZ / ACCEL_SCALE;
  
  gyroX = gyroRawX / GYRO_SCALE - gyroBiasX;
  gyroY = gyroRawY / GYRO_SCALE - gyroBiasY;
  gyroZ = gyroRawZ / GYRO_SCALE - gyroBiasZ;
  
  return true;
}

bool readMagnetometer() {
  uint8_t rawData[6];
  
  // Check if data is ready
  uint8_t status = readByte(QMC5883L_ADDR, QMC5883L_REG_STATUS);
  if (!(status & 0x01)) {
    return false;
  }
  
  if (!readBytes(QMC5883L_ADDR, QMC5883L_REG_X_LSB, 6, rawData)) {
    return false;
  }
  
  int16_t magRawX = ((int16_t)rawData[1] << 8) | rawData[0];
  int16_t magRawY = ((int16_t)rawData[3] << 8) | rawData[2];
  int16_t magRawZ = ((int16_t)rawData[5] << 8) | rawData[4];
  
  // Apply calibration
  magX = (magRawX - magXoffset) * magXscale;
  magY = (magRawY - magYoffset) * magYscale;
  magZ = (magRawZ - magZoffset) * magZscale;
  
  return true;
}

float calculateHeading() {
  if (magSensorPresent) {
    // Tilt-compensated compass heading
    float magXcomp = magX * cos(pitch * PI/180) + magZ * sin(pitch * PI/180);
    float magYcomp = magX * sin(roll * PI/180) * sin(pitch * PI/180) + 
                     magY * cos(roll * PI/180) - 
                     magZ * sin(roll * PI/180) * cos(pitch * PI/180);
    
    float heading = atan2(magYcomp, magXcomp) * 180.0 / PI;
    
    // Normalize to 0-360
    while (heading < 0) heading += 360;
    while (heading >= 360) heading -= 360;
    
    return heading;
  } else {
    // Use integrated gyro for heading when magnetometer is not available
    static float gyroHeading = 0.0;
    static unsigned long lastGyroUpdate = 0;
    
    unsigned long now = millis();
    float dt = (now - lastGyroUpdate) / 1000.0;
    
    if (lastGyroUpdate != 0 && dt > 0.001 && dt < 0.5) {
      gyroHeading += gyroZ * dt;
    }
    
    lastGyroUpdate = now;
    
    // Normalize to 0-360
    while (gyroHeading < 0) gyroHeading += 360;
    while (gyroHeading >= 360) gyroHeading -= 360;
    
    return gyroHeading;
  }
}

// =============== NAVIGATION FUNCTIONS ===============
void startStraightMode() {
  if (!sensorsInitialized) {
    Serial.println("Cannot start straight mode - sensors not initialized");
    return;
  }
  
  // Read current heading as target
  for (int i = 0; i < 10; i++) {
    readSensors();
    delay(20);
  }
  
  targetHeading = calculateHeading();
  Serial.print("Target heading set to: ");
  Serial.println(targetHeading);
  
  // Reset PID variables
  integral = 0.0;
  prevError = 0.0;
  lastPIDTime = millis();
  
  straightMode = true;
  turningMode = false;
  
  Serial.println("Straight mode started");
}

void updateStraightMode() {
  float currentHeading = calculateHeading();
  float correction = computePID(currentHeading);
  
  // Apply PID correction to base PWM values
  int leftPWM = baseLeftPWM - correction;
  int rightPWM = baseRightPWM + correction;
  
  // Constrain PWM values
  leftPWM = constrain(leftPWM, 0, 255);
  rightPWM = constrain(rightPWM, 0, 255);
  
  moveMotors(leftPWM, rightPWM);
  
  // Debug output occasionally
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 500) {
    Serial.print("Heading: ");
    Serial.print(currentHeading, 1);
    Serial.print("° Target: ");
    Serial.print(targetHeading, 1);
    Serial.print("° Correction: ");
    Serial.print(correction, 1);
    Serial.print(" PWM: L=");
    Serial.print(leftPWM);
    Serial.print(" R=");
    Serial.println(rightPWM);
    lastDebug = millis();
  }
}

void startTurnMode() {
  if (!sensorsInitialized) {
    Serial.println("Cannot start turn mode - sensors not initialized");
    return;
  }
  
  // Read current heading
  for (int i = 0; i < 5; i++) {
    readSensors();
    delay(20);
  }
  
  turnStartHeading = calculateHeading();
  turnStartTime = millis();
  
  straightMode = false;
  turningMode = true;
  
  Serial.print("Starting 90° turn ");
  Serial.print(turnDirection);
  Serial.print(" from heading: ");
  Serial.println(turnStartHeading);
}

void updateTurnMode() {
  float currentHeading = calculateHeading();
  float headingChange = currentHeading - turnStartHeading;
  
  // Normalize heading change to -180 to +180
  while (headingChange > 180) headingChange -= 360;
  while (headingChange < -180) headingChange += 360;
  
  bool turnComplete = false;
  int turnSpeed = 75; // Optimized speed for precision
  
  // FIXED LOGIC: Check absolute value and direction separately
  float absChange = abs(headingChange);
  
  if (turnDirection == "right") {
    moveMotors(turnSpeed, -turnSpeed); // Turn right (clockwise)
    // Right turn should give POSITIVE heading change
    // Stop when we've turned about 90 degrees clockwise (tightened range)
    if (headingChange > 85 && headingChange < 95) {
      turnComplete = true;
    }
  } else if (turnDirection == "left") {
    moveMotors(-turnSpeed, turnSpeed); // Turn left (counter-clockwise)  
    // Left turn should give NEGATIVE heading change
    // Stop when we've turned about 90 degrees counter-clockwise (tightened range)
    if (headingChange < -85 && headingChange > -95) {
      turnComplete = true;
    }
  }
  
  // Emergency stops for overshoot
  if (absChange > 110) {
    turnComplete = true;
    Serial.println("Emergency stop - overshoot detected");
  }
  
  // Timeout after 15 seconds
  if (millis() - turnStartTime > 15000) {
    turnComplete = true;
    Serial.println("Turn timeout reached");
  }
  
  if (turnComplete) {
    stopMotors();
    turningMode = false;
    
    Serial.print("Turn complete! Final heading: ");
    Serial.print(currentHeading, 1);
    Serial.print("° Start: ");
    Serial.print(turnStartHeading, 1);
    Serial.print("° Change: ");
    Serial.print(headingChange, 1);
    Serial.println("°");
    
    // Extra delay to ensure complete stop
    delay(200);
    
    // If this was part of rectangle mode, continue rectangle
    if (rectangleMode) {
      continueRectangleAfterTurn();
    }
  }
  
  // Debug output every 500ms during turn
  static unsigned long lastTurnDebug = 0;
  if (millis() - lastTurnDebug > 500) {
    Serial.print("TURN DEBUG - Direction: ");
    Serial.print(turnDirection);
    Serial.print(", Start: ");
    Serial.print(turnStartHeading, 1);
    Serial.print("°, Current: ");
    Serial.print(currentHeading, 1);
    Serial.print("°, Change: ");
    Serial.print(headingChange, 1);
    Serial.print("°, Target: ");
    Serial.println(turnDirection == "right" ? "+90°" : "-90°");
    lastTurnDebug = millis();
  }
}

float computePID(float currentHeading) {
  unsigned long now = millis();
  float dt = (now - lastPIDTime) / 1000.0;
  
  if (dt < 0.01) dt = 0.01; // Minimum time step
  
  // Calculate error
  float error = targetHeading - currentHeading;
  
  // Normalize error to -180 to +180
  while (error > 180) error -= 360;
  while (error < -180) error += 360;
  
  // Calculate integral
  integral += error * dt;
  
  // Limit integral windup
  if (integral > 20) integral = 20;
  if (integral < -20) integral = -20;
  
  // Calculate derivative
  float derivative = (error - prevError) / dt;
  
  // Calculate PID output
  float output = Kp * error + Ki * integral + Kd * derivative;
  
  // Limit output
  if (output > 50) output = 50;
  if (output < -50) output = -50;
  
  prevError = error;
  lastPIDTime = now;
  
  return output;
}

// =============== MANUAL NAVIGATION ===============
void handleManualNavigation(String direction, int speed) {
  Serial.print("Manual navigation: ");
  Serial.print(direction);
  Serial.print(" at speed ");
  Serial.println(speed);
  
  if (direction == "forward") {
    moveMotors(speed, speed);
  }
  else if (direction == "backward") {
    moveMotors(-speed, -speed);
  }
  else if (direction == "left") {
    // Gentle left turn
    moveMotors(speed/4, speed);
  }
  else if (direction == "right") {
    // Gentle right turn  
    moveMotors(speed, speed/4);
  }
  else {
    Serial.println("Unknown manual direction");
    stopMotors();
  }
}

// =============== RECTANGLE NAVIGATION ===============
void startRectangleMode() {
  if (!sensorsInitialized) {
    Serial.println("Cannot start rectangle mode - sensors not initialized");
    return;
  }
  
  // Check for magnet interference before starting
  pulseCount = 0;
  magnetInterferenceDetected = false;
  delay(100); // Brief delay to check for interference
  
  if (detectMagnetInterference()) {
    Serial.println("ERROR: Magnet interference detected! Remove magnet from wheel area and try again.");
    Serial2.println("MAGNET_INTERFERENCE_ERROR");
    return;
  }
  
  Serial.println("=== STARTING RECTANGLE NAVIGATION ===");
  Serial.print("Rectangle: ");
  Serial.print(rectLength);
  Serial.print(" x ");
  Serial.print(rectWidth);
  Serial.print(" pulses, ");
  Serial.print(rectDirection);
  Serial.print(", speed: ");
  Serial.println(rectSpeed);
  
  // Setup rectangle sequence: length, turn, width, turn, length, turn, width, turn
  rectCurrentPulses[0] = rectLength;  // First side (length)
  rectCurrentPulses[1] = rectWidth;   // Second side (width) 
  rectCurrentPulses[2] = rectLength;  // Third side (length)
  rectCurrentPulses[3] = rectWidth;   // Fourth side (width)
  
  rectCurrentSegment = 0;
  pulseCount = 0;
  consecutivePulses = 0;
  hallSensorEnabled = true;
  rectangleMode = true;
  
  // Set base PWM values for rectangle movement
  baseLeftPWM = rectSpeed;
  baseRightPWM = rectSpeed;
  
  Serial.println("Starting first segment (length)");
  startRectangleSegment();
}

void startRectangleSegment() {
  if (rectCurrentSegment >= 8) {
    // Rectangle complete
    finishRectangle();
    return;
  }
  
  bool isStraightSegment = (rectCurrentSegment % 2 == 0); // 0,2,4,6 are straight segments
  
  if (isStraightSegment) {
    // Straight segment - reinitialize heading and go straight
    int segmentIndex = rectCurrentSegment / 2; // 0,1,2,3
    rectCurrentTarget = rectCurrentPulses[segmentIndex];
    
    Serial.print("Segment ");
    Serial.print(rectCurrentSegment + 1);
    Serial.print("/8: Going straight for ");
    Serial.print(rectCurrentTarget);
    Serial.println(" pulses");
    
    // Reinitialize heading
    for (int i = 0; i < 10; i++) {
      readSensors();
      delay(20);
    }
    targetHeading = calculateHeading();
    
    // Reset PID
    integral = 0.0;
    prevError = 0.0;
    lastPIDTime = millis();
    
    // Reset pulse counter
    pulseCount = 0;
    hallSensorEnabled = true;
    
    // Start straight mode for this segment
    straightMode = true;
    
    Serial.print("Target heading set to: ");
    Serial.println(targetHeading);
  } else {
    // Turn segment
    Serial.print("Segment ");
    Serial.print(rectCurrentSegment + 1);
    Serial.print("/8: Making 90° turn ");
    
    // Determine turn direction based on rectangle direction setting
    if (rectDirection == "clockwise") {
      turnDirection = "right";
      Serial.println("RIGHT (clockwise)");
    } else {
      turnDirection = "left";
      Serial.println("LEFT (anticlockwise)");
    }
    
    // Start turn mode (this will call continueRectangleAfterTurn when done)
    hallSensorEnabled = false;
    straightMode = false;
    startTurnMode();
  }
}

void updateRectangleMode() {
  // Check for magnet interference during operation
  if (detectMagnetInterference()) {
    Serial.println("CRITICAL: Magnet interference detected during operation!");
    Serial.println("Stopping rectangle navigation for safety.");
    finishRectangle();
    Serial2.println("RECTANGLE_STOPPED_INTERFERENCE");
    return;
  }
  
  // Handle straight segments with pulse counting
  if (straightMode && hallSensorEnabled) {
    // Check if we've reached target pulses
    if (pulseCount >= rectCurrentTarget) {
      Serial.print("Reached target pulses: ");
      Serial.print(pulseCount);
      Serial.print("/");
      Serial.println(rectCurrentTarget);
      
      // Stop straight mode and continue to next segment
      straightMode = false;
      hallSensorEnabled = false;
      stopMotors();
      delay(500); // Brief pause between segments
      
      rectCurrentSegment++;
      startRectangleSegment();
    } else {
      // Continue straight with PID control
      updateStraightMode();
      
      // Debug output
      static unsigned long lastRectDebug = 0;
      if (millis() - lastRectDebug > 1000) {
        Serial.print("Rectangle straight: ");
        Serial.print(pulseCount);
        Serial.print("/");
        Serial.print(rectCurrentTarget);
        Serial.print(" pulses, Heading: ");
        Serial.println(calculateHeading(), 1);
        lastRectDebug = millis();
      }
    }
  }
}

void continueRectangleAfterTurn() {
  Serial.println("Turn completed, continuing rectangle");
  rectCurrentSegment++;
  delay(300); // Brief pause after turn
  startRectangleSegment();
}

void finishRectangle() {
  stopMotors();
  rectangleMode = false;
  hallSensorEnabled = false;
  straightMode = false;
  turningMode = false;
  magnetInterferenceDetected = false; // Reset interference flag
  
  Serial.println("=== RECTANGLE NAVIGATION COMPLETE ===");
  Serial.println("Robot should be back at starting position");
  
  // Blink LED to indicate completion
  for (int i = 0; i < 5; i++) {
    digitalWrite(STATUS_LED, LOW);
    delay(200);
    digitalWrite(STATUS_LED, HIGH);
    delay(200);
  }
}

// =============== MAGNET INTERFERENCE DETECTION ===============
bool detectMagnetInterference() {
  // If too many pulses occur too quickly, it's likely magnet interference
  unsigned long now = millis();
  
  // Check if we got more than 20 pulses in less than 1 second while stationary
  if (consecutivePulses > 20 && (now - lastValidPulseTime < 1000) && 
      (currentLeftSpeed == 0 && currentRightSpeed == 0)) {
    magnetInterferenceDetected = true;
    return true;
  }
  
  // Check for rapid pulse bursts (more than 10 pulses in 100ms)
  static unsigned long rapidPulseCount = 0;
  static unsigned long rapidPulseStartTime = 0;
  
  if (rapidPulseStartTime == 0) {
    rapidPulseStartTime = now;
    rapidPulseCount = 0;
  }
  
  if (now - rapidPulseStartTime < 100) {
    // Still in the 100ms window
    if (rapidPulseCount > 10) {
      magnetInterferenceDetected = true;
      Serial.println("Rapid pulse burst detected - magnet interference");
      return true;
    }
  } else {
    // Reset the rapid pulse detection window
    rapidPulseStartTime = now;
    rapidPulseCount = 0;
  }
  
  return false;
}

// =============== MOTOR CONTROL ===============
void moveMotors(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  
  currentLeftSpeed = leftSpeed;
  currentRightSpeed = rightSpeed;
  
  // Left motor
  if (leftSpeed > 0) {
    analogWrite(LEFT_FORWARD, leftSpeed);
    analogWrite(LEFT_BACKWARD, 0);
  } else if (leftSpeed < 0) {
    analogWrite(LEFT_FORWARD, 0);
    analogWrite(LEFT_BACKWARD, -leftSpeed);
  } else {
    analogWrite(LEFT_FORWARD, 0);
    analogWrite(LEFT_BACKWARD, 0);
  }
  
  // Right motor
  if (rightSpeed > 0) {
    analogWrite(RIGHT_FORWARD, rightSpeed);
    analogWrite(RIGHT_BACKWARD, 0);
  } else if (rightSpeed < 0) {
    analogWrite(RIGHT_FORWARD, 0);
    analogWrite(RIGHT_BACKWARD, -rightSpeed);
  } else {
    analogWrite(RIGHT_FORWARD, 0);
    analogWrite(RIGHT_BACKWARD, 0);
  }
}

void stopMotors() {
  analogWrite(LEFT_FORWARD, 0);
  analogWrite(LEFT_BACKWARD, 0);
  analogWrite(RIGHT_FORWARD, 0);
  analogWrite(RIGHT_BACKWARD, 0);
  
  currentLeftSpeed = 0;
  currentRightSpeed = 0;
  
  // Add small delay to ensure motors actually stop
  delay(50);
  
  Serial.println("Motors stopped");
}

// =============== HALL SENSOR INTERRUPT ===============
void hallSensorISR() {
  // Debounce the sensor
  unsigned long now = millis();
  if (now - lastPulseTime > 50) { // 50ms debounce
    lastPulseTime = now;
    
    // Only count pulses when hall sensor is enabled
    if (hallSensorEnabled) {
      // Check for potential magnet interference
      if (currentLeftSpeed == 0 && currentRightSpeed == 0) {
        // Motors are stopped, but getting pulses - possible magnet interference
        consecutivePulses++;
        if (consecutivePulses == 1) {
          lastValidPulseTime = now;
        }
        
        if (consecutivePulses > 5) {
          Serial.println("WARNING: Multiple pulses detected while motors stopped");
          magnetInterferenceDetected = true;
          return; // Don't count these pulses
        }
      } else {
        // Motors are running - reset consecutive pulse counter
        consecutivePulses = 0;
        lastValidPulseTime = now;
      }
      
      pulseCount++;
      
      // Brief LED flash to indicate pulse detected
      digitalWrite(STATUS_LED, LOW);
      delayMicroseconds(100);
      digitalWrite(STATUS_LED, HIGH);
      
      // Debug output for troubleshooting
      Serial.print("Pulse detected: ");
      Serial.print(pulseCount);
      Serial.print(" (Motors: L=");
      Serial.print(currentLeftSpeed);
      Serial.print(", R=");
      Serial.print(currentRightSpeed);
      Serial.println(")");
    }
  }
}

// =============== I2C UTILITY FUNCTIONS ===============
bool writeByte(uint8_t address, uint8_t reg, uint8_t data) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(data);
  return (Wire.endTransmission() == 0);
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
  if (Wire.endTransmission(false) != 0) {
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