// Arduino Mega - Straight Line PID Navigation with Obstacle Detection
// UART2: Pin 16 (TX2), Pin 17 (RX2) - Connected to ESP32

#include <Wire.h>

// Motor Pins (BTS7960)
#define LEFT_R_PWM 6    // Left motor forward
#define LEFT_L_PWM 5    // Left motor backward
#define RIGHT_R_PWM 10  // Right motor forward
#define RIGHT_L_PWM 9   // Right motor backward

// LED Pin
#define LED_PIN 36

// Hall Sensor
#define HALL_SENSOR_PIN 3

// Ultrasonic Sensor Pins
#define TRIG_PIN 23
#define ECHO_PIN 22

// Sensor Addresses
#define MPU_ADDR 0x68        // MPU6500/MPU6050
#define QMC5883L_ADDR 0x0D   // QMC5883L Magnetometer

// Navigation parameters
#define BASE_SPEED 150
#define MAX_CORRECTION 50
#define OBSTACLE_DISTANCE_CM 15  // Stop if obstacle within 15cm

// PID constants
float Kp = 2.0;
float Ki = 0.1; 
float Kd = 0.5;

// Sensor variables
float gyroZ = 0;
float magX = 0, magY = 0, magZ = 0;
float currentHeading = 0;
float targetHeading = 0;

// Magnetometer calibration (use your values)
float magXOffset = -46.0, magYOffset = -742.5, magZOffset = -207.5;
float magXScale = 1.261, magYScale = 1.000, magZScale = 1.143;

// PID variables
float integral = 0.0;
float prevError = 0.0;
unsigned long lastPIDTime = 0;

// Hall sensor variables
volatile unsigned long pulseCount = 0;
volatile unsigned long lastPulseTime = 0;
unsigned long targetPulses = 0;
bool navigationActive = false;

// Obstacle detection variables
bool obstacleDetected = false;
bool emergencyStop = false;
unsigned long lastObstacleCheck = 0;
unsigned long emergencyLEDBlink = 0;
bool emergencyLEDState = false;

// Auto-resume variables
bool autoResumeEnabled = true;  // Enable auto-resume by default
bool wasNavigatingBeforeStop = false;
unsigned long obstacleDetectedTime = 0;
#define OBSTACLE_CLEAR_DISTANCE 18  // Resume when obstacle is beyond 18cm (adds hysteresis)

// Scaling factors
const float GYRO_SCALE = 131.0;
const float ACCEL_SCALE = 16384.0;

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);
  
  // Setup pins
  pinMode(LEFT_R_PWM, OUTPUT);
  pinMode(LEFT_L_PWM, OUTPUT);
  pinMode(RIGHT_R_PWM, OUTPUT);
  pinMode(RIGHT_L_PWM, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);
  
  // Setup ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  
  digitalWrite(LED_PIN, LOW);
  stopMotors();
  
  // Initialize I2C
  Wire.begin();
  
  Serial.println("=== STRAIGHT LINE PID NAVIGATION WITH OBSTACLE DETECTION ===");
  
  // Initialize I2C
  Wire.begin();
  
  // Initialize sensors (continue even if some fail)
  bool sensorsOK = initSensors();
  if (sensorsOK) {
    Serial.println("All sensors initialized successfully");
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
  } else {
    Serial.println("Some sensors failed - continuing anyway");
    // Blink LED 3 times to indicate sensor issues
    for(int i = 0; i < 3; i++) {
      digitalWrite(LED_PIN, HIGH);
      delay(200);
      digitalWrite(LED_PIN, LOW);
      delay(200);
    }
  }
  
  // Setup hall sensor interrupt
  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hallSensorISR, FALLING);
  
  // Get initial heading
  calibrateHeading();
  
  // Test ultrasonic sensor
  Serial.println("Testing ultrasonic sensor...");
  float testDistance = getDistance();
  Serial.print("Initial distance reading: ");
  Serial.print(testDistance);
  Serial.println(" cm");
  
  Serial.println("System ready. Send START command with pulse count.");
}

void loop() {
  // Read sensors (only if needed)
  static unsigned long lastSensorRead = 0;
  if (millis() - lastSensorRead > 100) { // Read sensors every 100ms instead of every loop
    readSensors();
    lastSensorRead = millis();
  }
  
  // Check for obstacles every 50ms during navigation OR during emergency stop (for auto-resume)
  if ((navigationActive || emergencyStop) && (millis() - lastObstacleCheck > 50)) {
    checkObstacle();
    lastObstacleCheck = millis();
  }
  
  // Handle emergency stop LED blinking
  if (emergencyStop) {
    if (millis() - emergencyLEDBlink > 200) { // Blink every 200ms
      emergencyLEDState = !emergencyLEDState;
      digitalWrite(LED_PIN, emergencyLEDState);
      emergencyLEDBlink = millis();
    }
  }
  
  // Check for commands from ESP32
  if (Serial2.available()) {
    String command = "";
    
    // Read command with timeout protection
    unsigned long timeout = millis() + 1000;
    while (Serial2.available() && millis() < timeout) {
      char c = Serial2.read();
      if (c == '\n' || c == '\r') {
        break;
      }
      if (command.length() < 50) { // Limit command length
        command += c;
      }
      delay(1);
    }
    
    command.trim();
    
    if (command.length() > 0) {
      Serial.println("Received: [" + command + "]");
      
      // Process commands
      if (command.startsWith("START_")) {
        // Extract pulse count from command (e.g., "START_20")
        int pulseValue = command.substring(6).toInt();
        if (pulseValue > 0 && pulseValue < 1000) { // Limit range
          startNavigation(pulseValue);
        } else {
          Serial2.println("INVALID_PULSE_COUNT");
        }
      }
      else if (command == "STOP") {
        stopNavigation();
      }
      else if (command == "EMERGENCY_CLEAR") {
        clearEmergencyStop();
      }
      else if (command == "AUTO_RESUME_ON") {
        autoResumeEnabled = true;
        Serial.println("Auto-resume enabled");
        Serial2.println("AUTO_RESUME_ON_OK");
      }
      else if (command == "AUTO_RESUME_OFF") {
        autoResumeEnabled = false;
        Serial.println("Auto-resume disabled");
        Serial2.println("AUTO_RESUME_OFF_OK");
      }
      else if (command == "STATUS") {
        sendStatus();
      }
      else if (command == "LED_ON") {
        if (!emergencyStop) { // Don't override emergency LED
          digitalWrite(LED_PIN, HIGH);
        }
        Serial2.println("LED_ON_OK");
      }
      else if (command == "LED_OFF") {
        if (!emergencyStop) { // Don't override emergency LED
          digitalWrite(LED_PIN, LOW);
        }
        Serial2.println("LED_OFF_OK");
      }
      else if (command == "TEST_MOTORS") {
        testMotors();
        Serial2.println("MOTOR_TEST_COMPLETE");
      }
      else if (command == "RESET") {
        // Software reset
        Serial.println("*** RESETTING SYSTEM ***");
        Serial2.println("RESETTING");
        delay(100);
        asm volatile ("  jmp 0");  // Software reset
      }
      else if (command.startsWith("PID_")) {
        // Handle PID tuning: PID_P2.5_I0.2_D0.8
        updatePIDValues(command);
      }
      else if (command == "GET_PID") {
        sendPIDValues();
      }
      else if (command == "PING") {
        Serial.println("PING received");
        Serial2.println("PONG");
      }
      else if (command == "MANUAL_BACKWARD") {
        if (!emergencyStop) {
          manualBackward();
          Serial2.println("MANUAL_BACKWARD_OK");
        } else {
          Serial2.println("EMERGENCY_STOP_ACTIVE");
        }
      }
      else if (command == "MANUAL_LEFT") {
        if (!emergencyStop) {
          manualTurnLeft();
          Serial2.println("MANUAL_LEFT_OK");
        } else {
          Serial2.println("EMERGENCY_STOP_ACTIVE");
        }
      }
      else if (command == "MANUAL_RIGHT") {
        if (!emergencyStop) {
          manualTurnRight();
          Serial2.println("MANUAL_RIGHT_OK");
        } else {
          Serial2.println("EMERGENCY_STOP_ACTIVE");
        }
      }
      else if (command == "MANUAL_STOP") {
        stopMotors();
        Serial2.println("MANUAL_STOP_OK");
      }
      else if (command == "GET_DISTANCE") {
        float distance = getDistance();
        Serial2.print("DISTANCE:");
        Serial2.println(distance);
      }
      else {
        Serial2.println("UNKNOWN_COMMAND");
      }
      
      // Clear any remaining data in serial buffer
      while (Serial2.available()) {
        Serial2.read();
      }
    }
  }
  
  // Navigation logic (simplified to prevent crashes)
  if (navigationActive && !emergencyStop) {
    // Check if target reached
    if (pulseCount >= targetPulses) {
      stopNavigation();
      Serial.println("Target reached!");
      Serial2.println("TARGET_REACHED");
    } else {
      // Simple PID control with error checking
      float correction = computePID();
      if (!isnan(correction) && !isinf(correction)) { // Check for invalid values
        driveStraight(correction);
      } else {
        // If PID fails, drive straight without correction
        driveStraight(0);
      }
      
      // Debug info every 1 second (reduced frequency)
      static unsigned long lastDebug = 0;
      if (millis() - lastDebug > 1000) {
        Serial.print("Pulses: ");
        Serial.print(pulseCount);
        Serial.print("/");
        Serial.print(targetPulses);
        Serial.print(" Heading: ");
        Serial.print(currentHeading);
        Serial.print(" Distance: ");
        Serial.print(getDistance());
        Serial.println(" cm");
        lastDebug = millis();
      }
    }
  }
  
  delay(20); // Increased delay to reduce CPU load
}

// NEW FUNCTION: Get distance from ultrasonic sensor
float getDistance() {
  // Send trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Read echo pulse
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout
  
  // Calculate distance in cm
  if (duration == 0) {
    return 999; // No echo received, assume no obstacle
  }
  
  float distance = duration * 0.034 / 2;
  
  // Limit reasonable range
  if (distance > 400) distance = 400;
  if (distance < 2) distance = 2;
  
  return distance;
}

// ENHANCED FUNCTION: Check for obstacles with auto-resume
void checkObstacle() {
  float distance = getDistance();
  
  // Check for obstacle detection (within 15cm)
  if (distance <= OBSTACLE_DISTANCE_CM && distance > 0) {
    if (!obstacleDetected) {
      obstacleDetected = true;
      wasNavigatingBeforeStop = navigationActive;  // Remember if we were navigating
      emergencyStop = true;
      stopMotors();
      obstacleDetectedTime = millis();
      
      Serial.print("OBSTACLE DETECTED at ");
      Serial.print(distance);
      Serial.println(" cm - EMERGENCY STOP!");
      
      if (wasNavigatingBeforeStop) {
        Serial.print("Navigation paused at pulse ");
        Serial.print(pulseCount);
        Serial.print(" of ");
        Serial.println(targetPulses);
      }
      
      Serial2.println("EMERGENCY_STOP");
    }
  } 
  // Check for obstacle removal (beyond 18cm - hysteresis to prevent flickering)
  else if (distance > OBSTACLE_CLEAR_DISTANCE || distance == 999) {
    if (obstacleDetected && emergencyStop) {
      obstacleDetected = false;
      
      // Auto-resume navigation if it was active before and auto-resume is enabled
      if (wasNavigatingBeforeStop && autoResumeEnabled && pulseCount < targetPulses) {
        emergencyStop = false;
        digitalWrite(LED_PIN, LOW);
        emergencyLEDState = false;
        
        Serial.print("OBSTACLE CLEARED at ");
        Serial.print(distance);
        Serial.println(" cm - AUTO RESUMING NAVIGATION!");
        Serial.print("Continuing from pulse ");
        Serial.print(pulseCount);
        Serial.print(" to ");
        Serial.println(targetPulses);
        
        Serial2.println("AUTO_RESUMED");
      } else {
        // Just clear emergency but don't resume
        emergencyStop = false;
        digitalWrite(LED_PIN, LOW);
        emergencyLEDState = false;
        wasNavigatingBeforeStop = false;
        
        Serial.print("OBSTACLE CLEARED at ");
        Serial.print(distance);
        Serial.println(" cm - Emergency cleared but navigation not resumed");
        
        Serial2.println("EMERGENCY_CLEARED");
      }
    }
  }
}

// ENHANCED FUNCTION: Clear emergency stop
void clearEmergencyStop() {
  emergencyStop = false;
  obstacleDetected = false;
  wasNavigatingBeforeStop = false;  // Reset navigation memory
  digitalWrite(LED_PIN, LOW);
  emergencyLEDState = false;
  
  Serial.println("Emergency stop manually cleared");
  Serial2.println("EMERGENCY_CLEARED");
}

bool initSensors() {
  Serial.println("Initializing sensors...");
  
  bool mpuOK = false;
  bool magOK = false;
  
  // Try to initialize MPU6500/MPU6050
  Serial.print("MPU6500/6050: ");
  if (initMPU6050()) {
    Serial.println("OK");
    mpuOK = true;
  } else {
    Serial.println("FAILED");
  }
  
  // Try to initialize QMC5883L
  Serial.print("QMC5883L: ");
  if (initQMC5883L()) {
    Serial.println("OK");
    magOK = true;
  } else {
    Serial.println("FAILED");
  }
  
  // At least one sensor must work for basic operation
  if (!mpuOK && !magOK) {
    Serial.println("ERROR: No sensors working!");
    return false;
  }
  
  if (!mpuOK) {
    Serial.println("WARNING: Gyro not available - using magnetometer only");
  }
  
  if (!magOK) {
    Serial.println("WARNING: Magnetometer not available - using gyro integration");
  }
  
  return true;
}

bool initMPU6050() {
  // Check if device responds
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) {
    return false;
  }
  
  // Check WHO_AM_I register
  uint8_t whoAmI = readByte(MPU_ADDR, 0x75);
  if (whoAmI != 0x68 && whoAmI != 0x70) { // 0x68 for MPU6050, 0x70 for MPU6500
    Serial.print("Unexpected WHO_AM_I: 0x");
    Serial.println(whoAmI, HEX);
    return false;
  }
  
  // Reset device
  writeByte(MPU_ADDR, 0x6B, 0x80);
  delay(100);
  
  // Wake up device
  writeByte(MPU_ADDR, 0x6B, 0x01);
  delay(100);
  
  // Configure gyro (±250 deg/s)
  writeByte(MPU_ADDR, 0x1B, 0x00);
  
  return true;
}

bool initQMC5883L() {
  // Check if device responds
  Wire.beginTransmission(QMC5883L_ADDR);
  if (Wire.endTransmission() != 0) {
    return false;
  }
  
  // Reset device
  writeByte(QMC5883L_ADDR, 0x0B, 0x01);
  delay(100);
  
  // Configure device (8 samples avg, 200Hz, ±8 Gauss, continuous mode)
  writeByte(QMC5883L_ADDR, 0x09, 0x1D);
  delay(50);
  
  // Verify configuration
  uint8_t config = readByte(QMC5883L_ADDR, 0x09);
  if (config != 0x1D) {
    Serial.print("QMC5883L config failed, got: 0x");
    Serial.println(config, HEX);
    return false;
  }
  
  return true;
}

void readSensors() {
  // Read gyroscope with error checking
  int16_t gyroRawZ = readInt16(MPU_ADDR, 0x47);
  if (gyroRawZ != 0 || readByte(MPU_ADDR, 0x75) == 0x68 || readByte(MPU_ADDR, 0x75) == 0x70) {
    gyroZ = gyroRawZ / GYRO_SCALE;
  }
  
  // Read magnetometer with error checking
  uint8_t magStatus = readByte(QMC5883L_ADDR, 0x06);
  if (magStatus & 0x01) { // Data ready
    int16_t magRawX = readInt16(QMC5883L_ADDR, 0x00);
    int16_t magRawY = readInt16(QMC5883L_ADDR, 0x02);
    int16_t magRawZ = readInt16(QMC5883L_ADDR, 0x04);
    
    // Apply calibration only if values are reasonable
    if (magRawX != 0 || magRawY != 0) {
      magX = (magRawX - magXOffset) * magXScale;
      magY = (magRawY - magYOffset) * magYScale;
      magZ = (magRawZ - magZOffset) * magZScale;
      
      // Calculate heading with bounds checking
      float heading = atan2(magY, magX) * 180.0 / PI;
      if (heading < 0) heading += 360;
      
      // Only update if heading is reasonable (not NaN or infinity)
      if (!isnan(heading) && !isinf(heading) && heading >= 0 && heading < 360) {
        currentHeading = heading;
      }
    }
  }
}

void calibrateHeading() {
  Serial.println("Calibrating heading... Keep robot still for 3 seconds");
  
  float headingSum = 0;
  int samples = 0;
  
  for (int i = 0; i < 30; i++) {
    readSensors();
    headingSum += currentHeading;
    samples++;
    delay(100);
  }
  
  targetHeading = headingSum / samples;
  Serial.print("Target heading set to: ");
  Serial.println(targetHeading);
}

void startNavigation(int pulses) {
  // Don't start if emergency stop is active
  if (emergencyStop) {
    Serial.println("Cannot start - Emergency stop active!");
    Serial2.println("EMERGENCY_STOP_ACTIVE");
    return;
  }
  
  pulseCount = 0;
  targetPulses = pulses;
  navigationActive = true;
  integral = 0;
  prevError = 0;
  
  Serial.print("Starting navigation for ");
  Serial.print(pulses);
  Serial.println(" pulses");
  Serial2.println("NAVIGATION_STARTED");
}

void stopNavigation() {
  navigationActive = false;
  stopMotors();
  Serial.println("Navigation stopped");
  Serial2.println("NAVIGATION_STOPPED");
}

float computePID() {
  unsigned long now = millis();
  float dt = (now - lastPIDTime) / 1000.0;
  if (dt < 0.01) dt = 0.01;
  
  // Calculate error
  float error = targetHeading - currentHeading;
  
  // Normalize error to -180 to +180 degrees
  while (error > 180) error -= 360;
  while (error < -180) error += 360;
  
  // PID calculation
  integral += error * dt;
  if (integral > 20) integral = 20;
  if (integral < -20) integral = -20;
  
  float derivative = (error - prevError) / dt;
  float output = Kp * error + Ki * integral + Kd * derivative;
  
  // Limit output
  if (output > MAX_CORRECTION) output = MAX_CORRECTION;
  if (output < -MAX_CORRECTION) output = -MAX_CORRECTION;
  
  prevError = error;
  lastPIDTime = now;
  
  return output;
}

void driveStraight(float correction) {
  // Don't drive if emergency stop is active
  if (emergencyStop) {
    stopMotors();
    return;
  }
  
  int leftSpeed = BASE_SPEED - correction;
  int rightSpeed = BASE_SPEED + correction;
  
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
  // Apply to motors
  analogWrite(LEFT_R_PWM, leftSpeed);
  analogWrite(LEFT_L_PWM, 0);
  analogWrite(RIGHT_R_PWM, rightSpeed);
  analogWrite(RIGHT_L_PWM, 0);
}

void stopMotors() {
  analogWrite(LEFT_R_PWM, 0);
  analogWrite(LEFT_L_PWM, 0);
  analogWrite(RIGHT_R_PWM, 0);
  analogWrite(RIGHT_L_PWM, 0);
}

void sendStatus() {
  Serial2.print("STATUS:");
  Serial2.print(pulseCount);
  Serial2.print("/");
  Serial2.print(targetPulses);
  Serial2.print(",");
  Serial2.print(currentHeading);
  Serial2.print(",");
  Serial2.print(navigationActive ? "ACTIVE" : "STOPPED");
  Serial2.print(",");
  Serial2.print(emergencyStop ? "EMERGENCY" : "NORMAL");
  Serial2.print(",");
  Serial2.print(getDistance());
  Serial2.print("cm,");
  Serial2.print(autoResumeEnabled ? "AUTO_RESUME_ON" : "AUTO_RESUME_OFF");
  Serial2.println("");
}

void hallSensorISR() {
  unsigned long now = millis();
  if (navigationActive && (now - lastPulseTime > 50) && !emergencyStop) {
    pulseCount++;
    lastPulseTime = now;
    if (!emergencyStop) { // Don't blink during emergency
      digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink LED
    }
  }
}

// Helper functions
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

void testMotors() {
  if (emergencyStop) {
    Serial.println("Cannot test motors - Emergency stop active!");
    return;
  }
  
  Serial.println("--- MOTOR TEST WITHOUT SENSORS ---");
  
  Serial.println("Forward 2 sec");
  analogWrite(LEFT_R_PWM, 150);
  analogWrite(RIGHT_R_PWM, 150);
  analogWrite(LEFT_L_PWM, 0);
  analogWrite(RIGHT_L_PWM, 0);
  delay(2000);
  stopMotors();
  delay(1000);
  
  Serial.println("Left turn 2 sec");
  analogWrite(LEFT_R_PWM, 0);
  analogWrite(RIGHT_R_PWM, 120);
  analogWrite(LEFT_L_PWM, 120);
  analogWrite(RIGHT_L_PWM, 0);
  delay(2000);
  stopMotors();
  
  Serial.println("Motor test complete");
}

void updatePIDValues(String command) {
  // Parse command like: PID_P2.5_I0.2_D0.8
  Serial.println("Updating PID: " + command);
  
  // Simple parsing to avoid crashes
  float newKp = Kp; // Keep current values as default
  float newKi = Ki;
  float newKd = Kd;
  
  // Find P value
  int pPos = command.indexOf("P");
  if (pPos >= 0) {
    int pEnd = command.indexOf("_", pPos + 1);
    if (pEnd > pPos) {
      String pStr = command.substring(pPos + 1, pEnd);
      float val = pStr.toFloat();
      if (val >= 0 && val <= 10) newKp = val;
    }
  }
  
  // Find I value
  int iPos = command.indexOf("I");
  if (iPos >= 0) {
    int iEnd = command.indexOf("_", iPos + 1);
    if (iEnd > iPos) {
      String iStr = command.substring(iPos + 1, iEnd);
      float val = iStr.toFloat();
      if (val >= 0 && val <= 5) newKi = val;
    }
  }
  
  // Find D value
  int dPos = command.indexOf("D");
  if (dPos >= 0) {
    String dStr = command.substring(dPos + 1);
    float val = dStr.toFloat();
    if (val >= 0 && val <= 5) newKd = val;
  }
  
  // Update values
  Kp = newKp;
  Ki = newKi;
  Kd = newKd;
  
  // Reset PID
  integral = 0.0;
  prevError = 0.0;
  
  Serial.print("PID: Kp=");
  Serial.print(Kp);
  Serial.print(" Ki=");
  Serial.print(Ki);
  Serial.print(" Kd=");
  Serial.println(Kd);
  
  Serial2.println("PID_OK");
}

void sendPIDValues() {
  Serial2.print("PID_P");
  Serial2.print(Kp);
  Serial2.print("_I");
  Serial2.print(Ki);
  Serial2.print("_D");
  Serial2.print(Kd);
  Serial2.println("_END");
}

// Manual control functions (no sensors, no PID)
void manualBackward() {
  if (emergencyStop) {
    Serial.println("Cannot move - Emergency stop active!");
    return;
  }
  
  Serial.println("Manual: Backward");
  analogWrite(LEFT_R_PWM, 0);     // Left forward OFF
  analogWrite(LEFT_L_PWM, 150);   // Left backward
  analogWrite(RIGHT_R_PWM, 0);    // Right forward OFF  
  analogWrite(RIGHT_L_PWM, 150);  // Right backward
}

void manualTurnLeft() {
  if (emergencyStop) {
    Serial.println("Cannot move - Emergency stop active!");
    return;
  }
  
  Serial.println("Manual: Turn Left");
  analogWrite(LEFT_R_PWM, 0);     // Left forward OFF
  analogWrite(LEFT_L_PWM, 120);   // Left backward
  analogWrite(RIGHT_R_PWM, 120);  // Right forward
  analogWrite(RIGHT_L_PWM, 0);    // Right backward OFF
}

void manualTurnRight() {
  if (emergencyStop) {
    Serial.println("Cannot move - Emergency stop active!");
    return;
  }
  
  Serial.println("Manual: Turn Right");
  analogWrite(LEFT_R_PWM, 120);   // Left forward
  analogWrite(LEFT_L_PWM, 0);     // Left backward OFF
  analogWrite(RIGHT_R_PWM, 0);    // Right forward OFF
  analogWrite(RIGHT_L_PWM, 120);  // Right backward
}

int16_t readInt16(uint8_t address, uint8_t reg) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(address, (uint8_t)2);
  
  if (Wire.available() >= 2) {
    uint8_t low = Wire.read();
    uint8_t high = Wire.read();
    return (int16_t)((high << 8) | low);
  }
  return 0;
}