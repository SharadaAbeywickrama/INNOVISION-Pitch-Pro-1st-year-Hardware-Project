// Arduino Mega - Complete Control with All Features Integrated
// Features: Motor Control, Sensors, PID, Paint System, Line Width Control, Tank Monitoring,Obstacle detection

#include <Wire.h>
#include <Servo.h>

// =============== PIN DEFINITIONS ===============
// Motor Pins
#define LEFT_FORWARD 10
#define LEFT_BACKWARD 9
#define RIGHT_FORWARD 5
#define RIGHT_BACKWARD 6

// Status LED
#define STATUS_LED 36

// Hall Effect Sensor
#define HALL_SENSOR_PIN 3

// Paint System Pins
#define PAINT_RELAY_PIN 26    // Relay for paint pump
#define MIXER_IN3 45          // L298N IN3 for paint mixer
#define MIXER_IN4 43          // L298N IN4 for paint mixer  
#define MIXER_ENB 13          // L298N ENB (PWM) for paint mixer

// Line Width Control Pins
#define SERVO1_PIN 11         // First servo for linear actuator
#define SERVO2_PIN 12         // Second servo for linear actuator

// Paint Tank Monitoring Pins
#define ULTRASONIC_TRIG 27     // Ultrasonic trigger pin
#define ULTRASONIC_ECHO 22    // Ultrasonic echo pin
#define RED_LED_PIN 39        // Red LED for low paint level
#define YELLOW_LED_PIN 33     // Yellow LED for medium paint level  
#define GREEN_LED_PIN 35     // Green LED for high paint level
#define BUZZER_PIN 40          // Buzzer for alerts

// Obstacle Detection Pins
#define ULTRASONIC_TRIG1 24      // First ultrasonic trigger
#define ULTRASONIC_ECHO1 25      // First ultrasonic echo
#define ULTRASONIC_TRIG2 30      // Second ultrasonic trigger
#define ULTRASONIC_ECHO2 31      // Second ultrasonic echo



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
float lastTurnDegrees = 0.0;
unsigned long turnStartTime = 0;
bool turnFirstAttempt = true;
float turnTargetHeading = 0.0;
unsigned long lastSensorReadTime = 0;
float headingHistory[5] = {0, 0, 0, 0, 0};
int headingHistoryIndex = 0;
bool turnForceStop = false;
unsigned long correctionStartTime = 0;
float correctionErrorHistory[10];
int correctionErrorIndex = 0;
bool correctionSettled = false;
int correctionStableCount = 0;
unsigned long lastCorrectionMove = 0;
float deadbandZone = 2.0;
bool inDeadband = false;
int consecutiveDeadbandReadings = 0;

// Intelligent correction variables
bool correctionApplied = false;
bool verificationDone = false;
unsigned long correctionCompleteTime = 0;
unsigned long verificationCompleteTime = 0;
int correctionAttempts = 0;

// Rectangle navigation variables
int rectLength = 4;
int rectWidth = 2;
String rectDirection = "clockwise";
int rectSpeed = 120;
int rectCurrentSegment = 0;
int rectCurrentPulses[4] = {4, 0, 2, 0};
int rectCurrentTarget = 0;

// Hall sensor variables
volatile unsigned long pulseCount = 0;
volatile unsigned long lastPulseTime = 0;
volatile unsigned long consecutivePulses = 0;
volatile unsigned long lastValidPulseTime = 0;
bool hallSensorEnabled = false;
bool magnetInterferenceDetected = false;

// Paint System Variables
bool paintSystemEnabled = false;
bool mixerSystemEnabled = false;
bool paintWithNavigation = false;
unsigned long lastMixerUpdate = 0;
int mixerState = 0; // 0=stopped, 1=ramping up, 2=high speed, 3=ramping down
int currentMixerSpeed = 0;
unsigned long mixerStateStartTime = 0;
bool mixerCycleActive = false;
unsigned long nextMixerCycle = 0;

// Simple mixer control variables
bool simpleMixerEnabled = false;
unsigned long lastMixerToggle = 0;
bool mixerCurrentlyRunning = false;
const int MIXER_SPEED = 150;  // Fixed PWM speed
const unsigned long MIX_ON_TIME = 3000;   // 3 seconds ON
const unsigned long MIX_OFF_TIME = 2000;  // 2 seconds OFF


// Line Width Control Variables
Servo servo1, servo2;
float K1 = 50.0;  // Calibration constant for servo1
float K2 = 50.0;  // Calibration constant for servo2
float currentLineWidthInches = 2.0; // Default 2 inches

// Paint Tank Monitoring Variables
unsigned long lastTankCheck = 0;
const unsigned long TANK_CHECK_INTERVAL = 10000; // 30 seconds
int currentTankLevel = 1; // 0=empty, 1=low(red), 2=medium(yellow), 3=high(green)
int previousTankLevel = 1;
bool tankEmptyAlarm = false;
unsigned long buzzerLastToggle = 0;
bool buzzerState = false;

// // Tank level definitions (in cm from sensor)
// const int TANK_TOP_DISTANCE = 6;    // Distance from sensor to tank top
// const int TANK_DEPTH = 18;           // Total tank depth
// const int GREEN_MIN = 6, GREEN_MAX = 12;      // High level (20-26cm)
// const int YELLOW_MIN = 13, YELLOW_MAX = 19;    // Medium level (27-32cm)  
// const int RED_MIN = 19, RED_MAX = 25;          // Low level (33-38cm)
// const int EMPTY_THRESHOLD = 22;               // Critical empty level

// Obstacle Detection Variables
bool obstacleDetected = false;
bool previousObstacleState = false;
float obstacleDistance1 = 999.0;
float obstacleDistance2 = 999.0;
float minObstacleDistance = 999.0;
const float OBSTACLE_DETECTION_RANGE = 20.0; // 20cm detection range
unsigned long lastObstacleCheck = 0;
const unsigned long OBSTACLE_CHECK_INTERVAL = 200; // Check every 200ms
unsigned long obstacleDetectedTime = 0;
bool obstacleAlarmActive = false;
unsigned long lastObstacleBuzzer = 0;
bool obstacleBuzzerState = false;

// Navigation State Preservation
bool navigationPaused = false;
bool wasInStraightMode = false;
bool wasInRectangleMode = false;
bool wasInTurningMode = false;
bool wasHallSensorEnabled = false;
unsigned long pausedPulseCount = 0;
int pausedRectCurrentSegment = 0;
int pausedRectCurrentTarget = 0;
float pausedTargetHeading = 0.0;
String pausedTurnDirection = "";
bool wasPaintWithNavigation = false;

const int PULSE_MULTIPLIER = 4;  // 4x multiplier

// =============== SETUP FUNCTION ===============
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  
  // Initialize motor pins
  pinMode(LEFT_FORWARD, OUTPUT);
  pinMode(LEFT_BACKWARD, OUTPUT);
  pinMode(RIGHT_FORWARD, OUTPUT);
  pinMode(RIGHT_BACKWARD, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);
  
  // Setup hall sensor interrupt
  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hallSensorISR, FALLING);
  
  // Initialize paint system
  setupPaintSystem();
  
  // Initialize line width control and tank monitoring
  setupLineWidthAndTankMonitoring();

  setupObstacleDetection();
  
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
  Serial.println("Complete Robot Controller Ready");
  Serial2.println("READY");
}


// =============== MAIN LOOP ===============
void loop() {
  // Handle serial commands
  if (Serial2.available()) {
    String cmd = Serial2.readStringUntil('\n');
    cmd.trim();
    Serial.println("Received: " + cmd);
    handleCommandWithPaintAndWidth(cmd);
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
    updateRectangleModeWithPaint();
  }
  
  // Handle pulse navigation
  if (hallSensorEnabled && rectCurrentTarget > 0 && !rectangleMode) {
    updatePulseNavigation();
  }
  
  // Update paint system
 updateSimpleMixer();
  
  // Update line width control and tank monitoring
  updateLineWidthAndTankMonitoring();
  
  updateObstacleDetection();


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

// =============== PAINT SYSTEM FUNCTIONS ===============
void setupPaintSystem() {
  pinMode(PAINT_RELAY_PIN, OUTPUT);
  pinMode(MIXER_IN3, OUTPUT);
  pinMode(MIXER_IN4, OUTPUT);
  pinMode(MIXER_ENB, OUTPUT);
  
  // Initialize paint system to OFF
  digitalWrite(PAINT_RELAY_PIN, LOW);
  digitalWrite(MIXER_IN3, LOW);
  digitalWrite(MIXER_IN4, LOW);
  analogWrite(MIXER_ENB, 0);
  
  paintSystemEnabled = false;
  mixerSystemEnabled = false;
  
  Serial.println("Paint system initialized");
}
// Stop mixing
void stopSimpleMixer() {
  simpleMixerEnabled = false;
  mixerCurrentlyRunning = false;
  
  // Stop mixer immediately
  digitalWrite(MIXER_IN3, LOW);
  digitalWrite(MIXER_IN4, LOW);
  analogWrite(MIXER_ENB, 0);
  
  Serial.println("Simple mixer STOPPED");
}
// Start mixing manually or during painting
void startSimpleMixer() {
  simpleMixerEnabled = true;
  lastMixerToggle = millis();
  mixerCurrentlyRunning = true;
  
  // Start mixer immediately
  digitalWrite(MIXER_IN3, HIGH);
  digitalWrite(MIXER_IN4, LOW);
  analogWrite(MIXER_ENB, MIXER_SPEED);
  
  Serial.println("Simple mixer STARTED");
}

// Update mixer (call this in main loop)
void updateSimpleMixer() {
  if (!simpleMixerEnabled) return;
  
  unsigned long currentTime = millis();
  unsigned long elapsed = currentTime - lastMixerToggle;
  
  if (mixerCurrentlyRunning) {
    // Mixer is ON - check if time to turn OFF
    if (elapsed >= MIX_ON_TIME) {
      // Turn OFF mixer
      digitalWrite(MIXER_IN3, LOW);
      digitalWrite(MIXER_IN4, LOW);
      analogWrite(MIXER_ENB, 0);
      mixerCurrentlyRunning = false;
      lastMixerToggle = currentTime;
      Serial.println("Mixer: OFF (3sec completed)");
    }
  } else {
    // Mixer is OFF - check if time to turn ON
    if (elapsed >= MIX_OFF_TIME) {
      // Turn ON mixer
      digitalWrite(MIXER_IN3, HIGH);
      digitalWrite(MIXER_IN4, LOW);
      analogWrite(MIXER_ENB, MIXER_SPEED);
      mixerCurrentlyRunning = true;
      lastMixerToggle = currentTime;
      Serial.println("Mixer: ON (2sec wait completed)");
    }
  }
}

// Handle simple mixer commands
void handleSimpleMixerCommands(String cmd) {
  if (cmd.startsWith("MIX:")) {
    String action = cmd.substring(4);
    if (action == "start") {
      startSimpleMixer();
      Serial2.println("SIMPLE_MIX_STARTED");
    } else if (action == "stop") {
      stopSimpleMixer();
      Serial2.println("SIMPLE_MIX_STOPPED");
    }
  }
}
void updatePaintSystem() {
  // Handle automatic mixer cycling when paint navigation is active
  if (paintWithNavigation && mixerSystemEnabled) {
    handleMixerCycling();
  }
}

void startPaintPump() {
  digitalWrite(PAINT_RELAY_PIN, HIGH);
  paintSystemEnabled = true;
  Serial.println("Paint pump started");
}

void stopPaintPump() {
  digitalWrite(PAINT_RELAY_PIN, LOW);
  paintSystemEnabled = false;
  Serial.println("Paint pump stopped");
}

void startMixerSystem() {
  mixerSystemEnabled = true;
  if (paintWithNavigation) {
    // Start automatic cycling when in navigation mode
    nextMixerCycle = millis() + 5000; // First cycle in 5 seconds
    Serial.println("Mixer system enabled - automatic cycling");
  } else {
    // Manual mode - start mixing immediately
    startMixerCycle();
    Serial.println("Mixer started manually");
  }
}

void stopMixerSystem() {
  mixerSystemEnabled = false;
  mixerCycleActive = false;
  stopMixerMotor();
  Serial.println("Mixer system stopped");
}

void startMixerCycle() {
  if (!mixerSystemEnabled) return;
  
  mixerCycleActive = true;
  mixerState = 1; // Start ramping up
  currentMixerSpeed = 0;
  mixerStateStartTime = millis();
  
  digitalWrite(MIXER_IN3, HIGH);
  digitalWrite(MIXER_IN4, LOW);
  Serial.println("Mixer cycle started");
}

void stopMixerMotor() {
  digitalWrite(MIXER_IN3, LOW);
  digitalWrite(MIXER_IN4, LOW);
  analogWrite(MIXER_ENB, 0);
  currentMixerSpeed = 0;
  mixerState = 0;
  mixerCycleActive = false;
}

void handleMixerCycling() {
  if (!mixerSystemEnabled) return;
  
  unsigned long currentTime = millis();
  
  // Simple 5 second on, 5 second off cycle
  static unsigned long lastCycleChange = 0;
  static bool mixerRunning = false;
  
  if (currentTime - lastCycleChange >= 5000) {  // 5 second intervals
    mixerRunning = !mixerRunning;
    lastCycleChange = currentTime;
    
    if (mixerRunning) {
      digitalWrite(MIXER_IN3, HIGH);
      digitalWrite(MIXER_IN4, LOW);
      analogWrite(MIXER_ENB,200);  // Fixed speed
      Serial.println("Mixer: ON");
    } else {
      digitalWrite(MIXER_IN3, LOW);
      digitalWrite(MIXER_IN4, LOW);
      analogWrite(MIXER_ENB, 0);
      Serial.println("Mixer: OFF");
    }
  }
}

void handlePaintCommands(String cmd) {
  if (cmd.startsWith("PAINT:")) {
    String action = cmd.substring(6);
    if (action == "start") {
      startPaintPump();
      Serial2.println("PAINT_STARTED");
    } else if (action == "stop") {
      stopPaintPump();
      Serial2.println("PAINT_STOPPED");
    }
  }
  else {
    handleSimpleMixerCommands(cmd);  // Use simple mixer
  }
}

// =============== LINE WIDTH & TANK MONITORING ===============
void setupLineWidthAndTankMonitoring() {
  // Line Width Control Setup
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  
  // Initialize servos to neutral position (90 degrees)
  servo1.write(90);
  servo2.write(90);
  delay(500);
  
  // Set default line width using new logic
  setLineWidthServos(2.0);
  // Paint Tank Monitoring Setup
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  // Initialize LEDs (start with green - assuming full tank)
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, HIGH);
  digitalWrite(BUZZER_PIN, LOW);
  
  Serial.println("Line width control and tank monitoring initialized");
}




void setupObstacleDetection() {
  // Initialize obstacle detection pins
  pinMode(ULTRASONIC_TRIG1, OUTPUT);
  pinMode(ULTRASONIC_ECHO1, INPUT);
  pinMode(ULTRASONIC_TRIG2, OUTPUT);
  pinMode(ULTRASONIC_ECHO2, INPUT);
  
  // Initialize to no obstacle
  obstacleDetected = false;
  navigationPaused = false;
  
  Serial.println("Obstacle detection system initialized");
  Serial.print("Detection range: ");
  Serial.print(OBSTACLE_DETECTION_RANGE);
  Serial.println(" cm");
}

void updateLineWidthAndTankMonitoring() {
  // Check paint tank level every 30 seconds (optimized for low CPU load)
  unsigned long currentTime = millis();
  if (currentTime - lastTankCheck >= TANK_CHECK_INTERVAL) {
    checkPaintTankLevel();
    lastTankCheck = currentTime;
  }
  
  // Handle buzzer for empty tank alarm (lightweight check)
  if (tankEmptyAlarm && (currentTime - buzzerLastToggle > 500)) {
    buzzerState = !buzzerState;
    digitalWrite(BUZZER_PIN, buzzerState);
    buzzerLastToggle = currentTime;
  }
}

void updateObstacleDetection() {
  unsigned long currentTime = millis();
  
  // Check obstacles every 200ms (optimized for performance)
  if (currentTime - lastObstacleCheck >= OBSTACLE_CHECK_INTERVAL) {
    checkForObstacles();
    lastObstacleCheck = currentTime;
  }
  
  // Handle obstacle alarm (LED blinking and buzzer)
  if (obstacleAlarmActive && (currentTime - lastObstacleBuzzer > 300)) {
    obstacleBuzzerState = !obstacleBuzzerState;
    digitalWrite(STATUS_LED, obstacleBuzzerState ? LOW : HIGH); // Blink status LED
    digitalWrite(BUZZER_PIN, obstacleBuzzerState ? HIGH : LOW); // Buzz
    lastObstacleBuzzer = currentTime;
  }
}

// Map function for floats (from separate project)
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setLineWidthServos(float widthInches) {
  // Constants from separate project
  const float inputMin = 1.5;  // Changed to match your robot's range
  const float inputMax = 4.0;
  const int neutralPos = 90;
  const float wheelRadius = 0.5; // inches
  const float desiredDisplacement = 2.0; // inches per motor
  const float angleFromNeutral = (desiredDisplacement / wheelRadius) * (180.0 / 3.14159265);
  
  // Validate input range
  if (widthInches < inputMin || widthInches > inputMax) {
    Serial.println("ERROR: Line width out of range (1.5-4.0 inches)");
    return;
  }
  
  // Calculate movement using the mapping logic from separate project
  float movement = mapFloat(widthInches, inputMin, inputMax, 0.0, angleFromNeutral);
  
  // Calculate servo angles (both servos move the same way)
  float servo1Angle = neutralPos - movement;
  float servo2Angle = neutralPos - movement;
  
  // Constrain to servo limits
  servo1Angle = constrain(servo1Angle, 0, 180);
  servo2Angle = constrain(servo2Angle, 0, 180);
  
  // Move servos to new position
  servo1.write(servo1Angle);
  servo2.write(servo2Angle);
  
  // Update current width
  currentLineWidthInches = widthInches;
  
  Serial.print("Line width set to: ");
  Serial.print(widthInches, 2);
  Serial.print(" inches | Servo1: ");
  Serial.print(servo1Angle, 1);
  Serial.print("° | Servo2: ");
  Serial.print(servo2Angle, 1);
  Serial.println("°");
}
// 

void checkPaintTankLevel() {
  // Simple ultrasonic reading - NO DELAYS
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  
  long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 20000);
  if (duration == 0) return; // No reading, skip
  
  float distance = duration * 0.034 / 2;
  
  // Simple 3-level logic 
  if (distance <= 6) {
    // HIGH LEVEL - GREEN LED
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(YELLOW_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, HIGH);
    digitalWrite(BUZZER_PIN, LOW);
    currentTankLevel = 3;
    
  } else if (distance <= 12) {
    // MEDIUM LEVEL - YELLOW LED
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(YELLOW_LED_PIN, HIGH);
    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    currentTankLevel = 2;
    
  } else if (distance <= 18) {
    // LOW LEVEL - RED LED
    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(YELLOW_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    currentTankLevel = 1;
    
  } else {
    // EMPTY - RED LED + BUZZER
    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(YELLOW_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, HIGH);
    currentTankLevel = 0;
  }
}

float getUltrasonicDistance() {
  // Optimized single ultrasonic reading
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  
  long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 30000); // 30ms timeout
  if (duration == 0) return -1; // No echo received
  
  float distance = duration * 0.034 / 2; // Convert to cm
  return distance;
}

void updateTankLEDs() {
  // Turn off all LEDs first
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  
  // Turn on appropriate LED
  switch(currentTankLevel) {
    case 3: // High level
      digitalWrite(GREEN_LED_PIN, HIGH);
      break;
    case 2: // Medium level
      digitalWrite(YELLOW_LED_PIN, HIGH);
      break;
    case 1: // Low level
      digitalWrite(RED_LED_PIN, HIGH);
      break;
    case 0: // Empty - flash red LED
      static unsigned long lastFlash = 0;
      static bool redFlashState = false;
      if (millis() - lastFlash > 250) {
        redFlashState = !redFlashState;
        digitalWrite(RED_LED_PIN, redFlashState);
        lastFlash = millis();
      }
      break;
  }
}

void setManualLED(int ledPin) {
  // Turn off all LEDs first
  digitalWrite(RED_LED_PIN, LOW);    // Pin 39
  digitalWrite(YELLOW_LED_PIN, LOW); // Pin 35  
  digitalWrite(GREEN_LED_PIN, LOW);  // Pin 33
  
  // Turn on selected LED
  digitalWrite(ledPin, HIGH);
  
  Serial.print("Manual LED control: Pin ");
  Serial.print(ledPin);
  Serial.println(" ON, others OFF");
}

void handleLEDCommand(String cmd) {
  if (cmd == "LED:33") {
    setManualLED(GREEN_LED_PIN);   // Pin 33
  } else if (cmd == "LED:35") {
    setManualLED(YELLOW_LED_PIN);  // Pin 35
  } else if (cmd == "LED:39") {
    setManualLED(RED_LED_PIN);     // Pin 39
  } else if (cmd == "LED:OFF") {
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(YELLOW_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, LOW);
    Serial.println("All LEDs OFF");
  }
}
// =============== OBSTACLE DETECTION FUNCTIONS ===============

void checkForObstacles() {
  // Read both ultrasonic sensors
  obstacleDistance1 = getUltrasonicDistance(ULTRASONIC_TRIG1, ULTRASONIC_ECHO1);
  obstacleDistance2 = getUltrasonicDistance(ULTRASONIC_TRIG2, ULTRASONIC_ECHO2);
  
  // Get minimum distance (closest obstacle)
  minObstacleDistance = min(obstacleDistance1, obstacleDistance2);
  
  // Check if obstacle is detected
  bool currentObstacleState = (minObstacleDistance <= OBSTACLE_DETECTION_RANGE && minObstacleDistance > 0);
  
  // Handle obstacle state changes
  if (currentObstacleState != previousObstacleState) {
    if (currentObstacleState) {
      // Obstacle detected
      onObstacleDetected();
    } else {
      // Obstacle cleared
      onObstacleCleared();
    }
    previousObstacleState = currentObstacleState;
  }
  
  obstacleDetected = currentObstacleState;
}

float simpleUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 20000);  // 20ms timeout
  return (duration == 0) ? 999.0 : (duration * 0.034 / 2);
}



float getUltrasonicDistance(int trigPin, int echoPin) {
  return simpleUltrasonic(trigPin, echoPin);
}



void onObstacleDetected() {
  Serial.println("🚨 OBSTACLE DETECTED! 🚨");
  Serial.print("Distance: ");
  Serial.print(minObstacleDistance, 1);
  Serial.println(" cm");
  
  // Preserve current navigation state
  saveNavigationState();
  
  // Pause all navigation
  pauseNavigation();
  
  // Start obstacle alarm
  obstacleAlarmActive = true;
  obstacleDetectedTime = millis();
  
  // Notify ESP32
  Serial2.println("OBSTACLE_DETECTED:" + String(minObstacleDistance, 1));
  
  // Stop motors immediately
  stopMotors();
  
  Serial.println("Navigation paused - waiting for obstacle to clear");
}

void onObstacleCleared() {
  Serial.println("✅ OBSTACLE CLEARED!");
  
  // Stop obstacle alarm
  obstacleAlarmActive = false;
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(STATUS_LED, HIGH); // Solid LED
  
  // Notify ESP32
  Serial2.println("OBSTACLE_CLEARED");
  
  // Resume navigation from where it was paused
  resumeNavigation();
  
  Serial.println("Navigation resumed from pause point");
}

void saveNavigationState() {
  // Save current navigation state
  wasInStraightMode = straightMode;
  wasInRectangleMode = rectangleMode;
  wasInTurningMode = turningMode;
  wasHallSensorEnabled = hallSensorEnabled;
  wasPaintWithNavigation = paintWithNavigation;
  
  // Save specific state data
  pausedPulseCount = pulseCount;
  pausedRectCurrentSegment = rectCurrentSegment;
  pausedRectCurrentTarget = rectCurrentTarget;
  pausedTargetHeading = targetHeading;
  pausedTurnDirection = turnDirection;
  
  navigationPaused = true;
  
  Serial.println("Navigation state saved");
  Serial.print("Saved pulse count: ");
  Serial.println(pausedPulseCount);
  if (wasInRectangleMode) {
    Serial.print("Saved rectangle segment: ");
    Serial.print(pausedRectCurrentSegment);
    Serial.print("/8, target pulses: ");
    Serial.println(pausedRectCurrentTarget);
  }
}

void pauseNavigation() {
  // Stop all active navigation modes
  straightMode = false;
  rectangleMode = false;
  turningMode = false;
  hallSensorEnabled = false;
  
  // Pause paint operations but don't fully stop (for quick resume)
  if (paintWithNavigation) {
    stopPaintPump(); // Stop pump for safety
    // Keep mixer running for consistency
  }
  
  Serial.println("All navigation paused due to obstacle");
}

void resumeNavigation() {
  if (!navigationPaused) {
    Serial.println("No navigation to resume");
    return;
  }
  
  Serial.println("Resuming navigation from saved state...");
  
  // Restore paint operations if they were active
  if (wasPaintWithNavigation) {
    paintWithNavigation = true;
    // Paint will be controlled by navigation logic
  }
  
  // Resume based on what was running
  if (wasInRectangleMode) {
    Serial.println("Resuming rectangle navigation");
    resumeRectangleNavigation();
  } else if (wasInStraightMode) {
    Serial.println("Resuming straight line navigation");
    resumeStraightNavigation();
  } else if (wasInTurningMode) {
    Serial.println("Resuming turn navigation");
    resumeTurnNavigation();
  }
  
  navigationPaused = false;
  Serial.println("Navigation successfully resumed!");
}

void resumeRectangleNavigation() {
  // Restore rectangle state
  rectangleMode = true;
  rectCurrentSegment = pausedRectCurrentSegment;
  rectCurrentTarget = pausedRectCurrentTarget;
  
  // Restore pulse count to continue from where we left off
  pulseCount = pausedPulseCount;
  
  bool isStraightSegment = (rectCurrentSegment % 2 == 0);
  
  if (isStraightSegment) {
    // Resume straight segment
    Serial.print("Resuming rectangle segment ");
    Serial.print(rectCurrentSegment + 1);
    Serial.print("/8, continuing from pulse ");
    Serial.print(pulseCount);
    Serial.print("/");
    Serial.println(rectCurrentTarget);
    
    // Restore heading and PID
    targetHeading = pausedTargetHeading;
    integral = 0.0;
    prevError = 0.0;
    lastPIDTime = millis();
    
    straightMode = true;
    hallSensorEnabled = true;
    
    // Resume paint for painting segments
    if (wasPaintWithNavigation && (rectCurrentSegment == 0 || rectCurrentSegment == 4)) {
      startPaintPump();
      Serial.println("Paint resumed for rectangle segment");
    }
    
  } else {
    // Resume turn segment
    turnDirection = pausedTurnDirection;
    startTurnMode();
    Serial.println("Resuming rectangle turn");
  }
}

void resumeStraightNavigation() {
  // Restore straight line navigation
  straightMode = true;
  targetHeading = pausedTargetHeading;
  
  // Reset PID for smooth restart
  integral = 0.0;
  prevError = 0.0;
  lastPIDTime = millis();
  
  if (wasHallSensorEnabled) {
    // Pulse-based navigation
    hallSensorEnabled = true;
    pulseCount = pausedPulseCount;
    Serial.print("Resuming pulse navigation from pulse ");
    Serial.print(pulseCount);
    Serial.print("/");
    Serial.println(rectCurrentTarget);
    
    if (wasPaintWithNavigation) {
      startPaintPump();
      Serial.println("Paint resumed for pulse navigation");
    }
  } else {
    // Regular straight line
    if (wasPaintWithNavigation) {
      startPaintPump();
      Serial.println("Paint resumed for straight line");
    }
  }
}

void resumeTurnNavigation() {
  // Restore turn navigation
  turnDirection = pausedTurnDirection;
  startTurnMode();
  
  Serial.print("Resuming ");
  Serial.print(turnDirection);
  Serial.println(" turn");
}

// =============== COMMAND HANDLER ===============
void handleCommandWithPaintAndWidth(String cmd) {
  // Handle line width command
  if (cmd.startsWith("LINEWIDTH:")) {
    float width = cmd.substring(10).toFloat();
    if (width >= 1.5 && width <= 4.0) {
      setLineWidthServos(width);
      Serial2.println("LINEWIDTH_SET:" + String(width, 2));
    } else {
      Serial2.println("LINEWIDTH_ERROR:Invalid_range");
    }
    return;
  }
  
  // Handle existing commands with paint integration
  bool withPaint = cmd.indexOf(",paint") > 0;
  if (withPaint) {
    paintWithNavigation = true;
    cmd.replace(",paint", "");
  } else {
    paintWithNavigation = false;
  }
  
  // Handle PULSES command
  if (cmd.startsWith("PULSES:")) {
    int pulseTarget = cmd.substring(7).toInt();
    if (pulseTarget > 0) {
      startPulseBasedNavigation(pulseTarget);
      Serial2.println("PULSES_STARTED");
    }
  }
  // Handle existing STRAIGHT command
  else if (cmd.startsWith("STRAIGHT:")) {
    int commaPos = cmd.indexOf(',');
    if (commaPos > 0) {
      baseLeftPWM = cmd.substring(9, commaPos).toInt();
      baseRightPWM = cmd.substring(commaPos + 1).toInt();
      manualMode = false;
      rectangleMode = false;
      hallSensorEnabled = false;
      
      if (paintWithNavigation) {
        startPaintPump();
        startSimpleMixer();
      }
      
      startStraightMode();
      Serial2.println("STRAIGHT_STARTED");
    }
  }
  // Handle existing RECTANGLE command
  else if (cmd.startsWith("RECTANGLE:")) {
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
      
      if (paintWithNavigation) {
       startSimpleMixer();
      }
      
      startRectangleModeWithPaint();
      Serial2.println("RECTANGLE_STARTED");
    }
  }


  // Handle SQUARE command (if you have one)
else if (cmd.startsWith("SQUARE:")) {
  int commaPos = cmd.indexOf(',');
  if (commaPos > 0) {
    int squareSide = cmd.substring(7, commaPos).toInt();
    String direction = cmd.substring(commaPos + 1);
    
    // Apply multiplier to square
    rectLength = squareSide;   // Square uses same length/width
    rectWidth = squareSide;
    rectDirection = direction;
    rectSpeed = 120;  // Default speed
    
    Serial.print("Received square command: ");
    Serial.print(squareSide);
    Serial.print(" -> will execute ");
    Serial.print(squareSide * PULSE_MULTIPLIER);
    Serial.println("x");
    Serial.print(squareSide * PULSE_MULTIPLIER);
    Serial.println(" actual pulses");
    
    if (paintWithNavigation) {
      startSimpleMixer();
    }
    
    startRectangleModeWithPaint();  // This will apply the multiplier
    Serial2.println("SQUARE_STARTED");
  }
}
  // Handle MOVE command
  else if (cmd.startsWith("MOVE:")) {
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
  // Handle MANUAL command
  else if (cmd.startsWith("MANUAL:")) {
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
  // Handle TURN command
  else if (cmd.startsWith("TURN:")) {
    turnDirection = cmd.substring(5);
    manualMode = false;
    rectangleMode = false;
    hallSensorEnabled = false;
    startTurnMode();
    Serial2.println("TURN_STARTED:" + turnDirection);
  }
  // Handle PID command
  else if (cmd.startsWith("PID:")) {
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
  // Handle STOP command
  else if (cmd == "STOP") {
    stopAllOperations();
    Serial2.println("STOPPED");
  }
  // Handle STATUS command
  
else if (cmd == "STATUS") {
  Serial2.println(getEnhancedStatus());
}
// Handle LED commands
else if (cmd.startsWith("LED:")) {
  handleLEDCommand(cmd);
  Serial2.println("LED_COMMAND_EXECUTED");
}
// Handle other paint commands
else {
  handlePaintCommands(cmd);
}
}

// =============== NAVIGATION FUNCTIONS ===============
void startPulseBasedNavigation(int targetPulses) {
  if (!sensorsInitialized) {
    Serial.println("Cannot start pulse navigation - sensors not initialized");
    return;
  }
  
  Serial.print("Starting pulse-based navigation for ");
  Serial.print(targetPulses);
  Serial.println(" pulses");
  
  // Read current heading as target
  for (int i = 0; i < 10; i++) {
    readSensors();
    delay(20);
  }
  
  targetHeading = calculateHeading();
  
  // Reset PID variables
  integral = 0.0;
  prevError = 0.0;
  lastPIDTime = millis();
  
  // Setup pulse counting
  pulseCount = 0;
  int actualPulses = targetPulses * PULSE_MULTIPLIER;
rectCurrentTarget = actualPulses;
  hallSensorEnabled = true;
  
  // Start paint if enabled
  if (paintWithNavigation) {
    startPaintPump();
  }
  
  straightMode = true;
  turningMode = false;
  manualMode = false;
  rectangleMode = false;
  
  Serial.println("Pulse navigation started");
}

void updatePulseNavigation() {
  if (straightMode && hallSensorEnabled) {
    if (pulseCount >= rectCurrentTarget) {
      Serial.print("Pulse navigation complete: ");
      Serial.print(pulseCount);
      Serial.print("/");
      Serial.println(rectCurrentTarget);
      
      // Stop all operations
      straightMode = false;
      hallSensorEnabled = false;
      stopMotors();
      
      if (paintWithNavigation) {
        stopPaintPump();
       stopSimpleMixer();
        paintWithNavigation = false;
      }
      
      Serial2.println("PULSES_COMPLETE");
    } else {
      updateStraightMode();
      
      static unsigned long lastPulseDebug = 0;
      if (millis() - lastPulseDebug > 1000) {
        Serial.print("Pulse navigation: ");
        Serial.print(pulseCount);
        Serial.print("/");
        Serial.print(rectCurrentTarget);


        Serial.print(" pulses, Heading: ");
        Serial.print(calculateHeading(), 1);
        if (paintWithNavigation && paintSystemEnabled) {
          Serial.print(", PAINTING");
        }
        Serial.println();
        lastPulseDebug = millis();
      }
    }
  }
}

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
  turnFirstAttempt = true;
  turnForceStop = false;
  correctionStartTime = 0;
  correctionSettled = false;
  correctionStableCount = 0;
  lastCorrectionMove = 0;
  inDeadband = false;
  consecutiveDeadbandReadings = 0;
  
  // Initialize error history array
  for (int i = 0; i < 10; i++) {
    correctionErrorHistory[i] = 0;
  }
  correctionErrorIndex = 0;
  
  // Calculate precise target heading
  if (turnDirection == "right") {
    turnTargetHeading = turnStartHeading + 90.0;
  } else {
    turnTargetHeading = turnStartHeading - 90.0;
  }
  
  // Normalize target heading to 0-360
  while (turnTargetHeading < 0) turnTargetHeading += 360;
  while (turnTargetHeading >= 360) turnTargetHeading -= 360;
  
  straightMode = false;
  turningMode = true;
  
  Serial.print("Starting ULTRA-PRECISE 90° turn ");
  Serial.print(turnDirection);
  Serial.print(" from heading: ");
  Serial.print(turnStartHeading, 1);
  Serial.print("° to target: ");
  Serial.println(turnTargetHeading, 1);
}

void updateTurnMode() {
  //  Check force stop flag first - IMMEDIATE EXIT if set
  if (turnForceStop) {
    stopMotors();
    turningMode = false;
    Serial.println("🛑 FORCE STOP ACTIVE - Turn mode terminated");
    return;
  }
  
  // Force multiple sensor readings for better accuracy
  float headingSum = 0;
  int validReadings = 0;
  
  for (int i = 0; i < 3; i++) {
    readSensors();
    headingSum += calculateHeading();
    validReadings++;
    delay(10);
  }
  
  float currentHeading = headingSum / validReadings;
  float headingChange = currentHeading - turnStartHeading;
  
  // Normalize heading change to -180 to +180
  while (headingChange > 180) headingChange -= 360;
  while (headingChange < -180) headingChange += 360;
  
  // Store actual turn degrees for web display
  lastTurnDegrees = headingChange;
  
  // Calculate turning rate (degrees per second) for predictive stopping
  float turningRate = calculateTurningRate();
  
  bool turnComplete = false;
  
  if (turnFirstAttempt) {
    int turnSpeed = 140;
    float stopThreshold = 60.0;
    if (turningRate > 60) {
  stopThreshold = 65.0;  // Stop earlier due to higher speed
} else if (turningRate > 40) {
  stopThreshold = 78.0;
} else {
  stopThreshold = 80.0;
}
    
   if (turnDirection == "right") {
  moveMotors(turnSpeed, -turnSpeed);
  if (headingChange >= stopThreshold) {
    turnComplete = true;
  }
} else if (turnDirection == "left") {
  moveMotors(-turnSpeed, turnSpeed);
  if (headingChange <= -stopThreshold) {
    turnComplete = true;
  }
}
    
    // Emergency stop for major overshoot
    if (abs(headingChange) > 95) {
      turnComplete = true;
      Serial.println("EMERGENCY: Overshoot detected in first attempt");
    }
    
    // Timeout for first attempt
    if (millis() - turnStartTime > 12000) {
      turnComplete = true;
      Serial.println("First attempt timeout");
    }
    
    if (turnComplete) {
      stopMotors();
      Serial.print("FIRST ATTEMPT: Stopped at ");
      Serial.print(headingChange, 1);
      Serial.println("°");
      
      //  Extended settling time
      delay(1000);
      
      // Take multiple readings after settling
      float settledHeadingSum = 0;
      for (int i = 0; i < 5; i++) {
        readSensors();
        settledHeadingSum += calculateHeading();
        delay(100);
      }
      float settledHeading = settledHeadingSum / 5;
      
      // Calculate final error from target
      float errorFromTarget = turnTargetHeading - settledHeading;
      while (errorFromTarget > 180) errorFromTarget -= 360;
      while (errorFromTarget < -180) errorFromTarget += 360;
      
      Serial.println("===== FIRST ATTEMPT COMPLETE =====");
      Serial.print("Settled at: ");
      Serial.print(settledHeading, 1);
      Serial.print("° | Target: ");
      Serial.print(turnTargetHeading, 1);
      Serial.print("° | Error: ");
      Serial.print(errorFromTarget, 1);
      Serial.println("°");
      
      if (abs(errorFromTarget) <= 4.0) {
        Serial.println("✓ FIRST ATTEMPT SUCCESS - Turn complete");
        finalizeTurn();
      } else {
        Serial.println("⚠ NEEDS CORRECTION - Starting FINAL adjustment");
        startCorrectionAttempt(errorFromTarget, settledHeading);
      }
    }
  } else {
    // Second attempt: Intelligent correction
    float errorFromTarget = turnTargetHeading - currentHeading;
    while (errorFromTarget > 180) errorFromTarget -= 360;
    while (errorFromTarget < -180) errorFromTarget += 360;
    
    // Store error in history
    correctionErrorHistory[correctionErrorIndex] = errorFromTarget;
    correctionErrorIndex = (correctionErrorIndex + 1) % 10;
    
    // Intelligent correction logic
    if (!correctionApplied) {
      if (abs(errorFromTarget) <= 1.5) {
        Serial.println("✓ ERROR TOO SMALL - No correction needed");
        finalizeTurn();
        return;
      }
      
      correctionAttempts++;
      if (correctionAttempts > 1) {
        Serial.println("⚠ MAX CORRECTIONS REACHED - Finalizing");
        finalizeTurn();
        return;
      }
      
     // Calculate adaptive correction with HIGHER SPEEDS
float correctionSpeed;
unsigned long correctionDuration;

if (abs(errorFromTarget) > 12.0) {
  correctionSpeed = 110;  // Increased from 70 to 120
  correctionDuration = 750;  // Reduced time due to higher speed
} else if (abs(errorFromTarget) > 8.0) {
  correctionSpeed = 110;  // Increased from 65 to 110
  correctionDuration = 600;  // Reduced time
} else if (abs(errorFromTarget) > 5.0) {
  correctionSpeed = 95;   // Increased from 58 to 95
  correctionDuration = 400;  // Reduced time
} else if (abs(errorFromTarget) > 3.0) {
  correctionSpeed = 85;   // Increased from 50 to 85
  correctionDuration = 400;  // Reduced time
} else {
  correctionSpeed = 75;   // Increased from 42 to 75
  correctionDuration = 300;  // Reduced time
}
      Serial.println("===== APPLYING INTELLIGENT CORRECTION =====");
      Serial.print("Error: ");
      Serial.print(errorFromTarget, 1);
      Serial.print("° | Speed: ");
      Serial.print(correctionSpeed);
      Serial.print(" | Duration: ");
      Serial.print(correctionDuration);
      Serial.println("ms");
      
      // Apply correction
      if (errorFromTarget > 0) {
        moveMotors(correctionSpeed, -correctionSpeed);
      } else {
        moveMotors(-correctionSpeed, correctionSpeed);
      }
      
      correctionApplied = true;
      correctionCompleteTime = millis() + correctionDuration;
      
    } else if (!verificationDone) {
      if (millis() >= correctionCompleteTime) {
        stopMotors();
        Serial.println("===== CORRECTION APPLIED - VERIFYING =====");
        verificationDone = true;
        verificationCompleteTime = millis() + 1000;
      }
      
    } else {
      if (millis() >= verificationCompleteTime) {
        // Verification complete
        float verificationSum = 0;
        for (int i = 0; i < 5; i++) {
          readSensors();
          verificationSum += calculateHeading();
          delay(50);
        }
        float verifiedHeading = verificationSum / 5;
        
        float verifiedError = turnTargetHeading - verifiedHeading;
        while (verifiedError > 180) verifiedError -= 360;
        while (verifiedError < -180) verifiedError += 360;
        
        Serial.println("===== VERIFICATION COMPLETE =====");
        Serial.print("Verified heading: ");
        Serial.print(verifiedHeading, 1);
        Serial.print("° | Remaining error: ");
        Serial.print(verifiedError, 1);
        Serial.println("°");
        
        if (abs(verifiedError) <= 3.0 || correctionAttempts >= 2) {
          if (abs(verifiedError) <= 3.0) {
            Serial.println("✓ EXCELLENT PRECISION ACHIEVED");
          } else {
            Serial.println("⚠ MAXIMUM ATTEMPTS - ACCEPTING RESULT");
          }
          finalizeTurn();
          return;
        } else {
          Serial.println("🔄 NEEDS ADDITIONAL CORRECTION");
          correctionApplied = false;
          verificationDone = false;
        }
      }
    }
    
    // Safety timeout
    if (millis() - correctionStartTime > 15000) {
      Serial.println("⏰ TOTAL CORRECTION TIMEOUT");
      stopMotors();
      finalizeTurn();
      return;
    }
  }
  
  // Debug output
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 250) {
    Serial.print(turnFirstAttempt ? "[ATTEMPT-1]" : "[CORRECTION]");
    Serial.print(" ");
    Serial.print(turnDirection);
    Serial.print(" | H:");
    Serial.print(currentHeading, 1);
    Serial.print("° | Δ:");
    Serial.print(headingChange, 1);
    Serial.print("° | Rate:");
    Serial.print(turningRate, 1);
    Serial.println("°/s");
    lastDebug = millis();
  }
}

float calculateTurningRate() {
  if (headingHistoryIndex < 2) return 0;
  
  float oldHeading = headingHistory[(headingHistoryIndex + 3) % 5];
  float newHeading = headingHistory[(headingHistoryIndex + 4) % 5];
  
  float deltaHeading = newHeading - oldHeading;
  while (deltaHeading > 180) deltaHeading -= 360;
  while (deltaHeading < -180) deltaHeading += 360;
  
  return abs(deltaHeading) * 10;
}

void startCorrectionAttempt(float error, float currentHeading) {
  turnFirstAttempt = false;
  correctionStartTime = millis();
  
  correctionApplied = false;
  verificationDone = false;
  correctionCompleteTime = 0;
  verificationCompleteTime = 0;
  correctionAttempts = 0;
  
  Serial.println("===== STARTING INTELLIGENT CORRECTION =====");
  Serial.print("Current: ");
  Serial.print(currentHeading, 1);
  Serial.print("° | Target: ");
  Serial.print(turnTargetHeading, 1);
  Serial.print("° | Error: ");
  Serial.println(error, 1);
}

void finalizeTurn() {
  stopMotors();
  turningMode = false;
  turnForceStop = true;
  
  // Final verification
  float finalHeadingSum = 0;
  for (int i = 0; i < 3; i++) {
    readSensors();
    finalHeadingSum += calculateHeading();
    delay(100);
  }
  float finalHeading = finalHeadingSum / 3;
  
  float finalError = turnTargetHeading - finalHeading;
  while (finalError > 180) finalError -= 360;
  while (finalError < -180) finalError += 360;
  
  float actualTurnFromStart = finalHeading - turnStartHeading;
  while (actualTurnFromStart > 180) actualTurnFromStart -= 360;
  while (actualTurnFromStart < -180) actualTurnFromStart += 360;
  
  lastTurnDegrees = actualTurnFromStart;
  
  Serial.println("=========================================");
  Serial.println("      🛑 TURN COMPLETE 🛑");
  Serial.println("=========================================");
  Serial.print("Target turn:    90.0° ");
  Serial.println(turnDirection);
  Serial.print("Actual turn:    ");
  Serial.print(actualTurnFromStart, 1);
  Serial.println("°");
  Serial.print("Final error:    ");
  Serial.print(finalError, 1);
  Serial.println("°");
  Serial.println("=========================================");
  
  delay(800);
  turnForceStop = false;
  
  // If part of rectangle mode, continue
  if (rectangleMode) {
    continueRectangleAfterTurnWithPaint();
  }
}

float computePID(float currentHeading) {
  unsigned long now = millis();
  float dt = (now - lastPIDTime) / 1000.0;
  
  if (dt < 0.01) dt = 0.01;
  
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
    moveMotors(speed/4, speed);
  }
  else if (direction == "right") {
    moveMotors(speed, speed/4);
  }
  else {
    Serial.println("Unknown manual direction");
    stopMotors();
  }
}

// =============== RECTANGLE NAVIGATION WITH PAINT ===============
void startRectangleModeWithPaint() {
  if (!sensorsInitialized) {
    Serial.println("Cannot start rectangle mode - sensors not initialized");
    return;
  }
  
  // Check for magnet interference
  pulseCount = 0;
  magnetInterferenceDetected = false;
  delay(100);
  
  if (detectMagnetInterference()) {
    Serial.println("ERROR: Magnet interference detected!");
    Serial2.println("MAGNET_INTERFERENCE_ERROR");
    return;
  }
  
  Serial.println("=== STARTING RECTANGLE NAVIGATION WITH PAINT ===");
  Serial.print("Rectangle: ");
  Serial.print(rectLength);
  Serial.print(" x ");
  Serial.print(rectWidth);
  Serial.print(" pulses, ");
  Serial.print(rectDirection);
  Serial.print(", speed: ");
  Serial.print(rectSpeed);
  if (paintWithNavigation) {
    Serial.print(", WITH PAINTING");
  }
  Serial.println();
  
  // Apply multiplier to rectangle dimensions
int actualLength = rectLength * PULSE_MULTIPLIER;
int actualWidth = rectWidth * PULSE_MULTIPLIER;

  rectCurrentPulses[0] = actualLength;
rectCurrentPulses[1] = actualWidth;
rectCurrentPulses[2] = actualLength;
rectCurrentPulses[3] = actualWidth;

  
  rectCurrentSegment = 0;
  pulseCount = 0;
  consecutivePulses = 0;
  hallSensorEnabled = true;
  rectangleMode = true;
  
  baseLeftPWM = rectSpeed;
  baseRightPWM = rectSpeed;
  
  Serial.println("Starting first segment");
  startRectangleSegmentWithPaint();
}

void startRectangleSegmentWithPaint() {
  if (rectCurrentSegment >= 8) {
    finishRectangleWithPaint();
    return;
  }
  
  bool isStraightSegment = (rectCurrentSegment % 2 == 0);
  
  if (isStraightSegment) {
    // Straight segment
    int segmentIndex = rectCurrentSegment / 2;
    rectCurrentTarget = rectCurrentPulses[segmentIndex];
    
    Serial.print("Segment ");
    Serial.print(rectCurrentSegment + 1);
    Serial.print("/8: Going straight for ");
    Serial.print(rectCurrentTarget);
    Serial.println(" pulses");
    
    // Paint control logic
    if (paintWithNavigation) {
      if (rectCurrentSegment == 0 || rectCurrentSegment == 4) {
        // Segments 1 and 5 - START PAINTING
        startPaintPump();
        Serial.println("PAINT: Started for segment " + String(rectCurrentSegment + 1));
      } else if (rectCurrentSegment == 2 || rectCurrentSegment == 6) {
        // Segments 3 and 7 - STOP PAINTING
        stopPaintPump();
        Serial.println("PAINT: Stopped for segment " + String(rectCurrentSegment + 1));
      }
    }
    
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
    straightMode = true;
    
  } else {
    // Turn segment - always stop painting
    if (paintWithNavigation) {
      stopPaintPump();
      Serial.println("PAINT: Stopped for turn segment " + String(rectCurrentSegment + 1));
    }
    
    Serial.print("Segment ");
    Serial.print(rectCurrentSegment + 1);
    Serial.print("/8: Making 90° turn ");
    
    if (rectDirection == "clockwise") {
      turnDirection = "right";
      Serial.println("RIGHT (clockwise)");
    } else {
      turnDirection = "left";
      Serial.println("LEFT (anticlockwise)");
    }
    
    hallSensorEnabled = false;
    straightMode = false;
    startTurnMode();
  }
}

void updateRectangleModeWithPaint() {
  // Check for magnet interference
  if (detectMagnetInterference()) {
    Serial.println("CRITICAL: Magnet interference detected!");
    finishRectangleWithPaint();
    Serial2.println("RECTANGLE_STOPPED_INTERFERENCE");
    return;
  }
  
  // Handle straight segments with pulse counting
  if (straightMode && hallSensorEnabled) {
    if (pulseCount >= rectCurrentTarget) {
      Serial.print("Reached target pulses: ");
      Serial.print(pulseCount);
      Serial.print("/");
      Serial.println(rectCurrentTarget);
      
      straightMode = false;
      hallSensorEnabled = false;
      stopMotors();
      delay(500);
      
      rectCurrentSegment++;
      startRectangleSegmentWithPaint();
    } else {
      updateStraightMode();
      
      static unsigned long lastRectDebug = 0;
      if (millis() - lastRectDebug > 1000) {
        Serial.print("Rectangle straight: ");
        Serial.print(pulseCount);
        Serial.print("/");
        Serial.print(rectCurrentTarget);
        Serial.print(" pulses, Heading: ");
        Serial.print(calculateHeading(), 1);
        if (paintWithNavigation && paintSystemEnabled) {
          Serial.print(", PAINTING");
        }
        Serial.println();
        lastRectDebug = millis();
      }
    }
  }
}

void continueRectangleAfterTurnWithPaint() {
  Serial.println("Turn completed, continuing rectangle");
  rectCurrentSegment++;
  delay(300);
  startRectangleSegmentWithPaint();
}

void finishRectangleWithPaint() {
  stopMotors();
  
  // Stop all paint operations
  if (paintWithNavigation) {
    stopPaintPump();
   stopSimpleMixer();
    paintWithNavigation = false;
  }
  
  rectangleMode = false;
  hallSensorEnabled = false;
  straightMode = false;
  turningMode = false;
  magnetInterferenceDetected = false;
  
  Serial.println("=== RECTANGLE NAVIGATION COMPLETE ===");
  Serial.println("All paint operations stopped");
  
  // Blink LED to indicate completion
  for (int i = 0; i < 5; i++) {
    digitalWrite(STATUS_LED, LOW);
    delay(200);
    digitalWrite(STATUS_LED, HIGH);
    delay(200);
  }
}

// =============== UTILITY FUNCTIONS ===============
void stopAllOperations() {
  straightMode = false;
  turningMode = false;
  manualMode = false;
  rectangleMode = false;
  hallSensorEnabled = false;
  magnetInterferenceDetected = false;
  pulseCount = 0;
  
  // Stop obstacle alarm
  obstacleAlarmActive = false;
  navigationPaused = false;
  digitalWrite(BUZZER_PIN, LOW);
  
  // Stop paint operations
  stopPaintPump();
  stopSimpleMixer();
  paintWithNavigation = false;
  
  stopMotors();
  Serial.println("All operations stopped including obstacle detection");
}

String getEnhancedStatus() {
  String status = "STATUS:L=" + String(currentLeftSpeed) + ",R=" + String(currentRightSpeed);
  status += ",Pulses=" + String(pulseCount);
  status += ",TurnDegrees=" + String(lastTurnDegrees, 1);
  status += ",WIDTH:" + String(currentLineWidthInches, 1);
  
  // obstacle detection status
  status += ",OBSTACLE:" + String(obstacleDetected ? "YES" : "NO");
  if (obstacleDetected) {
    status += ",DIST:" + String(minObstacleDistance, 1) + "cm";
  }
  if (navigationPaused) {
    status += ",PAUSED:YES";
  }
  
  //  tank status
  String tankStatus = "";
  switch(currentTankLevel) {
    case 3: tankStatus = "GREEN"; break;
    case 2: tankStatus = "YELLOW"; break;
    case 1: tankStatus = "RED"; break;
    case 0: tankStatus = "EMPTY"; break;
  }
  status += ",TANK:" + tankStatus;
  
  if (magnetInterferenceDetected) {
    status += ",MAGNET_INTERFERENCE=TRUE";
  }
  if (sensorsInitialized) {
    status += ",H=" + String(calculateHeading(), 1);
    status += ",Mode=";
    if (navigationPaused) {
      status += "PAUSED";
    } else if (rectangleMode) {
      status += "RECTANGLE(" + String(rectCurrentSegment) + "/8)";
    } else if (straightMode) {
      status += "STRAIGHT";
    } else if (turningMode) {
      status += "TURNING";
    } else if (manualMode) {
      status += "MANUAL_NAV";
    } else {
      status += "MANUAL";
    }
  } else {
    status += ",Sensors=FAILED";
  }
  
  return status;
}

bool detectMagnetInterference() {
  unsigned long now = millis();
  
  if (consecutivePulses > 20 && (now - lastValidPulseTime < 1000) && 
      (currentLeftSpeed == 0 && currentRightSpeed == 0)) {
    magnetInterferenceDetected = true;
    return true;
  }
  
  static unsigned long rapidPulseCount = 0;
  static unsigned long rapidPulseStartTime = 0;
  
  if (rapidPulseStartTime == 0) {
    rapidPulseStartTime = now;
    rapidPulseCount = 0;
  }
  
  if (now - rapidPulseStartTime < 100) {
    if (rapidPulseCount > 10) {
      magnetInterferenceDetected = true;
      Serial.println("Rapid pulse burst detected - magnet interference");
      return true;
    }
  } else {
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
  
  delay(50);
  Serial.println("Motors stopped");
}

// =============== HALL SENSOR INTERRUPT ===============
void hallSensorISR() {
  unsigned long now = millis();
  if (now - lastPulseTime > 50) { // 50ms debounce
    lastPulseTime = now;
    
    if (hallSensorEnabled) {
      if (currentLeftSpeed == 0 && currentRightSpeed == 0) {
        consecutivePulses++;
        if (consecutivePulses == 1) {
          lastValidPulseTime = now;
        }
        
        if (consecutivePulses > 5) {
          Serial.println("WARNING: Multiple pulses while motors stopped");
          magnetInterferenceDetected = true;
          return;
        }
      } else {
        consecutivePulses = 0;
        lastValidPulseTime = now;
      }
      
      pulseCount++;
      
      // Brief LED flash
      digitalWrite(STATUS_LED, LOW);
      delayMicroseconds(100);
      digitalWrite(STATUS_LED, HIGH);
    }
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
  
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("MPU-6050 not found");
    return false;
  }
  
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
  
  // Update heading history for trend analysis
  lastSensorReadTime = millis();
  headingHistory[headingHistoryIndex] = calculateHeading();
  headingHistoryIndex = (headingHistoryIndex + 1) % 5;
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