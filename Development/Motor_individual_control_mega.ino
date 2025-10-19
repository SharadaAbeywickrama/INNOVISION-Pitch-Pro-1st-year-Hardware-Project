// Arduino Mega - Simplified Motor Control (Individual Motors + Stop Only)
#include <Wire.h>

// Motor Pins
#define LEFT_FORWARD 6
#define LEFT_BACKWARD 5
#define RIGHT_FORWARD 10
#define RIGHT_BACKWARD 9

// Current motor speeds
int currentLeftSpeed = 0;
int currentRightSpeed = 0;

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);
  
  pinMode(LEFT_FORWARD, OUTPUT);
  pinMode(LEFT_BACKWARD, OUTPUT);
  pinMode(RIGHT_FORWARD, OUTPUT);
  pinMode(RIGHT_BACKWARD, OUTPUT);
  
  stopMotors();
  Serial.println("Simple Motor Controller Ready");
  Serial2.println("READY");
}

void loop() {
  if (Serial2.available()) {
    String cmd = Serial2.readStringUntil('\n');
    cmd.trim();
    Serial.println("Received: " + cmd);
    
    if (cmd.startsWith("MOVE:")) {
      // Format: MOVE:left_speed,right_speed (e.g., MOVE:150,-100)
      int commaPos = cmd.indexOf(',');
      if (commaPos > 0) {
        int leftSpeed = cmd.substring(5, commaPos).toInt();
        int rightSpeed = cmd.substring(commaPos + 1).toInt();
        moveMotors(leftSpeed, rightSpeed);
        Serial2.println("MOVED:" + String(leftSpeed) + "," + String(rightSpeed));
      }
    }
    else if (cmd == "STOP") {
      stopMotors();
      Serial2.println("STOPPED");
    }
    else if (cmd == "STATUS") {
      Serial2.println("STATUS:L=" + String(currentLeftSpeed) + ",R=" + String(currentRightSpeed));
    }
  }
  
  delay(10);
}

void moveMotors(int leftSpeed, int rightSpeed) {
  // Constrain speeds
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  
  currentLeftSpeed = leftSpeed;
  currentRightSpeed = rightSpeed;
  
  // Control left motor
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
  
  // Control right motor
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
  
  Serial.println("Motors: L=" + String(leftSpeed) + " R=" + String(rightSpeed));
}

void stopMotors() {
  analogWrite(LEFT_FORWARD, 0);
  analogWrite(LEFT_BACKWARD, 0);
  analogWrite(RIGHT_FORWARD, 0);
  analogWrite(RIGHT_BACKWARD, 0);
  
  currentLeftSpeed = 0;
  currentRightSpeed = 0;
  
  Serial.println("Motors stopped");
}