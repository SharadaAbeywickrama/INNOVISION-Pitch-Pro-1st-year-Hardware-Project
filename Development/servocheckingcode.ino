#include <Servo.h>

Servo servo1;  // Create servo object for pin 11
Servo servo2;  // Create servo object for pin 12

const int servoPin1 = 11;
const int servoPin2 = 12;

void setup() {
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);

  // Move both servos to 0 degrees initially
  servo1.write(0);
  servo2.write(0);
  delay(1000);  // Wait 1 second before moving
}

void loop() {
  // Sweep servos from 0 to 90 degrees clockwise smoothly
  for (int angle = 0; angle <= 180; angle++) {
    servo1.write(angle);
    servo2.write(angle);
    delay(15);  // small delay for smooth motion
  }

  delay(2000); // Hold at 90 degrees for 2 seconds

  // Optionally, sweep back to 0 degrees (comment out if not needed)
  for (int angle = 180; angle >= 0; angle--) {
    servo1.write(angle);
    servo2.write(angle);
    delay(15);
  }

  delay(2000); // Hold at 0 degrees for 2 seconds
}
