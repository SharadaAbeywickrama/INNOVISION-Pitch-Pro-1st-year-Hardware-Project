// ===== BTS7960 Motor Test for Arduino Mega =====
// Left motor: PWM_L = 9, PWM_R = 10
// Right motor: PWM_L = 5, PWM_R = 6

// --- Pin Definitions ---
const int L_PWM_L = 10;
const int L_PWM_R = 9;
const int R_PWM_L = 5;
const int R_PWM_R = 6;

// --- Motor Speed (0-255) ---
int speedValue = 150; // Adjust for desired test speed

void setup() {
  Serial.begin(9600);

  // Set pins as outputs
  pinMode(L_PWM_L, OUTPUT);
  pinMode(L_PWM_R, OUTPUT);
  pinMode(R_PWM_L, OUTPUT);
  pinMode(R_PWM_R, OUTPUT);

  Serial.println("BTS7960 Motor Test Started");
}

void loop() {
  // ----- Forward -----
  Serial.println("Forward");
  analogWrite(L_PWM_L, speedValue);
  analogWrite(L_PWM_R, 0);
  analogWrite(R_PWM_L, speedValue);
  analogWrite(R_PWM_R, 0);
  delay(2000);

  // ----- Stop -----
  Serial.println("Stop");
  stopMotors();
  delay(1000);

  // ----- Backward -----
  Serial.println("Backward");
  analogWrite(L_PWM_L, 0);
  analogWrite(L_PWM_R, speedValue);
  analogWrite(R_PWM_L, 0);
  analogWrite(R_PWM_R, speedValue);
  delay(2000);

  // ----- Stop -----
  Serial.println("Stop");
  stopMotors();
  delay(1000);
}

// --- Helper function to stop all motors ---
void stopMotors() {
  analogWrite(L_PWM_L, 0);
  analogWrite(L_PWM_R, 0);
  analogWrite(R_PWM_L, 0);
  analogWrite(R_PWM_R, 0);
}