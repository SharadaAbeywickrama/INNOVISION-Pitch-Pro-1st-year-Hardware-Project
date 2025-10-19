// ESP32 Keypad Simple Row/Column Test
//1st pin cols -14,22,32,---rows 33,25,26,27last pin 
const int rowPins[4] = {33, 25, 26, 27}; // Row pins
const int colPins[3] = {14,22,32};     // Column pins

void setup() {
  Serial.begin(115200);

  // Initialize row pins as outputs HIGH
  for (int i = 0; i < 4; i++) {
    pinMode(rowPins[i], OUTPUT);
    digitalWrite(rowPins[i], HIGH);
  }

  // Initialize column pins as inputs with pull-up
  for (int i = 0; i < 3; i++) {
    pinMode(colPins[i], INPUT_PULLUP);
  }
}

void loop() {
  for (int row = 0; row < 4; row++) {
    // Activate one row at a time
    for (int i = 0; i < 4; i++) {
      digitalWrite(rowPins[i], (i == row) ? LOW : HIGH);
    }

    delayMicroseconds(10);

    // Check columns
    for (int col = 0; col < 3; col++) {
      if (digitalRead(colPins[col]) == LOW) {
        Serial.print("Row: ");
        Serial.print(row);
        Serial.print(", Col: ");
        Serial.println(col);

        // Wait until key released
        while (digitalRead(colPins[col]) == LOW) {
          delay(10);
        }
      }
    }
  }

  delay(10);
}