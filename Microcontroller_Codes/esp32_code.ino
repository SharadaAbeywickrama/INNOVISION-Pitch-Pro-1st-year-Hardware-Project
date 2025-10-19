// ESP32 - Code Display UI,TrackPad,Web Inteface
#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <Keypad.h>

#include <SoftwareSerial.h>



const char* ssid = "PitchPro";
const char* password = "12345678";

WebServer server(80);
TFT_eSPI tft = TFT_eSPI();

// GPS Configuration
SoftwareSerial gpsSerial(1, 3); // RX=GPIO1(TX0), TX=GPIO3(RX0)
struct GPSPoint {
  float latitude;
  float longitude;
  unsigned long timestamp;
};


// =============== KEYPAD CONFIGURATION ===============
const byte ROWS = 4;
const byte COLS = 3;

char keys[ROWS][COLS] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};

byte rowPins[ROWS] = {25, 33, 32, 22};  // R1, R2, R3, R4
byte colPins[COLS] = {14, 27, 26};      // C1, C2, C3

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// =============== PAINT SYSTEM PINS ===============
#define PAINT_RELAY_PIN 26     // Relay for paint pump (5V)
#define MIXER_IN3 45           // L298N IN3
#define MIXER_IN4 43           // L298N IN4  
#define MIXER_ENB 13           // L298N ENB (PWM)

// =============== UI STATE VARIABLES ===============
enum UIState {
  MAIN_MENU,
  RECTANGLE_INPUT,
  SQUARE_INPUT,
  STRAIGHT_INPUT,
  PULSE_INPUT,
  PAINT_OPTIONS,
  LINE_WIDTH_INPUT,
  RUNNING,
  STATUS_DISPLAY
};

UIState currentState = MAIN_MENU;
int selectedOption = 0;  // 0=Line, 1=Rectangle, 2=Square, 3=Turn Left, 4=Turn Right, 5=Paint, 6=Mix, 7=Line Width
String inputBuffer = "";
int rectLength = 0, rectWidth = 0, squareSide = 0, straightPulses = 0;
bool inputPhase = 0;  // 0=length/side, 1=width
String currentStatus = "Ready";
bool robotRunning = false;
bool paintEnabled = false;
bool mixingEnabled = false;
float currentLineWidth = 2.0; // Default 2 inches

// Paint mode tracking
bool paintWithNavigation = false;  // For navigation with painting
bool manualPaintMode = false;      // For manual paint control

// Line width presets (inches)
float lineWidthPresets[4] = {1.5, 2.0, 3.0, 4.0};


//GPS tracking variables
GPSPoint startPosition;
GPSPoint currentPosition;
GPSPoint pathPoints[100]; // Store up to 100 path points
int pathPointCount = 0;
bool gpsActive = false;
bool pathTracking = false;
unsigned long lastGPSRead = 0;
String rawGPSData = "";

// Obstacle Detection Variables
bool obstacleDetected = false;
float obstacleDistance = 999.0;
String obstacleStatus = "Clear";
unsigned long lastObstacleUpdate = 0;
bool navigationPaused = false;

// =============== DISPLAY COLORS ===============
#define BLACK       0x0000
#define WHITE       0xFFFF
#define RED         0xF800
#define GREEN       0x07E0
#define BLUE        0x001F
#define CYAN        0x07FF
#define MAGENTA     0xF81F
#define YELLOW      0xFFE0
#define ORANGE      0xFD20
#define PURPLE      0x8010

// Function declarations for obstacle detection
void showObstacleAlert(float distance);
void handleObstacleStatus();
void updateObstacleDisplayStatus();
void handleKeypadInput(char key);
void handleMainMenuInput(char key);
void handleStraightInputMenu(char key);
void handlePulseInput(char key);
void handleRectangleInput(char key);
void handleSquareInput(char key);
void handlePaintOptionsInput(char key);
void handleLineWidthInput(char key);
void handleRunningInput(char key);
void startSelectedNavigation();
void startRectangle();
void startSquare();
void showError(String error);

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  // Initialize GPS (add this after Serial.begin)
  initGPS();
  
  // Initialize paint system pins
  pinMode(PAINT_RELAY_PIN, OUTPUT);
  pinMode(MIXER_IN3, OUTPUT);
  pinMode(MIXER_IN4, OUTPUT);
  pinMode(MIXER_ENB, OUTPUT);
  
  // Initialize paint system to OFF
  digitalWrite(PAINT_RELAY_PIN, LOW);
  digitalWrite(MIXER_IN3, LOW);
  digitalWrite(MIXER_IN4, LOW);
  analogWrite(MIXER_ENB, 0);
  
  // Initialize TFT Display
  tft.init();
  tft.setRotation(1);  // Landscape mode
  tft.fillScreen(BLACK);
  tft.setTextColor(WHITE, BLACK);
  
  // Show startup screen
  showStartupScreen();
  delay(3000);
  
  // Initialize WiFi
  WiFi.softAP(ssid, password);
  Serial.println("WiFi AP started: " + String(WiFi.softAPIP()));
  
  // Setup web server routes
  server.on("/", handleRoot);
  server.on("/move", handleMove);
  server.on("/stop", handleStop);
  server.on("/straight", handleStraight);
  server.on("/turn", handleTurn);
  server.on("/pid", handlePID);
  server.on("/manual", handleManual);
  server.on("/rectangle", handleRectangle);
  server.on("/pulses", handlePulses);
  server.on("/paint", handlePaint);
  server.on("/mix", handleMix);
  server.on("/linewidth", handleLineWidth);
  server.on("/status", handleStatus);
  server.on("/led", handleLED);
 
    // obstacle detection route
  server.on("/obstacle", handleObstacleStatus);
    
  // GPS route to web server
  server.on("/gps", handleGPS);

   server.begin();
  Serial.println("Web server started");
  
  // Show main menu
  showMainMenu();
}

void loop() {
  server.handleClient();
  
  // Handle GPS data reading (add this)
  if (gpsActive) {
    readGPSData();
  }
  

  // Handle keypad input
  char key = keypad.getKey();
  if (key) {
    handleKeypadInput(key);
  }
  
  // Handle obstacle status display updates
if (obstacleDetected && currentState != STATUS_DISPLAY) {
  showObstacleAlert(obstacleDistance);
}

  // Handle Arduino responses
  if (Serial2.available()) {
    String response = Serial2.readStringUntil('\n');
    response.trim();
    Serial.println("Arduino: " + response);
    handleArduinoResponse(response);
  }
  
  // Update status periodically
  static unsigned long lastStatusUpdate = 0;
  if (millis() - lastStatusUpdate > 2000) {
    if (robotRunning) {
      requestStatus();
    }
    lastStatusUpdate = millis();
  }
  
  delay(10);
}

// =============== DISPLAY FUNCTIONS ===============
void showStartupScreen() {
  tft.fillScreen(BLACK);
  tft.setTextSize(2);
  tft.setTextColor(WHITE);
  
  String title = "Pitch Pro";
  tft.setCursor(80, 80);
  tft.println(title);
  
  tft.setTextSize(2);
  tft.setCursor(10, 120);
  tft.println("Autonomous Ground");
  tft.setCursor(10, 140);
  tft.println("Painting Robot");
  
  tft.setCursor(80, 200);
  tft.println("Initializing...");
}

void showMainMenu() {
  currentState = MAIN_MENU;
  tft.fillScreen(BLACK);
  tft.setTextSize(2);
  tft.setTextColor(WHITE);
  
  // Header
  tft.setCursor(10, 10);
  tft.println("Select Mode:");
  
  // Menu options
  const char* options[] = {"1. Line", "2. Rectangle", "3. Square", "4. Turn Left", "5. Turn Right", "6. Start Paint", "7. Mix Paint", "8. Line Width"};
  
  for (int i = 0; i < 8; i++) {
    tft.setCursor(20, 35 + i * 18);
    if (i == selectedOption) {
      tft.setTextColor(YELLOW);
      tft.print("> ");
    } else {
      tft.setTextColor(WHITE);
      tft.print("  ");
    }
    tft.println(options[i]);
  }
  
  // Current line width display
  tft.setTextSize(1);
  tft.setTextColor(CYAN);
  tft.setCursor(10, 185);
  tft.print("Current Width: ");
  tft.print(currentLineWidth, 1);
  tft.println(" inches");
  
  // Instructions
  tft.setCursor(10, 200);
  tft.println("1-8: Select  *: Back  #: Start  0: Stop");
}

void showStraightInput() {
  currentState = STRAIGHT_INPUT;
  tft.fillScreen(BLACK);
  tft.setTextSize(2);
  tft.setTextColor(WHITE);
  
  tft.setCursor(10, 10);
  tft.println("Straight Line Mode");
  
  tft.setCursor(10, 50);
  tft.setTextColor(YELLOW);
  tft.println("Choose Option:");
  
  tft.setCursor(20, 80);
  if (inputPhase == 0) {
    tft.setTextColor(YELLOW);
    tft.println("1. Normal Straight");
  } else {
    tft.setTextColor(WHITE);
    tft.println("1. Normal Straight");
  }
  
  tft.setCursor(20, 105);
  if (inputPhase == 1) {
    tft.setTextColor(YELLOW);
    tft.println("2. With Pulses Input");
  } else {
    tft.setTextColor(WHITE);
    tft.println("2. With Pulses Input");
  }
  
  tft.setTextSize(1);
  tft.setTextColor(CYAN);
  tft.setCursor(10, 180);
  tft.println("1-2: Select  *: Back  #: Continue");
}

void showPulseInput() {
  currentState = PULSE_INPUT;
  tft.fillScreen(BLACK);
  tft.setTextSize(2);
  tft.setTextColor(WHITE);
  
  tft.setCursor(10, 10);
  tft.println("Pulse-Based Navigation");
  
  tft.setCursor(10, 50);
  tft.setTextColor(YELLOW);
  tft.println("Enter Pulses:");
  
  tft.setTextSize(3);
  tft.setTextColor(WHITE);
  tft.setCursor(10, 100);
  tft.print("Input: ");
  tft.setTextColor(CYAN);
  tft.println(inputBuffer);
  
  tft.setTextSize(1);
  tft.setTextColor(CYAN);
  tft.setCursor(10, 180);
  tft.println("0-9: Enter digits  #: Continue  *: Back");
}

void showRectangleInput() {
  currentState = RECTANGLE_INPUT;
  tft.fillScreen(BLACK);
  tft.setTextSize(2);
  tft.setTextColor(WHITE);
  
  tft.setCursor(10, 10);
  tft.println("Rectangle Navigation");
  
  tft.setCursor(10, 50);
  if (inputPhase == 0) {
    tft.setTextColor(YELLOW);
    tft.println("Enter Length (pulses):");
  } else {
    tft.setTextColor(GREEN);
    tft.print("Length: ");
    tft.println(rectLength);
    tft.setTextColor(YELLOW);
    tft.println("Enter Width (pulses):");
  }
  
  tft.setTextSize(3);
  tft.setTextColor(WHITE);
  tft.setCursor(10, 120);
  tft.print("Input: ");
  tft.setTextColor(CYAN);
  tft.println(inputBuffer);
  
  tft.setTextSize(1);
  tft.setTextColor(CYAN);
  tft.setCursor(10, 180);
  tft.println("0-9: Enter digits  #: Confirm  *: Back");
}

void showSquareInput() {
  currentState = SQUARE_INPUT;
  tft.fillScreen(BLACK);
  tft.setTextSize(2);
  tft.setTextColor(WHITE);
  
  tft.setCursor(10, 10);
  tft.println("Square Navigation");
  
  tft.setCursor(10, 50);
  tft.setTextColor(YELLOW);
  tft.println("Enter Side Length (pulses):");
  
  tft.setTextSize(3);
  tft.setTextColor(WHITE);
  tft.setCursor(10, 120);
  tft.print("Input: ");
  tft.setTextColor(CYAN);
  tft.println(inputBuffer);
  
  tft.setTextSize(1);
  tft.setTextColor(CYAN);
  tft.setCursor(10, 180);
  tft.println("0-9: Enter digits  #: Confirm  *: Back");
}

void showPaintOptions(String mode) {
  currentState = PAINT_OPTIONS;
  tft.fillScreen(BLACK);
  tft.setTextSize(2);
  tft.setTextColor(WHITE);
  
  tft.setCursor(10, 10);
  tft.print(mode);
  tft.println(" Mode");
  
  tft.setCursor(10, 50);
  tft.setTextColor(YELLOW);
  tft.println("Choose Option:");
  
  tft.setCursor(20, 80);
  if (inputPhase == 0) {
    tft.setTextColor(YELLOW);
    tft.println("1. Normal (No Paint)");
  } else {
    tft.setTextColor(WHITE);
    tft.println("1. Normal (No Paint)");
  }
  
  tft.setCursor(20, 105);
  if (inputPhase == 1) {
    tft.setTextColor(YELLOW);
    tft.println("2. With Painting");
  } else {
    tft.setTextColor(WHITE);
    tft.println("2. With Painting");
  }
  
  tft.setTextSize(1);
  tft.setTextColor(CYAN);
  tft.setCursor(10, 150);
  tft.println("Note: Paint only during forward");
  tft.setCursor(10, 165);
  tft.println("movement, stops during turns");
  
  tft.setCursor(10, 190);
  tft.println("1-2: Select  *: Back  #: Start");
}

void showLineWidthInput() {
  currentState = LINE_WIDTH_INPUT;
  tft.fillScreen(BLACK);
  tft.setTextSize(2);
  tft.setTextColor(WHITE);
  
  tft.setCursor(10, 10);
  tft.println("Line Width Adjustment");
  
  tft.setCursor(10, 40);
  tft.setTextColor(CYAN);
  tft.print("Current: ");
  tft.print(currentLineWidth, 1);
  tft.println(" inches");
  
  tft.setCursor(10, 70);
  tft.setTextColor(YELLOW);
  tft.println("Select Preset:");
  
  // Preset options
  const char* presets[] = {"1. 1.5 inches", "2. 2.0 inches", "3. 3.0 inches", "4. 4.0 inches"};
  
  for (int i = 0; i < 4; i++) {
    tft.setCursor(20, 95 + i * 22);
    if (i == inputPhase) {
      tft.setTextColor(YELLOW);
      tft.print("> ");
    } else {
      tft.setTextColor(WHITE);
      tft.print("  ");
    }
    tft.println(presets[i]);
  }
  
  tft.setTextSize(1);
  tft.setTextColor(CYAN);
  tft.setCursor(10, 180);
  tft.println("1-4: Select preset  *: Back  #: Apply");
}

void showTurnMode(String direction) {
  currentState = STATUS_DISPLAY;
  tft.fillScreen(BLACK);
  tft.setTextSize(2);
  tft.setTextColor(GREEN);
  
  tft.setCursor(10, 60);
  tft.print("TURNING ");
  direction.toUpperCase();
  tft.println(direction);
  
  tft.setTextSize(1);
  tft.setTextColor(CYAN);
  tft.setCursor(10, 120);
  tft.println("90 Degree Turn");
  tft.setCursor(10, 140);
  tft.println("Status: " + currentStatus);
  
  tft.setTextColor(RED);
  tft.setCursor(10, 200);
  tft.println("Press 0 for Emergency Stop");
}

void showRunningStatus(String mode, String details = "") {
  currentState = STATUS_DISPLAY;
  tft.fillScreen(BLACK);
  tft.setTextSize(2);
  tft.setTextColor(GREEN);
  
  tft.setCursor(10, 10);
  tft.print("RUNNING: ");
  tft.println(mode);
  
  if (details.length() > 0) {
    tft.setTextColor(WHITE);
    tft.setCursor(10, 40);
    tft.println(details);
  }
  
  // Paint status
  if (paintWithNavigation) {
    tft.setTextColor(PURPLE);
    tft.setCursor(10, 65);
    tft.println("PAINT: Active");
  }
  
  tft.setTextColor(CYAN);
  tft.setCursor(10, 90);
  tft.println("Status: " + currentStatus);
  
  tft.setTextSize(1);
  tft.setTextColor(RED);
  tft.setCursor(10, 200);
  tft.println("Press 0 for Emergency Stop");
}

void showPaintStatus() {
  currentState = STATUS_DISPLAY;
  tft.fillScreen(BLACK);
  tft.setTextSize(2);
  tft.setTextColor(PURPLE);
  
  tft.setCursor(10, 60);
  tft.println("PAINT SYSTEM");
  
  tft.setTextColor(paintEnabled ? GREEN : RED);
  tft.setCursor(10, 90);
  tft.print("Pump: ");
  tft.println(paintEnabled ? "ON" : "OFF");
  
  tft.setTextColor(mixingEnabled ? GREEN : RED);
  tft.setCursor(10, 115);
  tft.print("Mixer: ");
  tft.println(mixingEnabled ? "ON" : "OFF");
  
  tft.setTextSize(1);
  tft.setTextColor(CYAN);
  tft.setCursor(10, 180);
  tft.println("*: Back to Menu  0: Emergency Stop");
}

void showError(String error) {
  tft.fillRect(0, 220, 320, 20, RED);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.setCursor(10, 225);
  tft.println("ERROR: " + error);
}

// =============== KEYPAD HANDLING ===============
void handleKeypadInput(char key) {
  switch (currentState) {
    case MAIN_MENU:
      handleMainMenuInput(key);
      break;
    case STRAIGHT_INPUT:
      handleStraightInputMenu(key);
      break;
    case PULSE_INPUT:
      handlePulseInput(key);
      break;
    case RECTANGLE_INPUT:
      handleRectangleInput(key);
      break;
    case SQUARE_INPUT:
      handleSquareInput(key);
      break;
    case PAINT_OPTIONS:
      handlePaintOptionsInput(key);
      break;
    case LINE_WIDTH_INPUT:
      handleLineWidthInput(key);
      break;
    case RUNNING:
    case STATUS_DISPLAY:
      handleRunningInput(key);
      break;
  }
}

void handleMainMenuInput(char key) {
  switch (key) {
    case '1': selectedOption = 0; showMainMenu(); break;
    case '2': selectedOption = 1; showMainMenu(); break;
    case '3': selectedOption = 2; showMainMenu(); break;
    case '4': selectedOption = 3; showMainMenu(); break;
    case '5': selectedOption = 4; showMainMenu(); break;
    case '6': selectedOption = 5; showMainMenu(); break;
    case '7': selectedOption = 6; showMainMenu(); break;
    case '8': selectedOption = 7; showMainMenu(); break;
    case '#':
      // Start selected mode
      if (selectedOption == 0) {
        // Line navigation - show options
        inputPhase = 0;
        showStraightInput();
      } else if (selectedOption == 1) {
        // Rectangle navigation
        inputBuffer = "";
        rectLength = 0; rectWidth = 0; inputPhase = 0;
        showRectangleInput();
      } else if (selectedOption == 2) {
        // Square navigation
        inputBuffer = "";
        squareSide = 0;
        showSquareInput();
      } else if (selectedOption == 3) {
        // Turn Left
        Serial2.println("TURN:left");
        robotRunning = true;
        showTurnMode("left");
      } else if (selectedOption == 4) {
        // Turn Right
        Serial2.println("TURN:right");
        robotRunning = true;
        showTurnMode("right");
      } else if (selectedOption == 5) {
        // Start Paint
        manualPaintMode = true;
        startPaint();
        showPaintStatus();
      } else if (selectedOption == 6) {
        // Mix Paint
        startMixing();
        showPaintStatus();
      } else if (selectedOption == 7) {
        // Line Width
        inputPhase = 0;
        showLineWidthInput();
      }
      break;
    case '0':
      emergencyStop();
      break;
  }
}

void handleStraightInputMenu(char key) {
  switch (key) {
    case '1':
      inputPhase = 0;
      showStraightInput();
      break;
    case '2':
      inputPhase = 1;
      showStraightInput();
      break;
    case '#':
      if (inputPhase == 0) {
        // Normal straight - show paint options
        showPaintOptions("Straight Line");
        inputPhase = 0;
      } else if (inputPhase == 1) {
        // Pulse-based straight - go to pulse input
        inputBuffer = "";
        showPulseInput();
      }
      break;
    case '*':
      showMainMenu();
      break;
    case '0':
      emergencyStop();
      break;
  }
}

void handlePulseInput(char key) {
  if (key >= '0' && key <= '9') {
    if (inputBuffer.length() < 5) {
      inputBuffer += key;
      showPulseInput();
    }
  } else if (key == '#') {
    if (inputBuffer.length() > 0) {
      straightPulses = inputBuffer.toInt();
      if (straightPulses > 0) {
        // Show paint options for pulse-based navigation
        showPaintOptions("Pulse-Based");
        inputPhase = 0;
      } else {
        showError("Invalid pulse count!");
      }
    }
  } else if (key == '*') {
    inputPhase = 1;
    showStraightInput();
  } else if (key == '0' && inputBuffer.length() == 0) {
    emergencyStop();
  }
}

void handleRunningInput(char key) {
  if (key == '0') {
    emergencyStop();
  } else if (key == '*') {
    if (obstacleDetected) {
      // Show override warning
      tft.fillScreen(ORANGE);
      tft.setTextColor(WHITE);
      tft.setTextSize(2);
      tft.setCursor(10, 100);
      tft.println("OBSTACLE DETECTED!");
      tft.setCursor(10, 130);
      tft.println("Cannot proceed");
      delay(2000);
      showObstacleAlert(obstacleDistance);
    } else {
      Serial2.println("STOP");
      stopAllPaintOperations();
      robotRunning = false;
      showMainMenu();
    }
  }
}

void handleRectangleInput(char key) {
  if (key >= '0' && key <= '9') {
    if (inputBuffer.length() < 5) {
      inputBuffer += key;
      showRectangleInput();
    }
  } else if (key == '#') {
    if (inputBuffer.length() > 0) {
      if (inputPhase == 0) {
        rectLength = inputBuffer.toInt();
        if (rectLength > 0) {
          inputBuffer = "";
          inputPhase = 1;
          showRectangleInput();
        } else {
          showError("Invalid length!");
        }
      } else {
        rectWidth = inputBuffer.toInt();
        if (rectWidth > 0) {
          showPaintOptions("Rectangle");
          inputPhase = 0;
        } else {
          showError("Invalid width!");
        }
      }
    }
  } else if (key == '*') {
    if (inputPhase == 1) {
      inputPhase = 0;
      inputBuffer = "";
      showRectangleInput();
    } else {
      showMainMenu();
    }
  } else if (key == '0' && inputBuffer.length() == 0) {
    emergencyStop();
  }
}

void handleSquareInput(char key) {
  if (key >= '0' && key <= '9') {
    if (inputBuffer.length() < 5) {
      inputBuffer += key;
      showSquareInput();
    }
  } else if (key == '#') {
    if (inputBuffer.length() > 0) {
      squareSide = inputBuffer.toInt();
      if (squareSide > 0) {
        showPaintOptions("Square");
        inputPhase = 0;
      } else {
        showError("Invalid side length!");
      }
    }
  } else if (key == '*') {
    showMainMenu();
  } else if (key == '0' && inputBuffer.length() == 0) {
    emergencyStop();
  }
}

void handlePaintOptionsInput(char key) {
  switch (key) {
    case '1':
      inputPhase = 0;
      showPaintOptions("Current");
      break;
    case '2':
      inputPhase = 1;
      showPaintOptions("Current");
      break;
    case '#':
      // Start the selected navigation mode
      paintWithNavigation = (inputPhase == 1);
      startSelectedNavigation();
      break;
    case '*':
      showMainMenu();
      break;
    case '0':
      emergencyStop();
      break;
  }
}

void handleLineWidthInput(char key) {
  switch (key) {
    case '1':
      inputPhase = 0;
      showLineWidthInput();
      break;
    case '2':
      inputPhase = 1;
      showLineWidthInput();
      break;
    case '3':
      inputPhase = 2;
      showLineWidthInput();
      break;
    case '4':
      inputPhase = 3;
      showLineWidthInput();
      break;
    case '#':
      // Apply selected width
      currentLineWidth = lineWidthPresets[inputPhase];
      setLineWidth(currentLineWidth);
      showMainMenu();
      break;
    case '*':
      showMainMenu();
      break;
    case '0':
      emergencyStop();
      break;
  }
}

void startSelectedNavigation() {
   // Start GPS tracking before navigation
  startPathTracking();

  if (selectedOption == 0) {
    // Straight line
    if (currentState == PULSE_INPUT || straightPulses > 0) {
      // Pulse-based straight
      String command = "PULSES:" + String(straightPulses);
      if (paintWithNavigation) command += ",paint";
      Serial2.println(command);
      robotRunning = true;
      String details = String(straightPulses) + " pulses";
      showRunningStatus("Pulse Navigation", details);
    } else {
      // Normal straight
      String command = "STRAIGHT:150,150";
      if (paintWithNavigation) command += ",paint";
      Serial2.println(command);
      robotRunning = true;
      showRunningStatus("Line", "PID Straight Line");
    }
  } else if (selectedOption == 1) {
    // Rectangle
    startRectangle();
  } else if (selectedOption == 2) {
    // Square
    startSquare();
  }
}

// Add obstacle alert display function
void showObstacleAlert(float distance) {
  tft.fillScreen(RED);
  tft.setTextColor(WHITE);
  tft.setTextSize(3);
  
  tft.setCursor(50, 60);
  tft.println("OBSTACLE");
  tft.setCursor(70, 100);
  tft.println("DETECTED!");
  
  tft.setTextSize(2);
  tft.setCursor(10, 150);
  tft.print("Distance: ");
  tft.print(distance, 1);
  tft.println(" cm");
  
  tft.setTextSize(1);
  tft.setCursor(10, 180);
  tft.println("Robot stopped for safety");
  tft.setCursor(10, 195);
  tft.println("Waiting for obstacle to clear...");
  
  tft.setTextColor(YELLOW);
  tft.setCursor(10, 220);
  tft.println("Press 0 for Emergency Stop");
}

// Add obstacle status web handler
void handleObstacleStatus() {
  String json = "{";
  json += "\"obstacleDetected\":" + String(obstacleDetected ? "true" : "false") + ",";
  json += "\"distance\":" + String(obstacleDistance, 1) + ",";
  json += "\"status\":\"" + obstacleStatus + "\",";
  json += "\"navigationPaused\":" + String(navigationPaused ? "true" : "false");
  json += "}";
  
  server.send(200, "application/json", json);
}

// =============== PAINT SYSTEM FUNCTIONS ===============
void startPaint() {
  digitalWrite(PAINT_RELAY_PIN, HIGH);
  paintEnabled = true;
  Serial.println("Paint pump started");
  Serial2.println("PAINT:start");
}

void stopPaint() {
  digitalWrite(PAINT_RELAY_PIN, LOW);
  paintEnabled = false;
  Serial.println("Paint pump stopped");
  Serial2.println("PAINT:stop");
}

void startMixing() {
  mixingEnabled = true;
  Serial.println("Paint mixing started");
  Serial2.println("MIX:start");
}

void stopMixing() {
  digitalWrite(MIXER_IN3, LOW);
  digitalWrite(MIXER_IN4, LOW);
  analogWrite(MIXER_ENB, 0);
  mixingEnabled = false;
  Serial.println("Paint mixing stopped");
  Serial2.println("MIX:stop");
}

void stopAllPaintOperations() {
  stopPaint();
  stopMixing();
  paintWithNavigation = false;
  manualPaintMode = false;
}

// =============== LINE WIDTH CONTROL FUNCTIONS ===============
void setLineWidth(float widthInches) {
  String command = "LINEWIDTH:" + String(widthInches, 2);
  Serial2.println(command);
  Serial.println("Setting line width to: " + String(widthInches, 2) + " inches");
  
  // Update display
  tft.fillRect(0, 205, 320, 15, BLACK);
  tft.setTextSize(1);
  tft.setTextColor(GREEN);
  tft.setCursor(10, 205);
  tft.print("Width set to: ");
  tft.print(widthInches, 1);
  tft.println(" inches");
}

// =============== ROBOT CONTROL FUNCTIONS ===============
void startRectangle() {

  startPathTracking(); // Add this line

  String command = "RECTANGLE:" + String(rectLength) + "," + String(rectWidth) + ",clockwise,120";
  if (paintWithNavigation) command += ",paint";
  Serial2.println(command);
  robotRunning = true;
  
  String details = String(rectLength) + " x " + String(rectWidth) + " pulses";
  showRunningStatus("Rectangle Navigation", details);
}

void startSquare() {
  String command = "RECTANGLE:" + String(squareSide) + "," + String(squareSide) + ",clockwise,120";
  if (paintWithNavigation) command += ",paint";
  Serial2.println(command);
  robotRunning = true;
  
  String details = String(squareSide) + " x " + String(squareSide) + " pulses";
  showRunningStatus("Square Navigation", details);
}

void emergencyStop() {
  Serial2.println("STOP");
  stopAllPaintOperations();

  stopPathTracking(); // Add this line

  robotRunning = false;
  
  tft.fillScreen(RED);
  tft.setTextColor(WHITE);
  tft.setTextSize(3);
  tft.setCursor(80, 100);
  tft.println("EMERGENCY");
  tft.setCursor(120, 140);
  tft.println("STOP!");
  
  delay(2000);
  showMainMenu();
}

void requestStatus() {
  Serial2.println("STATUS");
}

// =============== ARDUINO RESPONSE HANDLING ===============
void handleArduinoResponse(String response) {

  if (response.startsWith("OBSTACLE_DETECTED:")) {
    float distance = response.substring(18).toFloat();
    obstacleDetected = true;
    obstacleDistance = distance;
    navigationPaused = true;
    obstacleStatus = "Detected";
    showObstacleAlert(distance);
    currentStatus = "Obstacle Detected - Robot Paused";
    Serial.print("ESP32: Obstacle detected at ");
    Serial.print(distance);
    Serial.println(" cm");
    return; // Exit early
  }
  else if (response == "OBSTACLE_CLEARED") {
    obstacleDetected = false;
    obstacleDistance = 999.0;
    navigationPaused = false;
    obstacleStatus = "Clear";
    
    if (robotRunning) {
      if (selectedOption == 1) {
        String details = String(rectLength) + " x " + String(rectWidth) + " pulses";
        showRunningStatus("Rectangle Navigation", details);
      } else if (selectedOption == 2) {
        String details = String(squareSide) + " x " + String(squareSide) + " pulses";
        showRunningStatus("Square Navigation", details);
      } else {
        showRunningStatus("Navigation Resumed");
      }
    } else {
      showMainMenu();
    }
    currentStatus = "Obstacle Cleared - Navigation Resumed";
    Serial.println("ESP32: Obstacle cleared - navigation resumed");
    return; // Exit early
  }

  if (response.startsWith("STATUS:")) {
    parseStatus(response);
  } else if (response == "RECTANGLE_STARTED") {
    currentStatus = "Rectangle Started";
  } else if (response.startsWith("TURNED:")) {
    currentStatus = "Turn Complete";
  } else if (response == "STOPPED") {
    robotRunning = false;
    stopAllPaintOperations();

     stopPathTracking(); // Add this line

    currentStatus = "Stopped";
    showMainMenu();
  } else if (response == "PULSES_COMPLETE") {
    robotRunning = false;
    stopAllPaintOperations();

    stopPathTracking(); // Add this line

    currentStatus = "Pulse Navigation Complete";
    showMainMenu();
  }
  
  // Update display if in running mode
  if (currentState == STATUS_DISPLAY && robotRunning) {
    tft.fillRect(0, 90, 320, 20, BLACK);
    tft.setTextColor(CYAN);
    tft.setTextSize(2);
    tft.setCursor(10, 90);
    tft.println("Status: " + currentStatus);
  }

}

void parseStatus(String statusStr) {
  if (statusStr.indexOf("RECTANGLE") >= 0) {
    currentStatus = "Rectangle Running";
  } else if (statusStr.indexOf("STRAIGHT") >= 0) {
    currentStatus = "Going Straight";
  } else if (statusStr.indexOf("TURNING") >= 0) {
    currentStatus = "Turning";
  } else if (statusStr.indexOf("PULSES") >= 0) {
    currentStatus = "Pulse Navigation";
  } else {
    currentStatus = "Ready";
  }
}

// =============== WEB SERVER FUNCTIONS ===============
void handleRoot() {
  ///
  String html = "<!DOCTYPE html><html><head>";
  html += "<title>Robot Control System</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>";
  
  // Professional CSS Styling
  html += "* { margin: 0; padding: 0; box-sizing: border-box; }";
  html += "body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); min-height: 100vh; padding: 20px; }";
  html += ".container { max-width: 1200px; margin: 0 auto; background: white; border-radius: 15px; box-shadow: 0 20px 40px rgba(0,0,0,0.1); overflow: hidden; }";
  html += ".header { background: linear-gradient(135deg, #2c3e50 0%, #34495e 100%); color: white; padding: 30px; text-align: center; }";
  html += ".header h1 { font-size: 2.5em; font-weight: 300; margin-bottom: 10px; }";
  html += ".header p { font-size: 1.1em; opacity: 0.9; }";
  html += ".content { padding: 30px; }";
  
  // Section Styling
  html += ".section { margin-bottom: 30px; padding: 25px; border-radius: 10px; border: 1px solid #e1e8ed; background: #fafbfc; transition: all 0.3s ease; }";
  html += ".section:hover { transform: translateY(-2px); box-shadow: 0 8px 25px rgba(0,0,0,0.1); }";
  html += ".section h3 { color: #2c3e50; font-size: 1.4em; margin-bottom: 20px; padding-bottom: 10px; border-bottom: 2px solid #3498db; display: inline-block; }";
  
  // Button Styling
  html += "button { padding: 12px 24px; margin: 6px; font-size: 14px; font-weight: 500; border: none; border-radius: 6px; cursor: pointer; transition: all 0.3s ease; text-transform: uppercase; letter-spacing: 0.5px; }";
  html += "button:hover { transform: translateY(-2px); box-shadow: 0 4px 12px rgba(0,0,0,0.2); }";
  html += "button:active { transform: translateY(0px); }";
  
  // Button Color Classes
  html += ".btn-emergency { background: linear-gradient(135deg, #e74c3c, #c0392b); color: white; font-size: 16px; padding: 15px 30px; }";
  html += ".btn-emergency:hover { background: linear-gradient(135deg, #c0392b, #a93226); }";
  html += ".btn-primary { background: linear-gradient(135deg, #3498db, #2980b9); color: white; }";
  html += ".btn-primary:hover { background: linear-gradient(135deg, #2980b9, #21618c); }";
  html += ".btn-success { background: linear-gradient(135deg, #27ae60, #229954); color: white; }";
  html += ".btn-success:hover { background: linear-gradient(135deg, #229954, #1e8449); }";
  html += ".btn-warning { background: linear-gradient(135deg, #f39c12, #e67e22); color: white; }";
  html += ".btn-warning:hover { background: linear-gradient(135deg, #e67e22, #d35400); }";
  html += ".btn-info { background: linear-gradient(135deg, #9b59b6, #8e44ad); color: white; }";
  html += ".btn-info:hover { background: linear-gradient(135deg, #8e44ad, #7d3c98); }";
  html += ".btn-secondary { background: linear-gradient(135deg, #95a5a6, #7f8c8d); color: white; }";
  html += ".btn-secondary:hover { background: linear-gradient(135deg, #7f8c8d, #566573); }";
  
  // Input Styling
  html += "input[type='range'] { width: 200px; height: 6px; background: #ddd; border-radius: 3px; outline: none; margin: 10px; }";
  html += "input[type='range']::-webkit-slider-thumb { appearance: none; width: 18px; height: 18px; background: #3498db; border-radius: 50%; cursor: pointer; }";
  html += "input[type='number'] { width: 80px; padding: 8px; margin: 5px; border: 2px solid #bdc3c7; border-radius: 4px; text-align: center; }";
  html += "input[type='number']:focus { border-color: #3498db; outline: none; }";
  
  // Control Group Styling
  html += ".control-group { display: inline-block; margin: 15px; padding: 20px; border: 2px solid #ecf0f1; border-radius: 8px; background: white; vertical-align: top; min-width: 200px; }";
  html += ".control-group h4 { color: #2c3e50; margin-bottom: 15px; text-align: center; }";
  html += ".control-group .value { font-weight: bold; color: #3498db; font-size: 16px; }";
  
  // Status Display
  html += ".status-display { background: #34495e; color: white; padding: 20px; border-radius: 8px; margin: 15px 0; }";
  html += ".status-display h4 { color: #ecf0f1; margin-bottom: 10px; }";
  html += ".status-info { background: #2980b9; padding: 10px; border-radius: 4px; margin: 5px 0; font-family: monospace; }";
  
  // Special Sections
  html += ".emergency-section { border-color: #e74c3c; background: #fdf2f2; }";
  html += ".emergency-section h3 { color: #e74c3c; border-bottom-color: #e74c3c; }";
  html += ".paint-section { border-color: #e91e63; background: #fce4ec; }";
  html += ".paint-section h3 { color: #e91e63; border-bottom-color: #e91e63; }";
  html += ".navigation-section { border-color: #27ae60; background: #f1f8e9; }";
  html += ".navigation-section h3 { color: #27ae60; border-bottom-color: #27ae60; }";
  
  // Responsive Grid
  html += ".grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 20px; }";
  html += ".grid-item { background: white; padding: 20px; border-radius: 8px; border: 1px solid #e1e8ed; }";
  
  // Radio and Checkbox Styling
  html += "input[type='radio'], input[type='checkbox'] { margin: 0 8px 0 15px; transform: scale(1.2); }";
  html += "label { margin-right: 15px; color: #2c3e50; font-weight: 500; }";
  
  // Utility Classes
  html += ".text-center { text-align: center; }";
  html += ".mb-10 { margin-bottom: 10px; }";
  html += ".mb-20 { margin-bottom: 20px; }";
  html += ".highlight { background: #f7dc6f; padding: 2px 6px; border-radius: 3px; }";
  
  html += "</style></head><body>";
  
  // Header Section
  html += "<div class='container'>";
  html += "<div class='header'>";
  html += "<h1>Robot Control System</h1>";
  html += "<p>Advanced Autonomous Ground Painting Robot</p>";
  html += "</div>";
  
  html += "<div class='content'>";
  
  // Emergency Control
  html += "<div class='section emergency-section'>";
  html += "<h3>Emergency Control</h3>";
  html += "<div class='text-center'>";
  html += "<button class='btn-emergency' onclick='stopMotors()'>EMERGENCY STOP</button>";
  html += "</div>";
  html += "</div>";
  
  // Line Width Control
  html += "<div class='section'>";
  html += "<h3>Line Width Control</h3>";
  html += "<div class='control-group'>";
  html += "<h4>Current Width</h4>";
  html += "<div class='value'><span id='currentWidth'>" + String(currentLineWidth, 1) + "</span> inches</div>";
  html += "</div>";
  html += "<div class='control-group'>";
  html += "<h4>Custom Width</h4>";
  html += "<label>Width (1.5-4.0 inches):</label>";
  html += "<input type='number' id='customWidth' value='" + String(currentLineWidth, 1) + "' min='1.5' max='4.0' step='0.1'>";
  html += "<button class='btn-info' onclick='setCustomWidth()'>Apply Custom</button>";
  html += "</div>";
  html += "<div class='text-center mb-20'>";
  html += "<h4>Quick Presets</h4>";
  html += "<button class='btn-secondary' onclick='setPresetWidth(1.5)'>1.5 inch</button>";
  html += "<button class='btn-secondary' onclick='setPresetWidth(2.0)'>2.0 inch</button>";
  html += "<button class='btn-secondary' onclick='setPresetWidth(3.0)'>3.0 inch</button>";
  html += "<button class='btn-secondary' onclick='setPresetWidth(4.0)'>4.0 inch</button>";
  html += "</div>";
  html += "</div>";

  // Paint System Control
  html += "<div class='section paint-section'>";
  html += "<h3>Paint System Control</h3>";
  html += "<div class='grid'>";
  html += "<div class='grid-item'>";
  html += "<h4>Paint Pump</h4>";
  html += "<button class='btn-warning' onclick='startPaint()'>Start Paint</button>";
  html += "<button class='btn-emergency' onclick='stopPaint()'>Stop Paint</button>";
  html += "</div>";
  html += "<div class='grid-item'>";
  html += "<h4>Paint Mixer</h4>";
  html += "<button class='btn-info' onclick='startMix()'>Start Mixer</button>";
  html += "<button class='btn-emergency' onclick='stopMix()'>Stop Mixer</button>";
  html += "</div>";
  html += "</div>";
  html += "</div>";

  // Manual Navigation Controls
  html += "<div class='section navigation-section'>";
  html += "<h3>Manual Navigation</h3>";
  html += "<div class='control-group'>";
  html += "<h4>Speed Control</h4>";
  html += "<div>Speed: <span class='value' id='manualSpeedValue'>120</span></div>";
  html += "<input type='range' id='manualSpeedSlider' min='80' max='200' value='120' oninput='updateManualSpeed()'>";
  html += "</div>";
  html += "<div class='text-center'>";
  html += "<div class='mb-10'>";
  html += "<button class='btn-primary' onclick='startManual(\"forward\")'>Forward</button>";
  html += "</div>";
  html += "<div class='mb-10'>";
  html += "<button class='btn-primary' onclick='startManual(\"left\")'>Turn Left</button>";
  html += "<button class='btn-emergency' onclick='stopMotors()'>Stop</button>";
  html += "<button class='btn-primary' onclick='startManual(\"right\")'>Turn Right</button>";
  html += "</div>";
  html += "<div>";
  html += "<button class='btn-primary' onclick='startManual(\"backward\")'>Backward</button>";
  html += "</div>";
  html += "</div>";
  html += "</div>";
  
  // Straight Line Control
  html += "<div class='section'>";
  html += "<h3>Straight Line Control</h3>";
  html += "<div class='grid'>";
  html += "<div class='grid-item'>";
  html += "<h4>Left Motor PWM</h4>";
  html += "<div>PWM: <span class='value' id='leftPwmValue'>150</span></div>";
  html += "<input type='range' id='leftPwmSlider' min='50' max='255' value='150' oninput='updateLeftPwm()'>";
  html += "</div>";
  html += "<div class='grid-item'>";
  html += "<h4>Right Motor PWM</h4>";
  html += "<div>PWM: <span class='value' id='rightPwmValue'>150</span></div>";
  html += "<input type='range' id='rightPwmSlider' min='50' max='255' value='150' oninput='updateRightPwm()'>";
  html += "</div>";
  html += "</div>";
  html += "<div class='text-center'>";
  html += "<button class='btn-success' onclick='goStraight()'>Go Straight</button>";
  html += "<button class='btn-warning' onclick='goStraightWithPaint()'>Go Straight + Paint</button>";
  html += "</div>";
  html += "</div>";

  // Pulse-Based Navigation
  html += "<div class='section'>";
  html += "<h3>Pulse-Based Navigation</h3>";
  html += "<div class='control-group'>";
  html += "<h4>Target Pulses</h4>";
  html += "<label>Pulses:</label>";
  html += "<input type='number' id='targetPulses' value='10' min='1' max='100'>";
  html += "</div>";
  html += "<div class='text-center'>";
  html += "<button class='btn-success' onclick='goPulses()'>Go by Pulses</button>";
  html += "<button class='btn-warning' onclick='goPulsesWithPaint()'>Go by Pulses + Paint</button>";
  html += "</div>";
  html += "</div>";
  
  // Turn Control
  html += "<div class='section'>";
  html += "<h3>Turn Control</h3>";
  html += "<div class='text-center'>";
  html += "<button class='btn-primary' onclick='turnLeft()'>Turn Left 90°</button>";
  html += "<button class='btn-primary' onclick='turnRight()'>Turn Right 90°</button>";
  html += "</div>";
  html += "</div>";
  
  // Shape Navigation
  html += "<div class='section'>";
  html += "<h3>Shape Navigation</h3>";
  html += "<div class='text-center mb-20'>";
  html += "<button class='btn-secondary' onclick='selectShape(\"rectangle\")' id='rectBtn'>Rectangle Mode</button>";
  html += "<button class='btn-secondary' onclick='selectShape(\"square\")' id='squareBtn'>Square Mode</button>";
  html += "</div>";
  html += "<div id='rectangleInputs'>";
  html += "<div class='control-group'>";
  html += "<label>Length (pulses):</label>";
  html += "<input type='number' id='rectLength' value='4' min='1' max='20'>";
  html += "<label>Width (pulses):</label>";
  html += "<input type='number' id='rectWidth' value='2' min='1' max='20'>";
  html += "</div>";
  html += "</div>";
  html += "<div id='squareInputs' style='display: none;'>";
  html += "<div class='control-group'>";
  html += "<label>Side Length (pulses):</label>";
  html += "<input type='number' id='squareSide' value='4' min='1' max='20'>";
  html += "</div>";
  html += "</div>";
  html += "<div class='control-group'>";
  html += "<label>Direction:</label>";
  html += "<input type='radio' id='clockwise' name='direction' value='clockwise' checked>";
  html += "<label for='clockwise'>Clockwise</label>";
  html += "<input type='radio' id='anticlockwise' name='direction' value='anticlockwise'>";
  html += "<label for='anticlockwise'>Anti-clockwise</label>";
  html += "</div>";
  html += "<div class='control-group'>";
  html += "<label>Base Speed:</label>";
  html += "<input type='number' id='rectSpeed' value='120' min='80' max='200'>";
  html += "</div>";
  html += "<div class='control-group'>";
  html += "<input type='checkbox' id='withPaint'>";
  html += "<label for='withPaint'>Enable Painting (forward movement only)</label>";
  html += "</div>";
  html += "<div class='text-center'>";
  html += "<button class='btn-success' onclick='startShape()' id='startShapeBtn'>Start Rectangle</button>";
  html += "</div>";
  html += "</div>";
  
  // PID Settings
  html += "<div class='section'>";
  html += "<h3>PID Settings</h3>";
  html += "<div class='grid'>";
  html += "<div class='grid-item'>";
  html += "<label>Kp:</label>";
  html += "<input type='number' id='kp' value='2.0' step='0.1' min='0' max='10'>";
  html += "</div>";
  html += "<div class='grid-item'>";
  html += "<label>Ki:</label>";
  html += "<input type='number' id='ki' value='0.1' step='0.01' min='0' max='2'>";
  html += "</div>";
  html += "<div class='grid-item'>";
  html += "<label>Kd:</label>";
  html += "<input type='number' id='kd' value='0.5' step='0.1' min='0' max='5'>";
  html += "</div>";
  html += "</div>";
  html += "<div class='text-center'>";
  html += "<button class='btn-info' onclick='setPID()'>Apply PID Settings</button>";
  html += "</div>";
  html += "</div>";
  
  // Status Section
  html += "<div class='section'>";
  html += "<h3>System Status</h3>";
  html += "<div class='status-display'>";
  html += "<div class='status-info' id='status'>Click Get Status</div>";
  html += "<div class='status-info' id='paintStatus'></div>";
  html += "<div class='status-info' id='lineWidthStatus'></div>";
  html += "<div class='status-info' id='paintLevelStatus'></div>";
  html += "<div class='status-info' id='turnInfo'></div>";
  html += "</div>";
  html += "<div class='text-center'>";
  html += "<button class='btn-secondary' onclick='getStatus()'>Refresh Status</button>";
  html += "</div>";
  html += "</div>";

  // Obstacle Detection Section
  html += "<div class='section'>";
  html += "<h3>Obstacle Detection System</h3>";
  html += "<div class='status-info' id='obstacleStatus'>System Active - No Obstacles</div>";
  html += "<div class='grid'>";
  html += "<div class='grid-item'>";
  html += "<h4>Sensor 1 Distance</h4>";
  html += "<div class='value'><span id='distance1'>-- cm</span></div>";
  html += "</div>";
  html += "<div class='grid-item'>";
  html += "<h4>Sensor 2 Distance</h4>";
  html += "<div class='value'><span id='distance2'>-- cm</span></div>";
  html += "</div>";
  html += "<div class='grid-item'>";
  html += "<h4>Minimum Distance</h4>";
  html += "<div class='value'><span id='minDistance'>-- cm</span></div>";
  html += "</div>";
  html += "</div>";
  html += "<div class='highlight'>Detection Range: 20cm | Auto-pause and resume navigation</div>";
  html += "<div class='status-info' id='navigationStatus'></div>";
  html += "</div>";

  // GPS Path Tracking
  html += "<div class='section'>";
  html += "<h3>GPS Path Tracking</h3>";
  html += "<div class='status-info'>GPS Status: <span id='gpsState'>Inactive</span></div>";
  html += "<div class='text-center'>";
  html += "<canvas id='pathCanvas' width='600' height='400' style='border: 2px solid #bdc3c7; background: #f8f9fa; border-radius: 8px; max-width: 100%;'></canvas>";
  html += "</div>";
  html += "<div class='highlight'>Green dot: Start position | Red dot: Current position | Blue line: Robot path</div>";
  html += "</div>";
  
  html += "</div>"; // Close content
  html += "</div>"; // Close container
  
  // JavaScript (UPDATED)
  html += "<script>";
  html += "let selectedShape = 'rectangle';";
  html += "function selectShape(shape) {";
  html += "  selectedShape = shape;";
  html += "  if (shape === 'rectangle') {";
  html += "    document.getElementById('rectangleInputs').style.display = 'block';";
  html += "    document.getElementById('squareInputs').style.display = 'none';";
  html += "    document.getElementById('startShapeBtn').innerHTML = 'GO RECTANGLE';";
  html += "    document.getElementById('rectBtn').style.background = '#4CAF50';";
  html += "    document.getElementById('squareBtn').style.background = '#FF9800';";
  html += "  } else {";
  html += "    document.getElementById('rectangleInputs').style.display = 'none';";
  html += "    document.getElementById('squareInputs').style.display = 'block';";
  html += "    document.getElementById('startShapeBtn').innerHTML = 'GO SQUARE';";
  html += "    document.getElementById('rectBtn').style.background = '#FF9800';";
  html += "    document.getElementById('squareBtn').style.background = '#4CAF50';";
  html += "  }";
  html += "}";
  
  // Paint System Functions
  html += "function startPaint() { fetch('/paint?action=start'); }";
  html += "function stopPaint() { fetch('/paint?action=stop'); }";
  html += "function startMix() { fetch('/mix?action=start'); }";
  html += "function stopMix() { fetch('/mix?action=stop'); }";
  
  // Line Width Functions (NEW)
  html += "function setCustomWidth() { const width = parseFloat(document.getElementById('customWidth').value); if (width >= 1.5 && width <= 4.0) { fetch('/linewidth?width=' + width); document.getElementById('currentWidth').innerHTML = width.toFixed(1); } else { alert('Width must be between 1.5 and 4.0 inches'); } }";
  html += "function setPresetWidth(width) { fetch('/linewidth?width=' + width); document.getElementById('currentWidth').innerHTML = width.toFixed(1); document.getElementById('customWidth').value = width; }";
  
  // Updated Shape Function
  html += "function startShape() {";
  html += "  let length, width;";
  html += "  if (selectedShape === 'square') {";
  html += "    const side = parseInt(document.getElementById('squareSide').value);";
  html += "    length = side; width = side;";
  html += "  } else {";
  html += "    length = parseInt(document.getElementById('rectLength').value);";
  html += "    width = parseInt(document.getElementById('rectWidth').value);";
  html += "  }";
  html += "  const direction = document.querySelector('input[name=\"direction\"]:checked').value;";
  html += "  const speed = parseInt(document.getElementById('rectSpeed').value);";
  html += "  const withPaint = document.getElementById('withPaint').checked;";
  html += "  fetch('/rectangle?length=' + length + '&width=' + width + '&dir=' + direction + '&speed=' + speed + '&paint=' + withPaint);";
  html += "}";

  // Updated Straight Functions
  html += "function goStraight() { const leftPwm = parseInt(document.getElementById('leftPwmSlider').value); const rightPwm = parseInt(document.getElementById('rightPwmSlider').value); fetch('/straight?left=' + leftPwm + '&right=' + rightPwm); }";
  html += "function goStraightWithPaint() { const leftPwm = parseInt(document.getElementById('leftPwmSlider').value); const rightPwm = parseInt(document.getElementById('rightPwmSlider').value); fetch('/straight?left=' + leftPwm + '&right=' + rightPwm + '&paint=true'); }";

  // New Pulse Functions
  html += "function goPulses() { const pulses = parseInt(document.getElementById('targetPulses').value); fetch('/pulses?count=' + pulses); }";
  html += "function goPulsesWithPaint() { const pulses = parseInt(document.getElementById('targetPulses').value); fetch('/pulses?count=' + pulses + '&paint=true'); }";
  
  // Other existing functions
  html += "function updateManualSpeed() { document.getElementById('manualSpeedValue').innerHTML = document.getElementById('manualSpeedSlider').value; }";
  html += "function updateLeftPwm() { document.getElementById('leftPwmValue').innerHTML = document.getElementById('leftPwmSlider').value; }";
  html += "function updateRightPwm() { document.getElementById('rightPwmValue').innerHTML = document.getElementById('rightPwmSlider').value; }";
  html += "function startManual(direction) { const speed = parseInt(document.getElementById('manualSpeedSlider').value); fetch('/manual?dir=' + direction + '&speed=' + speed); }";
  html += "function turnLeft() { fetch('/turn?direction=left'); }";
  html += "function turnRight() { fetch('/turn?direction=right'); }";
  html += "function setPID() { const kp = parseFloat(document.getElementById('kp').value); const ki = parseFloat(document.getElementById('ki').value); const kd = parseFloat(document.getElementById('kd').value); fetch('/pid?kp=' + kp + '&ki=' + ki + '&kd=' + kd); }";
  html += "function stopMotors() { fetch('/stop'); }";
  html += "function setLED(pin) { if(pin == 0) { fetch('/led?cmd=OFF'); } else { fetch('/led?cmd=' + pin); } }";
  html += "function getStatus() { fetch('/status').then(function(response) { return response.text(); }).then(function(data) { document.getElementById('status').innerHTML = data; updatePaintStatus(); }); }";
  html += "function updatePaintStatus() { fetch('/status').then(function(response) { return response.text(); }).then(function(data) { if(data.includes('PAINT:ON')) { document.getElementById('paintStatus').innerHTML = '🎨 Paint: ACTIVE'; } else { document.getElementById('paintStatus').innerHTML = '🎨 Paint: OFF'; } if(data.includes('MIX:ON')) { document.getElementById('paintStatus').innerHTML += ' | 🔄 Mixer: ACTIVE'; } else { document.getElementById('paintStatus').innerHTML += ' | 🔄 Mixer: OFF'; } if(data.includes('WIDTH:')) { const widthMatch = data.match(/WIDTH:([0-9.]+)/); if(widthMatch) { document.getElementById('lineWidthStatus').innerHTML = '🎯 Line Width: ' + widthMatch[1] + '\"'; } } if(data.includes('TANK:')) { const tankMatch = data.match(/TANK:([A-Z]+)/); if(tankMatch) { let color = tankMatch[1] === 'GREEN' ? '#4CAF50' : tankMatch[1] === 'YELLOW' ? '#FF9800' : '#f44336'; document.getElementById('paintLevelStatus').innerHTML = '<span style=\"color:' + color + '\">🪣 Tank: ' + tankMatch[1] + '</span>'; } } }); }";
  html += "setInterval(getStatus, 2000); getStatus();";

 
html += "let pathCanvas = null;";
html += "let pathCtx = null;";
html += "let minLat = 999, maxLat = -999, minLon = 999, maxLon = -999;";

html += "function initGPSCanvas() {";
html += "  pathCanvas = document.getElementById(\"pathCanvas\");";
html += "  if (pathCanvas) {";
html += "    pathCtx = pathCanvas.getContext(\"2d\");";
html += "  }";
html += "}";

html += "function updateGPSPath() {";
html += "  fetch(\"/gps\").then(response => response.json()).then(data => {";
html += "    document.getElementById(\"gpsState\").innerHTML = data.totalPoints > 0 ? \"Active (\" + data.totalPoints + \" points)\" : \"Waiting for fix...\";";
html += "    if (data.pathPoints && data.pathPoints.length > 0 && pathCtx) {";
html += "      drawPath(data);";
html += "    }";
html += "  }).catch(err => {";
html += "    document.getElementById(\"gpsState\").innerHTML = \"Error\";";
html += "  });";
html += "}";

html += "function drawPath(data) {";
html += "  pathCtx.clearRect(0, 0, pathCanvas.width, pathCanvas.height);";
html += "  if (data.pathPoints.length < 2) return;";
html += "  minLat = Math.min(...data.pathPoints.map(p => p.lat));";
html += "  maxLat = Math.max(...data.pathPoints.map(p => p.lat));";
html += "  minLon = Math.min(...data.pathPoints.map(p => p.lon));";
html += "  maxLon = Math.max(...data.pathPoints.map(p => p.lon));";
html += "  const latPadding = (maxLat - minLat) * 0.1;";
html += "  const lonPadding = (maxLon - minLon) * 0.1;";
html += "  minLat -= latPadding; maxLat += latPadding;";
html += "  minLon -= lonPadding; maxLon += lonPadding;";
html += "  pathCtx.strokeStyle = \"#2196F3\";";
html += "  pathCtx.lineWidth = 2;";
html += "  pathCtx.beginPath();";
html += "  data.pathPoints.forEach((point, index) => {";
html += "    const x = ((point.lon - minLon) / (maxLon - minLon)) * pathCanvas.width;";
html += "    const y = pathCanvas.height - ((point.lat - minLat) / (maxLat - minLat)) * pathCanvas.height;";
html += "    if (index === 0) {";
html += "      pathCtx.moveTo(x, y);";
html += "      pathCtx.fillStyle = \"#4CAF50\";";
html += "      pathCtx.beginPath();";
html += "      pathCtx.arc(x, y, 6, 0, 2 * Math.PI);";
html += "      pathCtx.fill();";
html += "      pathCtx.strokeStyle = \"#2196F3\";";
html += "      pathCtx.beginPath();";
html += "      pathCtx.moveTo(x, y);";
html += "    } else {";
html += "      pathCtx.lineTo(x, y);";
html += "    }";
html += "  });";
html += "  pathCtx.stroke();";
html += "  if (data.pathPoints.length > 1) {";
html += "    const lastPoint = data.pathPoints[data.pathPoints.length - 1];";
html += "    const x = ((lastPoint.lon - minLon) / (maxLon - minLon)) * pathCanvas.width;";
html += "    const y = pathCanvas.height - ((lastPoint.lat - minLat) / (maxLat - minLat)) * pathCanvas.height;";
html += "    pathCtx.fillStyle = \"#f44336\";";
html += "    pathCtx.beginPath();";
html += "    pathCtx.arc(x, y, 5, 0, 2 * Math.PI);";
html += "    pathCtx.fill();";
html += "  }";
html += "}";

html += "setTimeout(initGPSCanvas, 1000);";
html += "setInterval(updateGPSPath, 3000);";


// Obstacle Detection JavaScript
html += "function updateObstacleStatus() {";
html += "  fetch('/obstacle').then(response => response.json()).then(data => {";
html += "    const statusDiv = document.getElementById('obstacleStatus');";
html += "    const navStatusDiv = document.getElementById('navigationStatus');";
html += "    ";
html += "    if (data.obstacleDetected) {";
html += "      statusDiv.className = 'obstacle-alert';";
html += "      statusDiv.innerHTML = '🚨 OBSTACLE DETECTED! Distance: ' + data.distance + ' cm';";
html += "      navStatusDiv.innerHTML = '⏸️ Navigation PAUSED for safety';";
html += "      navStatusDiv.style.color = '#f44336';";
html += "    } else {";
html += "      statusDiv.className = 'obstacle-clear';";
html += "      statusDiv.innerHTML = '✅ Path Clear - System Active';";
html += "      if (data.navigationPaused) {";
html += "        navStatusDiv.innerHTML = '▶️ Navigation RESUMING...';";
html += "        navStatusDiv.style.color = '#FF9800';";
html += "      } else {";
html += "        navStatusDiv.innerHTML = '✅ Navigation Active';";
html += "        navStatusDiv.style.color = '#4CAF50';";
html += "      }";
html += "    }";
html += "  }).catch(err => {";
html += "    document.getElementById('obstacleStatus').innerHTML = '❌ Sensor Error';";
html += "  });";
html += "}";
html += "setInterval(updateObstacleStatus, 500);";
html += "updateObstacleStatus();";

  html += "</script></body></html>";
  
  server.send(200, "text/html", html);
}

// New Paint System Handlers
void handlePaint() {
  if (server.hasArg("action")) {
    String action = server.arg("action");
    
    if (action == "start") {
      startPaint();
      server.send(200, "text/plain", "Paint started");
    } else if (action == "stop") {
      stopPaint();
      server.send(200, "text/plain", "Paint stopped");
    } else {
      server.send(400, "text/plain", "Invalid paint action");
    }
  } else {
    server.send(400, "text/plain", "Missing action parameter");
  }
}

void handleMix() {
  if (server.hasArg("action")) {
    String action = server.arg("action");
    
    if (action == "start") {
      startMixing();
      server.send(200, "text/plain", "Mixing started");
    } else if (action == "stop") {
      stopMixing();
      server.send(200, "text/plain", "Mixing stopped");
    } else {
      server.send(400, "text/plain", "Invalid mix action");
    }
  } else {
    server.send(400, "text/plain", "Missing action parameter");
  }
}

// New Line Width Handler
void handleLineWidth() {
  if (server.hasArg("width")) {
    float width = server.arg("width").toFloat();
    
    if (width >= 1.5 && width <= 4.0) {
      currentLineWidth = width;
      setLineWidth(width);
      server.send(200, "text/plain", "Line width set to " + String(width, 2) + " inches");
    } else {
      server.send(400, "text/plain", "Width must be between 1.5 and 4.0 inches");
    }
  } else {
    server.send(400, "text/plain", "Missing width parameter");
  }
}

// Updated Pulse Handler (NEW)
void handlePulses() {
  if (server.hasArg("count")) {
    int pulseCount = server.arg("count").toInt();
    bool withPaint = server.hasArg("paint") && server.arg("paint") == "true";
    
    String command = "PULSES:" + String(pulseCount);
    if (withPaint) {
      command += ",paint";
      paintWithNavigation = true;
    }
    
    Serial2.println(command);
    Serial.println("Sent: " + command);
    
    server.send(200, "text/plain", "Pulse navigation started: " + String(pulseCount) + " pulses" + (withPaint ? " with paint" : ""));
  } else {
    server.send(400, "text/plain", "Missing pulse count parameter");
  }
}

// Keep all existing web server handlers but update them for paint support
void handleMove() {
  if (server.hasArg("left") && server.hasArg("right")) {
    int leftSpeed = server.arg("left").toInt();
    int rightSpeed = server.arg("right").toInt();
    
    String command = "MOVE:" + String(leftSpeed) + "," + String(rightSpeed);
    Serial2.println(command);
    Serial.println("Sent: " + command);
    
    server.send(200, "text/plain", "Moving: L=" + String(leftSpeed) + " R=" + String(rightSpeed));
  } else {
    server.send(400, "text/plain", "Missing speed parameters");
  }
}

void handleStraight() {
  if (server.hasArg("left") && server.hasArg("right")) {
    int leftPwm = server.arg("left").toInt();
    int rightPwm = server.arg("right").toInt();
    bool withPaint = server.hasArg("paint") && server.arg("paint") == "true";
    
    String command = "STRAIGHT:" + String(leftPwm) + "," + String(rightPwm);
    if (withPaint) {
      command += ",paint";
      paintWithNavigation = true;
    }
    
    Serial2.println(command);
    Serial.println("Sent: " + command);
    
    server.send(200, "text/plain", "Going straight: L=" + String(leftPwm) + " R=" + String(rightPwm) + (withPaint ? " with paint" : ""));
  } else {
    server.send(400, "text/plain", "Missing PWM parameters");
  }
}

void handleTurn() {
  if (server.hasArg("direction")) {
    String direction = server.arg("direction");
    
    String command = "TURN:" + direction;
    Serial2.println(command);
    Serial.println("Sent: " + command);
    
    server.send(200, "text/plain", "Turning " + direction + " 90 degrees");
  } else {
    server.send(400, "text/plain", "Missing direction parameter");
  }
}

void handlePID() {
  if (server.hasArg("kp") && server.hasArg("ki") && server.hasArg("kd")) {
    float kp = server.arg("kp").toFloat();
    float ki = server.arg("ki").toFloat();
    float kd = server.arg("kd").toFloat();
    
    String command = "PID:" + String(kp) + "," + String(ki) + "," + String(kd);
    Serial2.println(command);
    Serial.println("Sent: " + command);
    
    server.send(200, "text/plain", "PID set: Kp=" + String(kp) + " Ki=" + String(ki) + " Kd=" + String(kd));
  } else {
    server.send(400, "text/plain", "Missing PID parameters");
  }
}

void handleManual() {
  if (server.hasArg("dir") && server.hasArg("speed")) {
    String direction = server.arg("dir");
    int speed = server.arg("speed").toInt();
    
    String command = "MANUAL:" + direction + "," + String(speed);
    Serial2.println(command);
    Serial.println("Sent: " + command);
    
    server.send(200, "text/plain", "Manual " + direction + " at speed " + String(speed));
  } else {
    server.send(400, "text/plain", "Missing manual parameters");
  }
}

void handleRectangle() {
  if (server.hasArg("length") && server.hasArg("width") && server.hasArg("dir") && server.hasArg("speed")) {
    int length = server.arg("length").toInt();
    int width = server.arg("width").toInt();
    String direction = server.arg("dir");
    int speed = server.arg("speed").toInt();
    bool withPaint = server.hasArg("paint") && server.arg("paint") == "true";
    
    String command = "RECTANGLE:" + String(length) + "," + String(width) + "," + direction + "," + String(speed);
    if (withPaint) {
      command += ",paint";
      paintWithNavigation = true;
    }
    
    Serial2.println(command);
    Serial.println("Sent: " + command);
    
    server.send(200, "text/plain", "Starting rectangle: " + String(length) + "x" + String(width) + " " + direction + (withPaint ? " with paint" : ""));
  } else {
    server.send(400, "text/plain", "Missing rectangle parameters");
  }
}

void handleStop() {
  Serial2.println("STOP");
  stopAllPaintOperations();
  Serial.println("Sent: STOP");
  server.send(200, "text/plain", "Stopped");
}

void handleStatus() {
  Serial2.println("STATUS");
  
  // Wait for response
  unsigned long timeout = millis() + 1000;
  String response = "";
  
  while (millis() < timeout) {
    if (Serial2.available()) {
      response = Serial2.readStringUntil('\n');
      response.trim();
      break;
    }
    delay(10);
  }
  
  // Add paint status to response
  String fullStatus = response;
  if (paintEnabled) fullStatus += " PAINT:ON";
  else fullStatus += " PAINT:OFF";
  if (mixingEnabled) fullStatus += " MIX:ON";
  else fullStatus += " MIX:OFF";
  fullStatus += " WIDTH:" + String(currentLineWidth, 1);
  
  if (fullStatus.length() > 0) {
    server.send(200, "text/plain", fullStatus);
  } else {
    server.send(200, "text/plain", "No response from Arduino PAINT:OFF MIX:OFF WIDTH:" + String(currentLineWidth, 1));
  }
}

void handleLED() {
  if (server.hasArg("cmd")) {
    String cmd = server.arg("cmd");
    String command = "LED:" + cmd;
    Serial2.println(command);
    Serial.println("Sent: " + command);
    server.send(200, "text/plain", "LED command sent: " + cmd);
  } else {
    server.send(400, "text/plain", "Missing LED command");
  }
}
// =============== GPS FUNCTIONS ===============
void initGPS() {
  gpsSerial.begin(9600);
  gpsActive = true;
  Serial.println("GPS initialized on TX0/RX0");
}

bool parseGPS(String data) {
  // Simple GPGGA parser for latitude/longitude
  if (data.startsWith("$GPGGA") || data.startsWith("$GNGGA")) {
    int commas[14];
    int commaCount = 0;
    
    // Find comma positions
    for (int i = 0; i < data.length() && commaCount < 14; i++) {
      if (data.charAt(i) == ',') {
        commas[commaCount] = i;
        commaCount++;
      }
    }
    
    if (commaCount >= 6) {
      String latStr = data.substring(commas[1] + 1, commas[2]);
      String latDir = data.substring(commas[2] + 1, commas[3]);
      String lonStr = data.substring(commas[3] + 1, commas[4]);
      String lonDir = data.substring(commas[4] + 1, commas[5]);
      String quality = data.substring(commas[5] + 1, commas[6]);
      
      if (latStr.length() > 0 && lonStr.length() > 0 && quality.toInt() > 0) {
        // Convert DDMM.MMMM to decimal degrees
        float lat = (latStr.substring(0, 2).toFloat() + 
                    latStr.substring(2).toFloat() / 60.0);
        if (latDir == "S") lat = -lat;
        
        float lon = (lonStr.substring(0, 3).toFloat() + 
                    lonStr.substring(3).toFloat() / 60.0);
        if (lonDir == "W") lon = -lon;
        
        currentPosition.latitude = lat;
        currentPosition.longitude = lon;
        currentPosition.timestamp = millis();
        return true;
      }
    }
  }
  return false;
}

void startPathTracking() {
  if (!gpsActive) {
    initGPS();
  }
  
  // Clear previous path
  pathPointCount = 0;
  
  // Try to get initial GPS position (timeout after 30 seconds)
  Serial.println("Getting initial GPS position...");
  unsigned long startTime = millis();
  bool gotInitialFix = false;
  
  while (millis() - startTime < 30000 && !gotInitialFix) {
    readGPSData();
    if (currentPosition.latitude != 0 && currentPosition.longitude != 0) {
      startPosition = currentPosition;
      addPathPoint(startPosition);
      gotInitialFix = true;
      pathTracking = true;
      Serial.println("GPS initial position acquired");
    }
    delay(100);
  }
  
  if (!gotInitialFix) {
    Serial.println("Warning: Could not get initial GPS fix");
  }
}

void readGPSData() {
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    rawGPSData += c;
    
    if (c == '\n') {
      rawGPSData.trim();
      if (parseGPS(rawGPSData)) {
        // Valid GPS data received
        if (pathTracking && millis() - lastGPSRead > 2000) { // Every 2 seconds
          addPathPoint(currentPosition);
          lastGPSRead = millis();
        }
      }
      rawGPSData = "";
    }
  }
}

void addPathPoint(GPSPoint point) {
  if (pathPointCount < 100) {
    pathPoints[pathPointCount] = point;
    pathPointCount++;
  }
}

void stopPathTracking() {
  pathTracking = false;
  Serial.print("Path tracking stopped. Total points: ");
  Serial.println(pathPointCount);
}

String getPathDataJSON() {
  String json = "{";
  json += "\"startLat\":" + String(startPosition.latitude, 7) + ",";
  json += "\"startLon\":" + String(startPosition.longitude, 7) + ",";
  json += "\"currentLat\":" + String(currentPosition.latitude, 7) + ",";
  json += "\"currentLon\":" + String(currentPosition.longitude, 7) + ",";
  json += "\"pathPoints\":[";
  
  for (int i = 0; i < pathPointCount; i++) {
    json += "{";
    json += "\"lat\":" + String(pathPoints[i].latitude, 7) + ",";
    json += "\"lon\":" + String(pathPoints[i].longitude, 7);
    json += "}";
    if (i < pathPointCount - 1) json += ",";
  }
  
  json += "],";
  json += "\"totalPoints\":" + String(pathPointCount);
  json += "}";
  return json;
}

void handleGPS() {
  server.send(200, "application/json", getPathDataJSON());
}

