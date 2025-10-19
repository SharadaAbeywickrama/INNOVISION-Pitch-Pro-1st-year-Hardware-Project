#include <WiFi.h>
#include <WebServer.h>
#include <TFT_eSPI.h>
#include <Keypad.h>
#include <SoftwareSerial.h>  // Use EspSoftwareSerial library
#include <TinyGPS++.h>

// TFT pin definitions
#define TFT_MISO 19
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS   15
#define TFT_DC   2
#define TFT_RST  4

// Arduino Uno uses hardware Serial (TX0/RX0)
// GPS pins for SoftwareSerial - Using completely free GPIO pins
#define GPS_RX_PIN 13  // ESP32 RX for GPS (GPS TX connects here) 
#define GPS_TX_PIN 21  // ESP32 TX for GPS (GPS RX connects here)

// Access Point credentials
const char* ssid = "ESP32-TFT-Control";
const char* password = "12345678";

// Keypad setup
const byte ROWS = 4;
const byte COLS = 3;
char keys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};
byte rowPins[ROWS] = {25, 33, 32, 22}; // R1, R2, R3, R4
byte colPins[COLS] = {14, 27, 26};     // C1, C2, C3

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

TFT_eSPI tft = TFT_eSPI();
WebServer server(80);
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);  // ESP32 SoftwareSerial
TinyGPSPlus gps;

// Current display color, LED state, and GPS data
uint16_t currentColor = TFT_BLACK;
bool ledState = false;
int displayMode = 0; // 0=color, 1=LED control, 2=GPS display

// GPS data variables
float latitude = 0.0;
float longitude = 0.0;
float altitude = 0.0;
float speed_kmh = 0.0;
int satellites = 0;
String gpsTime = "";
String gpsDate = "";
bool gpsValid = false;

// HTML web page
const char* htmlPage = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>ESP32 TFT & LED Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial; text-align: center; margin: 20px; background: #f0f0f0; }
        .container { max-width: 800px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 4px 6px rgba(0,0,0,0.1); }
        .color-btn { 
            width: 80px; 
            height: 50px; 
            margin: 5px; 
            border: none; 
            color: white; 
            font-weight: bold; 
            cursor: pointer;
            border-radius: 5px;
        }
        .led-btn {
            background: #4CAF50;
            color: white;
            padding: 15px 30px;
            margin: 10px;
            border: none;
            border-radius: 5px;
            cursor: pointer;
            font-size: 18px;
            font-weight: bold;
        }
        .led-btn:hover { background: #45a049; }
        .led-btn.off { background: #f44336; }
        .led-btn.off:hover { background: #da190b; }
        .control-btn {
            background: #2196F3;
            color: white;
            padding: 10px 20px;
            margin: 10px;
            border: none;
            border-radius: 5px;
            cursor: pointer;
            font-size: 16px;
        }
        .control-btn:hover { background: #1976D2; }
        .status-panel { 
            background: #f9f9f9; 
            padding: 15px; 
            margin: 15px 0; 
            border-radius: 5px; 
            border: 1px solid #ddd;
        }
        h1 { color: #333; margin-bottom: 30px; }
        h2 { color: #666; margin-top: 30px; }
        .status { font-weight: bold; font-size: 18px; }
        .led-on { color: #4CAF50; }
        .led-off { color: #f44336; }
    </style>
</head>
<body>
    <div class="container">
        <h1>ESP32 TFT & Arduino LED Control</h1>
        
        <h2>Arduino LED Control</h2>
        <div class="status-panel">
            <div class="status">LED Status: <span id="ledStatus" class="led-off">OFF</span></div>
        </div>
        
        <button id="ledToggle" class="led-btn" onclick="toggleLED()">Turn LED ON</button>
        <button class="led-btn" onclick="blinkLED()">Blink LED</button>
        <br>
        <button class="control-btn" onclick="showLEDDisplay()">Show LED Control on Display</button>
        <button class="control-btn" onclick="showColorDisplay()">Show Colors on Display</button>
        
        <h2>GPS Data</h2>
        <div class="status-panel">
            <div class="status">GPS Status: <span id="gpsStatus" class="led-off">Loading...</span></div>
            <div><strong>Latitude:</strong> <span id="latitude">--</span></div>
            <div><strong>Longitude:</strong> <span id="longitude">--</span></div>
            <div><strong>Altitude:</strong> <span id="altitude">--</span> m</div>
            <div><strong>Speed:</strong> <span id="speed">--</span> km/h</div>
            <div><strong>Satellites:</strong> <span id="satellites">--</span></div>
            <div><strong>Time:</strong> <span id="gpsTime">--</span></div>
            <div><strong>Date:</strong> <span id="gpsDate">--</span></div>
        </div>
        
        <button class="control-btn" onclick="showGPSDisplay()">Show GPS on Display</button>
        <button class="control-btn" onclick="refreshGPS()">Refresh GPS Data</button>
        <button class="led-btn" onclick="openGoogleMaps()">Open in Google Maps</button>
        
        <h2>Display Colors</h2>
        <p><strong>Keypad Colors:</strong> 1-Red, 2-Blue, 3-Green, 4-Yellow, 5-Purple, 6-Cyan, 7-White, 8-Black</p>
        <p><strong>Keypad LED:</strong> 9-LED ON, 0-LED OFF, *-Toggle LED, #-LED Display</p>
        <p><strong>Keypad GPS:</strong> A-GPS Display, B-GPS Refresh, C-Location Info, D-Color Mode</p>
        
        <button class="color-btn" style="background-color: red;" onclick="changeColor('red')">RED</button>
        <button class="color-btn" style="background-color: green;" onclick="changeColor('green')">GREEN</button>
        <button class="color-btn" style="background-color: blue;" onclick="changeColor('blue')">BLUE</button>
        <button class="color-btn" style="background-color: yellow; color: black;" onclick="changeColor('yellow')">YELLOW</button>
        <br>
        <button class="color-btn" style="background-color: purple;" onclick="changeColor('purple')">PURPLE</button>
        <button class="color-btn" style="background-color: cyan; color: black;" onclick="changeColor('cyan')">CYAN</button>
        <button class="color-btn" style="background-color: white; color: black;" onclick="changeColor('white')">WHITE</button>
        <button class="color-btn" style="background-color: black;" onclick="changeColor('black')">BLACK</button>
    </div>

    <script>
        function changeColor(color) {
            fetch('/color?c=' + color)
            .then(response => response.text())
            .then(data => console.log('Color changed to: ' + color));
        }
        
        function toggleLED() {
            fetch('/led-toggle')
            .then(response => response.json())
            .then(data => {
                updateLEDStatus(data.state);
            });
        }
        
        function blinkLED() {
            fetch('/led-blink')
            .then(response => response.text())
            .then(data => console.log('LED blink command sent'));
        }
        
        function showLEDDisplay() {
            fetch('/led-display')
            .then(response => response.text())
            .then(data => console.log('LED display mode activated'));
        }
        
        function showColorDisplay() {
            fetch('/color-display')
            .then(response => response.text())
            .then(data => console.log('Color display mode activated'));
        }
        
        function updateLEDStatus(state) {
            const statusElement = document.getElementById('ledStatus');
            const toggleButton = document.getElementById('ledToggle');
            
            if (state) {
                statusElement.textContent = 'ON';
                statusElement.className = 'led-on';
                toggleButton.textContent = 'Turn LED OFF';
                toggleButton.className = 'led-btn off';
            } else {
                statusElement.textContent = 'OFF';
                statusElement.className = 'led-off';
                toggleButton.textContent = 'Turn LED ON';
                toggleButton.className = 'led-btn';
            }
        }
        
        function checkLEDStatus() {
            fetch('/led-status')
            .then(response => response.json())
            .then(data => {
                updateLEDStatus(data.state);
            });
        }
        
        // Check LED status on page load and every 3 seconds
        checkLEDStatus();
        setInterval(checkLEDStatus, 3000);
        
        // Auto-refresh GPS data every 5 seconds
        updateGPSData();
        setInterval(updateGPSData, 5000);
    </script>
</body>
</html>
)rawliteral";

void setup() {
    Serial.begin(9600); // Use hardware Serial for Uno communication
    gpsSerial.begin(9600); // Initialize GPS SoftwareSerial
    
    // Initialize TFT display
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    
    // Display startup message
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    tft.drawString("Starting...", 10, 10);
    
    // Set up Access Point
    WiFi.softAP(ssid, password);
    IPAddress IP = WiFi.softAPIP();
    
    // Update display with AP info
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_GREEN);
    tft.setTextSize(2);
    tft.drawString("WiFi AP Ready", 10, 10);
    tft.setTextSize(1);
    tft.drawString("SSID: " + String(ssid), 10, 40);
    tft.drawString("IP: " + IP.toString(), 10, 60);
    tft.drawString("Arduino: Connected", 10, 80);
    tft.drawString("GPS: Initializing...", 10, 100);
    tft.drawString("Connect & go to IP", 10, 120);
    
    // Web server routes
    server.on("/", handleRoot);
    server.on("/color", handleColorChange);
    server.on("/led-toggle", handleLEDToggle);
    server.on("/led-blink", handleLEDBlink);
    server.on("/led-status", handleLEDStatus);
    server.on("/led-display", handleLEDDisplay);
    server.on("/color-display", handleColorDisplay);
    server.on("/gps-data", handleGPSData);
    server.on("/gps-display", handleGPSDisplay);
    
    server.begin();
    
    delay(2000); // Give Arduino and GPS time to initialize
    
    // Send initial command to Uno
    sendToUno("INIT");
}

void loop() {
    server.handleClient();
    
    // Read GPS data from SoftwareSerial
    while (gpsSerial.available() > 0) {
        if (gps.encode(gpsSerial.read())) {
            updateGPSData();
        }
    }
    
    // Check for keypad input
    char key = keypad.getKey();
    if (key) {
        handleKeypadInput(key);
    }
    
    // Read responses from Uno
    if (Serial.available()) {
        String response = Serial.readStringUntil('\n');
        // Process Uno responses here if needed
        // Note: Since we're using Serial for Uno communication,
        // debug messages won't appear in Serial Monitor
    }
    
    // Update GPS display if in GPS mode
    if (displayMode == 2) {
        static unsigned long lastGPSUpdate = 0;
        if (millis() - lastGPSUpdate > 2000) { // Update every 2 seconds
            displayGPSInfo();
            lastGPSUpdate = millis();
        }
    }
}

void handleRoot() {
    server.send(200, "text/html", htmlPage);
}

void handleColorChange() {
    String colorParam = server.arg("c");
    
    displayMode = 0; // Switch to color mode
    
    // Fixed color mapping based on your display behavior
    if (colorParam == "red") {
        currentColor = TFT_BLUE;
    } else if (colorParam == "green") {
        currentColor = TFT_GREEN;
    } else if (colorParam == "blue") {
        currentColor = TFT_RED;
    } else if (colorParam == "yellow") {
        currentColor = TFT_YELLOW;
    } else if (colorParam == "purple") {
        currentColor = TFT_PURPLE;
    } else if (colorParam == "cyan") {
        currentColor = TFT_GREEN;
    } else if (colorParam == "white") {
        currentColor = TFT_WHITE;
    } else if (colorParam == "black") {
        currentColor = TFT_BLACK;
    }
    
    changeDisplayColor(colorParam);
    // Note: Serial debug messages not available since Serial is used for Uno
    server.send(200, "text/plain", "Color changed to " + colorParam);
}

void handleLEDToggle() {
    ledState = !ledState;
    String command = ledState ? "LED_ON" : "LED_OFF";
    sendToUno(command);
    
    String response = "{\"state\":" + String(ledState ? "true" : "false") + "}";
    server.send(200, "application/json", response);
    
    // Update display if in LED mode
    if (displayMode == 1) {
        displayLEDControl();
    }
}

void handleLEDBlink() {
    sendToUno("LED_BLINK");
    server.send(200, "text/plain", "LED blink command sent");
}

void handleLEDStatus() {
    String response = "{\"state\":" + String(ledState ? "true" : "false") + "}";
    server.send(200, "application/json", response);
}

void handleLEDDisplay() {
    displayMode = 1;
    displayLEDControl();
    server.send(200, "text/plain", "LED display mode activated");
}

void handleColorDisplay() {
    displayMode = 0;
    changeDisplayColor("Color Mode");
    server.send(200, "text/plain", "Color display mode activated");
}

void handleKeypadInput(char key) {
    String colorName = "";
    
    switch(key) {
        case '1':
            displayMode = 0;
            currentColor = TFT_BLUE;
            colorName = "Red";
            changeDisplayColor(colorName);
            break;
        case '2':
            displayMode = 0;
            currentColor = TFT_RED;
            colorName = "Blue";
            changeDisplayColor(colorName);
            break;
        case '3':
            displayMode = 0;
            currentColor = TFT_GREEN;
            colorName = "Green";
            changeDisplayColor(colorName);
            break;
        case '4':
            displayMode = 0;
            currentColor = TFT_YELLOW;
            colorName = "Yellow";
            changeDisplayColor(colorName);
            break;
        case '5':
            displayMode = 0;
            currentColor = TFT_PURPLE;
            colorName = "Purple";
            changeDisplayColor(colorName);
            break;
        case '6':
            displayMode = 0;
            currentColor = TFT_GREEN;
            colorName = "Cyan";
            changeDisplayColor(colorName);
            break;
        case '7':
            displayMode = 0;
            currentColor = TFT_WHITE;
            colorName = "White";
            changeDisplayColor(colorName);
            break;
        case '8':
            displayMode = 0;
            currentColor = TFT_BLACK;
            colorName = "Black";
            changeDisplayColor(colorName);
            break;
        case '9':
            // LED ON
            ledState = true;
            sendToUno("LED_ON");
            if (displayMode == 1) displayLEDControl();
            break;
        case '0':
            // LED OFF
            ledState = false;
            sendToUno("LED_OFF");
            if (displayMode == 1) displayLEDControl();
            break;
        case '*':
            // Toggle LED
            ledState = !ledState;
            sendToUno(ledState ? "LED_ON" : "LED_OFF");
            if (displayMode == 1) displayLEDControl();
            break;
        case '#':
            // Switch to LED display mode
            displayMode = 1;
            displayLEDControl();
            break;
        default:
            return;
    }
    
    // Handle GPS and additional functions with * key combinations
    // Since we can't detect key combinations easily, we'll use A,B,C,D for GPS
    // These would be accessed by holding * then pressing 1,2,3,4 but we'll simulate with A,B,C,D
    // Note: This is for demonstration - actual keypad might not support A,B,C,D
    /*
    switch(key) {
        case 'A': // Simulate with available key if your keypad supports it
            displayMode = 2;
            displayGPSInfo();
            break;
        case 'B':
            // Refresh GPS
            if (displayMode == 2) displayGPSInfo();
            break;
        case 'C':
            // Show brief GPS info
            showGPSStatus();
            break;
        case 'D':
            displayMode = 0;
            changeDisplayColor("Color Mode");
            break;
    }
    */
    
    // Note: Debug messages removed since Serial is used for Uno communication
}

void changeDisplayColor(String colorName) {
    tft.fillScreen(currentColor);
    
    uint16_t textColor = (currentColor == TFT_BLACK || currentColor == TFT_BLUE || currentColor == TFT_RED) ? TFT_WHITE : TFT_BLACK;
    tft.setTextColor(textColor);
    tft.setTextSize(3);
    tft.drawString("Color:", 20, 30);
    tft.drawString(colorName, 20, 70);
    
    tft.setTextSize(1);
    tft.drawString("Colors: 1-8, LED: 9/0/*/#", 10, 120);
    tft.drawString("9-ON 0-OFF *-Toggle #-LED", 10, 135);
}

void displayLEDControl() {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    tft.drawString("LED Control", 10, 10);
    
    // LED Status
    tft.setTextSize(3);
    if (ledState) {
        tft.setTextColor(TFT_GREEN);
        tft.drawString("LED: ON", 20, 50);
    } else {
        tft.setTextColor(TFT_RED);
        tft.drawString("LED: OFF", 20, 50);
    }
    
    // Controls info
    tft.setTextColor(TFT_YELLOW);
    tft.setTextSize(1);
    tft.drawString("Controls:", 10, 100);
    tft.drawString("9 - Turn LED ON", 10, 115);
    tft.drawString("0 - Turn LED OFF", 10, 130);
    tft.drawString("* - Toggle LED", 10, 145);
    tft.drawString("# - LED Display Mode", 10, 160);
}

void sendToUno(String command) {
    Serial.println(command);
    // Note: Debug output not available since Serial is used for Uno communication
}

void updateGPSData() {
    if (gps.location.isValid()) {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
        gpsValid = true;
    }
    
    if (gps.altitude.isValid()) {
        altitude = gps.altitude.meters();
    }
    
    if (gps.speed.isValid()) {
        speed_kmh = gps.speed.kmph();
    }
    
    satellites = gps.satellites.value();
    
    if (gps.time.isValid() && gps.date.isValid()) {
        char timeStr[20];
        char dateStr[20];
        sprintf(timeStr, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
        sprintf(dateStr, "%02d/%02d/%04d", gps.date.day(), gps.date.month(), gps.date.year());
        gpsTime = String(timeStr);
        gpsDate = String(dateStr);
    }
}

void displayGPSInfo() {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    tft.drawString("GPS Data", 10, 5);
    
    tft.setTextSize(1);
    
    // GPS Status
    if (gpsValid) {
        tft.setTextColor(TFT_GREEN);
        tft.drawString("Status: GPS FIX", 10, 30);
    } else {
        tft.setTextColor(TFT_RED);
        tft.drawString("Status: NO FIX", 10, 30);
    }
    
    tft.setTextColor(TFT_WHITE);
    
    // Location
    tft.drawString("Lat: " + String(latitude, 6), 10, 50);
    tft.drawString("Lng: " + String(longitude, 6), 10, 65);
    
    // Additional info
    tft.drawString("Alt: " + String(altitude, 1) + "m", 10, 85);
    tft.drawString("Speed: " + String(speed_kmh, 1) + " km/h", 10, 100);
    tft.drawString("Sats: " + String(satellites), 10, 115);
    
    // Time and date
    tft.drawString("Time: " + gpsTime, 10, 135);
    tft.drawString("Date: " + gpsDate, 10, 150);
    
    // Controls
    tft.setTextColor(TFT_YELLOW);
    tft.drawString("Web interface for control", 10, 170);
}

void showGPSStatus() {
    // Briefly show GPS status, then return to previous mode
    int prevMode = displayMode;
    displayGPSInfo();
    delay(3000); // Show for 3 seconds
    
    if (prevMode == 0) {
        changeDisplayColor("Returned");
    } else if (prevMode == 1) {
        displayLEDControl();
    }
}

void handleGPSData() {
    String json = "{";
    json += "\"valid\":" + String(gpsValid ? "true" : "false") + ",";
    json += "\"latitude\":" + String(latitude, 6) + ",";
    json += "\"longitude\":" + String(longitude, 6) + ",";
    json += "\"altitude\":" + String(altitude, 1) + ",";
    json += "\"speed\":" + String(speed_kmh, 1) + ",";
    json += "\"satellites\":" + String(satellites) + ",";
    json += "\"time\":\"" + gpsTime + "\",";
    json += "\"date\":\"" + gpsDate + "\"";
    json += "}";
    
    server.send(200, "application/json", json);
}

void handleGPSDisplay() {
    displayMode = 2;
    displayGPSInfo();
    server.send(200, "text/plain", "GPS display mode activated");
}