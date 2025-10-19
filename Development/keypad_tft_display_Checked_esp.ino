#include <WiFi.h>
#include <WebServer.h>
#include <TFT_eSPI.h>
#include <Keypad.h>

// TFT pin definitions
#define TFT_MISO 19
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS   15
#define TFT_DC   2
#define TFT_RST  4

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

// Current display color
uint16_t currentColor = TFT_BLACK;

// HTML web page
const char* htmlPage = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>TFT Color Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial; text-align: center; margin: 50px; }
        .color-btn { 
            width: 80px; 
            height: 50px; 
            margin: 10px; 
            border: none; 
            color: white; 
            font-weight: bold; 
            cursor: pointer;
            border-radius: 5px;
        }
        h1 { color: #333; }
    </style>
</head>
<body>
    <h1>ESP32 TFT Display Control</h1>
    <p>Click a button to change the display color:</p>
    <p><strong>Keypad Controls:</strong><br>
    1-Red, 2-Blue, 3-Green, 4-Yellow, 5-Purple, 6-Cyan, 7-White, 8-Black</p>
    
    <button class="color-btn" style="background-color: red;" onclick="changeColor('red')">RED</button>
    <button class="color-btn" style="background-color: green;" onclick="changeColor('green')">GREEN</button>
    <button class="color-btn" style="background-color: blue;" onclick="changeColor('blue')">BLUE</button>
    <br>
    <button class="color-btn" style="background-color: yellow; color: black;" onclick="changeColor('yellow')">YELLOW</button>
    <button class="color-btn" style="background-color: purple;" onclick="changeColor('purple')">PURPLE</button>
    <button class="color-btn" style="background-color: cyan; color: black;" onclick="changeColor('cyan')">CYAN</button>
    <br>
    <button class="color-btn" style="background-color: white; color: black;" onclick="changeColor('white')">WHITE</button>
    <button class="color-btn" style="background-color: black;" onclick="changeColor('black')">BLACK</button>

    <script>
        function changeColor(color) {
            fetch('/color?c=' + color)
            .then(response => response.text())
            .then(data => {
                console.log('Color changed to: ' + color);
            });
        }
    </script>
</body>
</html>
)rawliteral";

void setup() {
    Serial.begin(115200);
    
    // Initialize TFT display
    tft.init();
    tft.setRotation(1); // Adjust rotation as needed
    tft.fillScreen(TFT_BLACK);
    
    // Display startup message
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    tft.drawString("Starting...", 10, 10);
    
    // Set up Access Point
    WiFi.softAP(ssid, password);
    IPAddress IP = WiFi.softAPIP();
    
    Serial.println("Access Point started");
    Serial.print("IP address: ");
    Serial.println(IP);
    
    // Update display with AP info
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_GREEN);
    tft.setTextSize(2);
    tft.drawString("WiFi AP Ready", 10, 10);
    tft.setTextSize(1);
    tft.drawString("SSID: " + String(ssid), 10, 40);
    tft.drawString("IP: " + IP.toString(), 10, 60);
    tft.drawString("Connect & go to IP", 10, 80);
    
    // Web server routes
    server.on("/", handleRoot);
    server.on("/color", handleColorChange);
    
    server.begin();
    Serial.println("Web server started");
}

void loop() {
    server.handleClient();
    
    // Check for keypad input
    char key = keypad.getKey();
    if (key) {
        handleKeypadInput(key);
    }
}

void handleRoot() {
    server.send(200, "text/html", htmlPage);
}

void handleColorChange() {
    String colorParam = server.arg("c");
    
    // Fixed color mapping based on your display behavior
    if (colorParam == "red") {
        currentColor = TFT_BLUE;  // Red shows blue, so use blue for red
    } else if (colorParam == "green") {
        currentColor = TFT_GREEN;
    } else if (colorParam == "blue") {
        currentColor = TFT_RED;   // Blue shows red, so use red for blue
    } else if (colorParam == "yellow") {
        currentColor = TFT_YELLOW;
    } else if (colorParam == "purple") {
        currentColor = TFT_PURPLE;
    } else if (colorParam == "cyan") {
        currentColor = TFT_GREEN; // Cyan shows green, so use green for cyan
    } else if (colorParam == "white") {
        currentColor = TFT_WHITE;
    } else if (colorParam == "black") {
        currentColor = TFT_BLACK;
    }
    
    // Change display color
    changeDisplayColor(colorParam);
    
    Serial.println("Web: Color changed to: " + colorParam);
    server.send(200, "text/plain", "Color changed to " + colorParam);
}

void handleKeypadInput(char key) {
    String colorName = "";
    
    switch(key) {
        case '1':
            currentColor = TFT_BLUE;  // Shows as red on your display
            colorName = "Red";
            break;
        case '2':
            currentColor = TFT_RED;   // Shows as blue on your display
            colorName = "Blue";
            break;
        case '3':
            currentColor = TFT_GREEN;
            colorName = "Green";
            break;
        case '4':
            currentColor = TFT_YELLOW;
            colorName = "Yellow";
            break;
        case '5':
            currentColor = TFT_PURPLE;
            colorName = "Purple";
            break;
        case '6':
            currentColor = TFT_GREEN; // Cyan shows as green on your display
            colorName = "Cyan";
            break;
        case '7':
            currentColor = TFT_WHITE;
            colorName = "White";
            break;
        case '8':
            currentColor = TFT_BLACK;
            colorName = "Black";
            break;
        default:
            return; // Invalid key, do nothing
    }
    
    changeDisplayColor(colorName);
    Serial.println("Keypad: Color changed to: " + colorName + " (Key: " + key + ")");
}

void changeDisplayColor(String colorName) {
    // Change display color
    tft.fillScreen(currentColor);
    
    // Add some text to see the color change better
    uint16_t textColor = (currentColor == TFT_BLACK || currentColor == TFT_BLUE || currentColor == TFT_RED) ? TFT_WHITE : TFT_BLACK;
    tft.setTextColor(textColor);
    tft.setTextSize(3);
    tft.drawString("Color:", 20, 30);
    tft.drawString(colorName, 20, 70);
    
    // Show keypad info
    tft.setTextSize(1);
    tft.drawString("Keys: 1-Red 2-Blue 3-Green", 10, 120);
    tft.drawString("4-Yellow 5-Purple 6-Cyan", 10, 135);
    tft.drawString("7-White 8-Black", 10, 150);
}