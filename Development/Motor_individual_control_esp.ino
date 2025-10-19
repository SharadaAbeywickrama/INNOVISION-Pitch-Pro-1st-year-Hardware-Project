// ESP32 - Simplified Robot Control (Individual Motors + Stop Only)
#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "RobotControl";
const char* password = "12345678";

WebServer server(80);

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);
  
  WiFi.softAP(ssid, password);
  Serial.println("WiFi AP started: " + String(WiFi.softAPIP()));
  
  server.on("/", handleRoot);
  server.on("/move", handleMove);
  server.on("/stop", handleStop);
  server.on("/status", handleStatus);
  
  server.begin();
  Serial.println("Web server started");
}

void loop() {
  server.handleClient();
  
  // Show responses from Arduino
  if (Serial2.available()) {
    String response = Serial2.readStringUntil('\n');
    Serial.println("Arduino: " + response);
  }
  
  delay(10);
}

void handleRoot() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<title>Robot Control</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>";
  html += "body { font-family: Arial; text-align: center; margin: 20px; background: #f0f0f0; }";
  html += ".container { max-width: 600px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; }";
  html += ".section { border: 2px solid #ddd; margin: 15px 0; padding: 15px; border-radius: 10px; }";
  html += "button { padding: 15px 25px; margin: 5px; font-size: 16px; border: none; border-radius: 8px; cursor: pointer; font-weight: bold; }";
  html += ".stop { background: #9E9E9E; color: white; }";
  html += "input[type='range'] { width: 200px; margin: 10px; }";
  html += ".motor-control { display: inline-block; margin: 20px; border: 1px solid #ccc; padding: 15px; border-radius: 5px; }";
  html += ".apply-btn { background: #9C27B0; color: white; }";
  html += ".status-btn { background: #795548; color: white; }";
  html += "</style></head><body>";
  
  html += "<div class='container'>";
  html += "<h1>Simple Robot Control</h1>";
  
  html += "<div class='section'>";
  html += "<h3>Emergency Stop</h3>";
  html += "<button class='stop' onclick='stopMotors()'>STOP ALL MOTORS</button>";
  html += "</div>";
  
  html += "<div class='section'>";
  html += "<h3>Individual Motor Control</h3>";
  html += "<div class='motor-control'>";
  html += "<h4>Left Motor</h4>";
  html += "<div>Speed: <span id='leftSpeedValue'>0</span></div>";
  html += "<input type='range' id='leftSpeedSlider' min='-255' max='255' value='0' oninput='updateLeftSpeed()'>";
  html += "<br><button onclick='setLeftSpeed(150)'>+150</button>";
  html += "<button onclick='setLeftSpeed(0)'>0</button>";
  html += "<button onclick='setLeftSpeed(-150)'>-150</button>";
  html += "</div>";
  
  html += "<div class='motor-control'>";
  html += "<h4>Right Motor</h4>";
  html += "<div>Speed: <span id='rightSpeedValue'>0</span></div>";
  html += "<input type='range' id='rightSpeedSlider' min='-255' max='255' value='0' oninput='updateRightSpeed()'>";
  html += "<br><button onclick='setRightSpeed(150)'>+150</button>";
  html += "<button onclick='setRightSpeed(0)'>0</button>";
  html += "<button onclick='setRightSpeed(-150)'>-150</button>";
  html += "</div>";
  
  html += "<br><button class='apply-btn' onclick='applyMotorSpeeds()'>APPLY SPEEDS</button>";
  html += "</div>";
  
  html += "<div class='section'>";
  html += "<h3>Status</h3>";
  html += "<div id='status'>Click Get Status</div>";
  html += "<br><button class='status-btn' onclick='getStatus()'>GET STATUS</button>";
  html += "</div>";
  
  html += "</div>";
  
  html += "<script>";
  html += "function updateLeftSpeed() { document.getElementById('leftSpeedValue').innerHTML = document.getElementById('leftSpeedSlider').value; }";
  html += "function updateRightSpeed() { document.getElementById('rightSpeedValue').innerHTML = document.getElementById('rightSpeedSlider').value; }";
  html += "function setLeftSpeed(speed) { document.getElementById('leftSpeedSlider').value = speed; updateLeftSpeed(); }";
  html += "function setRightSpeed(speed) { document.getElementById('rightSpeedSlider').value = speed; updateRightSpeed(); }";
  html += "function applyMotorSpeeds() {";
  html += "  const leftSpeed = parseInt(document.getElementById('leftSpeedSlider').value);";
  html += "  const rightSpeed = parseInt(document.getElementById('rightSpeedSlider').value);";
  html += "  sendMove(leftSpeed, rightSpeed);";
  html += "}";
  html += "function sendMove(leftSpeed, rightSpeed) { fetch('/move?left=' + leftSpeed + '&right=' + rightSpeed); }";
  html += "function stopMotors() { fetch('/stop'); }";
  html += "function getStatus() { fetch('/status').then(function(response) { return response.text(); }).then(function(data) { document.getElementById('status').innerHTML = data; }); }";
  html += "setInterval(getStatus, 3000); getStatus();";
  html += "</script></body></html>";
  
  server.send(200, "text/html", html);
}

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

void handleStop() {
  Serial2.println("STOP");
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
  
  if (response.length() > 0) {
    server.send(200, "text/plain", response);
  } else {
    server.send(200, "text/plain", "No response from Arduino");
  }
}