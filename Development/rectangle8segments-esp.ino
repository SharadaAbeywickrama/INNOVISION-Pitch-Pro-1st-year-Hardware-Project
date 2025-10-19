// ESP32 - Extended Robot Control with Magnetometer & Gyroscope
#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "RobotControl";
const char* password = "12345678";

WebServer server(80);

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  
  WiFi.softAP(ssid, password);
  Serial.println("WiFi AP started: " + String(WiFi.softAPIP()));
  
  server.on("/", handleRoot);
  server.on("/move", handleMove);
  server.on("/stop", handleStop);
  server.on("/straight", handleStraight);
  server.on("/turn", handleTurn);
  server.on("/pid", handlePID);
  server.on("/manual", handleManual);
  server.on("/rectangle", handleRectangle);
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
  html += "<title>Robot Control with Sensors</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>";
  html += "body { font-family: Arial; text-align: center; margin: 20px; background: #f0f0f0; }";
  html += ".container { max-width: 700px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; }";
  html += ".section { border: 2px solid #ddd; margin: 15px 0; padding: 15px; border-radius: 10px; }";
  html += "button { padding: 12px 20px; margin: 5px; font-size: 14px; border: none; border-radius: 8px; cursor: pointer; font-weight: bold; }";
  html += ".stop { background: #f44336; color: white; }";
  html += ".straight { background: #4CAF50; color: white; }";
  html += ".turn { background: #2196F3; color: white; }";
  html += ".apply { background: #9C27B0; color: white; }";
  html += ".status-btn { background: #795548; color: white; }";
  html += "input[type='range'] { width: 180px; margin: 5px; }";
  html += "input[type='number'] { width: 80px; padding: 5px; margin: 3px; }";
  html += ".motor-control { display: inline-block; margin: 15px; border: 1px solid #ccc; padding: 10px; border-radius: 5px; }";
  html += ".pid-control { display: inline-block; margin: 10px; border: 1px solid #ddd; padding: 10px; border-radius: 5px; }";
  html += "</style></head><body>";
  
  html += "<div class='container'>";
  html += "<h1>Robot Control with Sensors</h1>";
  
  // Emergency Stop
  html += "<div class='section'>";
  html += "<h3>Emergency Control</h3>";
  html += "<button class='stop' onclick='stopMotors()'>STOP ALL</button>";
  html += "</div>";
  
  // Manual Navigation Controls
  html += "<div class='section'>";
  html += "<h3>Manual Navigation</h3>";
  html += "<div>Speed: <span id='manualSpeedValue'>120</span></div>";
  html += "<input type='range' id='manualSpeedSlider' min='80' max='200' value='120' oninput='updateManualSpeed()'>";
  html += "<br><br>";
  html += "<button class='straight' onclick='startManual(\"forward\")'>FORWARD</button><br>";
  html += "<button class='turn' onclick='startManual(\"left\")'>LEFT</button>";
  html += "<button class='stop' onclick='stopMotors()'>STOP</button>";
  html += "<button class='turn' onclick='startManual(\"right\")'>RIGHT</button><br>";
  html += "<button class='straight' onclick='startManual(\"backward\")'>BACKWARD</button>";
  html += "</div>";
  
  // Straight Line Control
  html += "<div class='section'>";
  html += "<h3>Straight Line Control</h3>";
  html += "<div class='motor-control'>";
  html += "<h4>Left Motor Base PWM</h4>";
  html += "<div>PWM: <span id='leftPwmValue'>150</span></div>";
  html += "<input type='range' id='leftPwmSlider' min='50' max='255' value='150' oninput='updateLeftPwm()'>";
  html += "</div>";
  
  html += "<div class='motor-control'>";
  html += "<h4>Right Motor Base PWM</h4>";
  html += "<div>PWM: <span id='rightPwmValue'>150</span></div>";
  html += "<input type='range' id='rightPwmSlider' min='50' max='255' value='150' oninput='updateRightPwm()'>";
  html += "</div>";
  
  html += "<br><button class='straight' onclick='goStraight()'>GO STRAIGHT</button>";
  html += "</div>";
  
  // Turn Control
  html += "<div class='section'>";
  html += "<h3>Turn Control (90 degrees)</h3>";
  html += "<button class='turn' onclick='turnLeft()'>TURN LEFT 90°</button>";
  html += "<button class='turn' onclick='turnRight()'>TURN RIGHT 90°</button>";
  html += "</div>";
  
  // Rectangle Navigation
  html += "<div class='section'>";
  html += "<h3>Rectangle Navigation</h3>";
  html += "<div style='margin: 10px 0;'>";
  html += "<label>Length (pulses): </label>";
  html += "<input type='number' id='rectLength' value='4' min='1' max='20' style='width: 60px;'>";
  html += "<label style='margin-left: 15px;'>Width (pulses): </label>";
  html += "<input type='number' id='rectWidth' value='2' min='1' max='20' style='width: 60px;'>";
  html += "</div>";
  html += "<div style='margin: 10px 0;'>";
  html += "<label>Direction: </label>";
  html += "<input type='radio' id='clockwise' name='direction' value='clockwise' checked>";
  html += "<label for='clockwise'>Clockwise</label>";
  html += "<input type='radio' id='anticlockwise' name='direction' value='anticlockwise' style='margin-left: 15px;'>";
  html += "<label for='anticlockwise'>Anti-clockwise</label>";
  html += "</div>";
  html += "<div style='margin: 10px 0;'>";
  html += "<label>Base Speed: </label>";
  html += "<input type='number' id='rectSpeed' value='120' min='80' max='200' style='width: 60px;'>";
  html += "</div>";
  html += "<button class='straight' onclick='startRectangle()'>GO RECTANGLE</button>";
  html += "</div>";
  
  // PID Settings
  html += "<div class='section'>";
  html += "<h3>PID Settings</h3>";
  html += "<div class='pid-control'>";
  html += "<label>Kp: </label><input type='number' id='kp' value='2.0' step='0.1' min='0' max='10'>";
  html += "</div>";
  html += "<div class='pid-control'>";
  html += "<label>Ki: </label><input type='number' id='ki' value='0.1' step='0.01' min='0' max='2'>";
  html += "</div>";
  html += "<div class='pid-control'>";
  html += "<label>Kd: </label><input type='number' id='kd' value='0.5' step='0.1' min='0' max='5'>";
  html += "</div>";
  html += "<br><button class='apply' onclick='setPID()'>SET PID</button>";
  html += "</div>";
  
  // Individual Motor Control (backup)
  html += "<div class='section'>";
  html += "<h3>Manual Motor Control</h3>";
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
  
  html += "<br><button class='apply' onclick='applyMotorSpeeds()'>APPLY MANUAL SPEEDS</button>";
  html += "</div>";
  
  // Status
  html += "<div class='section'>";
  html += "<h3>Status</h3>";
  html += "<div id='status'>Click Get Status</div>";
  html += "<br><button class='status-btn' onclick='getStatus()'>GET STATUS</button>";
  html += "</div>";
  
  html += "</div>";
  
  // JavaScript
  html += "<script>";
  html += "let manualInterval;";
  html += "function updateManualSpeed() { document.getElementById('manualSpeedValue').innerHTML = document.getElementById('manualSpeedSlider').value; }";
  html += "function updateLeftPwm() { document.getElementById('leftPwmValue').innerHTML = document.getElementById('leftPwmSlider').value; }";
  html += "function updateRightPwm() { document.getElementById('rightPwmValue').innerHTML = document.getElementById('rightPwmSlider').value; }";
  html += "function updateLeftSpeed() { document.getElementById('leftSpeedValue').innerHTML = document.getElementById('leftSpeedSlider').value; }";
  html += "function updateRightSpeed() { document.getElementById('rightSpeedValue').innerHTML = document.getElementById('rightSpeedSlider').value; }";
  html += "function setLeftSpeed(speed) { document.getElementById('leftSpeedSlider').value = speed; updateLeftSpeed(); }";
  html += "function setRightSpeed(speed) { document.getElementById('rightSpeedSlider').value = speed; updateRightSpeed(); }";
  
  html += "function startManual(direction) {";
  html += "  const speed = parseInt(document.getElementById('manualSpeedSlider').value);";
  html += "  fetch('/manual?dir=' + direction + '&speed=' + speed);";
  html += "}";
  html += "function stopManual() { fetch('/stop'); }";
  
  html += "function goStraight() {";
  html += "  const leftPwm = parseInt(document.getElementById('leftPwmSlider').value);";
  html += "  const rightPwm = parseInt(document.getElementById('rightPwmSlider').value);";
  html += "  fetch('/straight?left=' + leftPwm + '&right=' + rightPwm);";
  html += "}";
  
  html += "function turnLeft() { fetch('/turn?direction=left'); }";
  html += "function turnRight() { fetch('/turn?direction=right'); }";
  
  html += "function startRectangle() {";
  html += "  const length = parseInt(document.getElementById('rectLength').value);";
  html += "  const width = parseInt(document.getElementById('rectWidth').value);";
  html += "  const direction = document.querySelector('input[name=\"direction\"]:checked').value;";
  html += "  const speed = parseInt(document.getElementById('rectSpeed').value);";
  html += "  fetch('/rectangle?length=' + length + '&width=' + width + '&dir=' + direction + '&speed=' + speed);";
  html += "}";
  
  html += "function setPID() {";
  html += "  const kp = parseFloat(document.getElementById('kp').value);";
  html += "  const ki = parseFloat(document.getElementById('ki').value);";
  html += "  const kd = parseFloat(document.getElementById('kd').value);";
  html += "  fetch('/pid?kp=' + kp + '&ki=' + ki + '&kd=' + kd);";
  html += "}";
  
  html += "function applyMotorSpeeds() {";
  html += "  const leftSpeed = parseInt(document.getElementById('leftSpeedSlider').value);";
  html += "  const rightSpeed = parseInt(document.getElementById('rightSpeedSlider').value);";
  html += "  fetch('/move?left=' + leftSpeed + '&right=' + rightSpeed);";
  html += "}";
  
  html += "function stopMotors() { clearInterval(manualInterval); fetch('/stop'); }";
  html += "function getStatus() { fetch('/status').then(function(response) { return response.text(); }).then(function(data) { document.getElementById('status').innerHTML = data; }); }";
  html += "setInterval(getStatus, 2000); getStatus();";
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

void handleStraight() {
  if (server.hasArg("left") && server.hasArg("right")) {
    int leftPwm = server.arg("left").toInt();
    int rightPwm = server.arg("right").toInt();
    
    String command = "STRAIGHT:" + String(leftPwm) + "," + String(rightPwm);
    Serial2.println(command);
    Serial.println("Sent: " + command);
    
    server.send(200, "text/plain", "Going straight: L=" + String(leftPwm) + " R=" + String(rightPwm));
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
    
    String command = "RECTANGLE:" + String(length) + "," + String(width) + "," + direction + "," + String(speed);
    Serial2.println(command);
    Serial.println("Sent: " + command);
    
    server.send(200, "text/plain", "Starting rectangle: " + String(length) + "x" + String(width) + " " + direction);
  } else {
    server.send(400, "text/plain", "Missing rectangle parameters");
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