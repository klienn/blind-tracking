/*
 * Light Tracking Vertical Blind System (Optimized)
 * - ESP32 + 2x ULN2003 + ADS1115 + 4x Photoresistors + 2x 28BYJ-48
 */

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <AccelStepper.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

// Wi-Fi credentials
const char *ssid = "scam ni";
const char *password = "Walakokabalo0123!";

// Web server
AsyncWebServer server(80);

// ADS1115 setup
Adafruit_ADS1115 ads;

// Motor pins - ULN2003 connection to ESP32
#define IN1_1 19
#define IN2_1 18
#define IN3_1 5
#define IN4_1 17
#define IN1_2 16
#define IN2_2 4
#define IN3_2 0
#define IN4_2 2

// Stepper motors
AccelStepper stepper1(AccelStepper::HALF4WIRE, IN1_1, IN3_1, IN2_1, IN4_1);
AccelStepper stepper2(AccelStepper::HALF4WIRE, IN1_2, IN3_2, IN2_2, IN4_2);

// System variables
#define SENSOR_READ_INTERVAL 5000
unsigned long lastSensorReadTime = 0;

// Cached light sensor values
int16_t topLeft = 0, topRight = 0, bottomLeft = 0, bottomRight = 0;

// Light sensor indices for ADS1115
#define TOP_LEFT_SENSOR 0
#define TOP_RIGHT_SENSOR 1
#define BOTTOM_LEFT_SENSOR 2
#define BOTTOM_RIGHT_SENSOR 3

// Motor parameters
#define MAX_MOTOR_SPEED 500
#define MAX_MOTOR_ACCEL 100
#define STEPS_PER_REVOLUTION 2048
int maxSteps = STEPS_PER_REVOLUTION * 2;

int openThreshold = 5000;
int closeThreshold = 20000;

// System modes
enum SystemMode { MODE_AUTO,
                  MODE_MANUAL,
                  MODE_CALIBRATION };
SystemMode currentMode = MODE_AUTO;
SystemMode lastMode = MODE_AUTO;
bool blindsClosed = false;
int manualPosition = 0;

// Function declarations
void setupWiFi();
void setupWebServer();
void readLightSensors();
void autoModeOperation();
void manualModeOperation();
void calibrationModeOperation();
void setMotorTargetPosition(int motorNumber, int percentage);
void closeBlinds();
void openBlinds();

void setup() {
  Serial.begin(115200);
  Serial.println("Light Tracking Vertical Blind System");

  if (!ads.begin()) {
    Serial.println("Could not find ADS1115. Check wiring!");
    while (1)
      ;
  }
  ads.setGain(GAIN_ONE);

  stepper1.setMaxSpeed(MAX_MOTOR_SPEED);
  stepper1.setAcceleration(MAX_MOTOR_ACCEL);
  stepper2.setMaxSpeed(MAX_MOTOR_SPEED);
  stepper2.setAcceleration(MAX_MOTOR_ACCEL);

  setupWiFi();
  setupWebServer();
}

void loop() {
  if (millis() - lastSensorReadTime > SENSOR_READ_INTERVAL) {
    readLightSensors();
    lastSensorReadTime = millis();
  }

  if (currentMode != lastMode) {
    if (currentMode == MODE_CALIBRATION) {
      stepper1.moveTo(0);
      stepper2.moveTo(0);
    }
    lastMode = currentMode;
  }

  switch (currentMode) {
    case MODE_AUTO: autoModeOperation(); break;
    case MODE_MANUAL: manualModeOperation(); break;
    case MODE_CALIBRATION: calibrationModeOperation(); break;
  }

  stepper1.run();
  stepper2.run();
}

void setupWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 5) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connection failed. Running in offline mode.");
  }
}

void setupWebServer() {
  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    DynamicJsonDocument doc(1024);
    doc["mode"] = currentMode == MODE_AUTO ? "auto" : (currentMode == MODE_MANUAL ? "manual" : "calibration");
    doc["blinds_closed"] = blindsClosed;
    doc["manual_position"] = manualPosition;
    JsonArray sensors = doc.createNestedArray("light_sensors");
    sensors.add(topLeft);
    sensors.add(topRight);
    sensors.add(bottomLeft);
    sensors.add(bottomRight);
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });

  server.on("/mode", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (request->hasParam("value", true)) {
      String modeStr = request->getParam("value", true)->value();
      if (modeStr == "auto") currentMode = MODE_AUTO;
      else if (modeStr == "manual") currentMode = MODE_MANUAL;
      else if (modeStr == "calibration") currentMode = MODE_CALIBRATION;
      request->send(200, "text/plain", "Mode updated");
    } else {
      request->send(400, "text/plain", "Missing mode value");
    }
  });

  server.on("/position", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (request->hasParam("value", true)) {
      manualPosition = request->getParam("value", true)->value().toInt();
      manualPosition = constrain(manualPosition, 0, 100);
      request->send(200, "text/plain", "Position updated");
    } else {
      request->send(400, "text/plain", "Missing position value");
    }
  });

  server.on("/settings", HTTP_GET, [](AsyncWebServerRequest *request) {
    DynamicJsonDocument doc(512);
    doc["open_threshold"] = openThreshold;
    doc["close_threshold"] = closeThreshold;
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });

  server.on("/settings", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (request->hasParam("open", true)) {
      openThreshold = request->getParam("open", true)->value().toInt();
    }
    if (request->hasParam("close", true)) {
      closeThreshold = request->getParam("close", true)->value().toInt();
    }
    request->send(200, "text/plain", "Settings updated");
  });

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    String html = R"rawliteral(
      <!DOCTYPE html>
      <html>
      <head><title>Vertical Blind Control</title><meta name="viewport" content="width=device-width, initial-scale=1">
      <style>body { font-family: Arial; margin: 0; padding: 20px; } .card { background: #f0f0f0; padding: 20px; border-radius: 10px; margin-bottom: 20px; } .slider { width: 100%; } button { padding: 10px; margin: 5px; cursor: pointer; } .active { background: #4CAF50; color: white; }</style>
      </head>
      <body>
      <h1>Light Tracking Vertical Blind System</h1>
      <div class="card">
        <h2>Mode Selection</h2>
        <button id="autoBtn" onclick="setMode('auto')">Automatic</button>
        <button id="manualBtn" onclick="setMode('manual')">Manual</button>
        <button id="calibBtn" onclick="setMode('calibration')">Calibration</button>
      </div>
      <div class="card">
        <h2>Manual Control</h2>
        <input type="range" min="0" max="100" value="50" class="slider" id="positionSlider">
        <p>Position: <span id="positionValue">50</span>%</p>
        <button onclick="setPosition()">Set Position</button>
      </div>
      <div class="card">
        <h2>System Status</h2>
        <p>Current Mode: <span id="currentMode">-</span></p>
        <p>Blinds Status: <span id="blindsStatus">-</span></p>
        <p>Light Sensors:</p>
        <ul>
          <li>Top Left: <span id="sensor0">-</span></li>
          <li>Top Right: <span id="sensor1">-</span></li>
          <li>Bottom Left: <span id="sensor2">-</span></li>
          <li>Bottom Right: <span id="sensor3">-</span></li>
        </ul>
      </div>
      <div class="card">
        <h2>Brightness Threshold Settings</h2>
        <label>Open Threshold: <input type="number" id="openThreshold" value="5000"></label><br>
        <label>Close Threshold: <input type="number" id="closeThreshold" value="20000"></label><br><br>
        <button onclick="updateSettings()">Update Settings</button>
      </div>
      <script>
        const slider = document.getElementById("positionSlider");
        const posValue = document.getElementById("positionValue");
        slider.oninput = function() { posValue.innerHTML = this.value; }
        function setMode(mode) {
          fetch('/mode', { method: 'POST', headers: { 'Content-Type': 'application/x-www-form-urlencoded' }, body: 'value=' + mode }).then(r => { if (r.ok) updateStatus(); });
        }
        function setPosition() {
          fetch('/position', { method: 'POST', headers: { 'Content-Type': 'application/x-www-form-urlencoded' }, body: 'value=' + slider.value }).then(r => { if (r.ok) updateStatus(); });
        }
        function updateSettings() {
          const openVal = document.getElementById("openThreshold").value;
          const closeVal = document.getElementById("closeThreshold").value;
          fetch('/settings', { method: 'POST', headers: { 'Content-Type': 'application/x-www-form-urlencoded' }, body: 'open=' + openVal + '&close=' + closeVal }).then(r => { if (r.ok) updateStatus(); });
        }
        function updateStatus() {
          fetch('/status')
            .then(r => r.json())
            .then(data => {
              document.getElementById("currentMode").textContent = data.mode;
              document.getElementById("blindsStatus").textContent = data.blinds_closed ? "Closed" : "Open " + data.manual_position + "%";
              document.getElementById("autoBtn").className = data.mode === "auto" ? "active" : "";
              document.getElementById("manualBtn").className = data.mode === "manual" ? "active" : "";
              document.getElementById("calibBtn").className = data.mode === "calibration" ? "active" : "";
              slider.value = data.manual_position;
              posValue.innerHTML = data.manual_position;
              for (let i = 0; i < 4; i++) document.getElementById("sensor" + i).textContent = data.light_sensors[i];
            });

          fetch('/settings')
            .then(r => r.json())
            .then(data => {
              document.getElementById("openThreshold").value = data.open_threshold;
              document.getElementById("closeThreshold").value = data.close_threshold;
            });
        }
        updateStatus();
        setInterval(updateStatus, 5000);
      </script>
      </body>
      </html>
    )rawliteral";
    request->send(200, "text/html", html);
  });

  server.begin();
}

void readLightSensors() {
  topLeft = ads.readADC_SingleEnded(TOP_LEFT_SENSOR);
  topRight = ads.readADC_SingleEnded(TOP_RIGHT_SENSOR);
  bottomLeft = ads.readADC_SingleEnded(BOTTOM_LEFT_SENSOR);
  bottomRight = ads.readADC_SingleEnded(BOTTOM_RIGHT_SENSOR);

  Serial.println("Light sensor readings:");
  Serial.print("Top Left: ");
  Serial.println(topLeft);
  Serial.print("Top Right: ");
  Serial.println(topRight);
  Serial.print("Bottom Left: ");
  Serial.println(bottomLeft);
  Serial.print("Bottom Right: ");
  Serial.println(bottomRight);
}

void autoModeOperation() {
  int avgLight = (topLeft + topRight + bottomLeft + bottomRight) / 4;

  if (avgLight > 20000) {
    if (!blindsClosed) {
      closeBlinds();
      blindsClosed = true;
    }
    return;
  }
  if (avgLight < 5000) {
    if (blindsClosed) {
      openBlinds();
      blindsClosed = false;
    }
    return;
  }

  int leftAvg = (topLeft + bottomLeft) / 2;
  int rightAvg = (topRight + bottomRight) / 2;
  int leftPosition = constrain(map(leftAvg, 5000, 20000, 0, 100), 0, 100);
  int rightPosition = constrain(map(rightAvg, 5000, 20000, 0, 100), 0, 100);

  setMotorTargetPosition(1, leftPosition);
  setMotorTargetPosition(2, rightPosition);

  blindsClosed = false;
}

void manualModeOperation() {
  setMotorTargetPosition(1, manualPosition);
  setMotorTargetPosition(2, manualPosition);
}

void calibrationModeOperation() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    if (input.startsWith("steps:")) {
      maxSteps = input.substring(6).toInt();
      Serial.print("Max steps set to: ");
      Serial.println(maxSteps);
    }
  }
}

void setMotorTargetPosition(int motorNumber, int percentage) {
  long targetSteps = map(percentage, 0, 100, 0, maxSteps);
  if (motorNumber == 1 && stepper1.currentPosition() != targetSteps) {
    stepper1.moveTo(targetSteps);
  } else if (motorNumber == 2 && stepper2.currentPosition() != targetSteps) {
    stepper2.moveTo(targetSteps);
  }
}

void closeBlinds() {
  setMotorTargetPosition(1, 100);
  setMotorTargetPosition(2, 100);
}
void openBlinds() {
  setMotorTargetPosition(1, 0);
  setMotorTargetPosition(2, 0);
}
