#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <AccelStepper.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

const char* ssid = "Gia";
const char* password = "12345678";
AsyncWebServer server(80);
Adafruit_ADS1115 ads;

// Stepper Motor Pins
#define IN1_1 19
#define IN2_1 18
#define IN3_1 5
#define IN4_1 17
#define IN1_2 16
#define IN2_2 4
#define IN3_2 2
#define IN4_2 15
#define IN1_3 26
#define IN2_3 25
#define IN3_3 33
#define IN4_3 32

#define LIMIT_SWITCH_OPEN 27
#define LIMIT_SWITCH_CLOSED 14

AccelStepper stepper1(AccelStepper::HALF4WIRE, IN1_1, IN3_1, IN2_1, IN4_1);  // tracking
AccelStepper stepper2(AccelStepper::HALF4WIRE, IN1_2, IN3_2, IN2_2, IN4_2);  // blind pull
AccelStepper stepper3(AccelStepper::HALF4WIRE, IN1_3, IN3_3, IN2_3, IN4_3);  // blind pull (reverse)

#define STEPS_PER_REVOLUTION 2048
#define MAX_MOTOR_SPEED 1200
#define MAX_MOTOR_ACCEL 200
#define SENSOR_READ_INTERVAL 5000

int maxStepsMotor1 = STEPS_PER_REVOLUTION * 12;  // 1080 degrees
bool motor1Reverse = true;

const float MOTOR_TO_BLIND_ANGLE_MULTIPLIER = 90.0 / 1080.0;  // mapping from motor rotation to real blind angle

enum SystemMode { MODE_AUTO,
                  MODE_MANUAL,
                  MODE_CALIBRATION };
SystemMode currentMode = MODE_AUTO;

enum BlindDirection { BLIND_NONE,
                      BLIND_OPENING,
                      BLIND_CLOSING };
BlindDirection blindDirection = BLIND_NONE;

String getBlindStatusString() {
  if (isBlindFullyClosed()) return "Closed";
  else if (isBlindFullyOpen()) return "Open";
  else if (blindDirection == BLIND_OPENING) return "Opening";
  else if (blindDirection == BLIND_CLOSING) return "Closing";
  return "Unknown";
}

int16_t ir1 = 0, ir2 = 0, ir3 = 0, ir4 = 0;
int16_t smooth_ir1 = 0, smooth_ir2 = 0, smooth_ir3 = 0, smooth_ir4 = 0;
unsigned long lastSensorReadTime = 0;

bool isBlindFullyClosed() {
  static bool stableState = false;
  static bool lastReading = false;
  static unsigned long lastChangeTime = 0;

  bool reading = digitalRead(LIMIT_SWITCH_CLOSED) == LOW;
  if (reading != lastReading) {
    lastChangeTime = millis();
    lastReading = reading;
  }
  if (millis() - lastChangeTime > 20) stableState = reading;
  return stableState;
}

bool isBlindFullyOpen() {
  static bool stableState = false;
  static bool lastReading = false;
  static unsigned long lastChangeTime = 0;

  bool reading = digitalRead(LIMIT_SWITCH_OPEN) == LOW;
  if (reading != lastReading) {
    lastChangeTime = millis();
    lastReading = reading;
  }
  if (millis() - lastChangeTime > 20) stableState = reading;
  return stableState;
}

void setMotorTargetPosition(int motorNumber, int percentage) {
  long steps = map(percentage, 0, 100, 0, maxStepsMotor1);
  if (motorNumber == 1 && motor1Reverse) steps = maxStepsMotor1 - steps;
  if (motorNumber == 1) stepper1.moveTo(steps);
}

void readLightSensors() {
  const float alpha = 0.4;  // smoothing factor (0 = slow react, 1 = no smoothing)

  // Raw readings
  int16_t raw1 = ads.readADC_SingleEnded(0);
  int16_t raw2 = ads.readADC_SingleEnded(1);
  int16_t raw3 = ads.readADC_SingleEnded(2);
  int16_t raw4 = ads.readADC_SingleEnded(3);

  // Apply exponential smoothing
  smooth_ir1 = alpha * raw1 + (1 - alpha) * smooth_ir1;
  smooth_ir2 = alpha * raw2 + (1 - alpha) * smooth_ir2;
  smooth_ir3 = alpha * raw3 + (1 - alpha) * smooth_ir3;
  smooth_ir4 = alpha * raw4 + (1 - alpha) * smooth_ir4;

  // Keep raw readings for status display
  ir1 = raw1;
  ir2 = raw2;
  ir3 = raw3;
  ir4 = raw4;
}


void autoModeOperation() {
  int total = smooth_ir1 + smooth_ir2 + smooth_ir3 + smooth_ir4;
  if (total == 0) return;

  float weight1 = smooth_ir1 / (float)total;
  float weight2 = smooth_ir2 / (float)total;
  float weight3 = smooth_ir3 / (float)total;
  float weight4 = smooth_ir4 / (float)total;

  float angle = 0;

  if (weight4 > 0.5) {
    angle = 0;  // Fully open
  } else if ((weight2 + weight3) > 0.6) {
    angle = 90;  // Closed (facing sun)
  } else if (weight1 > 0.4) {
    angle = 45;  // Half-closed
  } else {
    float middleWeight = (weight2 + weight3) * 0.5;
    angle = map(middleWeight * 100, 0, 50, 20, 90);
  }

  int anglePercent = round((angle / 90.0) * 100);
  setMotorTargetPosition(1, anglePercent);
}


void calibrationModeOperation() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    if (input.startsWith("steps1:")) maxStepsMotor1 = input.substring(7).toInt();
  }
}

void setupWebServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial; padding: 20px; }
    .card { background: #eee; padding: 20px; margin-bottom: 20px; border-radius: 10px; }
    button { padding: 10px; margin: 5px; }
    .slider { width: 100%; }
  </style>
</head>
<body>
  <h1>Vertical Blind System</h1>

  <div class="card">
    <button onclick="setMode('auto')" id="autoBtn">Auto</button>
    <button onclick="setMode('manual')" id="manualBtn">Manual</button>
    <button onclick="setMode('calibration')" id="calibBtn">Calibration</button>
  </div>

  <div class="card" id="manualControl" style="display:none;">
    <p>Motor1: <span id="motor1Position">-</span>%</p>
    <input type="range" min="0" max="100" id="motor1Slider" onchange="updateMotor1()" class="slider">
    <p>Blind Control:</p>
    <button onclick="controlBlind('open')">Open</button>
    <button onclick="controlBlind('close')">Close</button>
  </div>

  <div class="card">
    <h2>System Status</h2>
    <p>Current Mode: <span id="currentMode">-</span></p>
    <p>Blind Status: <span id="blindStatus">-</span></p>
    <p>Blind Tilt Angle: <span id="motor1PositionStatus">-</span>°</p>
    <p>Light Sensors:</p>
    <ul>
      <li>IR1: <span id="sensor0">-</span></li>
      <li>IR2: <span id="sensor1">-</span></li>
      <li>IR3: <span id="sensor2">-</span></li>
      <li>IR4: <span id="sensor3">-</span></li>
    </ul>
  </div>

  <script>
    function setMode(mode) {
      fetch('/mode', {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        body: 'value=' + mode
      }).then(updateStatus);
    }

    function updateMotor1() {
      const val = document.getElementById("motor1Slider").value;
      fetch('/position', {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        body: 'motor1=' + val
      });
    }

    function controlBlind(action) {
      fetch('/blind', {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        body: 'action=' + action
      });
    }

    function updateStatus() {
      fetch('/status')
        .then(r => r.json())
        .then(data => {
          document.getElementById("currentMode").textContent = data.mode;
          document.getElementById("blindStatus").textContent = data.blind_status;
          document.getElementById("motor1PositionStatus").textContent = data.motor1_position;
          document.getElementById("sensor0").textContent = data.light_sensors[0];
          document.getElementById("sensor1").textContent = data.light_sensors[1];
          document.getElementById("sensor2").textContent = data.light_sensors[2];
          document.getElementById("sensor3").textContent = data.light_sensors[3];

          document.getElementById("autoBtn").className = data.mode === "auto" ? "active" : "";
          document.getElementById("manualBtn").className = data.mode === "manual" ? "active" : "";
          document.getElementById("calibBtn").className = data.mode === "calibration" ? "active" : "";

          if (data.mode === "manual") {
            document.getElementById("manualControl").style.display = "block";
            document.getElementById("motor1Position").textContent = data.motor1_position;
            document.getElementById("motor1Slider").value = data.motor1_position;
          } else {
            document.getElementById("manualControl").style.display = "none";
          }
        });
    }

    updateStatus();
    setInterval(updateStatus, 5000);
  </script>
</body>
</html>
)rawliteral";
    req->send(200, "text/html", html);
  });

  server.on("/status", HTTP_GET, [](AsyncWebServerRequest* request) {
    DynamicJsonDocument doc(1024);
    doc["mode"] = currentMode == MODE_AUTO ? "auto" : (currentMode == MODE_MANUAL ? "manual" : "calibration");
    doc["blind_status"] = getBlindStatusString();
    JsonArray sensors = doc.createNestedArray("light_sensors");
    sensors.add(ir1);
    sensors.add(ir2);
    sensors.add(ir3);
    sensors.add(ir4);

    float rawSteps = stepper1.currentPosition();
    if (motor1Reverse) rawSteps = maxStepsMotor1 - rawSteps;

    float motorAngle = (360.0 * rawSteps) / STEPS_PER_REVOLUTION;
    float blindAngle = motorAngle * MOTOR_TO_BLIND_ANGLE_MULTIPLIER;
    doc["motor1_position"] = round((rawSteps / maxStepsMotor1) * 180);   // Real-world blind tilt angle in degrees

    String res;
    serializeJson(doc, res);
    request->send(200, "application/json", res);
  });

  server.on("/mode", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (request->hasParam("value", true)) {
      String val = request->getParam("value", true)->value();
      if (val == "auto") currentMode = MODE_AUTO;
      else if (val == "manual") currentMode = MODE_MANUAL;
      else if (val == "calibration") currentMode = MODE_CALIBRATION;
      request->send(200, "text/plain", "OK");
    }
  });

  server.on("/position", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (request->hasParam("motor1", true)) {
      int val = constrain(request->getParam("motor1", true)->value().toInt(), 0, 100);
      setMotorTargetPosition(1, val);
    }
    request->send(200, "text/plain", "Motor1 updated");
  });

  server.on("/blind", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (request->hasParam("action", true)) {
      String action = request->getParam("action", true)->value();
      if (action == "open") {
        stepper2.setSpeed(-1000);
        stepper3.setSpeed(1000);
        blindDirection = BLIND_OPENING;
      } else if (action == "close") {
        stepper2.setSpeed(1000);
        stepper3.setSpeed(-1000);
        blindDirection = BLIND_CLOSING;
      }
      request->send(200, "text/plain", "Blind moving");
    }
  });
}

void setup() {
  Serial.begin(115200);
  pinMode(LIMIT_SWITCH_OPEN, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_CLOSED, INPUT_PULLUP);

  stepper1.setMaxSpeed(MAX_MOTOR_SPEED);
  stepper1.setAcceleration(MAX_MOTOR_ACCEL);
  stepper2.setMaxSpeed(MAX_MOTOR_SPEED);
  stepper2.setAcceleration(MAX_MOTOR_ACCEL);
  stepper3.setMaxSpeed(MAX_MOTOR_SPEED);
  stepper3.setAcceleration(MAX_MOTOR_ACCEL);

  if (!ads.begin())
    while (1)
      ;
  ads.setGain(GAIN_ONE);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected. IP: " + WiFi.localIP().toString());

  if (!isBlindFullyOpen() && !isBlindFullyClosed()) {
    stepper2.setSpeed(1000);
    stepper3.setSpeed(-1000);
    blindDirection = BLIND_CLOSING;
  }

  setupWebServer();
  server.begin();
}

void loop() {
  if (millis() - lastSensorReadTime > SENSOR_READ_INTERVAL) {
    readLightSensors();
    lastSensorReadTime = millis();
  }

  if (currentMode == MODE_AUTO && isBlindFullyClosed()) autoModeOperation();
  if (currentMode == MODE_CALIBRATION) calibrationModeOperation();

  if (blindDirection == BLIND_OPENING) {
    if (isBlindFullyOpen()) {
      stepper2.setSpeed(0);
      stepper3.setSpeed(0);
      blindDirection = BLIND_NONE;
    } else {
      stepper2.runSpeed();
      stepper3.runSpeed();
    }
  } else if (blindDirection == BLIND_CLOSING) {
    if (isBlindFullyClosed()) {
      stepper2.setSpeed(0);
      stepper3.setSpeed(0);
      blindDirection = BLIND_NONE;
    } else {
      stepper2.runSpeed();
      stepper3.runSpeed();
    }
  }

  stepper1.run();
}