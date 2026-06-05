#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>

// WIFI SETTINGS
const char* ssid     = "___";
const char* password = "___";

// WEB SERVER
WebServer server(80);

// WHEEL PINS
// IN1/IN2/ENA = right
// IN3/IN4/ENB = left

#define ENA 25
#define IN1 26
#define IN2 27

#define ENB 14
#define IN3 23
#define IN4 13

// ULTRASONIC PINS
#define TRIG_PIN 5
#define ECHO_PIN 18

// SPEED SETTINGS
int leftMotorSpeed  = 130;
int rightMotorSpeed = 130;

int leftTurnSpeed  = 160;
int rightTurnSpeed = 160;

int backwardLeftSpeed  = 255;
int backwardRightSpeed = 255;

int searchSpinSpeed = 150;

int angleDeadZone = 5;

const float boostGain = 1.5f;
const int   maxBoost  = 60;

// SEARCH SETTINGS
const unsigned long searchTurnTime = 100;
const unsigned long searchWaitTime = 700;

bool          searchTurnActive    = false;
unsigned long searchTurnStartTime = 0;
unsigned long searchWaitEndTime   = 0;

// ULTRASONIC SETTINGS
long distanceCM          = -1;
int  wallCloseDistanceCM = 18;

const unsigned long backTime     = 500;
const unsigned long sideTurnTime = 550;

// CAMERA DATA
volatile int           aimAngle        = 404;
volatile int           lastTargetAngle = 0;
volatile unsigned long lastTargetSeen  = 0;
const unsigned long    targetLostTimeout = 1500;

const unsigned long potMemoryTime = 5000;

IPAddress camIP;
bool      camIPResolved = false;

TaskHandle_t camTaskHandle = NULL;

// SOIL DATA
int           soilStatus     = 9;
unsigned long lastSoilUpdate = 0;

// WALL AVOID STATE
bool          wallAvoidActive    = false;
int           wallAvoidStep      = 0;
unsigned long wallAvoidStartTime = 0;

bool arrivedWatering = false;

// ROBOT STATE
enum RobotState {
  STATE_SEARCH,
  STATE_FOLLOWING,
  STATE_WALL_AVOID,
  STATE_ARRIVED
};

RobotState robotState = STATE_SEARCH;

// LOG / WEB DATA
unsigned long requestCounter  = 0;
String        currentDecision = "Starting";

// PWM HELPER
int pwmLimit(int value) {
  if (value < 0)   return 0;
  if (value > 255) return 255;
  return value;
}

// TEXT HELPERS
String moistureText() {
  if (soilStatus == 1) return "DRY";
  if (soilStatus == 0) return "WET";
  return "UNKNOWN";
}

String stateText() {
  switch (robotState) {
    case STATE_SEARCH:     return "SEARCH";
    case STATE_FOLLOWING:  return "FOLLOWING";
    case STATE_WALL_AVOID: return "WALL AVOID";
    case STATE_ARRIVED:    return "ARRIVED";
  }
  return "UNKNOWN";
}

// MOTOR FUNCTIONS
void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.println("[MOTOR] STOP");
}

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, pwmLimit(rightMotorSpeed));
  analogWrite(ENB, pwmLimit(leftMotorSpeed));
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, pwmLimit(backwardRightSpeed));
  analogWrite(ENB, pwmLimit(backwardLeftSpeed));
  Serial.println("[MOTOR] BACKWARD");
}

void turnRight() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, pwmLimit(rightTurnSpeed));
  analogWrite(ENB, pwmLimit(leftTurnSpeed));
  Serial.println("[MOTOR] TURN RIGHT");
}

void turnLeft() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, pwmLimit(rightTurnSpeed));
  analogWrite(ENB, pwmLimit(leftTurnSpeed));
  Serial.println("[MOTOR] TURN LEFT");
}

void spinLeft() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, pwmLimit(searchSpinSpeed));
  analogWrite(ENB, pwmLimit(searchSpinSpeed));
}

void spinRight() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, pwmLimit(searchSpinSpeed));
  analogWrite(ENB, pwmLimit(searchSpinSpeed));
}

// ULTRASONIC
long readUltrasonicCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 10000);
  if (duration == 0) return -1;
  return duration * 0.034 / 2;
}

bool wallIsClose() {
  return (distanceCM > 0 && distanceCM <= wallCloseDistanceCM);
}
// POT MEMORY CHECK
bool potSeenRecently() {
  if (lastTargetSeen == 0) return false;
  return (millis() - lastTargetSeen <= potMemoryTime);
}

// CAMERA — target currently visible
bool targetFound() {
  return (aimAngle != 404) &&
         (millis() - lastTargetSeen <= targetLostTimeout);
}

void resolveCamIP() {
  Serial.println("[mDNS] Resolving wateringcam...");
  IPAddress ip = MDNS.queryHost("wateringcam", 2000);
  if (ip != INADDR_NONE) {
    camIP         = ip;
    camIPResolved = true;
    Serial.print("[mDNS] Resolved: ");
    Serial.println(camIP.toString());
  } else {
    camIPResolved = false;
    Serial.println("[mDNS] Failed");
  }
}

// CAMERA TASK — core 0
void cameraTask(void* parameter) {
  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      vTaskDelay(500 / portTICK_PERIOD_MS);
      continue;
    }

    if (!camIPResolved) {
      Serial.println("[CAM] Resolving...");
      IPAddress ip = MDNS.queryHost("wateringcam", 500);
      if (ip != INADDR_NONE) {
        camIP         = ip;
        camIPResolved = true;
        Serial.print("[CAM] Resolved: ");
        Serial.println(camIP.toString());
      } else {
        Serial.println("[CAM] Resolve failed, retry in 2s");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        continue;
      }
    }

    WiFiClient client;
    HTTPClient http;

    String url = "http://";
    url += camIP.toString();
    url += "/angle";

    http.begin(client, url);
    http.setTimeout(500);

    int code = http.GET();

    if (code == 200) {
      String body = http.getString();
      body.trim();
      int angle = body.toInt();
      aimAngle = angle;

      if (angle != 404) {
        lastTargetSeen  = millis();
        lastTargetAngle = angle;
      }

      Serial.print("[CAM] ");
      Serial.println(angle);
    } else {
      Serial.print("[CAM] fail: ");
      Serial.println(code);
      if (code < 0) {
        camIPResolved = false;
        Serial.println("[CAM] Lost — will re-resolve");
      }
    }

    http.end();
    vTaskDelay(150 / portTICK_PERIOD_MS);
  }
}

// WALL AVOID
void startWallAvoid() {
  wallAvoidActive    = true;
  wallAvoidStep      = 1;
  wallAvoidStartTime = millis();
  robotState         = STATE_WALL_AVOID;
  currentDecision    = "WALL: BACK";
  moveBackward();
}

void handleWallAvoid() {
  if (!wallAvoidActive) return;

  unsigned long elapsed = millis() - wallAvoidStartTime;

  if (wallAvoidStep == 1) {
    if (elapsed >= backTime) {
      wallAvoidStep      = 2;
      wallAvoidStartTime = millis();
      currentDecision    = "WALL: TURN";
      turnRight();
    }
    return;
  }

  if (wallAvoidStep == 2) {
    if (elapsed >= sideTurnTime) {
      wallAvoidActive = false;
      wallAvoidStep   = 0;
      stopMotors();
      robotState      = STATE_SEARCH;
      currentDecision = "WALL: DONE";
    }
    return;
  }
}

// SEARCH
void handleSearch() {

  if (targetFound()) {
    robotState      = STATE_FOLLOWING;
    currentDecision = "FOLLOWING";
    return;
  }

  if (searchTurnActive) {
    if (millis() - searchTurnStartTime >= searchTurnTime) {
      searchTurnActive  = false;
      searchWaitEndTime = millis() + searchWaitTime;
      stopMotors();
      currentDecision = "SEARCH: WAITING";
    }
    return;
  }

  if (millis() < searchWaitEndTime) {
    currentDecision = "SEARCH: WAITING";
    return;
  }

  searchTurnActive    = true;
  searchTurnStartTime = millis();

  if (lastTargetAngle >= 0) {
    currentDecision = "SEARCH: SPIN RIGHT";
    spinRight();
  } else {
    currentDecision = "SEARCH: SPIN LEFT";
    spinLeft();
  }
}

void handleFollowing() {

  if (!targetFound()) {
    stopMotors();
    robotState      = STATE_SEARCH;
    currentDecision = "SEARCH";
    return;
  }

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  if (aimAngle > angleDeadZone) {
    int boost = min((int)(aimAngle * boostGain), maxBoost);
    analogWrite(ENA, pwmLimit(rightMotorSpeed));
    analogWrite(ENB, pwmLimit(leftMotorSpeed + boost));
    currentDecision = "STEER RIGHT";

  } else if (aimAngle < -angleDeadZone) {
    int boost = min((int)(-aimAngle * boostGain), maxBoost);
    analogWrite(ENA, pwmLimit(rightMotorSpeed + boost));
    analogWrite(ENB, pwmLimit(leftMotorSpeed));
    currentDecision = "STEER LEFT";

  } else {
    analogWrite(ENA, pwmLimit(rightMotorSpeed));
    analogWrite(ENB, pwmLimit(leftMotorSpeed));
    currentDecision = "FORWARD";
  }
}

// ARRIVED — fine-tune alignment
bool          arrivedTurnActive    = false;
unsigned long arrivedTurnStartTime = 0;
bool          arrivedWaitingCam    = false;

void handleArrived() {
  arrivedWatering = true;

  if (aimAngle != 404 && aimAngle >= -angleDeadZone && aimAngle <= angleDeadZone) {
    stopMotors();
    arrivedTurnActive = false;
    arrivedWaitingCam = false;
    currentDecision   = "ARRIVED: ALIGNED";
    return;
  }

  if (arrivedWaitingCam) {
    stopMotors();
    currentDecision = "ARRIVED: WAITING CAM";
    if (aimAngle != 404) {
      arrivedWaitingCam = false;
    }
    return;
  }

  if (arrivedTurnActive) {
    if (millis() - arrivedTurnStartTime >= 150) {
      arrivedTurnActive = false;
      arrivedWaitingCam = true;
      stopMotors();
      currentDecision = "ARRIVED: WAITING CAM";
    }
    return;
  }

  int angle = (aimAngle != 404) ? aimAngle : lastTargetAngle;

  arrivedTurnActive    = true;
  arrivedTurnStartTime = millis();

  if (angle > angleDeadZone) {
    spinRight();
    currentDecision = "ARRIVED: SPIN RIGHT";
  } else if (angle < -angleDeadZone) {
    spinLeft();
    currentDecision = "ARRIVED: SPIN LEFT";
  } else {
    arrivedTurnActive = false;
    stopMotors();
    currentDecision = "ARRIVED: ALIGNED";
  }
}

// MAIN AUTO CONTROL
void autoControlRobot() {

  distanceCM = readUltrasonicCM();

  if (wallAvoidActive) {
    handleWallAvoid();
    return;
  }

  if (wallIsClose()) {
    stopMotors();

    if (potSeenRecently()) {
      Serial.println("[WALL] Pot confirmed — arrived");
      arrivedWatering = true;
      robotState      = STATE_ARRIVED;
      currentDecision = "ARRIVED";
    } else {
      Serial.println("[WALL] No pot — avoiding");
      arrivedWatering = false;
      startWallAvoid();
    }

    return;
  }

  switch (robotState) {
    case STATE_SEARCH:     handleSearch();    break;
    case STATE_FOLLOWING:  handleFollowing(); break;
    case STATE_WALL_AVOID: handleWallAvoid(); break;
    case STATE_ARRIVED:
      arrivedWatering = true;
      handleArrived();
      break;
  }
}

// WEB ROOT
void handleRoot() {
  requestCounter++;

  String html = "";
  html += "<!DOCTYPE html><html><head>";
  html += "<meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<meta http-equiv='refresh' content='1'>";
  html += "<title>Watering Robot</title>";
  html += "<style>";
  html += "body{background:#111;color:#00ff88;font-family:Arial;padding:20px;}";
  html += "h1{color:#00ccff;}";
  html += ".box{background:#1b1b1b;padding:20px;border-radius:12px;max-width:700px;}";
  html += ".line{margin:10px 0;font-size:18px;}";
  html += ".green{color:#00ff00;}.red{color:#ff4444;}.yellow{color:#ffff66;}.cyan{color:#00ccff;}";
  html += "</style></head><body><div class='box'>";
  html += "<h1>Watering Robot</h1>";

  html += "<div class='line'>IP: ";
  html += WiFi.localIP().toString();
  html += "</div>";

  html += "<div class='line'>Cam IP: ";
  if (camIPResolved) {
    html += "<span class='green'>";
    html += camIP.toString();
    html += "</span>";
  } else {
    html += "<span class='red'>NOT RESOLVED</span>";
  }
  html += "</div>";

  html += "<div class='line'>State: <span class='cyan'>";
  html += stateText();
  html += "</span></div>";

  html += "<div class='line'>Decision: ";
  html += currentDecision;
  html += "</div>";

  html += "<div class='line'>Moisture: ";
  if      (soilStatus == 1) html += "<span class='red'>DRY</span>";
  else if (soilStatus == 0) html += "<span class='green'>WET</span>";
  else                      html += "<span class='yellow'>UNKNOWN</span>";
  html += "</div>";

  html += "<div class='line'>Distance: ";
  html += String(distanceCM);
  html += " cm</div>";

  html += "<div class='line'>Angle: ";
  html += String(aimAngle);
  html += "</div>";

  html += "<div class='line'>Last target angle: ";
  html += String(lastTargetAngle);
  html += "</div>";

  html += "<div class='line'>Target: ";
  if (targetFound()) html += "<span class='green'>YES</span>";
  else               html += "<span class='red'>NO</span>";
  html += "</div>";

  html += "<div class='line'>Pot seen recently: ";
  if (potSeenRecently()) html += "<span class='green'>YES</span>";
  else                   html += "<span class='red'>NO</span>";
  html += "</div>";

  html += "<div class='line'>Wall close: ";
  if (wallIsClose()) html += "<span class='red'>YES</span>";
  else               html += "<span class='green'>NO</span>";
  html += "</div>";

  html += "<div class='line'>Arrived: ";
  if (arrivedWatering) html += "<span class='green'>YES</span>";
  else                 html += "<span class='red'>NO</span>";
  html += "</div>";

  html += "<div class='line'>Requests: ";
  html += String(requestCounter);
  html += "</div>";

  html += "</div></body></html>";

  server.send(200, "text/html", html);
}

// SOIL UPDATE
void handleSoilUpdate() {
  requestCounter++;
  if (server.hasArg("status")) {
    soilStatus     = server.arg("status").toInt();
    lastSoilUpdate = millis();
    server.send(200, "text/plain", "OK SOIL");
    return;
  }
  server.send(400, "text/plain", "Missing status");
}

// STATUS
void handleStatus() {
  String text = "";
  text += "IP: ";             text += WiFi.localIP().toString();            text += "\n";
  text += "Cam IP: ";         text += camIP.toString();                     text += "\n";
  text += "State: ";          text += stateText();                          text += "\n";
  text += "Decision: ";       text += currentDecision;                      text += "\n";
  text += "Angle: ";          text += String(aimAngle);                     text += "\n";
  text += "Last angle: ";     text += String(lastTargetAngle);              text += "\n";
  text += "Distance: ";       text += String(distanceCM);                   text += "\n";
  text += "Pot recent: ";     text += potSeenRecently() ? "YES" : "NO";     text += "\n";
  server.send(200, "text/plain", text);
}

void handleNotFound() {
  requestCounter++;
  server.send(404, "text/plain", "404");
}

// SETUP
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  stopMotors();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.println("[WiFi] Connecting...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("[WiFi] Connected");
  Serial.print("[WiFi] IP: ");
  Serial.println(WiFi.localIP());

  if (MDNS.begin("wateringrobot")) {
    Serial.println("[mDNS] http://wateringrobot.local");
  } else {
    Serial.println("[mDNS] FAILED");
  }

  xTaskCreatePinnedToCore(
    cameraTask,
    "cameraTask",
    4096,
    NULL,
    1,
    &camTaskHandle,
    0
  );

  server.on("/",       handleRoot);
  server.on("/soil",   handleSoilUpdate);
  server.on("/status", handleStatus);
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("[WEB] http://wateringrobot.local");
}

// LOOP — core 1
void loop() {

  server.handleClient();

  static unsigned long lastControlTime = 0;
  if (millis() - lastControlTime >= 50) {
    autoControlRobot();
    lastControlTime = millis();
  }

  static unsigned long lastWiFiCheck = 0;
  if (millis() - lastWiFiCheck >= 5000) {
    if (WiFi.status() != WL_CONNECTED) {
      stopMotors();
      WiFi.disconnect();
      WiFi.begin(ssid, password);
    }
    lastWiFiCheck = millis();
  }
}
