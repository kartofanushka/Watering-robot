#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

// ============== WIFI SETTINGS =================
const char* ssid = "___";
const char* password = "___";

// MAIN ESP32 HOSTNAME (mDNS)
const char* mainESP32_HOST = "wateringrobot.local";

// =========== PIN SETTINGS =============
#define SOIL_PIN 2

// Send every 10 seconds
unsigned long lastSendTime = 0;

const unsigned long sendInterval = 10000;

// SENSOR LOGIC
// HIGH = dry
// LOW  = wet
bool HIGH_MEANS_DRY = true;

// SEND DATA
void sendSoilStatus(int status) {

  if (WiFi.status() != WL_CONNECTED) {

    Serial.println("WiFi disconnected. Reconnecting...");

    WiFi.begin(ssid, password);

    return;
  }

  WiFiClient client;

  HTTPClient http;

  String url = "http://";

  url += mainESP32_HOST;

  url += "/soil?status=";

  url += String(status);

  Serial.println();
  Serial.println("========== SENDING ==========");
  Serial.print("URL: ");
  Serial.println(url);

  http.begin(client, url);

  int httpCode = http.GET();

  Serial.print("HTTP response: ");
  Serial.println(httpCode);

  if (httpCode > 0) {

    String payload = http.getString();

    Serial.print("Server response: ");
    Serial.println(payload);
  }
  else {

    Serial.println("HTTP request failed");
  }

  http.end();

  Serial.println("=============================");
  Serial.println();
}

// SETUP
void setup() {

  Serial.begin(115200);

  delay(1000);

  pinMode(SOIL_PIN, INPUT);

  WiFi.mode(WIFI_STA);

  WiFi.begin(ssid, password);

  Serial.println();
  Serial.println("ESP-01S connecting to WiFi...");

  while (WiFi.status() != WL_CONNECTED) {

    delay(500);

    Serial.print(".");
  }

  Serial.println();
  Serial.println("ESP-01S connected!");

  Serial.print("ESP-01S IP: ");
  Serial.println(WiFi.localIP());

  Serial.println();
  Serial.println("Using mDNS hostname:");
  Serial.println("http://wateringrobot.local");
}

// LOOP
void loop() {

  if (millis() - lastSendTime >= sendInterval) {

    int sensorValue = digitalRead(SOIL_PIN);

    int soilStatus;

    if (HIGH_MEANS_DRY) {

      soilStatus = (sensorValue == HIGH) ? 1 : 0;
    }
    else {

      soilStatus = (sensorValue == LOW) ? 1 : 0;
    }

    Serial.print("GPIO2 value: ");
    Serial.print(sensorValue);

    Serial.print(" | Soil status: ");

    Serial.println(soilStatus == 1 ? "Dry" : "Wet");

    sendSoilStatus(soilStatus);

    lastSendTime = millis();
  }
}
