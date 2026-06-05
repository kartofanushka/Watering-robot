#include "esp_camera.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ================= WIFI SETTINGS ============
const char* ssid = "___";
const char* password = "___";

// ================= MAIN ESP32 HOSTNAME ==============
const char* mainESP32_HOST = "wateringrobot.local";

// ================= CAMERA PINOUT AI THINKER =================
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ================= DATA =================
int currentAngle = 404;
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 200;
const int scanStep = 4;

const float CAMERA_FOV = 65.0; 

// SEND ANGLE TO MAIN ESP32
void sendAngleToMainESP32(int angle) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] Disconnected. Reconnecting...");
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    return;
  }

  WiFiClient client;
  HTTPClient http;

  String url = "http://";
  url += mainESP32_HOST;
  url += "/cam?angle=";
  url += String(angle);

  Serial.println();
  Serial.println("========== HTTP SEND ==========");
  Serial.print("[HTTP] URL: ");
  Serial.println(url);

  http.begin(client, url);
  http.setTimeout(500);

  int httpCode = http.GET();

  Serial.print("[HTTP] Angle: ");
  Serial.println(angle);
  Serial.print("[HTTP] Response: ");
  Serial.println(httpCode);

  if (httpCode > 0) {
    String payload = http.getString();
    Serial.print("[HTTP] Server reply: ");
    Serial.println(payload);
  } else {
    Serial.println("[HTTP] Request failed");
  }

  http.end();
  Serial.println("===============================");
}

// ================= CAMERA SETUP =================
void setupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size = FRAMESIZE_QQVGA;
  config.fb_count = 1;
  config.grab_mode = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[CAMERA] Init failed: 0x%x\n", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_whitebal(s, 1);
  s->set_exposure_ctrl(s, 1);
  s->set_gain_ctrl(s, 1);
  s->set_brightness(s, 0);
  s->set_contrast(s, 1);
  s->set_saturation(s, 2);

  Serial.println("[CAMERA] Ready");
}

// ================= WIFI SETUP ================
void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("[WiFi] Connecting...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n[WiFi] Connected!");
  Serial.print("[WiFi] IP: ");
  Serial.println(WiFi.localIP());
}

// ================= DETECT TARGET =================
int detectTargetAngle() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("[CAMERA] Failed to get frame");
    return 404;
  }

  uint32_t avg_x = 0;
  uint32_t red_pixels = 0;
  uint16_t *buf = (uint16_t *)fb->buf;

  // Fast scan (step 2)
  for (int y = 0; y < fb->height; y += scanStep) {
    for (int x = 0; x < fb->width; x += scanStep) {
      uint16_t pixel = buf[y * fb->width + x];
      pixel = (pixel >> 8) | (pixel << 8); // Endian swap
      
      uint8_t r = (pixel >> 11) & 0x1F; 
      uint8_t g = (pixel >> 5) & 0x3F;  
      uint8_t b = pixel & 0x1F;         

      // Sensitive color logic
      if (r > 12 && r > g && r > b) {
        avg_x += x;
        red_pixels++;
      }
    }
  }

  int angle = 404;


if (red_pixels > 6) {
    int centerX = avg_x / red_pixels;

    angle = round((centerX - 80.0) * (CAMERA_FOV / 2.0) / 80.0);

    if (angle >  (CAMERA_FOV / 2)) angle =  CAMERA_FOV / 2;
    if (angle < -(CAMERA_FOV / 2)) angle = -(CAMERA_FOV / 2);
}

  Serial.print("[CAMERA] Pixels: ");
  Serial.print(red_pixels);
  Serial.print(" | Angle: ");
  Serial.println(angle);

  esp_camera_fb_return(fb);
  return angle;
}

// ================= SETUP =================
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  delay(1000);

  setupWiFi();
  setupCamera();
  Serial.println("[SYSTEM] ESP32-CAM READY");
}

// ================= LOOP ===============
void loop() {
  currentAngle = detectTargetAngle();

  unsigned long now = millis();
  if (now - lastSendTime >= sendInterval) {
    sendAngleToMainESP32(currentAngle);
    lastSendTime = now;
  }

  delay(10);
}
