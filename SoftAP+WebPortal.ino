#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiManager.h>  
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Konfigurasi LCD I2C
LiquidCrystal_I2C lcd(0x27, 16, 2); // Alamat I2C 0x27

// Pilih Model Kamera
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

// Deklarasi Fungsi
void startCameraServer();
void updateLCD();
void initCamera();

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("\n=== ESP32-CAM System Boot ===");

  // Inisialisasi I2C dan LCD
  Wire.begin(14, 15); // SDA: GPIO 14, SCL: GPIO 15
  Wire.setClock(100000);
  lcd.begin();
  delay(100);
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ESP32-CAM Booting");

  // Inisialisasi WiFi Manager
  WiFiManager wm;
  lcd.setCursor(0, 1);
  lcd.print("WiFi Config...");

  // Kalau gagal connect WiFi → bikin AP "ESP32-CAM-Setup"
  if (!wm.autoConnect("ESP32-CAM-Setup")) {
    Serial.println("Gagal connect, reboot...");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi Failed!");
    delay(2000);
    ESP.restart();
  }

  Serial.println("Terhubung ke WiFi!");
  Serial.println("IP: " + WiFi.localIP().toString());

  // Inisialisasi Kamera
  initCamera();
  startCameraServer();
  updateLCD();
}

void loop() {
  static unsigned long lastCheck = 0;
  const unsigned long checkInterval = 5000; // Cek setiap 5 detik

  if (millis() - lastCheck >= checkInterval) {
    updateLCD();
    lastCheck = millis();
  }
}

void initCamera() {
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
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  if (psramFound()) {
    config.jpeg_quality = 10;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.jpeg_quality = 15;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[ERROR] Camera init failed: 0x%x\n", err);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Camera Failed");
    delay(2000);
    ESP.restart();
  }
  Serial.println("[INFO] Camera initialized successfully");
}

void updateLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ESP32-CAM Active");

  lcd.setCursor(0, 1);
  if (WiFi.status() == WL_CONNECTED) {
    String ip = WiFi.localIP().toString();
    if (ip.length() > 16) {
      lcd.print(ip.substring(0, 13) + "...");
    } else {
      lcd.print("IP: " + ip);
    }
  } else {
    lcd.print("WiFi Disconnected");
  }
}
