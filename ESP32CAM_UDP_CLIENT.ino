 #include "esp_camera.h"
#include <WiFi.h>
#include <WiFiUdp.h>

// Ganti dengan kredensial WiFi kamu
const char* ssid = "ESP32_Server";
const char* password = "12345678";

// IP dan port tujuan (ESP32 Receiver)
const char* receiverIP = "192.168.4.1"; 
const int udpPort = 8000;

WiFiUDP udp;
bool connected = false;

// Pin Kamera ESP32-CAM
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

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);

  // Konfigurasi kamera
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
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size = FRAMESIZE_QQVGA;  // 160x120
  config.fb_count = 1;

  // Inisialisasi kamera
  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("❌ Camera init failed");
    return;
  }

  // Koneksi WiFi
  WiFi.begin(ssid, password);
  Serial.print("🔄 Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ Connected to WiFi");
  Serial.print("📡 IP Address: ");
  Serial.println(WiFi.localIP());

  udp.begin(WiFi.localIP(), udpPort);
  connected = true;
}

void loop() {
  if (!connected) return;

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("❌ Camera capture failed");
    return;
  }

  const int width = fb->width;   // 160
  const int height = fb->height; // 120
  uint8_t oled_buffer[1024];     // 128x64 / 8 = 1024 bytes
  memset(oled_buffer, 0, sizeof(oled_buffer));

  // Konversi frame ke 128x64 1-bit bitmap untuk OLED SSD1306
  for (int y = 0; y < 64; y++) {
    for (int x = 0; x < 128; x++) {
      int src_x = x * width / 128;
      int src_y = y * height / 64;

      if (src_x >= width || src_y >= height) continue;

      uint8_t pixel = fb->buf[src_y * width + src_x];
      if (pixel > 127) {
        oled_buffer[x + (y / 8) * 128] |= (1 << (y % 8));
      }
    }
  }

  // Kirim ke receiver via UDP
  udp.beginPacket(receiverIP, udpPort);
  udp.write(oled_buffer, sizeof(oled_buffer));
  udp.endPacket();

  esp_camera_fb_return(fb);

  Serial.println("📤 Frame sent via UDP");
  delay(200); // Kirim tiap 200 ms (5 fps)
}