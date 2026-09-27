#include "esp_camera.h"
#include "esp_http_server.h"
#include <WiFi.h>

// =========================================================
// ESP32-CAM AI-THINKER
// Base: CameraWebServer0.ino
// =========================================================
// Responsibilities:
//   - Join the ESP32 DevKit SoftAP as a station
//   - Serve MJPEG directly to the laptop/browser on port 81
//   - Listen on Serial2 for lightweight optional housekeeping
//
// No motor, sensor, odometry, AI, or video-relay logic is here.
// =========================================================

// -------------------------
// DevKit AP credentials
// -------------------------
const char* WIFI_SSID = "ESP32-Rover-AP";
const char* WIFI_PASSWORD = "12345678";

// -------------------------
// Static camera IP on the DevKit AP subnet
// -------------------------
IPAddress CAM_IP(192, 168, 4, 2);
IPAddress CAM_GW(192, 168, 4, 1);
IPAddress CAM_MASK(255, 255, 255, 0);

// -------------------------
// AI-Thinker pin map
// -------------------------
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
#define LED_GPIO_NUM       4

// -------------------------
// Serial2 housekeeping UART
// -------------------------
// AI-Thinker ESP32-CAM exposes few spare GPIOs. These are intended for
// UART housekeeping when the onboard SD interface is not being used.
#define CAM_UART_RX 13
#define CAM_UART_TX 14
#define CAM_UART_BAUD 115200

// -------------------------
// Camera defaults from CameraWebServer0.ino
// -------------------------
framesize_t currentFrameSize = FRAMESIZE_QVGA;
int currentQuality = 12;

String uartLine;
httpd_handle_t controlHttpd = NULL;
httpd_handle_t streamHttpd = NULL;

static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=frame";
static const char* STREAM_BOUNDARY = "--frame";
static const char* STREAM_HEADER = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// =========================================================
// Camera server: direct MJPEG endpoint on port 81
// =========================================================
static esp_err_t stream_handler(httpd_req_t* req) {
  httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");

  char part_buf[64];
  while (true) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      return ESP_FAIL;
    }

    if (fb->format != PIXFORMAT_JPEG) {
      esp_camera_fb_return(fb);
      Serial.println("Unexpected non-JPEG frame");
      return ESP_FAIL;
    }

    size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_HEADER, fb->len);
    esp_err_t res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, "\r\n", 2);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, part_buf, hlen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, "\r\n", 2);

    esp_camera_fb_return(fb);
    if (res != ESP_OK) return res;
  }
}

static esp_err_t index_handler(httpd_req_t* req) {
  const char* body =
      "<html><head><title>ESP32-CAM</title></head><body>"
      "<h2>ESP32-CAM Rover Camera</h2>"
      "<p>MJPEG: <a href=\"/stream\">/stream</a></p>"
      "</body></html>";
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_handler(httpd_req_t* req) {
  sensor_t* s = esp_camera_sensor_get();
  String json = "{\"ip\":\"" + WiFi.localIP().toString() +
                "\",\"framesize\":" + String((int)currentFrameSize) +
                ",\"quality\":" + String(currentQuality) +
                ",\"sensor_pid\":" + String(s ? s->id.PID : 0) + "}";
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json.c_str(), HTTPD_RESP_USE_STRLEN);
}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.max_uri_handlers = 8;
  config.ctrl_port = 32768;

  if (httpd_start(&controlHttpd, &config) == ESP_OK) {
    httpd_uri_t index_uri = {};
    index_uri.uri = "/";
    index_uri.method = HTTP_GET;
    index_uri.handler = index_handler;
    httpd_register_uri_handler(controlHttpd, &index_uri);

    httpd_uri_t status_uri = {};
    status_uri.uri = "/status";
    status_uri.method = HTTP_GET;
    status_uri.handler = status_handler;
    httpd_register_uri_handler(controlHttpd, &status_uri);
  }

  httpd_config_t stream_config = HTTPD_DEFAULT_CONFIG();
  stream_config.server_port = 81;
  stream_config.ctrl_port = 32769;
  stream_config.max_uri_handlers = 4;

  if (httpd_start(&streamHttpd, &stream_config) == ESP_OK) {
    httpd_uri_t stream_uri = {};
    stream_uri.uri = "/stream";
    stream_uri.method = HTTP_GET;
    stream_uri.handler = stream_handler;
    httpd_register_uri_handler(streamHttpd, &stream_uri);

    httpd_uri_t stream_status_uri = {};
    stream_status_uri.uri = "/status";
    stream_status_uri.method = HTTP_GET;
    stream_status_uri.handler = status_handler;
    httpd_register_uri_handler(streamHttpd, &stream_status_uri);
  }
}

// =========================================================
// Sensor helpers
// =========================================================
framesize_t parseFrameSize(String token) {
  token.toUpperCase();
  if (token == "QQVGA") return FRAMESIZE_QQVGA;
  if (token == "QVGA") return FRAMESIZE_QVGA;
  if (token == "CIF") return FRAMESIZE_CIF;
  if (token == "VGA") return FRAMESIZE_VGA;
  if (token == "SVGA") return FRAMESIZE_SVGA;
  if (token == "XGA") return FRAMESIZE_XGA;
  if (token == "HD") return FRAMESIZE_HD;
  if (token == "SXGA") return FRAMESIZE_SXGA;
  if (token == "UXGA") return FRAMESIZE_UXGA;
  return FRAMESIZE_QVGA;
}

void sendAck(const String& msg) {
  Serial2.println(msg);
}

void processUartCommand(String cmd) {
  cmd.trim();
  if (!cmd.length()) return;

  sensor_t* s = esp_camera_sensor_get();
  if (!s) {
    sendAck("ERR:NO_SENSOR");
    return;
  }

  if (cmd == "FLASH_ON") {
    digitalWrite(LED_GPIO_NUM, HIGH);
    sendAck("ACK:FLASH_ON");
    return;
  }
  if (cmd == "FLASH_OFF") {
    digitalWrite(LED_GPIO_NUM, LOW);
    sendAck("ACK:FLASH_OFF");
    return;
  }

  if (cmd.startsWith("FRAME_")) {
    String v = cmd.substring(6);
    if (v == "QQVGA" || v == "QVGA" || v == "CIF" || v == "VGA" || v == "SVGA" || v == "XGA" || v == "HD" || v == "SXGA" || v == "UXGA") {
      currentFrameSize = parseFrameSize(v);
      s->set_framesize(s, currentFrameSize);
      sendAck("ACK:" + cmd);
    } else {
      sendAck("ERR:BAD_FRAME");
    }
    return;
  }

  if (cmd.startsWith("QUALITY_")) {
    int q = cmd.substring(8).toInt();
    if (q >= 10 && q <= 63) {
      currentQuality = q;
      s->set_quality(s, currentQuality);
      sendAck("ACK:" + cmd);
    } else {
      sendAck("ERR:BAD_QUALITY");
    }
    return;
  }

  if (cmd.startsWith("BRIGHTNESS_")) {
    int v = cmd.substring(12).toInt();
    if (v >= -2 && v <= 2) { s->set_brightness(s, v); sendAck("ACK:" + cmd); }
    else sendAck("ERR:BAD_BRIGHTNESS");
    return;
  }

  if (cmd.startsWith("CONTRAST_")) {
    int v = cmd.substring(9).toInt();
    if (v >= -2 && v <= 2) { s->set_contrast(s, v); sendAck("ACK:" + cmd); }
    else sendAck("ERR:BAD_CONTRAST");
    return;
  }

  if (cmd.startsWith("SATURATION_")) {
    int v = cmd.substring(11).toInt();
    if (v >= -2 && v <= 2) { s->set_saturation(s, v); sendAck("ACK:" + cmd); }
    else sendAck("ERR:BAD_SATURATION");
    return;
  }

  if (cmd.startsWith("HFLIP_")) {
    int v = cmd.substring(6).toInt();
    if (v == 0 || v == 1) { s->set_hmirror(s, v); sendAck("ACK:" + cmd); }
    else sendAck("ERR:BAD_HFLIP");
    return;
  }

  if (cmd.startsWith("VFLIP_")) {
    int v = cmd.substring(6).toInt();
    if (v == 0 || v == 1) { s->set_vflip(s, v); sendAck("ACK:" + cmd); }
    else sendAck("ERR:BAD_VFLIP");
    return;
  }

  if (cmd.startsWith("AWB_")) {
    int v = cmd.substring(4).toInt();
    if (v == 0 || v == 1) { s->set_whitebal(s, v); sendAck("ACK:" + cmd); }
    else sendAck("ERR:BAD_AWB");
    return;
  }

  if (cmd.startsWith("AGC_")) {
    int v = cmd.substring(4).toInt();
    if (v == 0 || v == 1) { s->set_gain_ctrl(s, v); sendAck("ACK:" + cmd); }
    else sendAck("ERR:BAD_AGC");
    return;
  }

  if (cmd.startsWith("AEC_")) {
    int v = cmd.substring(4).toInt();
    if (v == 0 || v == 1) { s->set_exposure_ctrl(s, v); sendAck("ACK:" + cmd); }
    else sendAck("ERR:BAD_AEC");
    return;
  }

  sendAck("ERR:UNKNOWN_COMMAND");
}

void pollUart() {
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    if (c == '\n') {
      uartLine.trim();
      if (uartLine.length()) processUartCommand(uartLine);
      uartLine = "";
    } else if (c != '\r' && uartLine.length() < 100) {
      uartLine += c;
    }
  }
}

// =========================================================
// Setup / loop
// =========================================================
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial2.begin(CAM_UART_BAUD, SERIAL_8N1, CAM_UART_RX, CAM_UART_TX);

  pinMode(LED_GPIO_NUM, OUTPUT);
  digitalWrite(LED_GPIO_NUM, LOW);

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

  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }

  sensor_t* s = esp_camera_sensor_get();
  if (s && s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }

  if (s) {
    s->set_framesize(s, FRAMESIZE_QVGA); // Base file lowers initial size for stream FPS.
    s->set_quality(s, currentQuality);
  }
  currentFrameSize = FRAMESIZE_QVGA;

  WiFi.mode(WIFI_STA);
  WiFi.config(CAM_IP, CAM_GW, CAM_MASK);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.setSleep(false);

  Serial.printf("Connecting to DevKit AP %s", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
  }
  Serial.println();
  Serial.print("Camera IP: ");
  Serial.println(WiFi.localIP());

  startCameraServer();

  Serial.printf("MJPEG stream: http://%s:81/stream\n", WiFi.localIP().toString().c_str());
  Serial.println("UART housekeeping ready; unplugging it does not stop WiFi/video.");
}

void loop() {
  pollUart();
  delay(2);
}
