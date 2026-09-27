#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include "esp_task_wdt.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// =========================================================
// ESP32 ROVER - DEVKIT V1
// Base: Rover_v16.txt
// =========================================================
// Responsibilities:
//   - WiFi SoftAP + dashboard HTTP server on port 80
//   - WebSocket telemetry/control server on port 81 (/ws)
//   - Differential/skid-steer motor control
//   - Hall wheel odometry + MPU6050 yaw dead reckoning
//   - Latched AI stop authority
//   - Optional UART housekeeping bridge to ESP32-CAM
//
// The ESP32-CAM is NOT used for video relay or AI data.
// =========================================================

// -------------------------
// WiFi / AP configuration
// -------------------------
const char* AP_SSID = "ESP32-Rover-AP";
const char* AP_PASSWORD = "12345678"; // >= 8 chars
IPAddress AP_IP(192, 168, 4, 1);
IPAddress AP_GW(192, 168, 4, 1);
IPAddress AP_MASK(255, 255, 255, 0);

// Default camera station address. The camera sketch uses the same value.
const char* DEFAULT_CAMERA_IP = "192.168.4.2";

// -------------------------
// Motor driver pins (from Rover_v16.txt)
// -------------------------
#define L_RPWM 25
#define L_LPWM 26
#define R_RPWM 27
#define R_LPWM 14

// -------------------------
// Sensors
// -------------------------
#define HALL_PIN 35

// I2C: default ESP32 SDA=21, SCL=22.
#define I2C_SDA 21
#define I2C_SCL 22

// Optional battery ADC. Disabled by default because the original
// rover firmware did not define a battery divider.
#define USE_BATTERY_ADC 0
#define BATTERY_ADC_PIN 33
const float BATTERY_R1_OHMS = 100000.0f;
const float BATTERY_R2_OHMS = 33000.0f;
const float ADC_REFERENCE_V = 3.30f;
const int ADC_MAX_COUNTS = 4095;

// -------------------------
// DevKit <-> ESP32-CAM UART
// -------------------------
// DevKit Serial2: RX16 / TX17
#define CAM_UART_RX 16
#define CAM_UART_TX 17
#define CAM_UART_BAUD 115200

// Camera pan servo (powered from an external 5V supply; common GND required).
#define CAMERA_SERVO_PIN 13
const int CAMERA_SERVO_LEFT = 150;
const int CAMERA_SERVO_CENTER = 90;
const int CAMERA_SERVO_RIGHT = 30;

// -------------------------
// PWM: ESP32 Arduino Core 3.x API
// -------------------------
const uint32_t PWM_FREQ = 5000;
const uint8_t PWM_RESOLUTION = 8;
const int PWM_MAX = 255;

// -------------------------
// Odometry calibration
// -------------------------
// These values come from Rover_v16.txt; recalibrate to your actual wheel/hall setup.
const float WHEEL_CIRCUMFERENCE_CM = 20.42f;
const uint16_t TICKS_PER_REV = 8;
const float CM_PER_TICK = WHEEL_CIRCUMFERENCE_CM / (float)TICKS_PER_REV;

// -------------------------
// Control timing
// -------------------------
const uint32_t KINEMATICS_PERIOD_MS = 20;  // 50 Hz
const uint32_t TELEMETRY_PERIOD_MS = 100; // 10 Hz
const uint32_t DRIVE_WATCHDOG_MS = 450;   // no joystick packet => motors stop

// AI safety lease. Laptop should refresh every 100-150 ms.
// Expiration fails closed and latches a motor stop.
const uint32_t AI_LEASE_TIMEOUT_MS = 450;

// Human-avoidance safety gates. The laptop must prove a side has been
// human-clear continuously for this time before the DevKit accepts AVOID.
const uint32_t AI_SIDE_CLEAR_HOLD_MS = 300;
const uint32_t AI_RECOVERY_CLEAR_HOLD_MS = 900;

// Normal autonomous AI cruise envelope. The laptop may ramp up to this value;
// the DevKit clamps it here.
const float AI_NORMAL_MAX_SPEED = 0.35f;
const float AI_NORMAL_MAX_TURN = 0.55f;
const uint32_t AI_NORMAL_RAMP_MIN_MS = 1000; // DevKit-side minimum recovery->normal ramp

// Slow recovery envelope before the normal-speed ramp begins.
const float AI_RECOVERY_MAX_SPEED = 0.18f;

// Motor-command slew limiting. All non-emergency movement passes through
// this limiter; safety stops bypass it and write zero immediately.
const float MOTOR_ACCEL_LIMIT_PER_SEC = 0.55f;
const float MOTOR_DECEL_LIMIT_PER_SEC = 1.80f;

// Independent motion-feedback watchdog. If the rover is commanded to move
// above a configurable minimum but the Hall sensor reports no wheel motion
// for too long, fail closed. This is a software cross-check, not a distance
// sensor.
const float HALL_FEEDBACK_MIN_COMMAND = 0.16f;
const uint32_t HALL_FEEDBACK_TIMEOUT_MS = 1200;
const uint32_t CONTROL_LOOP_OVERRUN_MS = 180;
const uint32_t CONTROL_TASK_WDT_TIMEOUT_MS = 1000;
const uint32_t MOTOR_COMMAND_EXPIRY_MS = 320;
const uint8_t AI_HEALTH_STOP_THRESHOLD = 35;
const uint8_t AI_HEALTH_CAUTION_THRESHOLD = 75;
const float AI_HEALTH_MIN_SPEED_SCALE = 0.45f;

// Side/block/recovery status is always published; these values also feed the
// local dashboard safety-health indicators.
const uint32_t SAFETY_STATUS_PERIOD_MS = 250;

// -------------------------
// Globals / servers
// -------------------------
AsyncWebServer webServer(80);
AsyncWebServer wsServer(81);
AsyncWebSocket ws("/ws");
Adafruit_MPU6050 mpu;

volatile uint32_t wheelTicks = 0;
uint32_t lastTicks = 0;
uint32_t lastKinematicsMs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t lastDriveCommandMs = 0;
uint32_t lastCamUartPollMs = 0;
uint32_t lastMotorOutputMs = 0;
uint32_t lastHallTickMs = 0;
uint32_t lastLoopEntryMs = 0;
uint32_t lastSafetyStatusMs = 0;
uint32_t lastAcceptedMotionCommandMs = 0;

bool mpuReady = false;
float gyroZOffset = 0.0f;
float yawAngleRad = 0.0f;
float posXcm = 0.0f;
float posYcm = 0.0f;
float totalDistanceCm = 0.0f;
float measuredSpeedCmS = 0.0f;

float joystickX = 0.0f; // left/right: -1 .. +1
float joystickY = 0.0f; // back/forward: -1 .. +1

// Explicit safety state machine.
enum SafetyState : uint8_t {
  STATE_MANUAL = 0,
  STATE_AI_SESSION,
  STATE_HUMAN_AVOID_LEFT,
  STATE_HUMAN_AVOID_RIGHT,
  STATE_HUMAN_RECOVERY,
  STATE_HUMAN_STOP,
  STATE_HUMANS_BLOCKED,
  STATE_AI_LINK_LOSS,
  STATE_WAIT_NEUTRAL,
  STATE_WAIT_NEW_DRIVE
};

volatile SafetyState safetyState = STATE_MANUAL;

// Hard-stop latch. A human STOP is only released by a valid AI GO,
// after which the joystick must be neutral before motion is allowed.
bool stoppedLatched = false;
bool aiStopReason = false;
bool requireFreshDriveAfterStop = false;
bool driveWatchdogActive = false;
volatile bool motorStopRequested = false;
volatile bool commandQueueFault = false;
String stopReason = "none";

uint8_t aiHealthScore = 0;
uint16_t aiFrameAgeMs = 0;
uint16_t aiInferenceMs = 0;
uint16_t aiWsRttMs = 0;
uint8_t aiVisionFps = 0;
float aiOpticalFlowMag = 0.0f;
uint16_t aiDecisionAgeMs = 0;
bool aiHealthValid = false;
bool taskWatchdogReady = false;

// Bounded AI avoidance command. The DevKit clamps these values.
float aiAvoidSpeed = 0.0f;
float aiAvoidTurn = 0.0f;
float aiDriveSpeed = 0.0f;
float aiDriveTurn = 0.0f;
float aiRecoverySpeed = 0.0f;

// Only the client that owns the AI lease can refresh/release it.
bool aiLeaseActive = false;
uint32_t aiClientId = 0;
uint32_t lastAiPacketMs = 0;
uint32_t lastAiSeq = 0;
bool lastAiSeqValid = false;

// Vision-side safety diagnostics relayed from the laptop.
bool aiLeftZoneOccupied = false;
bool aiRightZoneOccupied = false;
uint32_t aiSideClearHoldMs = 0;
uint32_t aiSideClearRequiredMs = AI_SIDE_CLEAR_HOLD_MS;
uint32_t aiBlockedHoldMs = 0;
uint32_t aiBlockedHoldRequiredMs = 0;
String aiSideCandidate = "none";
String aiResumeCondition = "AI session must start with STOP";
bool aiReleaseQualified = false;
String aiReleaseSide = "none";

String camUartLine;
Servo cameraServo;

// Explicit operator-selected control mode. MANUAL uses the dashboard joystick;
// AUTO permits the laptop AI lease. AUTO never enables motion by itself.
bool autonomousMode = false;

// WebSocket callbacks run outside loop(); never mutate motor/safety state
// directly from the callback. Parsed commands are copied into this queue and
// consumed serially by loop(), giving the DevKit one authoritative control
// context. If the queue fills, the safest fallback is an immediate stop flag.
enum WsCommandType : uint8_t {
  WS_CMD_DATA = 0,
  WS_CMD_DISCONNECT = 1
};

struct WsCommand {
  WsCommandType type = WS_CMD_DATA;
  uint32_t clientId = 0;
  bool hasAi = false;
  bool hasDrive = false;
  bool hasOdomReset = false;
  bool hasMode = false;
  bool hasServo = false;

  char mode[8] = {0};
  char servo[16] = {0};

  char ai[12] = {0};
  char reason[40] = {0};
  char chosenSide[12] = {0};
  char resumeCondition[160] = {0};

  uint32_t seq = 0;
  uint32_t clientMs = 0;
  uint32_t sideClearMs = 0;
  uint32_t sideClearRequiredMs = 0;
  uint32_t blockedMs = 0;
  uint32_t blockedRequiredMs = 0;
  uint32_t clearMs = 0;
  uint32_t rampMs = 0;

  float speed = 0.0f;
  float turn = 0.0f;
  float joyX = 0.0f;
  float joyY = 0.0f;
  bool leftBlocked = false;
  bool rightBlocked = false;
  bool releaseQualified = false;
  char releaseSide[12] = {0};
  uint8_t healthScore = 0;
  uint16_t frameAgeMs = 0;
  uint16_t inferenceMs = 0;
  uint16_t wsRttMs = 0;
  uint8_t visionFps = 0;
  uint16_t decisionAgeMs = 0;
  float opticalFlowMag = 0.0f;
};

QueueHandle_t wsCommandQueue = nullptr;

// =========================================================
// HTML: CONTROL DASHBOARD
// =========================================================
const char dashboard_html[] PROGMEM = R"HTML(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>VORTEX ROVER</title>
<style>
:root{--bg:#070910;--panel:rgba(18,24,38,.82);--line:rgba(120,190,255,.24);--text:#eef5ff;--muted:#8ea4c2;--cyan:#43d9ff;--green:#36e69a;--red:#ff4e68;--amber:#ffc857}
*{box-sizing:border-box}html,body{height:100%;overflow:hidden}body{margin:0;background:radial-gradient(circle at 20% 0%,#11203a 0,#080b13 44%,#05060b 100%);color:var(--text);font-family:Inter,Segoe UI,Arial,sans-serif;min-height:100vh}
.topbar{height:56px;display:flex;justify-content:space-between;align-items:center;gap:12px;padding:8px 14px;background:rgba(5,7,12,.78);backdrop-filter:blur(12px);border-bottom:1px solid var(--line);position:relative;z-index:5}
.brand{font-weight:800;letter-spacing:.12em;color:var(--cyan);font-size:13px}.links{display:flex;align-items:center;justify-content:flex-end;gap:7px;flex-wrap:nowrap}.navbtn{color:var(--text);text-decoration:none;border:1px solid var(--line);padding:7px 10px;border-radius:9px;background:rgba(255,255,255,.04);font-size:11px;white-space:nowrap}
.status{padding:8px 12px;border-radius:999px;border:1px solid var(--green);color:var(--green);font-weight:800;font-size:11px;letter-spacing:.07em;white-space:nowrap;box-shadow:0 0 18px rgba(54,230,154,.12)}
.status.stop{border-color:var(--red);color:var(--red);box-shadow:0 0 18px rgba(255,78,104,.24);animation:pulse 1s infinite alternate}
@keyframes pulse{to{box-shadow:0 0 26px rgba(255,78,104,.42)}}
.wrap{width:100%;height:calc(100vh - 56px);max-width:none;margin:0 auto;padding:10px;display:flex;flex-direction:column;min-height:0}
.camera-shell{position:relative;height:42vh;min-height:240px;max-height:460px;display:flex;justify-content:center;align-items:stretch;flex:0 0 auto;margin-bottom:10px}.camera-wrap{height:100%;width:auto;max-width:100%;aspect-ratio:16/9;overflow:hidden;border-radius:16px;border:1px solid var(--line);background:#020308;box-shadow:0 12px 36px rgba(0,0,0,.34)}
#camera{display:block;width:100%;height:100%;object-fit:contain;object-position:center;background:#000}.camera-note{position:absolute;left:10px;bottom:10px;padding:6px 9px;background:rgba(0,0,0,.58);border-radius:8px;font-size:10px;color:#b9c9dd}.camera-shell:after{content:'LANDSCAPE CAMERA';position:absolute;right:10px;top:10px;padding:5px 8px;border-radius:7px;background:rgba(0,0,0,.48);color:#b9c9dd;font-size:9px;letter-spacing:.08em;pointer-events:none}
.grid{display:grid;grid-template-columns:minmax(310px,.78fr) minmax(620px,1.42fr);gap:10px;flex:1;min-height:0}.panel{background:var(--panel);border:1px solid var(--line);border-radius:16px;box-shadow:0 9px 30px rgba(0,0,0,.22);backdrop-filter:blur(14px);padding:12px;min-height:0;overflow:hidden}.panel h2{font-size:12px;letter-spacing:.1em;margin:0 0 9px;color:var(--cyan)}
/* Explicit control mode + camera pan controls */
.mode-row,.servo-row{display:flex;gap:7px;align-items:center;flex-wrap:wrap;margin:6px 0}
.modebtn{font-weight:800}.modebtn.active{border-color:var(--green);color:var(--green)}
.servo-row span{font-size:10px;color:var(--muted);margin-right:2px}
/* Compact laptop joystick panel */.joystick-area{display:flex;justify-content:center;align-items:center;min-height:0;flex:1}.panel:first-child{display:flex;flex-direction:column}.joystick{width:min(31vh,260px);height:min(31vh,260px);max-width:260px;max-height:260px;min-width:190px;min-height:190px;border-radius:50%;border:1px solid rgba(67,217,255,.32);background:radial-gradient(circle,#142238 0,#0c1220 55%,#080c16 100%);position:relative;touch-action:none;user-select:none;box-shadow:inset 0 0 50px rgba(67,217,255,.05),0 0 30px rgba(67,217,255,.07)}
.joystick:before,.joystick:after{content:"";position:absolute;background:rgba(67,217,255,.14)}.joystick:before{width:1px;height:86%;left:50%;top:7%}.joystick:after{height:1px;width:86%;left:7%;top:50%}
.stick{position:absolute;width:30%;height:30%;left:35%;top:35%;border-radius:50%;background:linear-gradient(145deg,#49ddff,#13799c);border:2px solid #a9f2ff;box-shadow:0 0 28px rgba(67,217,255,.35);transform:translate(0,0)}
.axes{display:grid;grid-template-columns:1fr 1fr;gap:7px;margin-top:8px}.axis{background:rgba(255,255,255,.035);border:1px solid rgba(255,255,255,.06);border-radius:9px;padding:7px 9px;font-size:11px}.axis span{float:right;font-family:ui-monospace,monospace;color:var(--cyan)}
.telemetry-grid{display:grid;grid-template-columns:repeat(3,minmax(85px,1fr));gap:7px;margin-bottom:9px}.metric{padding:8px 9px;border-radius:10px;background:rgba(255,255,255,.035);border:1px solid rgba(255,255,255,.05)}.metric label{display:block;font-size:9px;color:var(--muted);text-transform:uppercase;letter-spacing:.08em}.metric strong{display:block;margin-top:4px;font-size:15px}
/* Landscape telemetry panel: avoidance on left, map on right */.grid>.panel:nth-child(2){display:grid;grid-template-columns:minmax(300px,.9fr) minmax(360px,1.1fr);grid-template-rows:auto auto 1fr auto auto;column-gap:10px;row-gap:7px}.grid>.panel:nth-child(2)>h2{grid-column:1/-1;grid-row:1}.grid>.panel:nth-child(2)>.telemetry-grid{grid-column:1;grid-row:2}.grid>.panel:nth-child(2)>.avoid-panel{grid-column:1;grid-row:3}.grid>.panel:nth-child(2)>.map-wrap{grid-column:2;grid-row:2/4;min-height:0;align-items:stretch}.grid>.panel:nth-child(2)>.map-actions{grid-column:2;grid-row:4}.grid>.panel:nth-child(2)>.footer-note{grid-column:2;grid-row:5}
.map-wrap{display:flex;justify-content:center;min-height:0}.map{width:100%;height:100%;max-width:none;aspect-ratio:auto;border-radius:11px;border:1px solid rgba(67,217,255,.25);background:#050811;display:block}.map-actions{display:flex;justify-content:flex-end;gap:7px;margin-top:0}.smallbtn{border:1px solid var(--line);background:rgba(255,255,255,.045);color:var(--text);padding:7px 10px;border-radius:8px;cursor:pointer;font-size:11px}.smallbtn:hover{border-color:var(--cyan)}
.footer-note{color:var(--muted);font-size:10px;line-height:1.4;margin-top:5px}.cam-off{display:none}
.avoid-panel{margin-top:0;padding:9px;border:1px solid var(--line);border-radius:11px;background:rgba(5,9,18,.35);min-height:0;overflow:hidden}.avoid-head{display:flex;justify-content:space-between;align-items:center;gap:8px}.avoid-head h3{margin:0;color:var(--cyan);font-size:10px;letter-spacing:.08em}.avoid-head span{font-size:9px;color:var(--muted);white-space:nowrap}.zone-row{display:grid;grid-template-columns:1fr 1fr;gap:7px;margin-top:7px}.zone{padding:7px 8px;border:1px solid var(--line);border-radius:8px;display:flex;justify-content:space-between;gap:6px;font-size:9px}.zone strong{letter-spacing:.06em}.zone.occupied{border-color:rgba(255,78,104,.7);color:#ff8b9b}.zone.clear{border-color:rgba(54,230,154,.45);color:#63efb0}.hold-row{display:flex;justify-content:space-between;margin-top:6px;font-size:9px;color:var(--muted)}.hold-row strong{color:var(--text)}.hold-bar{height:5px;border-radius:99px;overflow:hidden;background:#0b1120;border:1px solid var(--line);margin-top:3px}.hold-bar div{height:100%;width:0%;background:var(--amber);transition:width .08s linear}.resume-box{margin-top:7px;padding:7px 8px;border-radius:8px;background:rgba(255,200,87,.06);border:1px solid rgba(255,200,87,.2);font-size:9px;color:var(--muted)}.resume-box span{display:block;font-size:8px;letter-spacing:.08em;margin-bottom:3px}.resume-box strong{color:var(--text);font-size:10px;line-height:1.35}.release-box{margin-top:7px;padding:7px 8px;border-radius:8px;background:rgba(255,255,255,.03);border:1px solid var(--line);font-size:9px}.release-box span{display:block;font-size:8px;letter-spacing:.08em;margin-bottom:3px;color:var(--muted)}.release-box strong{color:#ff8b9b;font-size:10px;line-height:1.3}.release-box.qualified{border-color:rgba(54,230,154,.55);background:rgba(54,230,154,.07)}.release-box.qualified strong{color:#63efb0}.countdown-note{font-size:9px;line-height:1.3;color:var(--muted);margin-top:3px}
@media(max-width:1100px){.grid{grid-template-columns:minmax(270px,.7fr) minmax(540px,1.3fr)}.grid>.panel:nth-child(2){grid-template-columns:minmax(250px,.9fr) minmax(320px,1.1fr)}.joystick{width:min(29vh,230px);height:min(29vh,230px)}}
/* v10 landscape fit hardening */
@media(min-width:851px) and (max-height:900px){.grid>.panel:nth-child(2){grid-template-rows:auto auto minmax(225px,1fr) auto}.grid>.panel:nth-child(2)>.map-wrap{height:250px;max-height:250px;align-self:start}.grid>.panel:nth-child(2)>.footer-note{display:none}.grid>.panel:nth-child(2)>.map-actions{grid-column:2;grid-row:4}.grid{gap:8px}.panel{padding:10px;border-radius:14px}.panel h2{margin-bottom:7px}.avoid-panel{padding:8px}.zone{padding:6px 7px;font-size:9px}.hold-row{margin-top:5px;font-size:9px}.hold-bar{height:5px;margin-top:3px}.resume-box,.release-box{margin-top:6px;padding:6px 7px;font-size:9px}.resume-box strong,.release-box strong{font-size:10px}.countdown-note{font-size:8px;margin-top:2px}}
@media(min-width:851px) and (max-height:780px){.camera-shell{height:39vh;max-height:330px;min-height:220px}.grid>.panel:nth-child(2)>.map-wrap{height:220px;max-height:220px}.joystick{width:min(29vh,230px);height:min(29vh,230px);min-width:180px;min-height:180px}.axes{gap:5px;margin-top:5px}.axis{padding:5px 7px}.footer-note{font-size:9px}.avoid-panel{padding:7px}.zone{padding:5px 6px;font-size:8px}.hold-row{margin-top:4px;font-size:8px}.hold-bar{height:4px;margin-top:2px}.resume-box,.release-box{margin-top:5px;padding:5px 6px;font-size:8px}.resume-box strong,.release-box strong{font-size:9px}.countdown-note{font-size:8px;margin-top:2px}}
@media(max-width:850px),(orientation:portrait){html,body{height:auto;overflow:auto}.topbar{height:auto}.wrap{height:auto;padding:10px}.camera-shell{height:auto;max-height:none;min-height:0}.camera-wrap{width:100%;height:auto;aspect-ratio:16/9}.grid{height:auto;display:grid;grid-template-columns:1fr;flex:0 0 auto}.grid>.panel:nth-child(2){display:block}.joystick-area{min-height:320px}.joystick{width:min(78vw,320px);height:min(78vw,320px)}.map{height:auto;aspect-ratio:1.5}.telemetry-grid{grid-template-columns:repeat(2,1fr)}}.health-line{position:absolute;right:10px;bottom:10px;z-index:3;display:flex;align-items:center;justify-content:space-between;gap:7px;flex-wrap:nowrap;padding:4px 6px;border:1px solid rgba(120,190,255,.18);border-radius:8px;background:rgba(4,8,14,.78);backdrop-filter:blur(8px);font-size:7px}.health{display:flex;align-items:center;gap:3px;min-width:0;white-space:nowrap}.health span{color:var(--muted);font-size:7px;letter-spacing:.05em}.health strong{font-size:8px;color:var(--green)}.health.bad strong{color:var(--red)}.health.warn strong{color:var(--amber)}@media(max-width:850px){.health-line{left:10px;right:10px;justify-content:flex-start;flex-wrap:wrap}}</style>
</head>
<body>
<div class="topbar"><div class="brand">VORTEX // ROVER CONTROL</div><div class="links"><a class="navbtn" href="/settings">CAMERA SETTINGS</a><div id="aiLease" class="footer-note" style="margin:0">AI LEASE: --</div><div id="status" class="status">CONNECTING</div></div></div>
<div class="wrap">
  <div class="camera-shell"><div class="camera-wrap"><img id="camera" alt="ESP32-CAM MJPEG stream"></div><div class="camera-note" id="camLabel">CAMERA: --</div>
<div class="health-line" aria-label="Safety health">
          <div class="health" id="healthVision"><span>VISION</span><strong id="visionHealth">--</strong></div>
          <div class="health" id="healthLease"><span>AI LEASE</span><strong id="leaseHealth">--</strong></div>
          <div class="health" id="healthHall"><span>HALL FEEDBACK</span><strong id="hallHealth">--</strong></div>
          <div class="health" id="healthIntegrity"><span>CMD INTEGRITY</span><strong id="integrityHealth">--</strong></div>
          <div class="health" id="healthOdom"><span>ODOM</span><strong id="odomHealth">--</strong></div><div class="health" id="healthAi"><span>AI HEALTH</span><strong id="aiHealthValue">--</strong></div>
        </div>
</div>
  <div class="grid">
    <section class="panel">
      <h2>DRIVE JOYSTICK</h2><div class="mode-row">
  <button class="smallbtn modebtn" id="manualMode">MANUAL</button>
  <button class="smallbtn modebtn" id="autoMode">AUTONOMOUS AI</button>
</div>
<div class="servo-row">
  <span>CAMERA PAN</span>
  <button class="smallbtn" id="servoLeft">LEFT</button>
  <button class="smallbtn" id="servoCenter">CENTER</button>
  <button class="smallbtn" id="servoRight">RIGHT</button>
</div>
      <div class="joystick-area"><div id="joystick" class="joystick"><div id="stick" class="stick"></div></div></div>
      <div class="axes"><div class="axis">TURN <span id="jx">0.00</span></div><div class="axis">SPEED <span id="jy">0.00</span></div></div>
      <div class="footer-note">Vertical = forward/back speed. Horizontal = left/right turn. The DevKit ignores this input while the AI-stop latch is active.</div>
    </section>
    <section class="panel">
      <h2>LIVE TELEMETRY + DEAD-RECKONING</h2>
      <div class="telemetry-grid">
        <div class="metric"><label>Speed</label><strong id="speed">-- cm/s</strong></div>
        <div class="metric"><label>Heading</label><strong id="heading">--°</strong></div>
        <div class="metric"><label>Battery</label><strong id="battery">N/A</strong></div>
        <div class="metric"><label>X</label><strong id="x">-- cm</strong></div>
        <div class="metric"><label>Y</label><strong id="y">-- cm</strong></div>
        <div class="metric"><label>Distance</label><strong id="distance">-- cm</strong></div>
      </div>
      <div class="avoid-panel">
        <div class="avoid-head"><h3>HUMAN AVOIDANCE ZONES</h3><span id="avoidMode">--</span></div>
        <div class="zone-row">
          <div class="zone" id="leftZone"><span>LEFT ZONE</span><strong id="leftZoneState">--</strong></div>
          <div class="zone" id="rightZone"><span>RIGHT ZONE</span><strong id="rightZoneState">--</strong></div>
        </div><div class="hold-row"><span>Blocked hold</span><strong id="blockedHold">-- / -- ms</strong></div>
        <div class="hold-bar"><div id="blockedHoldBar"></div></div>
        <div class="hold-row"><span>Side-clear hold</span><strong id="sideHold">-- / -- ms</strong></div>
        <div class="hold-bar"><div id="sideHoldBar"></div></div>
        <div class="release-box" id="releaseBox"><span>RELEASE QUALIFIED</span><strong id="releaseQualified">FALSE — neither side has completed the clear hold</strong></div>
        <div class="hold-row"><span>Side-clear countdown</span><strong id="sideCountdown">--</strong></div>
        <div class="countdown-note" id="sideCountdownNote">Waiting for a LEFT or RIGHT side to qualify.</div>
        <div class="resume-box"><span>RESUME CONDITION</span><strong id="resumeCondition">--</strong></div>
      </div>
      <div class="map-wrap"><canvas id="map" class="map" width="720" height="480"></canvas></div>
      <div class="map-actions"><button class="smallbtn" id="resetMap">RESET ODOMETRY</button></div>
      <div class="footer-note">Trail is drawn from Hall-sensor distance plus integrated MPU6050 gyro yaw. This is dead reckoning, not a surveyed position.</div>
    </section>
  </div>
</div>
<script>
let socket=null, reconnectTimer=null, telemetry=null;
let sideCountdownSnapshot={qualified:false, side:null, remainingMs:null, requiredMs:0, packetAt:0};
let cameraIP='192.168.4.2'; try { cameraIP=localStorage.getItem('cameraIp') || cameraIP; } catch(e) { /* restricted storage context */ }
const camera=document.getElementById('camera'); const camLabel=document.getElementById('camLabel');
const statusEl=document.getElementById('status');
const map=document.getElementById('map'); const ctx=map.getContext('2d'); let trail=[];
const joystick=document.getElementById('joystick'), stick=document.getElementById('stick'); let activePointer=null; let joyX=0, joyY=0;
function cameraUrl(){return `http://${cameraIP}:81/stream`}
function loadCamera(){camera.src=cameraUrl();camLabel.textContent='CAMERA: '+cameraIP+':81';}
function drawGrid(){ctx.clearRect(0,0,map.width,map.height);ctx.fillStyle='#050811';ctx.fillRect(0,0,map.width,map.height);ctx.strokeStyle='rgba(67,217,255,.10)';ctx.lineWidth=1;for(let x=0;x<=map.width;x+=40){ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,map.height);ctx.stroke()}for(let y=0;y<=map.height;y+=40){ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(map.width,y);ctx.stroke()}}
function drawMap(){drawGrid(); if(!trail.length)return; const xs=trail.map(p=>p.x), ys=trail.map(p=>p.y);let maxAbs=Math.max(50,...xs.map(Math.abs),...ys.map(Math.abs));let scale=Math.min(map.width,map.height)/(2.1*maxAbs);let ox=map.width/2,oy=map.height/2;ctx.lineWidth=3;ctx.strokeStyle='#43d9ff';ctx.beginPath();trail.forEach((p,i)=>{let px=ox+p.x*scale,py=oy-p.y*scale;i?ctx.lineTo(px,py):ctx.moveTo(px,py)});ctx.stroke();let p=trail[trail.length-1],px=ox+p.x*scale,py=oy-p.y*scale;ctx.fillStyle='#36e69a';ctx.beginPath();ctx.arc(px,py,6,0,Math.PI*2);ctx.fill();let a=-p.heading*Math.PI/180;ctx.strokeStyle='#ffc857';ctx.lineWidth=3;ctx.beginPath();ctx.moveTo(px,py);ctx.lineTo(px+18*Math.cos(a),py+18*Math.sin(a));ctx.stroke()}
function clamp01(v){return Math.max(0,Math.min(1,Number(v)||0))}
function renderSideClearCountdown(){
  const el=document.getElementById('sideCountdown');
  const note=document.getElementById('sideCountdownNote');
  const s=sideCountdownSnapshot;
  if(s.qualified){
    el.textContent='0 ms';
    note.textContent=(s.side?s.side.toUpperCase():'SELECTED')+' side qualified — release_qualified is TRUE.';
    return;
  }
  if(!s.side || s.remainingMs===null){
    el.textContent='--';
    note.textContent='Waiting for a LEFT or RIGHT side to remain human-clear.';
    return;
  }
  const elapsed=Math.max(0,performance.now()-s.packetAt);
  // Never render 0 while the DevKit still reports release_qualified=false.
  const remaining=Math.max(1,Math.ceil(s.remainingMs-elapsed));
  el.textContent=remaining+' ms';
  note.textContent=s.side.toUpperCase()+' side must remain human-clear; countdown reaches 0 only when release_qualified becomes TRUE.';
}
function updateAvoidancePanel(d){
  const left=!!d.left_zone_occupied, right=!!d.right_zone_occupied;
  const lz=document.getElementById('leftZone'), rz=document.getElementById('rightZone');
  lz.className='zone '+(left?'occupied':'clear'); rz.className='zone '+(right?'occupied':'clear');
  document.getElementById('leftZoneState').textContent=left?'OCCUPIED':'CLEAR';
  document.getElementById('rightZoneState').textContent=right?'OCCUPIED':'CLEAR';
  const sideMs=Number(d.side_clear_ms)||0, sideReq=Number(d.side_clear_required_ms)||0;
  const blockMs=Number(d.blocked_hold_ms)||0, blockReq=Number(d.blocked_hold_required_ms)||0;
  document.getElementById('sideHold').textContent=Math.round(sideMs)+' / '+Math.round(sideReq)+' ms';
  document.getElementById('blockedHold').textContent=Math.round(blockMs)+' / '+Math.round(blockReq)+' ms';
  document.getElementById('sideHoldBar').style.width=(sideReq?Math.round(100*clamp01(sideMs/sideReq)):0)+'%';
  document.getElementById('blockedHoldBar').style.width=(blockReq?Math.round(100*clamp01(blockMs/blockReq)):0)+'%';

  const qualified=!!d.release_qualified;
  const releaseSide=(d.release_side&&d.release_side!=='none')?String(d.release_side).toLowerCase():null;
  const candidate=(d.side_candidate&&d.side_candidate!=='none')?String(d.side_candidate).toLowerCase():null;
  const qualifyingSide=releaseSide||candidate;
  let remaining=null;
  if(!qualified && qualifyingSide && ((qualifyingSide==='left'&&!left)||(qualifyingSide==='right'&&!right)) && sideReq>0){
    const explicit=Number(d.side_clear_remaining_ms);
    remaining=Number.isFinite(explicit)?Math.max(1,explicit):Math.max(1,sideReq-sideMs);
  }
  sideCountdownSnapshot={qualified,side:qualifyingSide,remainingMs:qualified?0:remaining,requiredMs:sideReq,packetAt:performance.now()};

  document.getElementById('avoidMode').textContent=qualifyingSide?'CANDIDATE: '+qualifyingSide.toUpperCase():String(d.state||'--').replaceAll('_',' ').toUpperCase();
  const releaseBox=document.getElementById('releaseBox');
  releaseBox.className='release-box'+(qualified?' qualified':'');
  document.getElementById('releaseQualified').textContent=qualified
    ? 'TRUE — '+String(d.release_side||qualifyingSide||'selected').toUpperCase()+' side completed the full clear hold'
    : 'FALSE — no side has completed the full clear hold';
  document.getElementById('resumeCondition').textContent=d.resume_condition||'Either LEFT or RIGHT side must remain human-clear for the configured hold period.';
  renderSideClearCountdown();
}

function setStatus(stopped, reason, aiLease, aiAge){
  const labels={
    human:'STOPPED — HUMAN DETECTED',
    blocked_by_humans:'STOPPED — BLOCKED BY HUMANS',
    ai_link_lost:'STOPPED — AI LINK LOST',
    camera_stale:'STOPPED — CAMERA STALE',
    startup:'STOPPED — AI STARTING',
    human_avoid_left:'SLOW AVOID — LEFT',
    human_avoid_right:'SLOW AVOID — RIGHT',
    human_recovery:'SLOW RECOVERY — FORWARD PATH CLEARING',
    manual:'MANUAL CONTROL — AI OVERRIDE',
    none:'DRIVING',
    drive_watchdog:'STOPPED — DRIVE LINK LOST',
    awaiting_neutral:'STOPPED — RELEASE JOYSTICK',
    awaiting_new_drive:'STOPPED — NEW DRIVE COMMAND'
  };
  statusEl.textContent=labels[reason] || (stopped?'STOPPED':'DRIVING');
  statusEl.className='status'+((stopped||reason.startsWith('human_avoid'))?' stop':'');
  const age=(Number(aiAge)>=0)?Math.round(Number(aiAge)):'--';
  document.getElementById('aiLease').textContent=aiLease?'AI LEASE: ACTIVE ('+age+' ms)':'AI LEASE: INACTIVE';
}
function setHealth(boxId, valueId, value, kind='ok'){const box=document.getElementById(boxId);document.getElementById(valueId).textContent=value;box.className='health'+(kind==='bad'?' bad':kind==='warn'?' warn':'')}
function updateHealth(d){
  const age=Number(d.ai_age_ms);
  const lease=!!d.ai_lease;
  const visionFresh=lease && Number.isFinite(age) && age < 350;
  setHealth('healthVision','visionHealth',lease?(visionFresh?'FRESH':'STALE'):'IDLE',lease?(visionFresh?'ok':'bad'):'warn');
  setHealth('healthLease','leaseHealth',lease?'ACTIVE':'INACTIVE',lease?'ok':'warn');
  const hallOk=d.hall_feedback_ok!==false;
  setHealth('healthHall','hallHealth',hallOk?'OK':'FAULT',hallOk?'ok':'bad');
  const integrityOk=d.command_integrity_ok!==false;
  setHealth('healthIntegrity','integrityHealth',integrityOk?'OK':'FAULT',integrityOk?'ok':'bad');
  const mpuOk=d.odom_mpu!==false;
  const odomScore=Number(d.odom_confidence);
  if(Number.isFinite(odomScore)){
    const odomText=Math.max(0,Math.min(100,Math.round(odomScore)))+'%';
    setHealth('healthOdom','odomHealth',odomText,odomScore>=65&&mpuOk?'ok':(odomScore>=35?'warn':'bad'));
  } else {
    setHealth('healthOdom','odomHealth',mpuOk?'OK':'MISSING',mpuOk?'ok':'bad');
  }
  const aiValid=d.ai_health_valid===true;
  const aiScore=Number(d.ai_health_score);
  const aiText=aiValid && Number.isFinite(aiScore) ? String(Math.max(0,Math.min(100,Math.round(aiScore)))) : '--';
  setHealth('healthAi','aiHealthValue',aiText,(!aiValid)?'warn':(aiScore<=35?'bad':(aiScore<75?'warn':'ok')));
  if(lease && aiValid){
    document.getElementById('aiLease').textContent='AI LEASE: ACTIVE ('+age+' ms) · HEALTH '+aiText;
  } else if(lease){
    document.getElementById('aiLease').textContent='AI LEASE: ACTIVE ('+age+' ms) · HEALTH --';
  }
}
function updateTelemetry(d){
  document.getElementById('speed').textContent=(Number(d.speed)||0).toFixed(1)+' cm/s';
  document.getElementById('heading').textContent=(Number(d.heading)||0).toFixed(1)+'°';
  document.getElementById('x').textContent=(Number(d.x)||0).toFixed(1)+' cm';
  document.getElementById('y').textContent=(Number(d.y)||0).toFixed(1)+' cm';
  document.getElementById('distance').textContent=(Number(d.distance)||0).toFixed(1)+' cm';
  document.getElementById('battery').textContent=(d.battery==null||Number(d.battery)<0)?'N/A':Number(d.battery).toFixed(2)+' V';
  setStatus(!!d.stopped,d.stop_reason||((d.ai)?'human':'none'),!!d.ai_lease,d.ai_age_ms);
  updateHealth(d);
  updateAvoidancePanel(d);
  if(trail.length===0||Math.hypot((trail[trail.length-1].x||0)-d.x,(trail[trail.length-1].y||0)-d.y)>0.4){trail.push({x:Number(d.x)||0,y:Number(d.y)||0,heading:Number(d.heading)||0});if(trail.length>2500)trail.shift();drawMap()}
}
setInterval(renderSideClearCountdown,50);
function connect(){clearTimeout(reconnectTimer);const host=location.hostname||'192.168.4.1';socket=new WebSocket(`ws://${host}:81/ws`);socket.onopen=()=>{console.log('[WS] connected');};socket.onclose=()=>{console.log('[WS] disconnected');setStatus(true,'ai_link_lost',false,-1);reconnectTimer=setTimeout(connect,1500)};socket.onerror=e=>console.warn('[WS] error',e);socket.onmessage=e=>{try{const d=JSON.parse(e.data);if(d.type==='telemetry')updateTelemetry(d);if(d.type==='ai_state'){setStatus(!!d.stopped,d.reason,!!d.ai_lease,d.ai_age_ms);updateAvoidancePanel(d);console.log('[AI STATE]',d.reason,'seq='+d.seq)}if(d.type==='ai_ack')console.log('[AI ACK]',d.accepted?'accepted':'rejected','seq='+d.seq,'reason='+d.reason);if(d.type==='cam_ack')console.log('[CAM]',d.message)}catch(err){console.warn('Bad telemetry',err)}}}
function send(obj){if(socket&&socket.readyState===1)socket.send(JSON.stringify(obj))}
function setMode(mode){
  send({mode:mode});
  document.getElementById('manualMode').classList.toggle('active',mode==='manual');
  document.getElementById('autoMode').classList.toggle('active',mode==='auto');
}
function setServo(position){send({servo:position});}

function sendDrive(){send({drive:{x:Number(joyX.toFixed(3)),y:Number(joyY.toFixed(3))}});document.getElementById('jx').textContent=joyX.toFixed(2);document.getElementById('jy').textContent=joyY.toFixed(2)}
function setJoy(x,y){const r=joystick.getBoundingClientRect(),cx=r.left+r.width/2,cy=r.top+r.height/2,rad=r.width*0.35;let dx=(x-cx)/rad,dy=(cy-y)/rad;const mag=Math.hypot(dx,dy);if(mag>1){dx/=mag;dy/=mag}const db=.04;joyX=Math.abs(dx)<db?0:dx;joyY=Math.abs(dy)<db?0:dy;let sx=joyX*(r.width*.35),sy=-joyY*(r.height*.35);stick.style.transform=`translate(${sx}px,${sy}px)`;sendDrive()}
function releaseJoy(){joyX=0;joyY=0;stick.style.transform='translate(0,0)';sendDrive()}
joystick.addEventListener('pointerdown',e=>{activePointer=e.pointerId;joystick.setPointerCapture(e.pointerId);setJoy(e.clientX,e.clientY)});joystick.addEventListener('pointermove',e=>{if(e.pointerId===activePointer)setJoy(e.clientX,e.clientY)});['pointerup','pointercancel','lostpointercapture'].forEach(ev=>joystick.addEventListener(ev,()=>{activePointer=null;releaseJoy()}));
document.getElementById('resetMap').onclick=()=>{send({odom:'reset'});trail=[];drawMap()};
window.addEventListener('resize',drawMap); window.addEventListener('beforeunload',()=>send({drive:{x:0,y:0}}));
document.getElementById('manualMode').onclick=()=>setMode('manual');
document.getElementById('autoMode').onclick=()=>setMode('auto');
document.getElementById('servoLeft').onclick=()=>setServo('left');
document.getElementById('servoCenter').onclick=()=>setServo('center');
document.getElementById('servoRight').onclick=()=>setServo('right');
setMode('manual');
loadCamera();drawMap();connect();setInterval(()=>{sendDrive()},120);
</script>
</body></html>)HTML";

// =========================================================
// HTML: CAMERA SETTINGS PAGE (served by DevKit)
// =========================================================
const char settings_html[] PROGMEM = R"HTML(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>VORTEX Camera Settings</title>
<style>
:root{--bg:#070910;--panel:rgba(18,24,38,.86);--line:rgba(120,190,255,.22);--cyan:#43d9ff;--muted:#8ea4c2;--green:#36e69a;--red:#ff4e68}
*{box-sizing:border-box}body{margin:0;background:radial-gradient(circle at 20% 0%,#11203a 0,#080b13 44%,#05060b 100%);color:#eef5ff;font-family:Inter,Segoe UI,Arial,sans-serif}.top{display:flex;justify-content:space-between;align-items:center;gap:10px;padding:14px 16px;border-bottom:1px solid var(--line);background:rgba(5,7,12,.72);backdrop-filter:blur(12px);position:sticky;top:0}.brand{font-weight:800;letter-spacing:.1em;color:var(--cyan);font-size:14px}.nav{color:#eef5ff;text-decoration:none;border:1px solid var(--line);padding:8px 12px;border-radius:10px}.wrap{max-width:1100px;margin:auto;padding:16px}.preview{width:100%;aspect-ratio:16/9;background:#000;border-radius:16px;border:1px solid var(--line);overflow:hidden;margin-bottom:14px}.preview img{width:100%;height:100%;object-fit:cover}.grid{display:grid;grid-template-columns:1fr 1fr;gap:14px}.card{background:var(--panel);border:1px solid var(--line);border-radius:16px;padding:16px}.card h2{font-size:13px;letter-spacing:.08em;color:var(--cyan);margin:0 0 14px}.row{display:grid;grid-template-columns:1fr auto;gap:10px;align-items:center;margin:11px 0}.row input,.row select{width:180px;max-width:55vw;background:#0b1120;border:1px solid var(--line);color:#eef5ff;border-radius:8px;padding:8px}.btns{display:flex;flex-wrap:wrap;gap:8px;margin-top:12px}.btn{cursor:pointer;border:1px solid var(--line);background:rgba(255,255,255,.04);color:#eef5ff;border-radius:9px;padding:9px 12px}.btn:hover{border-color:var(--cyan)}.danger{border-color:rgba(255,78,104,.6);color:#ff7b8d}.msg{margin-top:12px;color:var(--muted);font-size:12px;min-height:18px}.hint{color:var(--muted);font-size:11px;line-height:1.55;margin-top:10px}@media(max-width:780px){.grid{grid-template-columns:1fr}}
</style></head>
<body>
<div class="top"><div class="brand">VORTEX // CAMERA SETTINGS</div><a id="dashboardLink" class="nav" href="dashboard_preview.html">CONTROL DASHBOARD</a></div>
<div class="wrap">
<div class="preview"><img id="preview" alt="Camera preview"></div>
<div class="grid">
<section class="card"><h2>CONNECTION</h2>
<div class="row"><label>ESP32-CAM IP</label><input id="ip" value="192.168.4.2"></div>
<div class="btns"><button class="btn" onclick="saveIp()">SAVE CAMERA IP</button><button class="btn" onclick="testPreview()">REFRESH PREVIEW</button></div>
<div class="hint">The camera is configured as a WiFi station on the DevKit AP. The default static station address is 192.168.4.2. Video still travels directly from the camera to the browser/laptop.</div>
</section>
<section class="card"><h2>FLASH</h2><div class="btns"><button class="btn" onclick="cmd('FLASH_ON')">FLASH ON</button><button class="btn danger" onclick="cmd('FLASH_OFF')">FLASH OFF</button></div></section>
<section class="card"><h2>FRAME SIZE</h2><div class="row"><label>Resolution</label><select id="frame"><option value="QQVGA">QQVGA 160x120</option><option value="QVGA">QVGA 320x240</option><option value="CIF">CIF 352x288</option><option value="VGA">VGA 640x480</option><option value="SVGA">SVGA 800x600</option><option value="XGA">XGA 1024x768</option><option value="HD">HD 1280x720</option><option value="SXGA">SXGA 1280x1024</option><option value="UXGA">UXGA 1600x1200</option></select></div><button class="btn" onclick="cmd('FRAME_'+document.getElementById('frame').value)">APPLY FRAME SIZE</button></section>
<section class="card"><h2>JPEG QUALITY</h2><div class="row"><label>Quality (10 = higher)</label><input id="quality" type="number" min="10" max="63" value="12"></div><button class="btn" onclick="cmd('QUALITY_'+document.getElementById('quality').value)">APPLY QUALITY</button></section>
<section class="card"><h2>IMAGE TUNING</h2>
<div class="row"><label>Brightness (-2..2)</label><input id="brightness" type="number" min="-2" max="2" value="0"></div>
<div class="row"><label>Contrast (-2..2)</label><input id="contrast" type="number" min="-2" max="2" value="0"></div>
<div class="row"><label>Saturation (-2..2)</label><input id="saturation" type="number" min="-2" max="2" value="0"></div>
<div class="btns"><button class="btn" onclick="cmd('BRIGHTNESS_'+document.getElementById('brightness').value)">BRIGHTNESS</button><button class="btn" onclick="cmd('CONTRAST_'+document.getElementById('contrast').value)">CONTRAST</button><button class="btn" onclick="cmd('SATURATION_'+document.getElementById('saturation').value)">SATURATION</button></div>
</section>
<section class="card"><h2>ORIENTATION / AUTO CONTROLS</h2>
<div class="btns"><button class="btn" onclick="cmd('HFLIP_1')">H-MIRROR ON</button><button class="btn" onclick="cmd('HFLIP_0')">H-MIRROR OFF</button><button class="btn" onclick="cmd('VFLIP_1')">V-FLIP ON</button><button class="btn" onclick="cmd('VFLIP_0')">V-FLIP OFF</button><button class="btn" onclick="cmd('AWB_1')">AUTO WB ON</button><button class="btn" onclick="cmd('AWB_0')">AUTO WB OFF</button><button class="btn" onclick="cmd('AGC_1')">AUTO GAIN ON</button><button class="btn" onclick="cmd('AGC_0')">AUTO GAIN OFF</button><button class="btn" onclick="cmd('AEC_1')">AUTO EXPOSURE ON</button><button class="btn" onclick="cmd('AEC_0')">AUTO EXPOSURE OFF</button></div>
<div class="hint">These controls are intentionally sent as lightweight UART housekeeping commands through the DevKit. The camera firmware accepts a compact command line and returns an acknowledgement when UART is connected.</div>
</section>
</div>
<div id="msg" class="msg"></div>
</div>
<script>
const ipEl=document.getElementById('ip'),preview=document.getElementById('preview'),msg=document.getElementById('msg');
const LOCAL_PREVIEW=(location.protocol==='file:' || location.protocol==='about:' || location.hostname==='localhost' || location.hostname==='127.0.0.1' || new URLSearchParams(location.search).get('preview')==='1');
const LOCAL_CAMERA_SVG='data:image/svg+xml;charset=utf-8,'+encodeURIComponent(`<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 1600 900'><rect width='1600' height='900' fill='#07101b'/><rect x='80' y='80' width='1440' height='740' rx='36' fill='none' stroke='#43d9ff' opacity='.22'/><text x='800' y='420' fill='#8ea4c2' font-family='Arial' font-size='54' text-anchor='middle'>ESP32-CAM LOCAL PREVIEW</text><text x='800' y='485' fill='#43d9ff' font-family='Arial' font-size='28' text-anchor='middle'>MJPEG stream simulated for offline dashboard testing</text></svg>`);
if(LOCAL_PREVIEW) document.title='VORTEX Camera Settings — Local Preview';

function camIp(){return ipEl.value.trim()||'192.168.4.2'}
function saveIp(){try{localStorage.setItem('cameraIp',camIp())}catch(e){}testPreview();msg.textContent='Camera IP saved in this browser.'}
function testPreview(){try{localStorage.setItem('cameraIp',camIp())}catch(e){} if(LOCAL_PREVIEW){preview.src=LOCAL_CAMERA_SVG;msg.textContent='LOCAL PREVIEW: camera stream and UART commands are simulated.';} else {preview.src=`http://${camIp()}:81/stream?t=${Date.now()}`}}
async function cmd(command){msg.textContent='Sending '+command+'...'; if(LOCAL_PREVIEW){await new Promise(r=>setTimeout(r,120)); msg.textContent='LOCAL PREVIEW: '+command+' simulated successfully.'; return;} try{const r=await fetch('/camera/cmd?cmd='+encodeURIComponent(command),{cache:'no-store'});const t=await r.text();msg.textContent=t}catch(e){msg.textContent='Failed to send command: '+e}}
try{const saved=localStorage.getItem('cameraIp');if(saved)ipEl.value=saved}catch(e){} if(!LOCAL_PREVIEW){document.getElementById('dashboardLink').href='/';} else {document.getElementById('dashboardLink').href='dashboard_preview.html';} testPreview();
</script></body></html>)HTML";

// =========================================================
// Motor control
// =========================================================
float motorTargetLeftNorm = 0.0f;
float motorTargetRightNorm = 0.0f;
float motorAppliedLeftNorm = 0.0f;
float motorAppliedRightNorm = 0.0f;

void setMotors(int leftSigned, int rightSigned) {
  leftSigned = constrain(leftSigned, -PWM_MAX, PWM_MAX);
  rightSigned = constrain(rightSigned, -PWM_MAX, PWM_MAX);

  const int lf = max(leftSigned, 0);
  const int lr = max(-leftSigned, 0);
  const int rf = max(rightSigned, 0);
  const int rr = max(-rightSigned, 0);

  ledcWrite(L_RPWM, lf);
  ledcWrite(L_LPWM, lr);
  ledcWrite(R_RPWM, rf);
  ledcWrite(R_LPWM, rr);
}

void setMotorTargetNorm(float left, float right) {
  if (!isfinite(left) || !isfinite(right)) {
    motorTargetLeftNorm = 0.0f;
    motorTargetRightNorm = 0.0f;
    motorStopRequested = true;
    commandQueueFault = true;
    Serial.println("NON-FINITE MOTOR TARGET -> FAIL-CLOSED STOP");
    return;
  }

  motorTargetLeftNorm = constrain(left, -1.0f, 1.0f);
  motorTargetRightNorm = constrain(right, -1.0f, 1.0f);
}

void forceMotorStopNow() {
  motorTargetLeftNorm = 0.0f;
  motorTargetRightNorm = 0.0f;
  motorAppliedLeftNorm = 0.0f;
  motorAppliedRightNorm = 0.0f;
  setMotors(0, 0);
}

void emergencyMotorStop() {
  // Safe cross-task request only. loop() performs the actual PWM write.
  motorStopRequested = true;
}

void applyEmergencyMotorStop() {
  forceMotorStopNow();
  motorStopRequested = false;
}

float slewToward(float current, float target, float dt) {
  const float limit =
      (fabsf(target) < fabsf(current)) ? MOTOR_DECEL_LIMIT_PER_SEC
                                       : MOTOR_ACCEL_LIMIT_PER_SEC;
  const float maxStep = limit * dt;
  const float delta = target - current;
  if (delta > maxStep) return current + maxStep;
  if (delta < -maxStep) return current - maxStep;
  return target;
}

void updateMotorOutput(uint32_t now) {
  // Defense-in-depth: only these explicit states may ever reach PWM output.
  const bool explicitDriveState =
      safetyState == STATE_MANUAL ||
      safetyState == STATE_AI_SESSION ||
      safetyState == STATE_HUMAN_AVOID_LEFT ||
      safetyState == STATE_HUMAN_AVOID_RIGHT ||
      safetyState == STATE_HUMAN_RECOVERY;

  if (!explicitDriveState ||
      motorStopRequested || driveWatchdogActive || commandQueueFault) {
    applyEmergencyMotorStop();
    return;
  }

  if (!isfinite(motorTargetLeftNorm) || !isfinite(motorTargetRightNorm) ||
      !isfinite(motorAppliedLeftNorm) || !isfinite(motorAppliedRightNorm)) {
    forceMotorStopNow();
    commandQueueFault = true;
    motorStopRequested = false;
    Serial.println("NON-FINITE MOTOR STATE -> FAIL-CLOSED STOP");
    return;
  }

  if (lastMotorOutputMs == 0) lastMotorOutputMs = now;
  const float dt =
      constrain((now - lastMotorOutputMs) / 1000.0f, 0.001f, 0.10f);
  lastMotorOutputMs = now;

  motorAppliedLeftNorm =
      slewToward(motorAppliedLeftNorm, motorTargetLeftNorm, dt);
  motorAppliedRightNorm =
      slewToward(motorAppliedRightNorm, motorTargetRightNorm, dt);

  setMotors(
      (int)roundf(motorAppliedLeftNorm * PWM_MAX),
      (int)roundf(motorAppliedRightNorm * PWM_MAX)
  );
}

const float AI_AVOID_MAX_SPEED = 0.30f;
const float AI_AVOID_MAX_TURN = 0.55f;

void initControlTaskWatchdog() {
  esp_task_wdt_config_t config = {};
  config.timeout_ms = CONTROL_TASK_WDT_TIMEOUT_MS;
  config.idle_core_mask = (1U << portNUM_PROCESSORS) - 1U;
  config.trigger_panic = true;

  esp_err_t status = esp_task_wdt_status(NULL);
  if (status == ESP_ERR_INVALID_STATE) {
    status = esp_task_wdt_init(&config);
  } else if (status == ESP_OK || status == ESP_ERR_NOT_FOUND) {
    esp_err_t reconfig = esp_task_wdt_reconfigure(&config);
    if (reconfig != ESP_OK && reconfig != ESP_ERR_INVALID_STATE) {
      Serial.printf("Control task WDT reconfigure failed: %d\n", (int)reconfig);
    }
  }

  if (status != ESP_OK && status != ESP_ERR_INVALID_STATE) {
    Serial.printf("Control task WDT init failed: %d\n", (int)status);
    return;
  }

  status = esp_task_wdt_add(NULL);
  if (status == ESP_OK || status == ESP_ERR_INVALID_STATE) {
    taskWatchdogReady = true;
    Serial.printf("Control task WDT ready timeout=%lu ms\n", (unsigned long)CONTROL_TASK_WDT_TIMEOUT_MS);
  } else {
    Serial.printf("Control task WDT subscribe failed: %d\n", (int)status);
  }
}

void feedControlTaskWatchdog() {
  if (taskWatchdogReady) {
    (void)esp_task_wdt_reset();
  }
}

float aiHealthSpeedScale() {
  if (!aiHealthValid) return AI_HEALTH_MIN_SPEED_SCALE;
  if (aiHealthScore <= AI_HEALTH_STOP_THRESHOLD) return 0.0f;

  const float cautionMax = 0.75f;
  if (aiHealthScore <= AI_HEALTH_CAUTION_THRESHOLD) {
    const float t = (float)(aiHealthScore - AI_HEALTH_STOP_THRESHOLD) /
                    (float)max(1, (int)AI_HEALTH_CAUTION_THRESHOLD - (int)AI_HEALTH_STOP_THRESHOLD);
    return AI_HEALTH_MIN_SPEED_SCALE +
           t * (cautionMax - AI_HEALTH_MIN_SPEED_SCALE);
  }

  if (aiHealthScore >= 90) return 1.0f;
  const float t = (float)(aiHealthScore - AI_HEALTH_CAUTION_THRESHOLD) /
                  (float)(90 - AI_HEALTH_CAUTION_THRESHOLD);
  return cautionMax + t * (1.0f - cautionMax);
}


const char* safetyStateName(SafetyState state) {
  switch (state) {
    case STATE_MANUAL:            return "manual";
    case STATE_AI_SESSION:        return "ai_session";
    case STATE_HUMAN_AVOID_LEFT:  return "human_avoid_left";
    case STATE_HUMAN_AVOID_RIGHT: return "human_avoid_right";
    case STATE_HUMAN_RECOVERY:    return "human_recovery";
    case STATE_HUMAN_STOP:        return "human_stop";
    case STATE_HUMANS_BLOCKED:    return "blocked_by_humans";
    case STATE_AI_LINK_LOSS:      return "ai_link_loss";
    case STATE_WAIT_NEUTRAL:      return "wait_neutral";
    case STATE_WAIT_NEW_DRIVE:    return "wait_new_drive";
    default:                      return "unknown";
  }
}

const char* currentStopReason();

bool stateAllowsManualDrive() {
  return safetyState == STATE_MANUAL;
}

bool stateAllowsAiAvoidance() {
  return safetyState == STATE_HUMAN_AVOID_LEFT ||
         safetyState == STATE_HUMAN_AVOID_RIGHT;
}

bool stateAllowsAiRecovery() {
  return safetyState == STATE_HUMAN_RECOVERY;
}

bool stateAllowsAiDrive() {
  return safetyState == STATE_AI_SESSION;
}

void broadcastAiState(const char* reason, uint32_t seq) {
  StaticJsonDocument<640> doc;
  const SafetyState state = safetyState;

  doc["type"] = "ai_state";
  doc["state"] = safetyStateName(state);
  doc["control_mode"] = autonomousMode ? "auto" : "manual";
  doc["stopped"] =
      (state == STATE_HUMAN_STOP ||
       state == STATE_HUMANS_BLOCKED ||
       state == STATE_AI_LINK_LOSS ||
       state == STATE_WAIT_NEUTRAL ||
       state == STATE_WAIT_NEW_DRIVE ||
       driveWatchdogActive);
  const bool motorGateAllowed =
      !motorStopRequested && !driveWatchdogActive &&
      (stateAllowsManualDrive() ||
       stateAllowsAiDrive() ||
       stateAllowsAiAvoidance() ||
       stateAllowsAiRecovery());
  doc["motor_enabled"] = motorGateAllowed;
  doc["ai_drive_speed"] = aiDriveSpeed;
  doc["ai_drive_turn"] = aiDriveTurn;
  doc["reason"] = currentStopReason();
  doc["event_reason"] = reason;
  doc["seq"] = seq;
  doc["ai_lease"] = aiLeaseActive;
  doc["ai_age_ms"] = aiLeaseActive ? (millis() - lastAiPacketMs) : -1;
  doc["ai_speed"] = aiAvoidSpeed;
  doc["ai_turn"] = aiAvoidTurn;
  doc["ai_recovery_speed"] = aiRecoverySpeed;
  doc["ai_health_score"] = aiHealthScore;
  doc["ai_health_valid"] = aiHealthValid;
  doc["ai_frame_age_ms"] = aiFrameAgeMs;
  doc["ai_inference_ms"] = aiInferenceMs;
  doc["ai_ws_rtt_ms"] = aiWsRttMs;
  doc["ai_vision_fps"] = aiVisionFps;
  doc["ai_optical_flow_mag"] = aiOpticalFlowMag;
  doc["ai_decision_age_ms"] = aiDecisionAgeMs;
  doc["ai_speed_scale"] = aiHealthSpeedScale();
  doc["motor_target_left"] = motorTargetLeftNorm;
  doc["motor_target_right"] = motorTargetRightNorm;
  doc["motor_applied_left"] = motorAppliedLeftNorm;
  doc["motor_applied_right"] = motorAppliedRightNorm;

  doc["left_zone_occupied"] = aiLeftZoneOccupied;
  doc["right_zone_occupied"] = aiRightZoneOccupied;
  doc["side_clear_ms"] = aiSideClearHoldMs;
  doc["side_clear_required_ms"] = aiSideClearRequiredMs;
  doc["side_clear_remaining_ms"] = aiReleaseQualified ? 0UL : (aiSideClearHoldMs >= aiSideClearRequiredMs ? 0UL : (aiSideClearRequiredMs - aiSideClearHoldMs));
  doc["blocked_hold_ms"] = aiBlockedHoldMs;
  doc["blocked_hold_required_ms"] = aiBlockedHoldRequiredMs;
  doc["side_candidate"] = aiSideCandidate;
  doc["release_qualified"] = aiReleaseQualified;
  doc["release_side"] = aiReleaseSide;
  doc["resume_condition"] = aiResumeCondition;

  String out;
  serializeJson(doc, out);
  ws.textAll(out);
}

void latchHumanStop(uint32_t seq, const char* reasonText = "human") {
  stoppedLatched = true;
  aiStopReason = (strcmp(reasonText, "human") == 0);
  stopReason = reasonText;
  requireFreshDriveAfterStop = true;

  joystickX = 0.0f;
  joystickY = 0.0f;
  aiAvoidSpeed = 0.0f;
  aiAvoidTurn = 0.0f;
  aiDriveSpeed = 0.0f;
  aiDriveTurn = 0.0f;
  aiRecoverySpeed = 0.0f;

  aiReleaseQualified = false;
  aiReleaseSide = "none";

  safetyState = STATE_HUMAN_STOP;

  Serial.printf(
      "HUMAN STOP LATCHED seq=%lu\n",
      (unsigned long)seq
  );

  broadcastAiState(reasonText, seq);
}

void latchHumansBlocked(uint32_t seq) {
  stoppedLatched = true;
  stopReason = "blocked_by_humans";
  aiStopReason = true;
  requireFreshDriveAfterStop = true;

  joystickX = 0.0f;
  joystickY = 0.0f;
  aiAvoidSpeed = 0.0f;
  aiAvoidTurn = 0.0f;
  aiDriveSpeed = 0.0f;
  aiDriveTurn = 0.0f;
  aiRecoverySpeed = 0.0f;

  aiReleaseQualified = false;
  aiReleaseSide = "none";

  safetyState = STATE_HUMANS_BLOCKED;

  Serial.printf(
      "BOTH SIDES BLOCKED BY HUMANS -> HARD STOP seq=%lu\n",
      (unsigned long)seq
  );

  broadcastAiState("blocked_by_humans", seq);
}

void latchAiLinkLoss(uint32_t seq) {
  stoppedLatched = true;
  stopReason = "ai_link_lost";
  aiStopReason = false;
  requireFreshDriveAfterStop = true;

  joystickX = 0.0f;
  joystickY = 0.0f;
  aiAvoidSpeed = 0.0f;
  aiAvoidTurn = 0.0f;
  aiDriveSpeed = 0.0f;
  aiDriveTurn = 0.0f;
  aiRecoverySpeed = 0.0f;

  aiReleaseQualified = false;
  aiReleaseSide = "none";
  aiHealthValid = false;
  aiHealthScore = 0;
  aiOpticalFlowMag = 0.0f;
  aiDecisionAgeMs = 0;

  safetyState = STATE_AI_LINK_LOSS;

  Serial.printf(
      "AI LINK LOSS -> FAIL-CLOSED STOP seq=%lu\n",
      (unsigned long)seq
  );

  broadcastAiState("ai_link_lost", seq);
}

bool aiSequenceIsNew(uint32_t seq) {
  if (!lastAiSeqValid) return true;

  const uint32_t diff = seq - lastAiSeq;
  return diff != 0U && diff < 0x80000000UL;
}

void updateReleaseQualification(
    uint32_t sideClearMs,
    uint32_t requiredMs,
    const String& chosenSide) {

  aiSideClearRequiredMs =
      max((uint32_t)AI_SIDE_CLEAR_HOLD_MS, requiredMs);

  const bool leftChosen =
      chosenSide == "left";
  const bool rightChosen =
      chosenSide == "right";

  const bool chosenSideClear =
      (leftChosen && !aiLeftZoneOccupied) ||
      (rightChosen && !aiRightZoneOccupied);

  const bool holdComplete =
      sideClearMs >= aiSideClearRequiredMs;

  aiReleaseQualified =
      chosenSideClear && holdComplete;

  aiReleaseSide =
      aiReleaseQualified ? chosenSide : "none";
}

void enterAiSession(uint32_t seq) {
  stoppedLatched = false;
  aiStopReason = false;
  requireFreshDriveAfterStop = false;

  aiAvoidSpeed = 0.0f;
  aiAvoidTurn = 0.0f;
  aiDriveSpeed = 0.0f;
  aiDriveTurn = 0.0f;
  aiRecoverySpeed = 0.0f;

  aiReleaseQualified = false;
  aiReleaseSide = "none";

  safetyState = STATE_AI_SESSION;

  Serial.printf(
      "AI SESSION ACTIVE seq=%lu\n",
      (unsigned long)seq
  );

  broadcastAiState("ai_session", seq);
}

bool acceptAiGo(uint32_t seq, uint32_t clearMs) {
  if (safetyState == STATE_AI_LINK_LOSS) {
    Serial.printf(
        "AI GO rejected while link-loss is active seq=%lu\n",
        (unsigned long)seq
    );
    return false;
  }

  // Recovery must finish through the gradual DRIVE ramp. GO is never a
  // shortcut out of recovery.
  if (safetyState == STATE_HUMAN_RECOVERY) {
    Serial.printf(
        "AI GO rejected from recovery: DRIVE ramp required seq=%lu\n",
        (unsigned long)seq
    );
    motorStopRequested = true;
    return false;
  }

  // BLOCKED_BY_HUMANS may NEVER be released by GO.
  // It can only clear through a side-clear-confirmed AVOID packet, which
  // becomes the beginning of the normal avoidance -> recovery sequence.
  if (safetyState == STATE_HUMANS_BLOCKED) {
    emergencyMotorStop();
    Serial.printf(
        "AI GO rejected from blocked state: side-clear AVOID required seq=%lu\n",
        (unsigned long)seq
    );
    broadcastAiState("blocked_requires_side_clear_avoid", seq);
    return false;
  }

  // GO is never a motor-enable command after a hard stop.
  // Only the actual HUMAN detection stop can enter WAIT_NEUTRAL here.
  // Sensor/control-integrity stops must recover through a new AI session.
  if (safetyState == STATE_HUMAN_STOP &&
      (stopReason == "human" || stopReason == "vision_inference_fault" ||
       stopReason == "camera_content_frozen" || stopReason == "vision_command_stale")) {
    // Existing clear-hold behavior remains enforced on the laptop.
    // The DevKit also requires a non-zero clear interval to prevent an
    // accidental immediate GO after a STOP packet.
    if (clearMs < AI_SIDE_CLEAR_HOLD_MS) {
      Serial.printf(
          "AI GO rejected: clear hold=%lu ms < %lu ms seq=%lu\n",
          (unsigned long)clearMs,
          (unsigned long)AI_SIDE_CLEAR_HOLD_MS,
          (unsigned long)seq
      );
      return false;
    }

    stoppedLatched = false;
    aiStopReason = false;
    requireFreshDriveAfterStop = true;
    aiReleaseQualified = false;
    aiReleaseSide = "none";

    joystickX = 0.0f;
    joystickY = 0.0f;
    aiAvoidSpeed = 0.0f;
    aiAvoidTurn = 0.0f;
    aiRecoverySpeed = 0.0f;

    safetyState = STATE_WAIT_NEUTRAL;

    Serial.printf(
        "AI GO -> WAIT_NEUTRAL seq=%lu\n",
        (unsigned long)seq
    );

    emergencyMotorStop();
    broadcastAiState("go_wait_neutral", seq);
    return true;
  }

  if (safetyState == STATE_AI_SESSION) {
    aiReleaseQualified = false;
    aiReleaseSide = "none";
    broadcastAiState("go_refresh", seq);
    return true;
  }

  // AVOID states must finish through RECOVERY, not directly through GO.
  if (stateAllowsAiAvoidance()) {
    Serial.printf(
        "AI GO rejected from avoidance; require RECOVER first seq=%lu\n",
        (unsigned long)seq
    );
    return false;
  }

  // No AI session: GO alone can never establish motion authority.
  Serial.printf(
      "AI GO ignored in state=%s seq=%lu\n",
      safetyStateName(safetyState),
      (unsigned long)seq
  );
  return false;
}

bool acceptAiAvoid(
    uint32_t seq,
    float speed,
    float turn,
    uint32_t sideClearMs) {

  if (!aiLeaseActive) return false;

  // AVOID may start from normal AI driving, a previous avoidance,
  // recovery, or a hard human stop that is being re-evaluated.
  const bool allowed =
      safetyState == STATE_AI_SESSION ||
      stateAllowsAiAvoidance() ||
      stateAllowsAiRecovery() ||
      safetyState == STATE_HUMAN_STOP ||
      safetyState == STATE_HUMANS_BLOCKED;

  if (!allowed) {
    Serial.printf(
        "AI AVOID rejected in state=%s seq=%lu\n",
        safetyStateName(safetyState),
        (unsigned long)seq
    );
    return false;
  }

  // Firmware-side validation: an AVOID command is accepted only when the
  // packet's selected side, occupancy flags, and completed hold are internally
  // consistent. The DevKit cannot independently see the camera, so it treats
  // inconsistent/missing proof as unsafe and keeps the current stop state.
  if (!aiReleaseQualified || aiReleaseSide == "none" ||
      sideClearMs < AI_SIDE_CLEAR_HOLD_MS) {
    Serial.printf(
        "AI AVOID rejected: side qualification incomplete seq=%lu clear=%lu required=%lu side=%s\n",
        (unsigned long)seq,
        (unsigned long)sideClearMs,
        (unsigned long)AI_SIDE_CLEAR_HOLD_MS,
        aiReleaseSide.c_str()
    );

    motorStopRequested = true;
    if (safetyState == STATE_HUMANS_BLOCKED) {
      stoppedLatched = true;
      aiStopReason = true;
      requireFreshDriveAfterStop = true;
    } else {
      latchHumanStop(seq);
    }
    return false;
  }

  const bool chosenSideMatchesTurn =
      (aiReleaseSide == "left" && turn < -0.001f) ||
      (aiReleaseSide == "right" && turn > 0.001f);

  if (!chosenSideMatchesTurn) {
    Serial.printf(
        "AI AVOID rejected: turn does not match qualified side=%s seq=%lu\n",
        aiReleaseSide.c_str(),
        (unsigned long)seq
    );
    motorStopRequested = true;
    if (safetyState == STATE_HUMANS_BLOCKED) {
      stoppedLatched = true;
      aiStopReason = true;
      requireFreshDriveAfterStop = true;
    } else {
      latchHumanStop(seq);
    }
    return false;
  }

  // Reject a claimed clear side if the same packet says that side is occupied.
  if ((aiReleaseSide == "left" && aiLeftZoneOccupied) ||
      (aiReleaseSide == "right" && aiRightZoneOccupied)) {
    Serial.printf(
        "AI AVOID rejected: qualified side reported occupied seq=%lu\n",
        (unsigned long)seq
    );
    motorStopRequested = true;
    if (safetyState == STATE_HUMANS_BLOCKED) {
      stoppedLatched = true;
      aiStopReason = true;
      requireFreshDriveAfterStop = true;
    } else {
      latchHumanStop(seq);
    }
    return false;
  }

  const float healthScale = aiHealthSpeedScale();
  aiAvoidSpeed =
      constrain(speed, 0.0f, AI_AVOID_MAX_SPEED) * healthScale;
  aiAvoidTurn =
      constrain(turn, -AI_AVOID_MAX_TURN, AI_AVOID_MAX_TURN);
  lastAcceptedMotionCommandMs = millis();
  aiRecoverySpeed = 0.0f;

  if (aiAvoidTurn < -0.001f) {
    safetyState = STATE_HUMAN_AVOID_LEFT;
  } else if (aiAvoidTurn > 0.001f) {
    safetyState = STATE_HUMAN_AVOID_RIGHT;
  } else {
    // A zero-turn AVOID is not a meaningful side-steer.
    Serial.printf(
        "AI AVOID rejected: zero turn seq=%lu\n",
        (unsigned long)seq
    );
    latchHumanStop(seq);
    return false;
  }

  stoppedLatched = false;
  aiStopReason = false;
  requireFreshDriveAfterStop = false;

  Serial.printf(
      "AI AVOID %s speed=%.2f turn=%.2f clear=%lu ms seq=%lu\n",
      safetyState == STATE_HUMAN_AVOID_LEFT ? "LEFT" : "RIGHT",
      aiAvoidSpeed,
      aiAvoidTurn,
      (unsigned long)sideClearMs,
      (unsigned long)seq
  );

  broadcastAiState("human_avoid", seq);
  return true;
}

bool acceptAiRecovery(uint32_t seq, float speed) {
  if (!aiLeaseActive) return false;

  const bool allowed =
      safetyState == STATE_AI_SESSION ||
      stateAllowsAiAvoidance() ||
      safetyState == STATE_HUMAN_RECOVERY;

  if (!allowed) {
    Serial.printf(
        "AI RECOVER rejected in state=%s seq=%lu\n",
        safetyStateName(safetyState),
        (unsigned long)seq
    );
    return false;
  }

  aiRecoverySpeed =
      constrain(speed, 0.0f, AI_NORMAL_MAX_SPEED) * aiHealthSpeedScale();
  lastAcceptedMotionCommandMs = millis();

  aiAvoidSpeed = 0.0f;
  aiAvoidTurn = 0.0f;
  stoppedLatched = false;
  aiStopReason = false;
  requireFreshDriveAfterStop = false;

  safetyState = STATE_HUMAN_RECOVERY;

  Serial.printf(
      "AI RECOVERY speed=%.2f seq=%lu\n",
      aiRecoverySpeed,
      (unsigned long)seq
  );

  broadcastAiState("human_recovery", seq);
  return true;
}

bool acceptAiDrive(
    uint32_t seq,
    float speed,
    float turn,
    uint32_t clearMs,
    uint32_t rampMs) {

  (void)clearMs;

  if (!aiLeaseActive) return false;

  // AI DRIVE is never an escape hatch from a hard human stop or a
  // blocked-by-humans state. Those states must recover through AVOID and
  // the established slow recovery sequence.
  if (safetyState == STATE_HUMAN_STOP ||
      safetyState == STATE_HUMANS_BLOCKED) {
    emergencyMotorStop();
    Serial.printf(
        "AI DRIVE rejected from safety stop state=%s seq=%lu\n",
        safetyStateName(safetyState),
        (unsigned long)seq
    );
    return false;
  }

  // Recovery -> normal AI driving transition: the laptop must present a
  // completed speed ramp, and the DevKit independently enforces that the
  // requested ramp time is long enough.
  if (safetyState == STATE_HUMAN_RECOVERY) {
    if (clearMs < AI_RECOVERY_CLEAR_HOLD_MS) {
      Serial.printf(
          "AI DRIVE rejected: recovery clear=%lu ms < required=%lu ms seq=%lu\n",
          (unsigned long)clearMs,
          (unsigned long)AI_RECOVERY_CLEAR_HOLD_MS,
          (unsigned long)seq
      );
      motorStopRequested = true;
      return false;
    }

    if (rampMs < AI_NORMAL_RAMP_MIN_MS) {
      Serial.printf(
          "AI DRIVE rejected: recovery ramp=%lu ms < minimum=%lu ms seq=%lu\n",
          (unsigned long)rampMs,
          (unsigned long)AI_NORMAL_RAMP_MIN_MS,
          (unsigned long)seq
      );
      motorStopRequested = true;
      return false;
    }
  }

  if (safetyState == STATE_WAIT_NEUTRAL ||
      safetyState == STATE_WAIT_NEW_DRIVE ||
      safetyState == STATE_AI_LINK_LOSS) {
    return false;
  }

  const float s = constrain(speed, 0.0f, AI_NORMAL_MAX_SPEED) * aiHealthSpeedScale();
  const float t = constrain(turn, -AI_NORMAL_MAX_TURN, AI_NORMAL_MAX_TURN);

  aiDriveSpeed = s;
  lastAcceptedMotionCommandMs = millis();
  aiDriveTurn = t;

  if (safetyState == STATE_HUMAN_RECOVERY) {
    // Final recovery packet is the endpoint of the laptop's gradual ramp.
    aiRecoverySpeed = s;
  }

  safetyState = STATE_AI_SESSION;
  requireFreshDriveAfterStop = false;

  Serial.printf(
      "AI DRIVE speed=%.3f turn=%.3f seq=%lu\n",
      aiDriveSpeed,
      aiDriveTurn,
      (unsigned long)seq
  );

  broadcastAiState("ai_drive", seq);
  return true;
}

void applyAiDrive() {
  if (!stateAllowsAiDrive() || driveWatchdogActive) {
    setMotorTargetNorm(0.0f, 0.0f);
    return;
  }

  const float speed = constrain(aiDriveSpeed, 0.0f, AI_NORMAL_MAX_SPEED);
  const float turn = constrain(aiDriveTurn, -AI_NORMAL_MAX_TURN, AI_NORMAL_MAX_TURN);
  float left = speed + turn;
  float right = speed - turn;
  const float maxMag = max(1.0f, max(fabsf(left), fabsf(right)));
  left /= maxMag;
  right /= maxMag;
  setMotorTargetNorm(left, right);
}

void applyJoystickDrive() {
  if (!stateAllowsManualDrive() || driveWatchdogActive) {
    setMotorTargetNorm(0.0f, 0.0f);
    return;
  }

  float left = joystickY + joystickX;
  float right = joystickY - joystickX;
  const float maxMag = max(1.0f, max(fabsf(left), fabsf(right)));
  left /= maxMag;
  right /= maxMag;
  setMotorTargetNorm(left, right);
}

void applyAiAvoidance() {
  if (!stateAllowsAiAvoidance() || driveWatchdogActive) {
    setMotorTargetNorm(0.0f, 0.0f);
    return;
  }

  const float speed = constrain(aiAvoidSpeed, 0.0f, AI_AVOID_MAX_SPEED);
  const float maxTurnForSpeed = min(AI_AVOID_MAX_TURN, speed);
  const float turn = constrain(aiAvoidTurn, -maxTurnForSpeed, maxTurnForSpeed);
  float left = speed + turn;
  float right = speed - turn;
  const float maxMag = max(1.0f, max(fabsf(left), fabsf(right)));
  left /= maxMag;
  right /= maxMag;
  setMotorTargetNorm(left, right);
}

void applyAiRecovery() {
  if (!stateAllowsAiRecovery() || driveWatchdogActive) {
    setMotorTargetNorm(0.0f, 0.0f);
    return;
  }

  const float speed = constrain(aiRecoverySpeed, 0.0f, AI_NORMAL_MAX_SPEED);
  setMotorTargetNorm(speed, speed);
}

// =========================================================
// Hall ISR
// =========================================================
void IRAM_ATTR countTick() {
  wheelTicks++;
}

// =========================================================
// MPU6050 calibration / reset
// =========================================================
void calibrateGyro() {
  if (!mpuReady) return;
  Serial.println("Calibrating MPU6050 gyro Z... keep rover still.");
  const int samples = 500;
  double sum = 0.0;
  sensors_event_t a, g, temp;
  for (int i = 0; i < samples; i++) {
    mpu.getEvent(&a, &g, &temp);
    sum += g.gyro.z;
    delay(5);
  }
  gyroZOffset = (float)(sum / samples);
  Serial.printf("Gyro Z offset = %.7f rad/s\n", gyroZOffset);
}

void resetOdometry() {
  noInterrupts();
  lastTicks = wheelTicks;
  interrupts();
  yawAngleRad = 0.0f;
  posXcm = 0.0f;
  posYcm = 0.0f;
  totalDistanceCm = 0.0f;
  measuredSpeedCmS = 0.0f;
}

float normalizedHeadingDeg() {
  float d = yawAngleRad * 180.0f / PI;
  d = fmodf(d, 360.0f);
  if (d < 0) d += 360.0f;
  return d;
}

// =========================================================
// Optional battery
// =========================================================
uint8_t computeOdomConfidence() {
  uint8_t score = mpuReady ? 65 : 25;
  const bool motion = stateAllowsManualDrive() || stateAllowsAiDrive() ||
                      stateAllowsAiAvoidance() || stateAllowsAiRecovery();
  if (motion && max(fabsf(motorTargetLeftNorm), fabsf(motorTargetRightNorm)) >= HALL_FEEDBACK_MIN_COMMAND) {
    const uint32_t hallAge = (lastHallTickMs == 0) ? 0UL : (millis() - lastHallTickMs);
    if (hallAge > HALL_FEEDBACK_TIMEOUT_MS) score = 0;
    else if (hallAge > 300) score = min<uint8_t>(score, 35);
    else if (hallAge > 180) score = min<uint8_t>(score, 50);
  }
  if (!mpuReady && motion) score = min<uint8_t>(score, 20);
  return score;
}

float readBatteryVoltage() {
#if USE_BATTERY_ADC
  int raw = analogRead(BATTERY_ADC_PIN);
  float vadc = (raw * ADC_REFERENCE_V) / ADC_MAX_COUNTS;
  return vadc * (BATTERY_R1_OHMS + BATTERY_R2_OHMS) / BATTERY_R2_OHMS;
#else
  return -1.0f;
#endif
}

// =========================================================
// Telemetry
// =========================================================
const char* currentStopReason() {
  switch (safetyState) {
    case STATE_HUMAN_STOP:        return stopReason.c_str();
    case STATE_HUMANS_BLOCKED:    return "blocked_by_humans";
    case STATE_AI_LINK_LOSS:      return "ai_link_lost";
    case STATE_WAIT_NEUTRAL:      return "awaiting_neutral";
    case STATE_WAIT_NEW_DRIVE:    return "awaiting_new_drive";
    case STATE_HUMAN_AVOID_LEFT:  return "human_avoid_left";
    case STATE_HUMAN_AVOID_RIGHT: return "human_avoid_right";
    case STATE_HUMAN_RECOVERY:    return "human_recovery";
    case STATE_MANUAL:            return "manual";
    case STATE_AI_SESSION:        return "ai_session";
    default:                      return driveWatchdogActive ? "drive_watchdog" : "none";
  }
}

String makeTelemetryJson() {
  StaticJsonDocument<640> doc;

  const SafetyState state = safetyState;
  const bool motorEnabled =
      (state == STATE_MANUAL ||
       state == STATE_AI_SESSION ||
       state == STATE_HUMAN_AVOID_LEFT ||
       state == STATE_HUMAN_AVOID_RIGHT ||
       state == STATE_HUMAN_RECOVERY);

  const bool actualStopped =
      (state == STATE_HUMAN_STOP ||
       state == STATE_HUMANS_BLOCKED ||
       state == STATE_AI_LINK_LOSS ||
       state == STATE_WAIT_NEUTRAL ||
       state == STATE_WAIT_NEW_DRIVE ||
       driveWatchdogActive);

  doc["type"] = "telemetry";
  doc["state"] = safetyStateName(state);
  doc["control_mode"] = autonomousMode ? "auto" : "manual";
  doc["motor_enabled"] = motorEnabled;

  doc["speed"] = measuredSpeedCmS;
  doc["heading"] = normalizedHeadingDeg();
  doc["x"] = posXcm;
  doc["y"] = posYcm;
  doc["motor_target_left"] = motorTargetLeftNorm;
  doc["motor_target_right"] = motorTargetRightNorm;
  doc["motor_applied_left"] = motorAppliedLeftNorm;
  doc["motor_applied_right"] = motorAppliedRightNorm;
  doc["odom_mpu"] = mpuReady;
  doc["odom_hall_age_ms"] = (lastHallTickMs == 0 ? 0UL : (millis() - lastHallTickMs));
  doc["odom_confidence"] = computeOdomConfidence();
  doc["distance"] = totalDistanceCm;
  doc["battery"] = readBatteryVoltage();
  doc["hall_feedback_ok"] = !(lastHallTickMs > 0 &&
                               ((stateAllowsManualDrive() || stateAllowsAiDrive() ||
                                 stateAllowsAiAvoidance() || stateAllowsAiRecovery()) &&
                                max(fabsf(motorTargetLeftNorm), fabsf(motorTargetRightNorm)) >= HALL_FEEDBACK_MIN_COMMAND &&
                                millis() - lastHallTickMs > HALL_FEEDBACK_TIMEOUT_MS));
  doc["command_integrity_ok"] = !commandQueueFault;
  doc["control_loop_health"] = true;

  doc["stopped"] = actualStopped;
  doc["ai"] = aiStopReason;
  doc["stop_reason"] = currentStopReason();

  doc["ai_lease"] = aiLeaseActive;
  doc["ai_age_ms"] =
      aiLeaseActive ? (millis() - lastAiPacketMs) : -1;
  doc["ai_seq"] =
      lastAiSeqValid ? lastAiSeq : 0;

  doc["ai_speed"] = aiAvoidSpeed;
  doc["ai_turn"] = aiAvoidTurn;
  doc["ai_recovery_speed"] = aiRecoverySpeed;
  doc["ai_health_score"] = aiHealthScore;
  doc["ai_health_valid"] = aiHealthValid;
  doc["ai_frame_age_ms"] = aiFrameAgeMs;
  doc["ai_inference_ms"] = aiInferenceMs;
  doc["ai_ws_rtt_ms"] = aiWsRttMs;
  doc["ai_vision_fps"] = aiVisionFps;
  doc["ai_optical_flow_mag"] = aiOpticalFlowMag;
  doc["ai_decision_age_ms"] = aiDecisionAgeMs;
  doc["ai_speed_scale"] = aiHealthSpeedScale();
  doc["ai_drive_speed"] = aiDriveSpeed;
  doc["ai_drive_turn"] = aiDriveTurn;

  doc["left_zone_occupied"] = aiLeftZoneOccupied;
  doc["right_zone_occupied"] = aiRightZoneOccupied;
  doc["side_clear_ms"] = aiSideClearHoldMs;
  doc["side_clear_required_ms"] = aiSideClearRequiredMs;
  doc["side_clear_remaining_ms"] = aiReleaseQualified ? 0UL : (aiSideClearHoldMs >= aiSideClearRequiredMs ? 0UL : (aiSideClearRequiredMs - aiSideClearHoldMs));
  doc["blocked_hold_ms"] = aiBlockedHoldMs;
  doc["blocked_hold_required_ms"] = aiBlockedHoldRequiredMs;
  doc["side_candidate"] = aiSideCandidate;
  doc["release_qualified"] = aiReleaseQualified;
  doc["release_side"] = aiReleaseSide;
  doc["resume_condition"] = aiResumeCondition;

  doc["joy_x"] = joystickX;
  doc["joy_y"] = joystickY;

  String out;
  serializeJson(doc, out);
  return out;
}

void broadcastTelemetry() {
  ws.textAll(makeTelemetryJson());
}

// =========================================================
// ESP32-CAM UART bridge
// =========================================================
void sendCamUartCommand(const String& cmd) {
  Serial2.println(cmd);
  Serial.printf("CAM UART TX: %s\n", cmd.c_str());
}

void pollCamUart() {
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    if (c == '\n') {
      camUartLine.trim();
      if (camUartLine.length()) {
        StaticJsonDocument<192> doc;
        doc["type"] = "cam_ack";
        doc["message"] = camUartLine;
        String out;
        serializeJson(doc, out);
        ws.textAll(out);
        Serial.printf("CAM UART RX: %s\n", camUartLine.c_str());
      }
      camUartLine = "";
    } else if (c != '\r' && camUartLine.length() < 120) {
      camUartLine += c;
    }
  }
}

bool validCamCommand(const String& cmd) {
  if (cmd == "FLASH_ON" || cmd == "FLASH_OFF") return true;
  if (cmd.startsWith("FRAME_")) return true;
  if (cmd.startsWith("QUALITY_")) return true;
  if (cmd.startsWith("BRIGHTNESS_")) return true;
  if (cmd.startsWith("CONTRAST_")) return true;
  if (cmd.startsWith("SATURATION_")) return true;
  if (cmd == "HFLIP_0" || cmd == "HFLIP_1") return true;
  if (cmd == "VFLIP_0" || cmd == "VFLIP_1") return true;
  if (cmd == "AWB_0" || cmd == "AWB_1") return true;
  if (cmd == "AGC_0" || cmd == "AGC_1") return true;
  if (cmd == "AEC_0" || cmd == "AEC_1") return true;
  return false;
}

// =========================================================
// WebSocket event handler: parse/copy only; state changes happen in loop().
// =========================================================
void queueWsCommand(const WsCommand& cmd) {
  if (wsCommandQueue != nullptr &&
      xQueueSend(wsCommandQueue, &cmd, 0) == pdPASS) {
    return;
  }

  // Queue overflow must never become permission to keep driving.
  motorStopRequested = true;
  commandQueueFault = true;
  Serial.println("WS command queue full -> FAIL-CLOSED STOP");
}

template<size_t N> void copyDocString(StaticJsonDocument<N>& doc, const char* key, char* dst, size_t cap) {
  const char* value = doc[key] | "";
  if (cap == 0) return;
  strncpy(dst, value, cap - 1);
  dst[cap - 1] = '\0';
}

void onWsEvent(
    AsyncWebSocket* server,
    AsyncWebSocketClient* client,
    AwsEventType type,
    void* arg,
    uint8_t* data,
    size_t len) {

  (void)server;
  (void)arg;

  if (type == WS_EVT_DISCONNECT) {
    WsCommand cmd;
    cmd.type = WS_CMD_DISCONNECT;
    cmd.clientId = client->id();
    queueWsCommand(cmd);
    return;
  }

  if (type != WS_EVT_DATA) return;

  AwsFrameInfo* info = (AwsFrameInfo*)arg;
  if (!(info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)) {
    return;
  }

  StaticJsonDocument<448> doc;
  DeserializationError error = deserializeJson(doc, data, len);
  if (error) {
    Serial.printf("WS JSON error: %s\n", error.c_str());
    motorStopRequested = true;
    commandQueueFault = true;
    return;
  }

  const bool hasAi = doc.containsKey("ai");
  const bool hasDrive = doc.containsKey("drive");
  const bool hasMode = doc.containsKey("mode");
  const bool hasServo = doc.containsKey("servo");

  // Mixed safety + joystick packet is always rejected.
  if (hasAi && hasDrive) {
    Serial.println("WS rejected ambiguous packet: ai + drive -> STOP");
    motorStopRequested = true;
    commandQueueFault = true;
    return;
  }

  WsCommand cmd;
  cmd.clientId = client->id();
  cmd.hasAi = hasAi;
  cmd.hasDrive = hasDrive;
  cmd.hasMode = hasMode;
  cmd.hasServo = hasServo;

  if (hasMode) {
    copyDocString(doc, "mode", cmd.mode, sizeof(cmd.mode));
  }

  if (hasServo) {
    copyDocString(doc, "servo", cmd.servo, sizeof(cmd.servo));
  }

  if (hasAi) {
    copyDocString(doc, "ai", cmd.ai, sizeof(cmd.ai));
    copyDocString(doc, "reason", cmd.reason, sizeof(cmd.reason));
    copyDocString(doc, "chosen_side", cmd.chosenSide, sizeof(cmd.chosenSide));
    copyDocString(doc, "resume_condition", cmd.resumeCondition, sizeof(cmd.resumeCondition));
    copyDocString(doc, "release_side", cmd.releaseSide, sizeof(cmd.releaseSide));
    cmd.seq = doc["seq"] | 0UL;
    cmd.clientMs = doc["client_ms"] | 0UL;
    cmd.sideClearMs = doc["side_clear_ms"] | 0UL;
    cmd.sideClearRequiredMs = doc["side_clear_required_ms"] | AI_SIDE_CLEAR_HOLD_MS;
    cmd.blockedMs = doc["blocked_ms"] | 0UL;
    cmd.blockedRequiredMs = doc["blocked_hold_required_ms"] | 0UL;
    cmd.clearMs = doc["clear_ms"] | 0UL;
    cmd.rampMs = doc["ramp_ms"] | 0UL;
    cmd.speed = doc["speed"] | 0.0f;
    cmd.turn = doc["turn"] | 0.0f;
    cmd.leftBlocked = doc["left_blocked"] | false;
    cmd.rightBlocked = doc["right_blocked"] | false;
    cmd.releaseQualified = doc["release_qualified"] | false;
    cmd.healthScore = doc["ai_health_score"] | 0;
    cmd.frameAgeMs = doc["ai_frame_age_ms"] | 0;
    cmd.inferenceMs = doc["ai_inference_ms"] | 0;
    cmd.wsRttMs = doc["ai_ws_rtt_ms"] | 0;
    cmd.visionFps = doc["ai_vision_fps"] | 0;
    cmd.decisionAgeMs = doc["ai_decision_age_ms"] | 0;
    cmd.opticalFlowMag = doc["ai_optical_flow_mag"] | 0.0f;
  }

  if (hasDrive) {
    JsonObject drive = doc["drive"].as<JsonObject>();
    cmd.joyX = constrain(drive["x"] | 0.0f, -1.0f, 1.0f);
    cmd.joyY = constrain(drive["y"] | 0.0f, -1.0f, 1.0f);
  }

  if (doc.containsKey("odom")) {
    const char* odom = doc["odom"] | "";
    cmd.hasOdomReset = strcmp(odom, "reset") == 0;
  }

  queueWsCommand(cmd);
}

void sendAiAck(
    uint32_t seq,
    uint32_t clientMs,
    bool accepted,
    const char* reason) {

  StaticJsonDocument<640> doc;
  doc["type"] = "ai_ack";
  doc["seq"] = seq;
  doc["client_ms"] = clientMs;
  doc["server_ms"] = millis();
  doc["accepted"] = accepted;
  doc["state"] = safetyStateName(safetyState);
  doc["stopped"] =
      !stateAllowsManualDrive() &&
      !stateAllowsAiDrive() &&
      !stateAllowsAiAvoidance() &&
      !stateAllowsAiRecovery() || driveWatchdogActive;
  const bool motorGateAllowed =
      !motorStopRequested && !driveWatchdogActive &&
      (stateAllowsManualDrive() ||
       stateAllowsAiDrive() ||
       stateAllowsAiAvoidance() ||
       stateAllowsAiRecovery());
  doc["motor_enabled"] = motorGateAllowed;
  doc["motor_left"] = motorAppliedLeftNorm;
  doc["motor_right"] = motorAppliedRightNorm;
  doc["reason"] = reason;
  doc["stop_reason"] = currentStopReason();
  doc["ai_lease"] = aiLeaseActive;
  doc["ai_age_ms"] = aiLeaseActive ? (millis() - lastAiPacketMs) : -1;
  doc["ai_seq"] = lastAiSeqValid ? lastAiSeq : 0UL;

  doc["left_zone_occupied"] = aiLeftZoneOccupied;
  doc["right_zone_occupied"] = aiRightZoneOccupied;
  doc["side_clear_ms"] = aiSideClearHoldMs;
  doc["side_clear_required_ms"] = aiSideClearRequiredMs;
  doc["side_clear_remaining_ms"] = aiReleaseQualified ? 0UL : (aiSideClearHoldMs >= aiSideClearRequiredMs ? 0UL : (aiSideClearRequiredMs - aiSideClearHoldMs));
  doc["blocked_hold_ms"] = aiBlockedHoldMs;
  doc["blocked_hold_required_ms"] = aiBlockedHoldRequiredMs;
  doc["side_candidate"] = aiSideCandidate;
  doc["release_qualified"] = aiReleaseQualified;
  doc["release_side"] = aiReleaseSide;
  doc["resume_condition"] = aiResumeCondition;

  String out;
  serializeJson(doc, out);
  ws.textAll(out);
}


void processModeCommand(const WsCommand& cmd) {
  const bool wantAuto = strcmp(cmd.mode, "auto") == 0;
  const bool wantManual = strcmp(cmd.mode, "manual") == 0;

  if (!wantAuto && !wantManual) {
    ws.textAll("{\"type\":\"mode_ack\",\"accepted\":false,\"reason\":\"invalid_mode\"}");
    return;
  }

  // Switching mode always stops first. This prevents a mode change from
  // becoming an accidental motor-enable command.
  emergencyMotorStop();
  joystickX = 0.0f;
  joystickY = 0.0f;
  aiAvoidSpeed = 0.0f;
  aiAvoidTurn = 0.0f;
  aiDriveSpeed = 0.0f;
  aiDriveTurn = 0.0f;
  aiRecoverySpeed = 0.0f;
  stoppedLatched = false;
  aiStopReason = false;
  requireFreshDriveAfterStop = false;
  lastDriveCommandMs = millis();

  aiLeaseActive = false;
  aiClientId = 0;
  lastAiSeqValid = false;

  autonomousMode = wantAuto;

  if (wantManual) {
    safetyState = STATE_MANUAL;
    stopReason = "manual";
    broadcastAiState("manual_mode_selected", 0);
  } else {
    // AUTO is armed but remains stopped until the laptop AI claims a lease
    // with the required initial STOP packet.
    safetyState = STATE_AI_LINK_LOSS;
    stopReason = "awaiting_ai";
    broadcastAiState("autonomous_mode_selected", 0);
  }

  StaticJsonDocument<192> doc;
  doc["type"] = "mode_ack";
  doc["accepted"] = true;
  doc["mode"] = autonomousMode ? "auto" : "manual";
  doc["stopped"] = true;
  String out;
  serializeJson(doc, out);
  ws.textAll(out);

  Serial.printf("CONTROL MODE -> %s (stopped until valid drive authority)\n",
                autonomousMode ? "AUTONOMOUS" : "MANUAL");
}

void processServoCommand(const WsCommand& cmd) {
  if (strcmp(cmd.servo, "left") == 0) {
    cameraServo.write(CAMERA_SERVO_LEFT);
  } else if (strcmp(cmd.servo, "center") == 0) {
    cameraServo.write(CAMERA_SERVO_CENTER);
  } else if (strcmp(cmd.servo, "right") == 0) {
    cameraServo.write(CAMERA_SERVO_RIGHT);
  } else {
    ws.textAll("{\"type\":\"servo_ack\",\"accepted\":false,\"reason\":\"invalid_servo_position\"}");
    return;
  }

  StaticJsonDocument<128> doc;
  doc["type"] = "servo_ack";
  doc["accepted"] = true;
  doc["position"] = cmd.servo;
  String out;
  serializeJson(doc, out);
  ws.textAll(out);
}

void processJoystickCommand(const WsCommand& cmd) {
  if (autonomousMode) {
    // In explicit AUTO mode, joystick input is ignored. Use the mode button
    // to return to MANUAL.
    setMotorTargetNorm(0.0f, 0.0f);
    return;
  }

  const float newX = cmd.joyX;
  const float newY = cmd.joyY;

  joystickX = newX;
  joystickY = newY;
  lastDriveCommandMs = millis();
  lastAcceptedMotionCommandMs = lastDriveCommandMs;
  driveWatchdogActive = false;

  const bool neutral = fabsf(newX) <= 0.05f && fabsf(newY) <= 0.05f;
  const bool meaningfulDrive = fabsf(newX) >= 0.08f || fabsf(newY) >= 0.08f;

  if (meaningfulDrive &&
      (safetyState == STATE_AI_SESSION ||
       stateAllowsAiAvoidance() ||
       stateAllowsAiRecovery())) {

    aiLeaseActive = false;
    aiClientId = 0;
    lastAiSeqValid = false;
    stoppedLatched = false;
    aiStopReason = false;
    requireFreshDriveAfterStop = false;
    aiAvoidSpeed = 0.0f;
    aiAvoidTurn = 0.0f;
    aiRecoverySpeed = 0.0f;
    setMotorTargetNorm(0.0f, 0.0f);
    safetyState = STATE_MANUAL;

    Serial.printf("MANUAL OVERRIDE from joystick x=%.2f y=%.2f\n", newX, newY);
    broadcastAiState("manual_override", lastAiSeqValid ? lastAiSeq : 0);

    applyJoystickDrive();
    return;
  }

  if (safetyState == STATE_WAIT_NEUTRAL) {
    if (neutral) {
      safetyState = STATE_WAIT_NEW_DRIVE;
      setMotorTargetNorm(0.0f, 0.0f);
      Serial.println("Joystick neutral observed -> WAIT_NEW_DRIVE");
      broadcastAiState("neutral_observed", lastAiSeqValid ? lastAiSeq : 0);
    } else {
      setMotorTargetNorm(0.0f, 0.0f);
    }
    return;
  }

  if (safetyState == STATE_WAIT_NEW_DRIVE) {
    if (!neutral) {
      aiLeaseActive = false;
      aiClientId = 0;
      lastAiSeqValid = false;
      stoppedLatched = false;
      aiStopReason = false;
      requireFreshDriveAfterStop = false;
      aiAvoidSpeed = 0.0f;
      aiAvoidTurn = 0.0f;
      aiRecoverySpeed = 0.0f;
      safetyState = STATE_MANUAL;
      setMotorTargetNorm(0.0f, 0.0f);
      Serial.println("New non-zero drive -> MANUAL override");
      broadcastAiState("manual_override_after_stop", 0);
      applyJoystickDrive();
    } else {
      setMotorTargetNorm(0.0f, 0.0f);
    }
    return;
  }

  if (safetyState == STATE_HUMAN_STOP ||
      safetyState == STATE_HUMANS_BLOCKED ||
      safetyState == STATE_AI_LINK_LOSS) {
    setMotorTargetNorm(0.0f, 0.0f);
    return;
  }

  if (stateAllowsAiAvoidance() || stateAllowsAiRecovery()) {
    setMotorTargetNorm(0.0f, 0.0f);
    return;
  }

  if (stateAllowsManualDrive()) {
    applyJoystickDrive();
  }
}

void processAiCommand(const WsCommand& cmd) {
  if (!autonomousMode) {
    sendAiAck(cmd.seq, cmd.clientMs, false, "manual_mode_active");
    return;
  }

  const bool isStop = strcmp(cmd.ai, "stop") == 0;
  const bool isGo = strcmp(cmd.ai, "go") == 0;
  const bool isAvoid = strcmp(cmd.ai, "avoid") == 0;
  const bool isRecover = strcmp(cmd.ai, "recover") == 0;
  const bool isDrive = strcmp(cmd.ai, "drive") == 0;

  if (!isStop && !isGo && !isAvoid && !isRecover && !isDrive) {
    sendAiAck(cmd.seq, cmd.clientMs, false, "invalid_ai_command");
    return;
  }

  // A new AI session must begin with STOP. This is also the recovery path
  // after a previous AI client disconnected or its lease expired.
  if (!aiLeaseActive) {
    if (!isStop) {
      sendAiAck(cmd.seq, cmd.clientMs, false, "session_must_start_with_stop");
      return;
    }
    aiLeaseActive = true;
    aiClientId = cmd.clientId;
    lastAiSeqValid = false;
    Serial.printf("AI lease claimed by WS client #%u\n", (unsigned)aiClientId);
  }

  if (aiClientId != cmd.clientId) {
    sendAiAck(cmd.seq, cmd.clientMs, false, "ai_lease_owned");
    return;
  }

  if (!aiSequenceIsNew(cmd.seq)) {
    sendAiAck(cmd.seq, cmd.clientMs, false, "stale_seq");
    return;
  }

  // Only valid, increasing packets refresh the lease.
  lastAiPacketMs = millis();
  lastAiSeq = cmd.seq;
  lastAiSeqValid = true;

  aiHealthScore = constrain(cmd.healthScore, (uint8_t)0, (uint8_t)100);
  aiFrameAgeMs = cmd.frameAgeMs;
  aiInferenceMs = cmd.inferenceMs;
  aiWsRttMs = cmd.wsRttMs;
  aiVisionFps = cmd.visionFps;
  aiDecisionAgeMs = cmd.decisionAgeMs;
  aiOpticalFlowMag = isfinite(cmd.opticalFlowMag) ? max(0.0f, cmd.opticalFlowMag) : 0.0f;
  aiHealthValid = (cmd.healthScore > 0 || cmd.frameAgeMs > 0 || cmd.inferenceMs > 0 || cmd.visionFps > 0 || cmd.decisionAgeMs > 0);

  aiLeftZoneOccupied = cmd.leftBlocked;
  aiRightZoneOccupied = cmd.rightBlocked;
  aiSideClearHoldMs = cmd.sideClearMs;
  aiSideClearRequiredMs = max((uint32_t)AI_SIDE_CLEAR_HOLD_MS, cmd.sideClearRequiredMs);
  aiBlockedHoldMs = cmd.blockedMs;
  aiBlockedHoldRequiredMs = cmd.blockedRequiredMs;
  aiSideCandidate = String(cmd.chosenSide[0] ? cmd.chosenSide : "none");
  aiResumeCondition = String(cmd.resumeCondition);
  aiReleaseQualified = cmd.releaseQualified;
  aiReleaseSide = String(cmd.releaseSide[0] ? cmd.releaseSide : "none");

  updateReleaseQualification(
      cmd.sideClearMs,
      cmd.sideClearRequiredMs,
      aiSideCandidate
  );

  if (!isStop && aiHealthValid && aiHealthScore <= AI_HEALTH_STOP_THRESHOLD) {
    latchAiLinkLoss(cmd.seq);
    aiLeaseActive = false;
    aiClientId = 0;
    sendAiAck(cmd.seq, cmd.clientMs, false, "ai_health_low");
    return;
  }

  if (isStop) {
    const char* reason = cmd.reason[0] ? cmd.reason : "human";

    if (safetyState == STATE_HUMANS_BLOCKED &&
        strcmp(reason, "side_clear_pending") == 0) {
      emergencyMotorStop();
      stoppedLatched = true;
      aiStopReason = true;
      requireFreshDriveAfterStop = true;
      sendAiAck(cmd.seq, cmd.clientMs, true, "blocked_side_clear_pending");
      return;
    }

    if (strcmp(reason, "blocked_by_humans") == 0) {
      latchHumansBlocked(cmd.seq);
    } else {
      latchHumanStop(cmd.seq);
    }

    sendAiAck(cmd.seq, cmd.clientMs, true, currentStopReason());
    return;
  }

  if (isAvoid) {
    const bool accepted = acceptAiAvoid(
        cmd.seq,
        cmd.speed,
        cmd.turn,
        cmd.sideClearMs
    );
    sendAiAck(cmd.seq, cmd.clientMs, accepted, currentStopReason());
    return;
  }

  if (isRecover) {
    const bool accepted = acceptAiRecovery(cmd.seq, cmd.speed);
    sendAiAck(cmd.seq, cmd.clientMs, accepted, currentStopReason());
    return;
  }

  if (isDrive) {
    const bool accepted = acceptAiDrive(
        cmd.seq,
        cmd.speed,
        cmd.turn,
        cmd.clearMs,
        cmd.rampMs
    );
    sendAiAck(cmd.seq, cmd.clientMs, accepted, accepted ? "ai_drive" : "ai_drive_rejected");
    return;
  }

  // GO.
  const bool accepted = acceptAiGo(cmd.seq, cmd.clearMs);
  sendAiAck(cmd.seq, cmd.clientMs, accepted, currentStopReason());
}

void processWsCommand(const WsCommand& cmd) {
  if (cmd.type == WS_CMD_DISCONNECT) {
    if (aiLeaseActive && aiClientId == cmd.clientId) {
      const uint32_t lostSeq = lastAiSeqValid ? lastAiSeq : 0;
      aiLeaseActive = false;
      aiClientId = 0;
      lastAiSeqValid = false;
      latchAiLinkLoss(lostSeq);
      Serial.println("AI CLIENT DISCONNECTED: FAIL-CLOSED STOP");
    }
    return;
  }

  if (cmd.hasMode) {
    processModeCommand(cmd);
  }

  if (cmd.hasServo) {
    processServoCommand(cmd);
  }

  if (cmd.hasAi) {
    processAiCommand(cmd);
    return;
  }

  if (cmd.hasDrive) {
    processJoystickCommand(cmd);
    return;
  }

  if (cmd.hasOdomReset) {
    resetOdometry();
    ws.textAll("{\"type\":\"odom_reset\",\"ok\":true}");
  }
}

void processQueuedWsCommands() {
  if (wsCommandQueue == nullptr) return;

  WsCommand cmd;
  while (xQueueReceive(wsCommandQueue, &cmd, 0) == pdPASS) {
    processWsCommand(cmd);
  }
}

// =========================================================
// Camera settings HTTP route
// =========================================================
void handleCameraCommand(AsyncWebServerRequest* request) {
  if (!request->hasParam("cmd")) {
    request->send(400, "text/plain", "Missing cmd");
    return;
  }
  String cmd = request->getParam("cmd")->value();
  cmd.trim();
  if (!validCamCommand(cmd)) {
    request->send(400, "text/plain", "Invalid camera command");
    return;
  }
  sendCamUartCommand(cmd);
  request->send(200, "text/plain", "Command sent to camera UART: " + cmd + " (UART is optional; wait for ACK if connected)");
}

// =========================================================
// Setup
// =========================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  // Motor pins default LOW before attaching PWM so reset/startup cannot request motion.
  pinMode(L_RPWM, OUTPUT);
  pinMode(L_LPWM, OUTPUT);
  pinMode(R_RPWM, OUTPUT);
  pinMode(R_LPWM, OUTPUT);
  digitalWrite(L_RPWM, LOW);
  digitalWrite(L_LPWM, LOW);
  digitalWrite(R_RPWM, LOW);
  digitalWrite(R_LPWM, LOW);

  // Motor pins / PWM (Arduino-ESP32 Core 3.x)
  ledcAttach(L_RPWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(L_LPWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(R_RPWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(R_LPWM, PWM_FREQ, PWM_RESOLUTION);
  emergencyMotorStop();
  initControlTaskWatchdog();

  // Hall sensor. GPIO35 is input-only and has no internal pull-up.
  // Use an external pull-up to 3.3V if your Hall sensor output requires one.
  pinMode(HALL_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), countTick, RISING);

  // I2C + MPU6050
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setTimeOut(50);
  if (mpu.begin()) {
    mpuReady = true;
    mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    calibrateGyro();
  } else {
    mpuReady = false;
    Serial.println("WARNING: MPU6050 not found. Heading will remain 0 and odometry heading integration is unavailable.");
  }

  // UART bridge. This is optional; it must not block rover startup.
  Serial2.begin(CAM_UART_BAUD, SERIAL_8N1, CAM_UART_RX, CAM_UART_TX);
  Serial.printf("CAM UART ready RX=%d TX=%d @ %d\n", CAM_UART_RX, CAM_UART_TX, CAM_UART_BAUD);

  // Camera pan servo.
  // Use an external 5V supply for the servo and connect its GND to ESP32 GND.
  cameraServo.setPeriodHertz(50);
  cameraServo.attach(CAMERA_SERVO_PIN, 500, 2400);
  cameraServo.write(CAMERA_SERVO_CENTER);
  Serial.printf("Camera servo ready on GPIO %d\n", CAMERA_SERVO_PIN);

  // DevKit always hosts the AP.
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);
  bool apOk = WiFi.softAP(AP_SSID, AP_PASSWORD, 6, 0, 8);
  WiFi.setSleep(false);

  Serial.printf("SoftAP %s, IP=%s, AP started=%s\n", AP_SSID, WiFi.softAPIP().toString().c_str(), apOk ? "YES" : "NO");
  Serial.printf("Dashboard: http://%s/\n", WiFi.softAPIP().toString().c_str());
  Serial.printf("WebSocket: ws://%s:81/ws\n", WiFi.softAPIP().toString().c_str());
  Serial.printf("Default camera stream: http://%s:81/stream\n", DEFAULT_CAMERA_IP);

  wsCommandQueue = xQueueCreate(32, sizeof(WsCommand));
  if (wsCommandQueue == nullptr) {
    Serial.println("FATAL: WebSocket command queue allocation failed");
    forceMotorStopNow();
  }

  lastMotorOutputMs = millis();
  lastLoopEntryMs = lastMotorOutputMs;

  // HTTP server: dashboard + settings page.
  webServer.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html; charset=utf-8", dashboard_html);
  });

  webServer.on("/settings", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html; charset=utf-8", settings_html);
  });

  webServer.on("/camera/cmd", HTTP_GET, handleCameraCommand);

  webServer.onNotFound([](AsyncWebServerRequest* request) {
    request->send(404, "text/plain", "Not found");
  });

  ws.onEvent(onWsEvent);
  wsServer.addHandler(&ws);

  webServer.begin();
  wsServer.begin();

  lastKinematicsMs = millis();
  lastTelemetryMs = millis();
  lastDriveCommandMs = millis();
  lastAcceptedMotionCommandMs = lastDriveCommandMs;
  driveWatchdogActive = false;
  motorStopRequested = false;
  stoppedLatched = false;
  requireFreshDriveAfterStop = false;
  aiLeaseActive = false;
  aiClientId = 0;
  lastAiPacketMs = 0;
  lastAiSeq = 0;
  lastAiSeqValid = false;
  stopReason = "none";
  commandQueueFault = false;
  aiDriveSpeed = 0.0f;
  aiDriveTurn = 0.0f;
  aiRecoverySpeed = 0.0f;
  aiLeftZoneOccupied = false;
  aiRightZoneOccupied = false;
  aiSideClearHoldMs = 0;
  aiSideClearRequiredMs = AI_SIDE_CLEAR_HOLD_MS;
  aiBlockedHoldMs = 0;
  aiBlockedHoldRequiredMs = 0;
  aiSideCandidate = "none";
  aiResumeCondition = "AI session must start with STOP";
  safetyState = STATE_MANUAL;
  Serial.printf("AI lease timeout=%lu ms\n", (unsigned long)AI_LEASE_TIMEOUT_MS);
  Serial.println("Rover firmware ready.");
}

// =========================================================
// Main loop
// =========================================================
void loop() {
  const uint32_t now = millis();

  // Independent control-loop health watchdog. If an I2C call, network call,
  // or unexpected blocking operation delays the loop beyond this threshold,
  // fail closed before another motion command can be applied.
  if (lastLoopEntryMs != 0 && (now - lastLoopEntryMs) > CONTROL_LOOP_OVERRUN_MS) {
    if (aiLeaseActive || safetyState != STATE_MANUAL) {
      motorStopRequested = true;
      stoppedLatched = true;
      requireFreshDriveAfterStop = true;
      stopReason = "control_loop_overrun";
      aiStopReason = false;
      aiLeaseActive = false;
      aiClientId = 0;
      safetyState = STATE_AI_LINK_LOSS;
      Serial.printf("CONTROL LOOP OVERRUN: %lu ms -> FAIL-CLOSED STOP\n", (unsigned long)(now - lastLoopEntryMs));
      broadcastAiState("control_loop_overrun", lastAiSeqValid ? lastAiSeq : 0);
    }
  }
  lastLoopEntryMs = now;

  // Serialize all WebSocket-originated state changes through loop().
  processQueuedWsCommands();

  // Queue/integrity fault is sticky until a full reboot. A malformed,
  // ambiguous, or overflowed control stream must never recover by itself.
  if (commandQueueFault) {
    motorStopRequested = true;
    stoppedLatched = true;
    requireFreshDriveAfterStop = true;
    stopReason = "command_integrity_fault";
    aiStopReason = false;
    safetyState = STATE_AI_LINK_LOSS;
    aiLeaseActive = false;
    aiClientId = 0;
  }

  // -------------------------------------------------------
  // AI lease timeout FIRST.
  // Safety decisions happen before any motor output.
  // -------------------------------------------------------
  if (aiLeaseActive &&
      (now - lastAiPacketMs > AI_LEASE_TIMEOUT_MS)) {

    aiLeaseActive = false;
    aiClientId = 0;

    Serial.printf(
        "AI LEASE EXPIRED: last packet %lu ms ago\n",
        (unsigned long)(now - lastAiPacketMs)
    );

    latchAiLinkLoss(
        lastAiSeqValid ? lastAiSeq : 0
    );
  }

  // Independent motor-command validity window. Motion expires unless a
  // fresh joystick/AI motion command has been accepted. This is separate
  // from the AI lease and dashboard watchdogs.
  const bool anyMotionState =
      stateAllowsManualDrive() || stateAllowsAiDrive() ||
      stateAllowsAiAvoidance() || stateAllowsAiRecovery();
  if (anyMotionState &&
      max(fabsf(motorTargetLeftNorm), fabsf(motorTargetRightNorm)) >= 0.05f &&
      now - lastAcceptedMotionCommandMs > MOTOR_COMMAND_EXPIRY_MS) {
    motorStopRequested = true;
    stoppedLatched = true;
    requireFreshDriveAfterStop = true;
    stopReason = "motor_command_expired";
    aiStopReason = false;
    joystickX = 0.0f;
    joystickY = 0.0f;
    if (aiLeaseActive) {
      aiLeaseActive = false;
      aiClientId = 0;
      aiHealthValid = false;
      safetyState = STATE_AI_LINK_LOSS;
    } else {
      // Manual command expiry is also fail-closed, but the operator can
      // re-arm manual motion through the existing neutral -> new-drive gate.
      safetyState = STATE_WAIT_NEUTRAL;
    }
    Serial.printf("MOTOR COMMAND EXPIRED: age=%lu ms -> FAIL-CLOSED STOP\n", (unsigned long)(now - lastAcceptedMotionCommandMs));
  }

  // -------------------------------------------------------
  // 50 Hz kinematics / dead reckoning.
  // -------------------------------------------------------
  if (now - lastKinematicsMs >= KINEMATICS_PERIOD_MS) {
    const float dt =
        (now - lastKinematicsMs) / 1000.0f;

    lastKinematicsMs = now;

    float gyroZSample = 0.0f;
    if (mpuReady) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      gyroZSample = g.gyro.z;
      const float correctedZ =
          gyroZSample - gyroZOffset;

      if (fabsf(correctedZ) > 0.02f) {
        yawAngleRad += correctedZ * dt;
      }

      yawAngleRad = fmodf(yawAngleRad, 2.0f * PI);
    }

    noInterrupts();
    const uint32_t ticksNow = wheelTicks;
    interrupts();

    const uint32_t deltaTicks =
        ticksNow - lastTicks;

    if (deltaTicks > 0) {
      lastHallTickMs = now;
    }
    if (lastHallTickMs == 0) {
      lastHallTickMs = now;
    }

    if (mpuReady) {
      // Reuse the already-read gyro sample for bias adaptation so the I2C
      // sensor is sampled once per 20 ms cycle, not twice.
      static uint32_t gyroStillSince = 0;
      const float gyroResidual = gyroZSample - gyroZOffset;
      const bool commandedStill =
          fabsf(motorTargetLeftNorm) < 0.01f &&
          fabsf(motorTargetRightNorm) < 0.01f;
      if (commandedStill && deltaTicks == 0 && fabsf(gyroResidual) < 0.08f) {
        if (gyroStillSince == 0) gyroStillSince = now;
        if (now - gyroStillSince > 800) {
          gyroZOffset = 0.995f * gyroZOffset + 0.005f * gyroZSample;
        }
      } else {
        gyroStillSince = 0;
      }
    }

    if (deltaTicks > 0) {
      const float deltaDist =
          deltaTicks * CM_PER_TICK;

      totalDistanceCm += deltaDist;

      float dirSign = 0.0f;
      const float commandForward = (motorTargetLeftNorm + motorTargetRightNorm) * 0.5f;
      if (fabsf(commandForward) > 0.02f) {
        dirSign = commandForward > 0.0f ? 1.0f : -1.0f;
      }

      if (stateAllowsManualDrive() ||
          stateAllowsAiDrive() ||
          stateAllowsAiAvoidance() ||
          stateAllowsAiRecovery()) {

        if (fabsf(dirSign) > 0.0f) {
          const float signedDist = deltaDist * dirSign;
          const float midYaw = yawAngleRad;

          posXcm += signedDist * cosf(midYaw);
          posYcm += signedDist * sinf(midYaw);
          const float rawSpeedCmS = signedDist / max(dt, 0.001f);
          measuredSpeedCmS = 0.70f * measuredSpeedCmS + 0.30f * rawSpeedCmS;
        }
      }

      lastTicks = ticksNow;
    }

    if (deltaTicks == 0 && now - lastHallTickMs > 250) {
      measuredSpeedCmS = 0.0f;
    }

    // Independent motion-feedback safety check. This only trips when a
    // meaningful motor command is present; very slow commands are exempt
    // because a Hall sensor may legitimately not pulse every control tick.
    const float maxAbsMotorTarget =
        max(fabsf(motorTargetLeftNorm), fabsf(motorTargetRightNorm));
    const bool motionState =
        stateAllowsAiDrive() ||
        stateAllowsAiAvoidance() ||
        stateAllowsAiRecovery() ||
        stateAllowsManualDrive();

    if (motionState &&
        maxAbsMotorTarget >= HALL_FEEDBACK_MIN_COMMAND &&
        now - lastHallTickMs > HALL_FEEDBACK_TIMEOUT_MS) {

      const uint32_t age = now - lastHallTickMs;
      motorStopRequested = true;
      stoppedLatched = true;
      requireFreshDriveAfterStop = true;
      stopReason = "hall_motion_feedback_lost";
      aiStopReason = false;
      joystickX = 0.0f;
      joystickY = 0.0f;

      Serial.printf(
          "HALL FEEDBACK LOST: commanded=%.2f age=%lu ms -> FAIL-CLOSED STOP\n",
          maxAbsMotorTarget,
          (unsigned long)age
      );

      if (aiLeaseActive) {
        safetyState = STATE_AI_LINK_LOSS;
        aiLeaseActive = false;
        aiClientId = 0;
      } else {
        safetyState = STATE_HUMAN_STOP;
      }
    }

    // -----------------------------------------------------
    // Local joystick failsafe.
    // -----------------------------------------------------
    if (safetyState == STATE_MANUAL &&
        now - lastDriveCommandMs > DRIVE_WATCHDOG_MS) {

      joystickX = 0.0f;
      joystickY = 0.0f;

      if (!driveWatchdogActive) {
        driveWatchdogActive = true;

        emergencyMotorStop();

        Serial.println(
            "DRIVE WATCHDOG: motors stopped"
        );

        broadcastAiState(
            "drive_watchdog",
            lastAiSeqValid ? lastAiSeq : 0
        );
      }
    } else {
      driveWatchdogActive = false;
    }

    // -----------------------------------------------------
    // SINGLE motor-output point.
    //
    // Safety states always win.
    // -----------------------------------------------------
    if (motorStopRequested ||
        driveWatchdogActive ||
        safetyState == STATE_HUMAN_STOP ||
        safetyState == STATE_HUMANS_BLOCKED ||
        safetyState == STATE_AI_LINK_LOSS ||
        safetyState == STATE_WAIT_NEUTRAL ||
        safetyState == STATE_WAIT_NEW_DRIVE) {

      applyEmergencyMotorStop();

    } else if (stateAllowsAiAvoidance()) {

      motorStopRequested = false;
      applyAiAvoidance();
      updateMotorOutput(now);

    } else if (stateAllowsAiRecovery()) {

      motorStopRequested = false;
      applyAiRecovery();
      updateMotorOutput(now);

    } else if (stateAllowsAiDrive()) {

      motorStopRequested = false;
      applyAiDrive();
      updateMotorOutput(now);

    } else if (stateAllowsManualDrive()) {

      motorStopRequested = false;
      applyJoystickDrive();
      updateMotorOutput(now);

    } else {

      applyEmergencyMotorStop();
    }
  }

  // Telemetry.
  if (now - lastTelemetryMs >=
      TELEMETRY_PERIOD_MS) {

    lastTelemetryMs = now;
    broadcastTelemetry();
  }

  // Optional camera UART housekeeping.
  if (now - lastCamUartPollMs >= 5) {
    lastCamUartPollMs = now;
    pollCamUart();
  }

  ws.cleanupClients();
  feedControlTaskWatchdog();
}
