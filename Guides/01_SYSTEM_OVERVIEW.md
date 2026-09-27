# 1. System Overview

## 1.1 Purpose

VORTEX Rover is a small differential/skid-steer rover using:

- AI-Thinker ESP32-CAM for camera capture and MJPEG streaming.
- ESP32 DevKit V1 for Wi-Fi AP, web dashboard, WebSocket server, motor control and onboard sensing.
- Laptop for person detection/tracking and high-level AI decisions.
- Hall sensor for wheel-pulse/distance estimation.
- MPU6050 for yaw-rate/heading integration.
- Optional battery-voltage ADC on the DevKit.

No gas, methane, temperature, humidity, DHT11, MQ4, GPS, LiDAR, ToF or other additional sensor is required by this build.

## 1.2 Fixed topology

```text
                         WIFI AP: ESP32-Rover-AP
                  ┌──────────────────────────────────────┐
                  │                                      │
ESP32-CAM         │         ESP32 DevKit V1              │
Camera only       │         motor/safety authority        │
192.168.4.2       │         192.168.4.1                  │
     │             │                                      │
     │ MJPEG       │  HTTP :80      WS :81 /ws           │
     ├─────────────┼─────────────────────────────────────┤
     │             │                                      │
     ▼             ▼                                      │
Laptop AI ──────── WebSocket requests ────────────────► │
     │                                                  │
     │ STOP / AVOID / RECOVER / DRIVE                  │
     │                                                  ▼
     │                                             BTS7960 motors
     │
     └──── YOLOv8n + BoT-SORT + prediction
```

Video never travels through the DevKit. The laptop reads the camera's `http://192.168.4.2:81/stream` endpoint directly.

## 1.3 Responsibilities

### ESP32-CAM

- Initializes the AI-Thinker camera.
- Connects as a station to the DevKit AP.
- Streams MJPEG on port 81 at `/stream`.
- Hosts basic camera-status information.
- Accepts optional Serial2 housekeeping commands from the DevKit for flash/resolution/image tuning.
- Does not control motors or read rover sensors.

### ESP32 DevKit V1

- Creates the Wi-Fi AP.
- Serves the control dashboard on HTTP port 80.
- Serves WebSocket control/telemetry on port 81.
- Owns motor PWM and all final safety decisions.
- Runs Hall + MPU6050 odometry.
- Maintains AI lease and fail-closed safety state machine.
- Applies motor command expiry, slew limiting and motion-feedback checks.
- Relays optional camera-settings commands over UART.

### Laptop

- Joins the DevKit AP.
- Captures the newest camera frame.
- Runs YOLOv8n class 0 person detection.
- Runs BoT-SORT by default.
- Tracks position/size/velocity of people.
- Projects tracked humans forward in time.
- Computes side occupancy, side-clear qualification, blocked condition, avoidance and recovery decisions.
- Computes AI-health telemetry.
- Sends bounded AI requests to the DevKit.
- Never sends motor PWM.

## 1.4 Authority rule

The laptop makes requests. The DevKit decides whether a request is legal.

Safety stops cannot be cleared by a lower-priority source. The browser cannot override an AI hard stop. A stale AI packet cannot refresh the AI lease. A new AI connection must establish a STOP handshake before autonomous motion is accepted.

## 1.5 Network defaults

```text
SSID:       ESP32-Rover-AP
Password:   12345678
DevKit IP:  192.168.4.1
CAM IP:     192.168.4.2
Dashboard:  http://192.168.4.1/
Settings:   http://192.168.4.1/settings
WS:         ws://192.168.4.1:81/ws
MJPEG:      http://192.168.4.2:81/stream
```

The addresses are defaults from the firmware. Change them only if the corresponding firmware/configuration is changed consistently.
