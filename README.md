
## V15.1 additions in this package
- Added a pan servo on DevKit GPIO13 with LEFT/CENTER/RIGHT control.
- Added explicit MANUAL and AUTONOMOUS AI mode buttons to the dashboard.
- MANUAL mode disables the AI lease; AUTONOMOUS mode keeps the rover stopped until the laptop AI handshake is accepted.
- Laptop AI now commands the pan servo toward the selected avoidance side and returns it to CENTER during normal driving/recovery.
- Existing fail-closed AI/human-avoidance state machine is retained.

# VORTEX Rover v15 — Clean Final Release

This is the compact final software package for the VORTEX ESP32-CAM + ESP32 DevKit + laptop human-detection rover.

## Read first

1. `Guides/00_DOCUMENTATION_INDEX.md`
2. `Guides/13_OPERATOR_QUICK_START.md`
3. `Guides/14_HARDWARE_VALIDATION_CHECKLIST.md`
4. `Guides/03_HARDWARE_WIRING_AND_FLASHING.md`
5. `Guides/05_AI_ML_INTEGRATION.md`

## System topology

```text
ESP32-CAM --direct MJPEG--> Laptop YOLO/ML
Laptop --high-level AI requests--> ESP32 DevKit
ESP32 DevKit --motor PWM--> BTS7960 motors
Hall + MPU6050 --> ESP32 DevKit odometry
ESP32 DevKit --optional UART--> ESP32-CAM housekeeping
```

The DevKit is the single motor and safety authority.

## Core features

- Differential/skid-steer single-joystick driving.
- AI-supervised driving with bounded high-level commands.
- YOLOv8n person detection with BoT-SORT tracking.
- Predictive human-motion avoidance.
- Side-clear hold before steering.
- Both-sides-occupied `blocked_by_humans` hard stop.
- Release qualification and side-clear countdown.
- Slow recovery with configurable recovery-to-cruise speed ramp.
- Manual joystick takeover from AI operation.
- AI heartbeat lease and fail-closed timeout.
- Sequence validation and reconnect STOP handshake.
- Motor command expiry and acceleration/deceleration limiting.
- Hall feedback and control-loop watchdogs.
- ESP-IDF Task Watchdog integration.
- AI-health degradation and fail-closed behavior.
- Camera stale/freeze protection.
- Hall + MPU6050 dead-reckoning with confidence diagnostics.
- Separate camera settings page.
- Local dashboard preview without rover hardware.

## No-extra-hardware constraint

This release uses only the existing ESP32-CAM, Hall sensor, MPU6050, motors and optional battery ADC. No gas, temperature, humidity, GPS, LiDAR or ToF subsystem is included.

## Real rover addresses

```text
Dashboard:       http://192.168.4.1/
Camera settings: http://192.168.4.1/settings
WebSocket:       ws://192.168.4.1:81/ws
MJPEG:           http://192.168.4.2:81/stream
```

## Local preview

Use `Dashboard/start_preview.bat` and open:

```text
http://127.0.0.1:8765/
http://127.0.0.1:8765/settings
```

The preview starts with blank telemetry and is explicitly marked `LOCAL PREVIEW / SIMULATION`. Sample values appear only when a simulation scenario is selected.

## Safety boundary

The software is fail-closed and extensively tested, but it is not a certified collision-avoidance system. A single camera cannot prove that a human-free region contains no wall, drop, furniture or other non-human obstacle. Physical low-speed validation is required before autonomous operation.
