# 9. Dashboard and Camera Settings Guide

## 9.1 Real dashboard

Open:

```text
http://192.168.4.1/
```

The laptop is expected to be connected to `ESP32-Rover-AP`.

## 9.2 Landscape layout

The dashboard is optimized for laptop landscape screens, including 1366x768, 1536x864 and 1920x1080 test targets.

The main areas are:

```text
Top:       camera feed
Bottom L:  one joystick
Bottom R:  telemetry + avoidance status + map
```

## 9.3 Status information

The top/status region can show:

```text
DRIVING
SLOW AVOID — LEFT/RIGHT
SLOW RECOVERY — FORWARD PATH CLEARING
STOPPED — HUMAN DETECTED
STOPPED — BLOCKED BY HUMANS
STOPPED — AI LINK LOST
STOPPED — CAMERA STALE
MANUAL CONTROL — AI OVERRIDE
```

## 9.4 Human-zone panel

The dashboard shows:

- LEFT ZONE: OCCUPIED/CLEAR
- RIGHT ZONE: OCCUPIED/CLEAR
- blocked hold
- side-clear hold
- release-qualified state
- qualifying side
- side-clear countdown
- exact resume condition

The countdown should display zero only when the corresponding release-qualified condition has actually been reported.

## 9.5 AI-health panel

Health diagnostics include information derived from:

- AI lease freshness;
- AI-health score;
- vision/FPS condition;
- odometry confidence;
- frame age and other timing data where available.

## 9.6 Local dashboard preview

Open `Dashboard/dashboard_preview.html` when you want to inspect the UI without a rover.

The preview starts blank and explicitly identifies itself as:

```text
LOCAL PREVIEW / SIMULATION
```

Select a `SIM:` scenario to populate sample values and demonstrate states.

### Recommended Windows method

Use:

```text
Dashboard/start_preview.bat
```

Then browse to:

```text
http://127.0.0.1:8765/
http://127.0.0.1:8765/settings
```

The bundled local server avoids file-origin routing problems with `/settings`.

## 9.7 Camera settings page

Real rover:

```text
http://192.168.4.1/settings
```

Local preview:

```text
http://127.0.0.1:8765/settings
```

The page provides controls for the camera configuration supported by the ESP32-CAM sketch.

## 9.8 Camera stream path

The dashboard references the camera directly:

```text
http://192.168.4.2:81/stream
```

The DevKit does not relay the video.
