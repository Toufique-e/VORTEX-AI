# 12. Troubleshooting

## 12.1 Dashboard does not open

Check:

1. laptop is connected to `ESP32-Rover-AP`;
2. DevKit is running;
3. try `http://192.168.4.1/`;
4. check DevKit Serial Monitor for AP IP.

For offline UI work, use `Dashboard/start_preview.bat` and open `http://127.0.0.1:8765/`.

## 12. Camera settings page does not open

Real rover:

```text
http://192.168.4.1/settings
```

It is hosted by the DevKit. The ESP32-CAM itself is not the dashboard server.

Local preview:

```text
http://127.0.0.1:8765/settings
```

## 12. Camera stream blank

Check:

- camera joined `ESP32-Rover-AP`;
- CAM IP is `192.168.4.2` or the configured address;
- direct `/stream` URL works;
- camera was flashed with `CameraWebServer0.ino`;
- the laptop can ping/reach the camera IP on the AP.

## 12. AI stays stopped

This is often intentional. Check dashboard state and laptop log for:

```text
AI LINK LOST
CAMERA STALE
CAMERA CONTENT FROZEN
VISION COMMAND STALE
AI HEALTH LOW
HUMAN STOP
BLOCKED BY HUMANS
WAIT NEUTRAL
WAIT NEW DRIVE
```

## 12. AI starts but never drives

Verify:

- AI STOP handshake completed;
- current scene is clear;
- `clear-hold` completed;
- joystick is neutral where required;
- AI health is above the stop threshold;
- no safety-state rejection is reported by the DevKit ACK.

## 12. Rover chooses the wrong side

Check camera horizontal mirroring. The current image geometry assumes left/right image coordinates are not unintentionally mirrored.

Also check:

- `--side-corridor`;
- `--prediction-horizon`;
- `--prediction-margin`;
- camera mounting angle;
- tracker continuity.

## 12. Rover is blocked even after one side looks clear

This is expected until the side-clear hold reaches the required duration. The dashboard's countdown and `RELEASE QUALIFIED` indicator show the actual gate.

## 12. Rover resumes unexpectedly

Stop testing and inspect:

- whether the state was actually a hard-stop state;
- whether a new AI GO/DRIVE was accepted;
- whether the joystick takeover/recovery gate behaved correctly;
- AI sequence logs;
- lease age;
- command expiry;
- motor target telemetry.

Unexpected motion on a powered test is a release blocker until diagnosed.

## 12. Map drifts

This is expected over time with Hall + gyro-only dead reckoning. Verify wheel circumference/ticks calibration and gyro bias, but do not expect GPS-level accuracy.
