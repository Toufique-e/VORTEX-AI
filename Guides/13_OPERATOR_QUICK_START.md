# VORTEX Rover — Operator Quick-Start Guide

## 1. Purpose

Use this guide for day-to-day operation of the final VORTEX ESP32-CAM + ESP32 DevKit + laptop AI rover release.

**The DevKit is the motor and safety authority.** The laptop requests high-level AI actions; it never writes motor PWM directly.

## 2. Before Power-On

- Keep the rover on a stand or in a large, clear, low-speed test area for the first run.
- Confirm the joystick is centered/neutral.
- Confirm the BTS7960 and battery connections are correct.
- Confirm the camera is mounted so left/right image geometry matches the avoidance logic; do not mirror the camera unless intentionally accounted for.

## 3. Startup

1. Power the ESP32 DevKit.
2. Wait for the `ESP32-Rover-AP` Wi-Fi network.
3. Power the ESP32-CAM and allow it to join the DevKit AP.
4. Connect the laptop to `ESP32-Rover-AP`.
5. Open the rover dashboard:
   `http://192.168.4.1/`
6. Confirm the dashboard reports the expected DevKit connection and a stopped/idle state.
7. Verify the camera stream:
   `http://192.168.4.2:81/stream`
8. Start the laptop AI program from `Laptop_AI` using the documented command.
9. Confirm the AI lease becomes active.
10. Confirm the rover does **not** move during AI startup and initial clear-scene qualification.

## 4. Normal AI Operation

When AI control is active:

- The laptop captures the newest camera frame rather than building a backlog.
- YOLO detects people and the tracker maintains continuity between frames.
- Human motion is predicted ahead of the current frame when sufficient track information exists.
- A detected human causes speed reduction/avoidance evaluation.
- A side must remain human-clear for the configured side-clear hold before steering into that side.
- If both side corridors are occupied, the rover hard-stops and reports `blocked_by_humans`.
- After avoidance, the rover enters slow recovery; the forward path must remain clear for the configured recovery hold, then speed ramps gradually back to the normal AI cruise value.
- The dashboard is the source of operator feedback for AI lease, safety state, left/right occupancy, release qualification, countdowns, recovery, AI health and telemetry.

## 5. Manual Takeover

To intentionally take control while AI is driving:

1. Move the dashboard joystick.
2. The DevKit treats meaningful joystick movement as a manual takeover request.
3. AI ownership is revoked.
4. The dashboard should change to `MANUAL` / `MANUAL CONTROL — AI OVERRIDE`.
5. The joystick now controls the rover.

Safety stops such as `human_stop`, `blocked_by_humans` and `ai_link_loss` remain protected and cannot be bypassed by simply moving the joystick.

## 6. Emergency Stop

For an immediate stop during testing or operation:

1. Release the joystick to neutral.
2. Use the available physical motor-power/emergency-stop method for the rover if one is installed.
3. If the AI system is driving, stopping the laptop AI process will cause the DevKit AI lease to expire and fail closed; do not rely on this as the primary emergency action.
4. Confirm the dashboard reports a stopped safety state before approaching the rover.

The software safety system is not a substitute for a physical emergency-stop mechanism.

## 7. Shutdown

1. Stop normal driving and return the joystick to neutral.
2. Stop the laptop AI process.
3. Verify the rover is stopped.
4. Remove motor/battery power.
5. Power down the ESP32 boards.

## 8. Common Fault Recovery

### AI LINK LOST

**Expected:** `STOPPED — AI LINK LOST`.

1. Verify the laptop is still connected to `ESP32-Rover-AP`.
2. Verify `http://192.168.4.1/` still responds.
3. Restart the AI program if necessary.
4. The new AI session must establish the required STOP handshake.
5. Allow the normal clear-vision and recovery gates to complete before motion resumes.

### CAMERA STALE / CAMERA NOT AVAILABLE

1. Check camera power.
2. Confirm the camera is connected to the DevKit AP.
3. Open `http://192.168.4.2:81/stream`.
4. Check camera IP/settings.
5. If the stream does not recover, keep the rover stopped.

### BLOCKED BY HUMANS

**Expected:** `STOPPED — BLOCKED BY HUMANS`.

1. Do not attempt to bypass the state with the joystick.
2. Allow one side to become human-clear.
3. That side must remain clear for the configured side-clear hold.
4. The dashboard `RELEASE QUALIFIED` indicator must become `TRUE` and identify the qualifying side.
5. A fresh avoidance command is then allowed; recovery remains slow and controlled.

### HUMAN STOP / RELEASE JOYSTICK

If the rover remains in a post-stop release state:

1. Release the joystick completely.
2. Wait for the dashboard to show neutral accepted / new-drive required.
3. Give a new deliberate movement command only after the safety state permits it.

### MANUAL OVERRIDE

If AI unexpectedly changes to MANUAL:

1. Return the joystick to neutral.
2. Decide whether manual control should continue.
3. To return to AI, stop the manual motion and restart/reauthorize the AI session according to the AI integration guide.

### LOW AI HEALTH

If the dashboard reports degraded AI health or reduced AI speed:

1. Slow or stop the rover.
2. Check camera frame age and laptop inference time.
3. Reduce unnecessary laptop load.
4. Check Wi-Fi signal/range.
5. Resume only after AI health returns to the expected range.

## 9. Operator Rule

When the dashboard and physical behavior disagree, **treat the rover as unsafe and stop it physically**. Never use the preview/simulation dashboard as evidence that the real rover is healthy.
