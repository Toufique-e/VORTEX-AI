# 4. Operating Guide

## 4.1 Start sequence

1. Put the rover on a stand or safe open test area.
2. Power the DevKit.
3. Connect your laptop to `ESP32-Rover-AP`.
4. Confirm `192.168.4.1` opens in a browser.
5. Power the ESP32-CAM.
6. Confirm the camera stream at `http://192.168.4.2:81/stream`.
7. Open the dashboard at `http://192.168.4.1/`.
8. Verify joystick is physically/virtually at neutral.
9. Start `Laptop_AI/ai_rover.py`.
10. Confirm the AI lease becomes active.
11. The AI starts in STOP and must establish a fresh clear-vision condition before autonomous drive is allowed.

## 4.2 Manual driving

Move the single joystick:

- Up/down = forward/reverse.
- Left/right = turn.

Manual drive is subject to the DevKit command watchdog and motor safety gate.

If a previously stopped state requires neutral-first recovery, release the joystick to neutral before giving a new movement command.

## 4.3 Starting AI control

Run, for example:

```bash
python ai_rover.py --cam-ip 192.168.4.2 --devkit-ip 192.168.4.1 --show
```

The laptop first establishes a STOP handshake. Do not interpret the dashboard preview's simulated AI status as live rover status.

## 4.4 Human detected

Expected behavior:

```text
Person detected
    -> decelerate/stop as required
    -> evaluate forward and side occupancy
    -> confirm chosen side clear for hold
    -> AVOID LEFT or AVOID RIGHT
```

A weak-confidence forward-person detection is intentionally conservative and can result in STOP rather than steering.

## 4.5 Both sides blocked

If both side corridors are occupied or predicted occupied:

```text
STOPPED — BLOCKED BY HUMANS
```

The rover does not attempt to pass between people. To recover, one side must remain human-clear for the configured hold period, after which a fresh avoidance request is required.

## 4.6 Recovery

After avoidance the rover enters:

```text
SLOW RECOVERY — FORWARD PATH CLEARING
```

The forward path must remain continuously clear for the configured recovery interval. Then the system ramps toward normal AI cruise speed. The ramp prevents an abrupt return to cruise speed.

## 4.7 Manual takeover

While AI is actively controlling the rover, intentionally moving the dashboard joystick requests manual takeover. The DevKit revokes AI ownership and enters MANUAL.

The exception is a hard safety state such as `human_stop` or `ai_link_loss`; those states keep the motors stopped until their specific recovery conditions are met.

## 4.8 Camera settings

On the real rover:

```text
http://192.168.4.1/settings
```

The settings page controls the camera through DevKit->CAM UART housekeeping messages. Video itself remains direct CAM->browser/laptop.

Available controls include:

- camera IP
- flash on/off
- frame size
- JPEG quality
- brightness
- contrast
- saturation
- horizontal mirror
- vertical flip
- auto-exposure/related camera tuning exposed by the page

Avoid horizontal mirroring when testing left/right avoidance unless you intentionally account for the changed image geometry.

## 4.9 Shutdown

1. Stop AI operation first.
2. Return the joystick to neutral.
3. Verify the dashboard reports a stopped/idle state.
4. Remove motor power.
5. Power down the ESP32 boards.

Never stop only the laptop application while leaving a moving rover in an uncontrolled state; the DevKit will fail closed on AI lease expiry, but you should still perform orderly shutdowns.
