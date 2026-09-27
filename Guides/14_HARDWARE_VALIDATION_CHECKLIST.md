# VORTEX Rover — Hardware Validation Checklist

## Purpose

This is the field checklist for validating the final release on the actual rover. Complete tests in order. Early tests should be performed with motors unloaded or the rover physically restrained; floor tests must use a low speed and a clear area.

## Result Rules

Mark each item **PASS** only when the observed behavior matches the criterion on every required repetition. Mark **FAIL** for any unsafe motion, unexpected resume, missing stop, contradictory status, excessive latency, or unstable hardware behavior.

For any safety-test failure: **STOP TESTING, remove motor power, record the failure, and fix it before proceeding.**

---

## A. ESP32 DevKit Flashing and Boot

### A1. Firmware upload

- [ ] Correct ESP32 DevKit V1 board/port selected.
- [ ] `DevKit_Rover/DevKit_Rover.ino` compiles with the installed ESP32 Arduino environment and required libraries.
- [ ] Upload completes without errors.
- [ ] Serial Monitor shows clean boot output.
- [ ] No continuous reboot/reset loop is present.

**PASS:** firmware uploads and boots repeatedly without reset loops.

### A2. Safe boot

- [ ] With motor power connected but wheels unloaded, boot the DevKit.
- [ ] Verify motor outputs remain zero during boot and initialization.

**PASS:** no commanded motor motion occurs during boot.

---

## B. ESP32-CAM Flashing and Streaming

### B1. Camera firmware

- [ ] Flash `ESP32_CAM/CameraWebServer0.ino` for the AI-Thinker camera.
- [ ] Camera boots without repeated resets.
- [ ] Camera joins the DevKit AP.

**PASS:** camera remains connected and stable for at least 2 minutes.

### B2. Direct MJPEG stream

- [ ] Open `http://192.168.4.2:81/stream` from the laptop.
- [ ] Verify continuous moving video.
- [ ] Check that the image geometry is not unintentionally mirrored/flipped.
- [ ] Check for severe frame freezes or long stalls.

**PASS:** stream remains live for at least 2 minutes with no unexplained persistent freeze.

---

## C. DevKit Dashboard and WebSocket

- [ ] Join `ESP32-Rover-AP` from the laptop.
- [ ] Open `http://192.168.4.1/`.
- [ ] Verify dashboard loads without missing resources.
- [ ] Verify telemetry updates.
- [ ] Verify WebSocket connection remains active.
- [ ] Verify `/settings` opens at `http://192.168.4.1/settings`.

**PASS:** dashboard and settings page operate for at least 2 minutes without repeated WebSocket disconnects or page errors.

---

## D. AI Connection and Lease

- [ ] Install `Laptop_AI/requirements.txt`.
- [ ] Start `Laptop_AI/ai_rover.py` using the documented camera and DevKit IPs.
- [ ] Confirm startup begins in a stopped state.
- [ ] Confirm the AI WebSocket connects.
- [ ] Confirm the DevKit reports an active AI lease.
- [ ] Confirm heartbeat traffic remains within the configured lease interval.

### D1. AI disconnect test

1. Start AI control while the rover is stationary.
2. Allow supervised driving only after the normal gates are satisfied.
3. Disconnect Wi-Fi or terminate the AI process.
4. Observe the DevKit safety state.

**PASS:** the DevKit fails closed on AI lease loss and does not resume motion after reconnect without a fresh AI safety handshake and normal recovery conditions.

The default AI lease timeout is approximately **450 ms**; verify the actual configured value in the current firmware before formal timing acceptance.

---

## E. Joystick and Manual Takeover

- [ ] Verify neutral joystick produces no motion.
- [ ] Verify forward/backward control.
- [ ] Verify left/right turning.
- [ ] Verify release to neutral stops command output as expected.

### E1. AI-to-manual takeover

1. Put the rover into normal AI control.
2. Deliberately move the dashboard joystick.
3. Verify AI ownership is revoked.
4. Verify dashboard changes to MANUAL.
5. Verify the new joystick command controls the rover.

**PASS:** intentional joystick movement changes control ownership to MANUAL without an ambiguous intermediate state.

---

## F. Emergency Stop / Safety-State Tests

### F1. Human-stop test

Use a controlled person/dummy test at low speed and adequate clearance.

**PASS:** person detection causes the expected speed reduction/stop and never causes an uncommanded acceleration.

### F2. Held-joystick restart test

1. Hold the joystick in a non-zero position.
2. Trigger a human stop.
3. Send/allow the AI release condition.
4. Keep the joystick held.

**PASS:** the rover remains stopped until neutral is observed and a new deliberate non-zero drive command is issued.

### F3. Blocked-by-humans test

1. Populate both side human zones.
2. Maintain the condition through the configured blocked hold.
3. Verify hard stop.
4. Verify dashboard state `blocked_by_humans`.
5. Verify normal `GO` cannot bypass the state.

**PASS:** both occupied sides cause a hard stop and a persistent distinct blocked state.

### F4. Side-clear release

1. Leave both sides blocked.
2. Make only one side human-clear.
3. Hold that side clear for the configured side-clear interval.
4. Observe dashboard qualification.

**PASS:** `release_qualified` remains FALSE until the full hold is satisfied, then becomes TRUE and identifies the qualifying side.

---

## G. Human Avoidance and Predictive Behavior

### G1. Single-person side selection

At very low speed in a controlled area:

- [ ] Person left → rover evaluates right-side avoidance.
- [ ] Person right → rover evaluates left-side avoidance.
- [ ] Person center → rover evaluates the available side only after the side-clear hold.

**PASS:** steering never begins into a side containing or predicted-to-contain a human.

### G2. Predictive crossing test

Use a controlled person moving laterally across the rover's intended path.

**PASS:** if the tracked/predicted human is expected to occupy the candidate corridor during the prediction horizon, that side is rejected rather than treated as clear.

### G3. Both sides occupied

**PASS:** rover remains hard-stopped and reports `blocked_by_humans`.

---

## H. Braking-Distance Validation

Do not begin this test at normal maximum speed. Start at the lowest practical autonomous speed.

### H1. Procedure

1. Mark a start line and a stop line.
2. Run a straight approach on a clear surface.
3. Trigger a controlled stop condition.
4. Measure distance from the stop-trigger reference point to the point at which wheel motion has fully ceased.
5. Repeat at least 3 times at the same speed.
6. Repeat at additional speeds only after the lower-speed result is acceptable.

Record:

```text
Test speed:
Stop trigger time:
AI frame age:
YOLO inference time:
WebSocket RTT:
DevKit stop-event time:
Stopping distance:
Surface:
Battery voltage:
```

**PASS:** stopping distance is repeatable, within the pre-declared maximum for that operating speed/surface, and there is no run with unexpected continued drive.

Do **not** use a generic distance value as a universal safety limit. Establish the rover-specific maximum in your actual test environment before increasing speed.

---

## I. Obstacle / Clearance Validation

The system has no independent non-human obstacle sensor. These tests therefore validate only the visual human-avoidance corridor, not guaranteed obstacle avoidance.

### I1. Human-free side corridor

- [ ] Place a controlled human in/near the forward path.
- [ ] Keep the intended avoidance side visibly human-clear.
- [ ] Verify the side-clear hold is completed before steering.

**PASS:** no steering into the occupied side and no side-switch oscillation.

### I2. Narrow/ambiguous corridor

Use a soft, non-damaging test environment.

**PASS:** when the visual side corridor is ambiguous or becomes occupied, the rover slows/stops rather than forcing a maneuver.

### I3. Post-avoidance recovery

- [ ] Verify `SLOW RECOVERY — FORWARD PATH CLEARING`.
- [ ] Maintain a clear forward path for the configured recovery hold.
- [ ] Verify gradual speed ramp to the configured AI cruise speed.

**PASS:** no abrupt jump from recovery speed to cruise speed and no recovery while the forward path is still considered occupied.

---

## J. Battery Behavior

### J1. Startup voltage

- [ ] Measure actual battery voltage with an appropriate external meter.
- [ ] Compare with dashboard telemetry if the optional battery ADC is enabled.

**PASS:** dashboard reading is reasonable and consistent with the measured pack voltage after accounting for the configured divider/calibration.

### J2. Low-voltage behavior

The firmware's exact low-voltage threshold must be taken from the deployed configuration; do not assume an unspecified threshold.

- [ ] Observe telemetry while the pack voltage changes during a controlled test.
- [ ] Check for resets, brownouts, camera dropout, or AI instability.
- [ ] Stop the test before the battery reaches an unsafe discharge condition for the pack.

**PASS:** no uncontrolled motion, repeated brownout/reset cycle, or misleading battery reading is observed during the approved operating range.

---

## K. Long-Run Stability

- [ ] Run dashboard + camera + AI for at least 15 minutes at low/no motion.
- [ ] Observe WebSocket stability.
- [ ] Observe camera stability.
- [ ] Observe AI frame age and inference latency.
- [ ] Observe DevKit resets/free-heap or health diagnostics if exposed.

**PASS:** no unexplained resets, runaway memory behavior, stuck motors, or progressive communication degradation.

---

## L. Final Release Gate

Do not mark the rover field-ready until all of the following are PASS:

- [ ] DevKit flashing/boot
- [ ] CAM flashing/stream
- [ ] Dashboard/WebSocket
- [ ] AI lease and reconnect
- [ ] Manual takeover
- [ ] Human stop
- [ ] Blocked-by-humans
- [ ] Side-clear/release qualification
- [ ] Predictive side rejection test
- [ ] Slow recovery and speed ramp
- [ ] Braking distance
- [ ] Clearance tests appropriate to the environment
- [ ] Battery telemetry/behavior
- [ ] Long-run stability

**Final decision:** PASS only when every safety item is PASS and the measured braking/latency data is recorded.
