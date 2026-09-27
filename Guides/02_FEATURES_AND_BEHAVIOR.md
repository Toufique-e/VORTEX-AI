# 2. Features and Behavior

## 2.1 Driving

- Single joystick: vertical = forward/reverse, horizontal = turn.
- Differential mixing produces continuously variable left/right motor targets.
- DevKit clamps autonomous speed and turn envelopes.
- Motor acceleration and deceleration are slew-limited.
- A motor command expires if it is not refreshed.
- A dashboard drive watchdog stops the rover when joystick packets stop arriving.

## 2.2 AI supervision

AI packets carry:

- action (`stop`, `go`, `avoid`, `recover`, `drive`)
- `seq` sequence number
- `client_ms`
- reason
- side occupancy/hold information where relevant
- AI health and timing data

The DevKit rejects malformed packets, stale sequences, ambiguous AI+joystick packets and commands that are forbidden by the current safety state.

## 2.3 Human detection

The laptop uses YOLOv8n with `classes=[0]`, retaining person detections only. BoT-SORT tracking is the default for the moving camera. Temporal memory prevents a single missed frame from immediately clearing a recent forward-person condition.

## 2.4 Predictive human avoidance

For tracked people the laptop maintains normalized center/size history and estimates motion. It projects each tracked bounding box into the near future using a configurable prediction horizon and adds a conservative prediction margin.

Predicted occupancy can:

- stop forward motion;
- invalidate LEFT;
- invalidate RIGHT;
- make both sides blocked.

Prediction is conservative: it can remove options or request a stop, but it cannot bypass the DevKit's side-clear hold or motor safety gate.

## 2.5 Side-clear avoidance

The rover does not steer as soon as a side looks visually empty.

The selected side must remain human-clear for the configured side-clear hold. If both sides are occupied/predicted occupied, the rover enters the blocked-by-humans safety condition.

## 2.6 Blocked-by-humans

When both side corridors remain blocked for the configured blocked hold:

```text
STOPPED — BLOCKED BY HUMANS
```

The state cannot be cleared by normal `GO` or `DRIVE`. One side must subsequently remain human-clear for the configured hold, and a fresh confirmed avoidance request must be accepted before the recovery sequence can proceed.

## 2.7 Slow recovery

After avoidance, the rover remains in slow recovery while the forward path is continuously clear for the configured recovery-clear hold. After that hold, the AI ramps from slow recovery speed toward normal cruise speed over the configured ramp duration. The DevKit enforces a minimum ramp duration so the laptop cannot request an abrupt jump to normal speed.

## 2.8 Manual takeover

While the rover is in AI session, avoidance or recovery, meaningful dashboard joystick motion revokes AI ownership and enters manual control.

Hard safety states remain protected:

- human stop
- blocked by humans
- AI link loss
- wait-neutral
- wait-new-drive

## 2.9 Fail-safe features

The v15 build includes multiple software fail-closed paths:

- AI heartbeat/lease timeout
- AI client disconnect handling
- new-session STOP handshake
- uint32 sequence validation
- stale/duplicate rejection
- command queue overflow stop
- mixed-packet rejection
- non-finite command rejection
- motor-command expiry
- drive watchdog
- Hall motion-feedback watchdog
- control-loop overrun protection
- ESP-IDF Task Watchdog integration
- camera frame-age watchdog
- camera-content freeze watchdog
- YOLO/tracker exception STOP
- vision command-stale STOP
- AI-health stop threshold
- camera-motion/optical-flow health contribution
- avoidance timeout
- avoidance-side switch watchdog
- centralized motor safety gate

## 2.10 Dashboard

The laptop-oriented dashboard contains:

- full-width camera area
- single rover joystick
- speed/heading/X/Y/distance/battery telemetry
- AI lease and AI-health indicators
- odometry confidence
- left/right human-zone occupancy
- blocked hold progress
- side-clear hold progress
- release qualification and qualifying side
- side-clear countdown
- exact resume condition
- avoidance/recovery state
- 2D dead-reckoning trail
- separate camera-settings page

## 2.11 Local preview

The standalone dashboard preview opens in a clearly labeled `LOCAL PREVIEW / SIMULATION` mode. It does not claim to be live rover telemetry. Synthetic values appear only after a `SIM:` scenario is selected.
