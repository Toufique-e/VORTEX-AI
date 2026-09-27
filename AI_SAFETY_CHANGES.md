# AI Safety Changes — v15

## Communication lease

- AI heartbeat default: 125 ms.
- DevKit lease timeout: 450 ms.
- Only valid, strictly newer sequence numbers refresh the lease.
- uint32 serial arithmetic accepts normal wraparound and rejects duplicates/old packets.
- New AI sessions must begin with STOP.
- AI disconnect is fail-closed.

## Laptop fail-closed layers

- latest-frame-only capture
- camera frame-age timeout
- camera content freeze detection
- YOLO/tracker exception -> STOP
- vision-command stale timeout
- independent AI network/heartbeat thread
- reconnect STOP handshake
- tracking-ID fallback
- predictive forward-human STOP
- predictive side-occupancy rejection
- rapid human-approach STOP

## DevKit fail-closed layers

- command queue serialization
- malformed/ambiguous command rejection
- queue-overflow/integrity fault latch
- AI lease timeout
- motor-command expiry
- Hall motion-feedback watchdog
- control-loop overrun detection
- ESP-IDF Task Watchdog monitoring
- non-finite motor target rejection
- centralized motor safety gate
- motor acceleration/deceleration slew limiting
- low AI-health STOP / degraded-health speed limiting

## Recovery gates

- Human STOP cannot be released by joystick.
- blocked_by_humans cannot be cleared by GO or DRIVE.
- A side must be human-clear for the complete configured hold before an avoidance command can steer.
- After avoidance, forward path must remain clear for the recovery hold.
- Recovery-to-normal speed uses a minimum ramp time.
- AI GO does not directly enable motors; neutral/new-drive gating remains active.
- Meaningful joystick movement during AI operation revokes AI and enters MANUAL when that state is allowed.

## AI health telemetry

The laptop sends:

- ai_health_score
- ai_frame_age_ms
- ai_inference_ms
- ai_ws_rtt_ms
- ai_vision_fps
- ai_optical_flow_mag
- ai_decision_age_ms

AI health can reduce speed or stop motion, but can never by itself authorize motion.
