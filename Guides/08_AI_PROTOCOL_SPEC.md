# AI Protocol Specification

## Transport

```text
WebSocket: ws://192.168.4.1:81/ws
```

The laptop opens one AI WebSocket session to the DevKit. The DevKit assigns the session to the connected client and accepts only monotonically newer sequence numbers for that session.

## Common fields

```json
{
  "ai": "stop|go|avoid|recover|drive",
  "seq": 1234,
  "client_ms": 12345678,
  "reason": "text"
}
```

Additional fields may carry:

```text
speed
turn
clear_ms
ramp_ms
left_blocked
right_blocked
side_clear_ms
side_clear_required_ms
blocked_ms
blocked_hold_required_ms
chosen_side
resume_condition
release_qualified
release_side
ai_health_score
ai_frame_age_ms
ai_inference_ms
ai_ws_rtt_ms
ai_vision_fps
ai_optical_flow_mag
ai_decision_age_ms
```

## STOP

STOP is the strongest AI motion request. It immediately causes the DevKit to remove the motor target through its centralized safety mechanism.

Typical reasons:

```text
human_detected
camera_stale
camera_content_frozen
vision_inference_fault
vision_empty_result
vision_command_stale
avoidance_timeout
ai_link_lost
```

## GO

GO is not a raw motor-enable command. The DevKit uses it only when the current state permits release and then applies the required neutral/new-drive safety gates.

GO cannot bypass:

- blocked-by-humans;
- AI-link-loss;
- wait-neutral;
- wait-new-drive;
- command expiry;
- health/safety faults.

## AVOID

AVOID is a bounded high-level request:

```text
speed = normalized forward command
turn  = normalized turn command
```

The DevKit clamps the envelope and requires its own safety-state conditions plus the side-clear hold reported by the AI layer.

The selected side must be explicit:

```text
chosen_side = left | right
```

## RECOVER

RECOVER requests slow forward recovery. It is valid only while the safety state allows recovery.

Recovery is followed by a sustained forward-clear requirement and a gradual transition toward normal AI cruise.

## DRIVE

DRIVE requests normal autonomous forward/turn behavior. It is bounded by the DevKit.

During `HUMAN_RECOVERY`, DRIVE is accepted only after:

```text
clear_ms >= configured recovery clear hold
ramp_ms  >= DevKit minimum ramp
```

## Sequence handling

The sequence is 32-bit unsigned. The DevKit uses wrap-safe serial arithmetic. New sessions reset the sequence acceptance baseline after the required STOP handshake.

A duplicate/stale sequence:

```text
must not refresh the AI lease
must not authorize motion
```

## Manual takeover

Dashboard joystick packets are separate from AI packets. A meaningful joystick movement during AI session/avoidance/recovery transfers authority to MANUAL and revokes the AI lease.

Mixed AI + joystick packets are rejected as an integrity fault.

## ACKs

The DevKit returns an ACK containing acceptance information and timing/state data where enabled. Laptop logs interpret its RTT as a round-trip measurement, not one-way network latency.
