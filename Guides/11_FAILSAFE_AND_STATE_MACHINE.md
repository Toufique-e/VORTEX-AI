# 8. Fail-Safe Architecture and State Machine

## 8.1 States

```text
MANUAL
AI_SESSION
HUMAN_AVOID_LEFT
HUMAN_AVOID_RIGHT
HUMAN_RECOVERY
HUMAN_STOP
BLOCKED_BY_HUMANS
AI_LINK_LOSS
WAIT_NEUTRAL
WAIT_NEW_DRIVE
```

## 8.2 Hard-stop states

The following states force zero motor target:

```text
HUMAN_STOP
BLOCKED_BY_HUMANS
AI_LINK_LOSS
WAIT_NEUTRAL
WAIT_NEW_DRIVE
```

## 8.3 Allowed recovery paths

```text
MANUAL -> AI_SESSION
AI_SESSION -> HUMAN_STOP
AI_SESSION -> HUMAN_AVOID_LEFT/RIGHT
AI_SESSION -> AI_LINK_LOSS
HUMAN_STOP -> WAIT_NEUTRAL       (valid GO/clear conditions)
BLOCKED_BY_HUMANS -> HUMAN_AVOID_LEFT/RIGHT (only after side-clear hold)
HUMAN_AVOID_* -> HUMAN_RECOVERY
HUMAN_RECOVERY -> AI_SESSION    (after clear hold + ramp)
WAIT_NEUTRAL -> WAIT_NEW_DRIVE
WAIT_NEW_DRIVE -> AI_SESSION    (fresh AI/drive conditions)
AI_LINK_LOSS -> new AI session + STOP handshake -> recovery path
```

## 8.4 Forbidden transitions

```text
HUMAN_STOP -> DRIVE
BLOCKED_BY_HUMANS -> DRIVE
AI_LINK_LOSS -> DRIVE
WAIT_NEUTRAL -> DRIVE on non-neutral stale input
WAIT_NEW_DRIVE -> DRIVE without the required fresh command
new AI connection + GO -> DRIVE
stale sequence -> refresh lease
joystick -> clear HUMAN_STOP
joystick -> clear AI_LINK_LOSS
```

## 8.5 Lease

The laptop refreshes the DevKit AI lease every 125 ms by default. The DevKit expires the lease after 450 ms without a valid packet.

A valid AI packet must pass sequence validation. A stale/duplicate packet does not refresh the lease.

## 8.6 Sequence wraparound

The sequence check uses serial-number arithmetic across `uint32_t` wraparound. Normal wrap:

```text
0xFFFFFFFF -> 0
```

is valid. Duplicate and backward packets are rejected.

## 8.7 Motor command expiry

Even when an AI lease is healthy, a non-zero motor target must be refreshed within the motor-command validity window. This provides an additional stop path if the motion stream stalls.

## 8.8 Control-loop watchdogs

The DevKit also uses:

- control-loop overrun detection;
- ESP-IDF Task Watchdog;
- Hall motion-feedback watchdog;
- WebSocket command queue integrity checks.

The safety objective is that a failure of one monitored software path does not silently become continued motion.

## 8.9 Manual override

Meaningful joystick movement during AI session, avoidance or recovery transfers authority to MANUAL. The DevKit revokes the AI lease so the laptop cannot immediately reclaim control.

Hard safety states remain protected.
