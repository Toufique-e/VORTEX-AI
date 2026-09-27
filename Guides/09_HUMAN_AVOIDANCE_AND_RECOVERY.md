# 7. Human Avoidance and Recovery

## 7.1 Decision flow

```text
fresh camera frame
      |
      v
person detection/tracking
      |
      +--> no reliable frame ----------> STOP
      |
      +--> forward human --------------> STOP / avoid evaluation
      |
      +--> both sides blocked ---------> BLOCKED BY HUMANS
      |
      v
predict human positions
      |
      v
select candidate side
      |
      v
side clear continuously for hold?
      |
   no +----> remain stopped / candidate
      |
     yes
      |
      v
AVOID
      |
      v
forward path clear continuously?
      |
      v
SLOW RECOVERY
      |
      v
recovery clear hold complete
      |
      v
ramp to normal cruise
      |
      v
AI_SESSION
```

## 7.2 Side qualification

A side is not considered valid merely because its current frame is empty. The system considers current and predicted human occupancy. Then it requires the side-clear timer to reach the configured hold.

The dashboard exposes:

- left zone occupied/clear;
- right zone occupied/clear;
- hold elapsed;
- hold required;
- countdown remaining;
- release qualification;
- qualifying side.

## 7.3 Blocked-by-humans state

Both side corridors occupied/predicted occupied result in a hard stop. After the blocked state is latched, normal `GO` and `DRIVE` cannot clear it.

One side must become human-clear and stay clear for the configured hold. The laptop then submits a confirmed `AVOID` for that side. The DevKit accepts the request only if the AI lease/session and safety conditions are valid.

## 7.4 Predictive behavior

Prediction is based on short track history and a constant-velocity projection. It is not true depth estimation.

The planner uses prediction to be conservative. For example, a side that is clear now but is predicted to become occupied can be rejected before the rover commits to that side.

## 7.5 Rapid approach

Fast growth in a person's normalized bounding-box area is treated as a rapid-approach condition. The planner requests STOP instead of waiting for the normal avoidance sequence.

## 7.6 Recovery

After the avoidance maneuver, the rover should not immediately resume normal cruise.

It first remains in slow recovery. The forward path must remain clear for the configured hold, after which the speed target ramps from recovery maximum toward the normal AI cruise target.

The DevKit also imposes its own minimum ramp duration.

## 7.7 What this does not guarantee

Because the only semantic obstacle sensor is a single camera, a human-free side is not proof of a physically clear path. The system can miss walls, furniture, boxes, steps, drops and other non-human hazards.

Use low speed and controlled environments during physical testing.
