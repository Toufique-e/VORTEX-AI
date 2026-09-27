# Human Avoidance — v15 Predictive

## Decision hierarchy

The laptop can request STOP, AVOID, RECOVER or DRIVE. The DevKit validates every request and remains the final motor authority.

## Predictive avoidance

The laptop uses YOLOv8n class 0 detections with BoT-SORT tracking by default. Current track state contains normalized center, bounding-box dimensions, velocity and recent area growth.

For each tracked person, the planner projects the box forward by `--prediction-horizon` and expands it by `--prediction-margin`.

Predicted occupancy can:

- force a forward STOP
- invalidate LEFT
- invalidate RIGHT
- make both sides blocked

It can never bypass the side-clear hold.

## Rapid approach

Rapid normalized bounding-box area growth above `--approach-area-rate` causes hard STOP. This is intended to catch a person approaching rapidly even when the box is not yet at the normal close-stop size.

## Side selection

A chosen side is only steered after it remains human-clear for `--side-clear-hold` continuously. A side blocked by a current or predicted human is not eligible.

If both side corridors are occupied or predicted occupied, the action is `blocked_candidate`, and the rover remains hard-stopped. Once the blocked state is latched, one side must subsequently remain human-clear for the same configured hold before a fresh AVOID can be accepted.

## Recovery

After successful avoidance, the rover enters slow recovery. The forward path must remain clear for `--recovery-clear-hold`. The laptop then sends a gradual recovery-to-cruise ramp; the DevKit enforces the minimum ramp duration. No direct speed jump to normal cruise is permitted.

## Manual takeover

A meaningful dashboard joystick movement while AI is in `AI_SESSION`, avoidance or recovery revokes AI ownership and enters MANUAL control. Human stop and AI-link-loss safety states remain protected.

## Fundamental limitation

This remains monocular camera-only obstacle perception. A side with no detected human is not proven clear of walls, drops, furniture, vehicles or other non-human hazards.
