# Predictive Human Avoidance Guide

## What the planner does

1. YOLOv8n detects persons.
2. BoT-SORT maintains track IDs when available.
3. The laptop records recent normalized position, size and velocity per track.
4. Human boxes are projected forward by `--prediction-horizon` (default 0.35 s).
5. A prediction margin is added with `--prediction-margin` (default 0.06 normalized).
6. Predicted forward occupancy causes STOP.
7. A side predicted to become occupied is not considered a valid avoidance side.
8. Rapid bounding-box area growth above `--approach-area-rate` causes STOP.
9. The chosen side must still pass the configured side-clear hold before steering is sent.

## Important safety property

Prediction is conservative. It is allowed to remove options, slow or stop the rover; it is never allowed to bypass the DevKit motor safety gate.

## Suggested starting parameters

- `--tracker botsort.yaml`
- `--prediction-horizon 0.35`
- `--prediction-margin 0.06`
- `--approach-area-rate 0.70`
- `--side-clear-hold 0.30`
- `--avoid-timeout 3.0`

Tune only after recording your actual camera footage and measuring false detections/latency.

## Predictive test scenarios

Test at walking speed or lower first:

- Person stationary in center.
- Person moving from right toward the center.
- Person moving from left toward the center.
- Person crossing into the predicted avoidance side.
- Person rapidly approaching the camera.
- Two people occupying both side corridors.
- A briefly missed person track.
- Camera stream freeze.
