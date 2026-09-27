# Laptop AI Controller — v15

The laptop joins `ESP32-Rover-AP`, reads the ESP32-CAM MJPEG stream directly and sends bounded AI requests to the DevKit WebSocket. It never drives motors directly.

## Default pipeline

Camera latest-frame capture -> YOLOv8n person detection -> BoT-SORT tracking -> temporal safety filters -> predictive human planner -> side-clear/blocked/recovery state logic -> AI heartbeat/network thread -> DevKit safety state machine.

BoT-SORT is the default tracker for this moving-camera rover. Ultralytics supports tracker YAML selection and documents BoT-SORT with configurable camera-motion compensation.

## Safety/watchdogs

- AI starts with STOP.
- New WebSocket sessions require a STOP handshake.
- 125 ms heartbeat refreshes the DevKit lease.
- Camera frame-age timeout requests STOP.
- Camera content freeze while motion is requested requests STOP.
- YOLO/tracker exceptions request STOP.
- Vision command staleness requests STOP.
- Missing tracker IDs degrade to detection-only safety behavior.
- Recent-forward-human memory prevents a single missed frame from immediately causing GO.
- Rapid human bounding-box growth causes hard STOP.
- Predicted forward occupancy causes STOP.
- Predicted side occupancy invalidates that side for avoidance.
- Both side corridors occupied/predicted occupied -> blocked-by-humans.
- Side-clear hold is required before steering.
- Avoidance duration and side-switch instability are bounded.
- Recovery requires sustained forward clearance and a gradual speed ramp.

## Predictive parameters

```text
--tracker botsort.yaml
--prediction-horizon 0.35
--prediction-margin 0.06
--approach-area-rate 0.70
--track-memory 0.35
--side-clear-hold 0.30
--blocked-hold 0.30
--avoid-timeout 3.0
```

## AI-health parameters

```text
--health-stop 35
--health-caution 75
--min-vision-fps 8.0
--optical-flow-sample-every 3
--optical-flow-jump 18.0
--optical-flow-freeze 0.35
```

AI health combines frame freshness, inference latency, FPS, WebSocket RTT and optical-flow consistency. Lower health reduces requested speed; very low health stops autonomous motion.

## Example

```bash
python ai_rover.py --cam-ip 192.168.4.2 --devkit-ip 192.168.4.1 --show
```

For actual autonomous testing, use a low physical rover speed and an open controlled test area.
