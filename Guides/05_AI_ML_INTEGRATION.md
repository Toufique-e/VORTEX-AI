# 5. AI/ML Integration Guide

## 5.1 Role of the laptop ML layer

The laptop is the perception and planning computer. It receives camera frames and produces high-level requests. It must never contain the final motor authority.

```text
ESP32-CAM
   | HTTP MJPEG
   v
OpenCV latest-frame capture
   v
YOLOv8n person detection
   v
BoT-SORT tracking
   v
Temporal filtering
   v
Predictive human-motion projection
   v
Side/forward occupancy + safety planner
   v
AI command channel
   v
ESP32 DevKit safety arbiter
   v
motors
```

## 5.2 Environment setup

From `Laptop_AI`:

### Windows PowerShell

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
pip install -r requirements.txt
```

### Windows Command Prompt

```bat
python -m venv .venv
.venv\Scripts\activate
python -m pip install --upgrade pip
pip install -r requirements.txt
```

Verify imports:

```bash
python -c "import cv2, websocket; from ultralytics import YOLO; print('AI environment OK')"
```

## 5.3 Model selection

The default model path is:

```text
yolov8n.pt
```

The current script calls the Ultralytics tracker with:

```python
classes=[0]
```

so only the configured person class is retained by the planner.

For a custom model, change `--model` to the path to your trained weights. The custom model must preserve a person class mapping compatible with the planner's `classes=[0]` assumption, or the source must be changed and re-tested.

Example:

```bash
python ai_rover.py --model path/to/best.pt
```

## 5.4 Starting the AI controller

Default network addresses:

```bash
python ai_rover.py \
  --devkit-ip 192.168.4.1 \
  --cam-ip 192.168.4.2 \
  --show
```

On Windows CMD, the same command can be written on one line:

```cmd
python ai_rover.py --devkit-ip 192.168.4.1 --cam-ip 192.168.4.2 --show
```

## 5.5 What the AI thread does

### Capture thread

The camera capture thread continuously drains the MJPEG source and retains only the newest frame. This avoids building a stale queue when inference is slower than camera delivery.

### Inference thread

The planner processes the newest available frame with YOLO tracking. It records frame age, inference time, vision FPS and other metrics.

### Network thread

A separate thread owns the DevKit WebSocket. It:

- reconnects after a failure;
- begins each new session with STOP;
- sends AI packets at the configured heartbeat interval;
- keeps sequence numbers;
- monitors command freshness;
- records ACK round-trip time.

This separation prevents a blocked send from directly blocking camera/inference processing.

## 5.6 AI packet contract

The laptop sends packets conceptually like:

```json
{
  "ai": "stop",
  "seq": 1001,
  "client_ms": 123456,
  "reason": "human_detected"
}
```

Avoidance:

```json
{
  "ai": "avoid",
  "seq": 1002,
  "client_ms": 123581,
  "reason": "avoid_right_confirmed",
  "speed": 0.16,
  "turn": 0.12,
  "side_clear_ms": 300,
  "chosen_side": "right",
  "release_qualified": true,
  "release_side": "right"
}
```

Recovery:

```json
{
  "ai": "recover",
  "seq": 1003,
  "client_ms": 123706,
  "reason": "recovery_clear_pending",
  "speed": 0.10
}
```

Normal drive:

```json
{
  "ai": "drive",
  "seq": 1004,
  "client_ms": 123831,
  "reason": "recovery_to_ai_complete",
  "speed": 0.30,
  "turn": 0.00,
  "clear_ms": 900,
  "ramp_ms": 1500
}
```

The DevKit clamps/validates these values and may reject them based on safety state, lease, sequence or timing.

## 5.7 Adding a custom ML model

A practical integration workflow:

1. Record representative frames from the actual ESP32-CAM at the installed camera height/angle.
2. Include difficult cases: low light, backlight, partial occlusion, fast movement, edge-of-frame people and multiple people.
3. Annotate `person` consistently.
4. Train/validate a custom Ultralytics detector separately from the rover.
5. Confirm the exported model returns reliable person detections on the actual camera frames.
6. Place the resulting weights on the laptop.
7. Run with `--model <weights>`.
8. Keep `classes=[0]` only if the custom model keeps person at index 0.
9. Re-run the predictive-avoidance tests.
10. Re-measure frame age/inference latency before using the custom model on the floor.

Do not reduce confidence thresholds merely to force avoidance behavior to trigger. Tune based on recorded false-positive/false-negative results.

## 5.8 Tracker integration

The current default is:

```text
--tracker botsort.yaml
```

BoT-SORT supplies track continuity suitable for a moving camera. The planner still has a conservative fallback: when IDs are missing, detection-only safety logic is used rather than crashing.

## 5.9 Predictive layer

For each person track the planner estimates:

```text
current center
current width/height
recent velocity
recent area growth
```

It projects the box forward by:

```text
--prediction-horizon 0.35
```

and expands it by:

```text
--prediction-margin 0.06
```

If the projected human occupies the forward corridor, the planner can request STOP. If the projection intersects a side corridor, that side is rejected.

## 5.10 AI-health integration

The laptop computes a 0-100 health score from:

- frame age;
- inference duration;
- vision FPS;
- WebSocket RTT;
- optical-flow consistency.

The DevKit treats low health as a restriction only:

```text
healthy       -> normal envelope
caution       -> reduced autonomous speed
critical      -> autonomous stop
```

Health can never authorize movement through a safety stop.

## 5.11 ML performance tuning

The default inference size is:

```text
--imgsz 416
```

Benchmark 320/416/512/640 on the actual laptop and camera rather than assuming higher resolution is safer. Measure:

- FPS;
- p95 inference time;
- frame age;
- missed detections;
- false detections;
- stop latency.

A fresh lower-resolution frame can be more useful to a real-time safety loop than a much older high-resolution frame.

## 5.12 Keep the safety boundary intact

The laptop can request:

```text
STOP
AVOID
RECOVER
DRIVE
```

It cannot request:

```text
raw motor PWM
BTS7960 duty cycle
arbitrary acceleration
```

Those remain inside the DevKit authority boundary.
