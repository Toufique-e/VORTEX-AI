# 6. AI Configuration and Tuning

This guide uses the current source defaults. Tune only after the baseline system works.

## 6.1 Core settings

| Option | Default | Meaning |
|---|---:|---|
| `--confidence` | 0.45 | Strong person confidence used for autonomous avoidance |
| `--stop-confidence` | 0.25 | Lower threshold retained for conservative forward stopping |
| `--keepalive` | 0.125 s | AI packet interval |
| `--lease-timeout` | 0.45 s | Expected DevKit AI lease timeout |
| `--clear-hold` | 0.80 s | Continuous clear time before normal GO when no recovery is active |
| `--frame-timeout` | 0.45 s | Maximum camera-frame age |
| `--imgsz` | 416 | YOLO input size |
| `--tracker` | `botsort.yaml` | Tracking configuration |

## 6.2 Avoidance settings

| Option | Default | Meaning |
|---|---:|---|
| `--front-y` | 0.55 | Lower-image threshold defining forward occupancy |
| `--avoid-max-speed` | 0.30 | Maximum normalized avoidance speed requested |
| `--avoid-min-speed` | 0.14 | Minimum normalized avoidance speed |
| `--avoid-max-turn` | 0.50 | Maximum normalized avoidance turn |
| `--close-stop-height` | 0.70 | Large person box height fraction that triggers hard stop |
| `--side-corridor` | 0.26 | Side-zone geometry parameter |
| `--side-clear-hold` | 0.30 s | Side must stay clear before steering |
| `--blocked-hold` | 0.30 s | Both-side occupancy confirmation |
| `--avoid-timeout` | 3.0 s | Maximum continuous avoidance duration |
| `--avoid-switch-window` | 2.0 s | Window for side-switch instability detection |
| `--avoid-switch-max` | 2 | Side switches allowed in that window |

## 6.3 Prediction settings

| Option | Default | Meaning |
|---|---:|---|
| `--prediction-horizon` | 0.35 s | How far to project a track forward |
| `--prediction-margin` | 0.06 | Conservative expansion around projected box |
| `--approach-area-rate` | 0.70 | Rapid box-area growth threshold |
| `--track-memory` | 0.35 s | Recent forward-human memory after a track is temporarily lost |

## 6.4 Recovery settings

| Option | Default | Meaning |
|---|---:|---|
| `--recovery-clear-hold` | 0.90 s | Required forward-clear interval |
| `--recovery-start-speed` | 0.08 | Recovery starting speed |
| `--recovery-max-speed` | 0.18 | Slow recovery maximum |
| `--recovery-ramp` | 1.50 s | Ramp duration toward normal cruise |
| `--ai-cruise-speed` | 0.30 | Normal AI cruise request |

## 6.5 AI-health settings

| Option | Default | Meaning |
|---|---:|---|
| `--health-stop` | 35 | At/below this health score: autonomous stop |
| `--health-caution` | 75 | Below this score: speed reduction |
| `--min-vision-fps` | 8.0 | Expected minimum vision rate |
| `--optical-flow-sample-every` | 3 | Compute optical-flow health every N processed frames |
| `--optical-flow-jump` | 18.0 | Large motion jump treated as instability |
| `--optical-flow-freeze` | 0.35 | Very low flow while moving contributes a health penalty |

## 6.6 Watchdogs

| Option | Default | Meaning |
|---|---:|---|
| `--freeze-timeout` | 0.35 s | Camera content freeze timeout |
| `--command-stale-timeout` | 0.25 s | Vision command freshness limit |
| `--io-timeout` | 0.05 s | WebSocket receive timeout in network thread |
| `--reconnect` | 0.25 s | Reconnect interval |

## 6.7 Safe tuning procedure

Tune one variable at a time.

Recommended order:

1. `--imgsz` and laptop performance.
2. `--confidence` / `--stop-confidence`.
3. side corridor geometry.
4. prediction horizon/margin.
5. side-clear hold.
6. avoid speed/turn limits.
7. recovery speed/ramp.
8. AI-health thresholds.

For each change record:

```text
parameter set
lighting
camera resolution
rover speed
vision FPS
frame age
inference p95
false stops
missed humans
avoidance side decisions
```

Never increase speed simply because detection looks stable. Verify latency and physical stopping distance first.
