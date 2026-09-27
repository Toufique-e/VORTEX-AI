# AI Health and Latency Guide

## AI health inputs

The laptop computes a 0-100 health score from:

- camera frame age
- YOLO inference latency
- vision FPS
- WebSocket round-trip time
- optical-flow consistency

The DevKit uses this score only as a restriction: degraded health reduces speed; health at/below the stop threshold stops autonomous motion.

## Timing logs

The laptop logs:

- frame age
- inference duration
- vision FPS
- WebSocket RTT
- optical-flow magnitude
- decision age from frame capture to AI decision
- requested AI action/reason
- sequence number

WebSocket RTT is round-trip time. It must not be interpreted as one-way packet latency.

## Physical latency measurement

For a real rover, instrument these events:

T0 camera exposure/frame capture
T1 laptop receives fresh frame
T2 YOLO starts
T3 YOLO ends
T4 STOP decision created
T5 STOP packet transmitted
T6 DevKit receives/accepts STOP
T7 DevKit PWM reaches zero
T8 rover physically stops

The useful safety metric is T8-T0 plus the physical braking distance.
