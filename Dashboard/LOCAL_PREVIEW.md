# Local Dashboard Preview

## Windows

Double-click:

```text
start_preview.bat
```

Then open:

```text
http://127.0.0.1:8765/
```

Camera settings:

```text
http://127.0.0.1:8765/settings
```

To stop the server:

```text
stop_preview.bat
```

The local preview does not require ESP32 hardware. It simulates dashboard telemetry, human-zone state, release qualification, countdown, recovery, and camera settings controls.

## Real rover

When the DevKit is running, use:

```text
http://192.168.4.1/
http://192.168.4.1/settings
```

The camera stream itself remains direct from the ESP32-CAM, normally:

```text
http://192.168.4.2:81/stream
```
