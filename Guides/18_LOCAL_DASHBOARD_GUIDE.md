# Local Dashboard Guide

## Quick preview on Windows

Double-click:

```text
Dashboard/start_preview.bat
```

Then open:

```text
http://127.0.0.1:8765/
```

Camera settings:

```text
http://127.0.0.1:8765/settings
```

## Stop the preview server

Double-click:

```text
Dashboard/stop_preview.bat
```

## Offline files

- `Dashboard/dashboard_preview.html`
- `Dashboard/camera_settings.html`

The offline settings page simulates the camera preview and lightweight UART commands. On the real DevKit, the same settings page is served from `/settings` and commands are sent through the DevKit UART bridge.
