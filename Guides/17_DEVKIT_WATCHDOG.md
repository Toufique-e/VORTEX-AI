# DevKit Watchdog Guide

The DevKit uses the ESP-IDF Task Watchdog Timer (TWDT) as a control-task liveness monitor.

The current code:

- configures a 1000 ms timeout
- subscribes the current control task with `esp_task_wdt_add(NULL)`
- feeds it from the control loop
- configures `trigger_panic = true`

This is a reset/liveness safety net. It does not replace the application-level AI lease, motor command expiry, Hall feedback watchdog or command integrity checks.

Espressif documents that a subscribed task must periodically call `esp_task_wdt_reset()` and that `trigger_panic` can be used to cause panic behavior on timeout.

Do not shorten the timeout until the real ESP32 build has been measured for worst-case I2C, network and startup paths.
