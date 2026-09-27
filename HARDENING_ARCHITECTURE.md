# v15 Hardening Architecture

## Safety hierarchy

The DevKit is the final authority. Laptop decisions are requests only.

Safety precedence:

1. Motor stop request / integrity fault.
2. AI lease loss.
3. Human stop / blocked-by-humans.
4. Wait-neutral / wait-new-drive gates.
5. AI health low.
6. AI avoidance/recovery.
7. AI normal drive.
8. Manual drive when AI is not active and safety gates permit it.

A lower-priority source can never clear a higher-priority stop state.

## Independent fail-closed paths

### AI communication lease
The laptop refreshes the AI lease continuously. A stale lease or AI-client disconnect transitions the DevKit to AI_LINK_LOSS and forces motor output to zero.

### Motor command expiry
A non-zero motor target must be refreshed within MOTOR_COMMAND_EXPIRY_MS. Otherwise the target is cleared and the rover is stopped.

### Control-loop overrun
The main control loop tracks its entry interval. Excessive delay transitions supervised control to a fail-closed state.

### ESP32 Task Watchdog
The DevKit subscribes the running control task to the ESP-IDF Task Watchdog Timer. A monitored task must periodically call esp_task_wdt_reset(); on configured timeout the watchdog can invoke panic/reset behavior. See Espressif ESP-IDF Task Watchdog documentation.

### Hall feedback watchdog
Meaningful motor demand without expected Hall pulses for the configured interval produces a fail-closed stop.

### Command integrity
Malformed JSON, ambiguous mixed AI+joystick packets, non-finite motor targets, queue overflow or stale AI sequence numbers cannot authorize motion.

### AI-health gate
Laptop vision/network health is carried with every AI packet. Low health stops autonomous motion; degraded health reduces speed. Health can never release a safety stop.

## Predictive avoidance

The laptop maintains short track history for each visible person. For a configurable prediction horizon it projects the current center/size using constant velocity. Predicted occupancy is conservative: it can reject an avoidance side or trigger STOP, but it cannot bypass the required side-clear hold.

Rapid bounding-box growth is treated as rapid approach and requests STOP.

## Limits

This is software redundancy using the existing sensing channels. It is not independent sensor redundancy. A camera-only perception system cannot guarantee that a human-free image region contains no wall, drop, furniture or other non-human obstacle.
