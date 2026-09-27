# Wiring quick reference

## DevKit V1

| Function | GPIO |
|---|---:|
| Left BTS7960 RPWM | 25 |
| Left BTS7960 LPWM | 26 |
| Right BTS7960 RPWM | 27 |
| Right BTS7960 LPWM | 14 |
| Hall sensor | 35 |
| MPU6050 SDA | 21 |
| MPU6050 SCL | 22 |
| CAM UART RX2 | 16 |
| CAM UART TX2 | 17 |
| Optional battery ADC | 33 |

## AI-Thinker ESP32-CAM

| Function | GPIO |
|---|---:|
| Flash LED | 4 |
| CAM UART RX2 | 13 |
| CAM UART TX2 | 14 |

UART:

```text
DevKit TX2 17 -> Camera RX2 13
DevKit RX2 16 <- Camera TX2 14
DevKit GND    --- Camera GND
```

Do not use the camera SD interface while using the GPIO13/14 housekeeping UART arrangement.

## Camera pan servo (added)
- Servo signal: ESP32 DevKit GPIO13.
- Servo V+: use an external regulated 5V supply sized for the servo.
- Servo GND: connect to the external supply GND and ESP32 GND (common ground).
- Positions: LEFT=150°, CENTER=90°, RIGHT=30° (adjust in `DevKit_Rover.ino` for the actual bracket).
- Do not power a high-current servo from the ESP32 3.3V pin.

## Explicit control modes (added)
- `MANUAL`: dashboard joystick controls the rover; AI packets are rejected.
- `AUTONOMOUS AI`: joystick is ignored; laptop AI must claim the AI lease and pass the existing safety state machine.
- Switching modes always stops the motors first.
