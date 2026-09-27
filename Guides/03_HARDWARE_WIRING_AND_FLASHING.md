# 3. Hardware, Wiring and Firmware Flashing

## 3.1 DevKit V1 wiring

| Function | GPIO |
|---|---:|
| Left BTS7960 RPWM | 25 |
| Left BTS7960 LPWM | 26 |
| Right BTS7960 RPWM | 27 |
| Right BTS7960 LPWM | 14 |
| Hall sensor | 35 |
| MPU6050 SDA | 21 |
| MPU6050 SCL | 22 |
| Optional battery ADC | 33 |
| CAM UART RX2 | 16 |
| CAM UART TX2 | 17 |

GPIO35 is input-only on the ESP32 and does not provide an internal pull-up. The Hall sensor interface must therefore provide an appropriate external signal/pull-up arrangement for your Hall device.

## 3.2 AI-Thinker ESP32-CAM

| Function | GPIO |
|---|---:|
| Flash LED | 4 |
| UART RX2 | 13 |
| UART TX2 | 14 |

UART:

```text
DevKit TX2 GPIO17 -> CAM RX2 GPIO13
DevKit RX2 GPIO16 <- CAM TX2 GPIO14
DevKit GND         -> CAM GND
```

UART is optional. The rover still works without it; the camera simply loses remote housekeeping control.

## 3.3 Motor driver

The firmware uses four PWM outputs for BTS7960-style differential motor control.

Before connecting the motor power supply, confirm:

- left/right motor polarity;
- all grounds are common as required by the driver/interface;
- PWM signal levels are compatible with the driver;
- the motor driver can be held disabled during ESP32 reset/boot in your actual wiring.

The software safety logic is not a substitute for a physical motor-power safety method.

## 3.4 I2C

The MPU6050 uses the DevKit default I2C pins configured explicitly as:

```text
SDA = GPIO21
SCL = GPIO22
```

The firmware performs startup gyro calibration while the rover must remain still.

## 3.5 Required software libraries

### DevKit

The sketch uses:

- ESP32 Arduino WiFi
- AsyncTCP
- ESPAsyncWebServer
- ArduinoJson
- Wire
- Adafruit MPU6050
- Adafruit Unified Sensor
- ESP-IDF Task Watchdog API bundled with the ESP32 environment

The source targets the Arduino-ESP32 Core 3.x LEDC API.

### Camera

The camera sketch uses ESP32 WiFi/camera/http-server support.

### Laptop

`Laptop_AI/requirements.txt` contains:

```text
opencv-python
ultralytics
websocket-client
```

## 3.6 Flashing order

1. Install Arduino IDE/PlatformIO with an ESP32 board package matching the source API expectations.
2. Install the DevKit libraries listed above.
3. Open `DevKit_Rover/DevKit_Rover.ino`.
4. Select the correct ESP32 DevKit V1 board/port.
5. Compile and upload.
6. Open Serial Monitor and verify the AP address and MPU6050 startup result.
7. Open `ESP32_CAM/CameraWebServer0.ino`.
8. Select the AI-Thinker ESP32-CAM board definition appropriate to your camera module.
9. Compile and upload.
10. Confirm the camera joins the DevKit AP and prints `192.168.4.2` by default.
11. Only after the network/camera tests pass, connect the laptop.

## 3.7 First power-up rule

For the first powered test, keep the rover lifted or remove wheel load. Verify zero motor output at boot, after WebSocket disconnect, after AI-link loss and after any safety fault.

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
