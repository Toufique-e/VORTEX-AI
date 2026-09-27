# Camera UART Housekeeping Protocol

The UART is optional. It carries only lightweight housekeeping commands between the DevKit and ESP32-CAM.

It is not used for video or AI data.

## Wiring

```text
DevKit TX2 GPIO17 -> CAM RX2 GPIO13
DevKit RX2 GPIO16 <- CAM TX2 GPIO14
GND                -> GND
```

Baud:

```text
115200 8N1
```

## Commands

### Flash

```text
FLASH_ON
FLASH_OFF
```

### Frame size

```text
FRAME_QQVGA
FRAME_QVGA
FRAME_CIF
FRAME_VGA
FRAME_SVGA
FRAME_XGA
FRAME_HD
FRAME_SXGA
FRAME_UXGA
```

### JPEG quality

```text
QUALITY_10 ... QUALITY_63
```

The camera accepts values from 10 through 63.

### Image tuning

```text
BRIGHTNESS_-2 ... BRIGHTNESS_2
CONTRAST_-2 ... CONTRAST_2
SATURATION_-2 ... SATURATION_2
```

### Image orientation

```text
HFLIP_0 / HFLIP_1
VFLIP_0 / VFLIP_1
```

### Auto controls

```text
AWB_0 / AWB_1
AGC_0 / AGC_1
AEC_0 / AEC_1
```

## ACK format

Successful commands return:

```text
ACK:<command>
```

Errors return values such as:

```text
ERR:NO_SENSOR
ERR:BAD_FRAME
ERR:BAD_QUALITY
ERR:UNKNOWN_COMMAND
```

## Important

If UART is unplugged, the system can still stream camera video and run rover control. Only remote camera housekeeping is lost.
