# VESS Calibration Sketch

Standalone ESP32 code for checking the VESS signal on GPIO26 without changing
the Cluster firmware.

## Wiring

```text
ESP32 GPIO26 -> VESS signal input
ESP32 GND    -> VESS GND/common ground
```

Power the VESS module according to its own power requirements. Do not power it
from a weak ESP32 rail if the module needs more current.

## Current Cluster PWM Reference

The current Cluster firmware drives VESS with 50 Hz servo-style PWM:

| Logical throttle | Meaning | Pulse width | Team sound level label |
| --- | --- | --- | --- |
| -1.0 | Reverse throttle | 1000 us | 15 |
| 0.0 | Idle / neutral | 1500 us | 20 |
| +1.0 | Full forward throttle | 2000 us | 30 |

The reverse pulse is provisional for calibration because the production reverse
calibration value has not been confirmed yet.

## Upload

From this folder:

```powershell
$env:PLATFORMIO_CORE_DIR='C:\heven\cluster\.pio-core'
& 'C:\Users\김민철\AppData\Roaming\Python\Python311\Scripts\platformio.exe' run -t upload --upload-port COM4
```

## Serial Commands

Open the serial monitor at 115200 baud.

```text
i     idle, throttle 0.0
f     full forward, throttle +1.0
r     reverse, throttle -1.0
0..9  forward throttle 0.0..0.9
+     increase throttle by 0.05
-     decrease throttle by 0.05
s     slow sweep -1.0 -> 0.0 -> +1.0 -> 0.0
p     print current status
h/?   help
```
