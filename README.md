# psoc6-drone-flight-controller

AHRS Drone Flight Controller for CY8CKIT-062S2-AI with BMI270, BMM350, DPS368 sensors and WebSerial visualizer.

## Files
- `flight_controller.ino` – complete flight controller sketch (100Hz sensor+AHRS JSON stream).
- `web/index.html` – WebSerial visualizer (3D attitude, compass, altitude graph, telemetry).

## Arduino libraries
Install from Library Manager:
- `SparkFun BMI270 Arduino Library` (SparkFun)
- `BMM350-Magnetometer` (CancianMatteo)
- `xensiv-dps3xx` (Infineon)

## Serial JSON protocol
Each line is a JSON object:
- `{"type":"sensor","data":{...},"timestamp":ms}`
- `{"type":"ahrs","data":{...},"timestamp":ms}`
- `{"type":"status","data":{...},"timestamp":ms}`

## Runtime commands
Send over serial:
- `IDLE:1` / `IDLE:0` – enter/exit low-power idle mode.
- `DEBUG:1` / `DEBUG:0` – enable/disable debug status output.
