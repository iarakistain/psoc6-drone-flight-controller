# psoc6-drone-flight-controller

AHRS Drone Flight Controller for CY8CKIT-062S2-AI with BMI270, BMM350, DPS368 sensors and WebSerial visualizer.

## Files
- `flight_controller.ino` – complete flight controller sketch (100Hz sensor+AHRS JSON stream).
- `web/index.html` – WebSerial visualizer (3D attitude, compass, altitude graph, telemetry).

## Arduino libraries
Install from Library Manager:
- `SparkFun BMI270 Arduino Library` by SparkFun, version `1.0.3` (header: `SparkFun_BMI270_Arduino_Library.h`)
- `BMM350` by Matteo Cancian (`BMM350-Magnetometer` repo), version `1.3.0` (header: `BMM350.h`)
- `XENSIV Digital Pressure Sensor` by Infineon, version `1.0.3` (header: `Dps3xx.h`)

## Board package
Install the Infineon PSoC 6 Arduino board package that includes `CY8CKIT-062S2-AI` in Arduino IDE Boards Manager before compiling `flight_controller.ino`.

## Serial JSON protocol
Each line is a JSON object:
- `{"type":"sensor","data":{...},"timestamp":ms}`
- `{"type":"ahrs","data":{...},"timestamp":ms}`
- `{"type":"status","data":{...},"timestamp":ms}`

`sensor.data.baro` includes both `altitudeM` (standard pressure altitude) and `compensatedAltitudeM` (temperature-compensated altitude).

## Runtime commands
Send over serial:
- `IDLE:1` / `IDLE:0` – enter/exit low-power idle mode.
- `DEBUG:1` / `DEBUG:0` – enable/disable debug status output.
