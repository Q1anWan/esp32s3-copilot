# Copilot MQTT Protocol

Topic layout (default):
- Command: `copilot/<device_id>/cmd`
- Status:  `copilot/<device_id>/status`

`device_id` comes from `CONFIG_COPILOT_DEVICE_ID`. If empty, a MAC-based ID is used.

## Commands (JSON)

### Expression
```json
{"type":"emotion","name":"happy","duration_ms":420,"prelight_ms":500,"sound":"chime"}
```
`type` can be `"emotion"` or `"expression"` (both accepted).
Fields:
- `name`: `neutral|happy|sad|angry|surprised|sleepy|dizzy`
- `id`: numeric expression index (0..6), optional alternative to `name`
- `duration_ms`: transition time (default `CONFIG_COPILOT_EXPR_TRANSITION_MS`)
- `prelight_ms`: ring prelight time (default `CONFIG_COPILOT_PRELIGHT_MS`)
- `sound`: optional sound id or `true` to use a default beep

### Motion
```json
{"type":"motion","ax":0.3,"ay":-0.2,"yaw":12.0,"speed":15.0}
```
Fields:
- `ax`: forward/back acceleration (g, roughly -1..1)
- `ay`: lateral acceleration (g, roughly -1..1)
- `yaw`: steering yaw (deg)
- `speed`: optional, reserved for future use (accepted but currently unused)

Quaternion mode (for external IMU):
```json
{"type":"motion","qw":1.0,"qx":0.0,"qy":0.0,"qz":0.0}
```

### Sound
```json
{"type":"sound","id":"beep_short","prelight_ms":350}
```
IDs: `beep_short|beep_long|chime|tap`

### Ring
```json
{"type":"ring","on":true}
```

### Calibrate (IMU)
```json
{"type":"calibrate","target":"gyro"}
```
Starts gyroscope zero-bias calibration. The device must be kept **stationary** for approximately 3 seconds during calibration.

Fields:
- `target`: calibration target, currently only `"gyro"` is supported

The calibration result is automatically saved to NVS (flash) and will be loaded on next boot.

### Status Query
```json
{"type":"status","query":"imu"}
```
Queries device status. Results are logged to the device console (not sent via MQTT).
Note: status logging requires `CONFIG_COPILOT_LOG_APP=y`.

Fields:
- `query`: `"imu"` or `"all"`

Response (logged on device):
```
IMU status: ready=1 calibrating=0 bias=2.35 dps
```

## Demo Script

Use the Python script for cross-platform testing (Windows/Linux/macOS):

```bash
python tools/mqtt_demo.py
```

Requires `paho-mqtt`:

```bash
python -m pip install paho-mqtt
```

You can also use the Bash script on Linux/macOS:

```bash
# Default (localhost broker)
./tools/mqtt_demo.sh

# Custom broker
./tools/mqtt_demo.sh -h 192.168.1.100

# With custom device ID
./tools/mqtt_demo.sh -h 192.168.1.100 -d my_device

# Show help
./tools/mqtt_demo.sh --help
```

Environment variables:
- `MQTT_HOST`: broker hostname
- `MQTT_PORT`: broker port (default: 1883)
- `MQTT_PREFIX`: topic prefix (default: `copilot`)
- `MQTT_DEVICE`: device ID (default: `s3_copilot`)
- `MQTT_USER`: MQTT username (optional)
- `MQTT_PASS`: MQTT password (optional)

## MQTT Server (Local Broker)

Use the Python launcher to run Mosquitto on Windows/Linux/macOS:

```bash
python tools/mqtt_server.py
```

Options:
- `-p/--port`: broker port (default: 1883)
- `-c/--conf`: path to a custom mosquitto.conf
- `--mosquitto`: explicit path to the mosquitto binary

Mosquitto installation:
- Ubuntu/Debian: `sudo apt install mosquitto`
- Windows: https://mosquitto.org/download/
