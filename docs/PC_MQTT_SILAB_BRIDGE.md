# PC ESP32 Bridge

## Target Topology

```text
SILAB / vehicle state source
        |
        v
Python logic decision program --TCP 7777--> Experiment PC Bridge GUI
                                             |
                                             +--direct TCP play--> ESP32
                                             +--MQTT status/fallback--> ESP32
```

The formal Bridge no longer communicates with HRT. If an experiment still needs
HRT records, send them from the logic decision program as a separate optional
link. Do not route HRT traffic through the ESP32 Bridge.

## Logic Packet

The logic decision program sends one UTF-8 text line per sample:

```text
TRIGGER;TIMESTAMP;SCENE;SEQ;SPEED<LF>
```

Rules:

- `TRIGGER >= 0.5` is active.
- Playback happens only on a `0 -> 1` rising edge.
- Continuous fixed-rate `TRIGGER=1` packets are ignored until a `TRIGGER=0`
  packet resets the latch.
- `TIMESTAMP` is a complete Unix timestamp in seconds, integer or decimal.
- `SCENE` is the TF-card audio folder.
- `SEQ` is the file stem. Numeric `1` maps to `001.wav` on ESP32; text IDs map
  directly, for example `common/left_rear_vehicle_merge.wav`.
- `SPEED` is displayed for observation/debug only.

Example:

```text
1;1779452836.933;common;left_rear_vehicle_merge;1.2
```

The GUI forwards this play payload to ESP32:

```json
{"type":"play","scene":"common","seq":"left_rear_vehicle_merge","message_id":"logic_..."}
```

ESP32 then plays:

```text
/sdcard/audio/common/left_rear_vehicle_merge.wav
```

## Run On Experiment PC

Install dependencies:

```bash
python -m pip install paho-mqtt pyserial
```

Start the GUI:

```bash
python3 tools/silab_mqtt_bridge_gui.py
```

In the GUI:

1. Use an external Mosquitto service for formal tests. The GUI `Start Dev Broker`
   button is only for local developer checks.
2. Set the broker host/port used by the GUI. `127.0.0.1:1883` is fine when the
   broker runs on the experiment PC.
3. Set `ESP32 USB Setup / Broker URI` to the experiment PC WiFi LAN IP, for
   example `mqtt://192.168.0.10:1883`.
4. Press `Save MQTT` with ESP32 connected over USB.
5. Press `Connect` in the MQTT panel.
6. Wait for ESP32 status; the GUI auto-fills `ESP Host` from `tcp_host` or WiFi
   IP.
7. Keep `Direct ESP TCP` enabled for low-latency playback.
8. In `Logic TCP Host`, set `Bind = 0.0.0.0`, `Port = 7777`, then press
   `Start TCP Host`.
9. Point the Python logic decision program to the experiment PC IP and port
   `7777`.

## ESP32 MQTT Topics

Defaults:

```text
command: copilot/s3_copilot/cmd
status:  copilot/s3_copilot/status
```

Manual status query:

```json
{"type":"status","query":"status"}
```

Manual playback:

```json
{"type":"play","scene":"common","seq":"left_rear_vehicle_merge"}
```

Manual stop:

```json
{"type":"audio","action":"stop"}
```

## USB Quick Configuration

The broker URI can also be set without the GUI:

```bash
python3 tools/copilot_usb_config.py --port /dev/ttyACM2 mqtt --broker mqtt://192.168.0.10:1883
```

WiFi configuration remains:

```bash
python3 tools/copilot_usb_config.py --port /dev/ttyACM2 wifi --ssid YOUR_WIFI --password YOUR_PASS
```

## Packet Probe

Before connecting the real logic program to the GUI, validate the packet format:

```bash
python3 tools/silab_tcp_probe.py --host 0.0.0.0 --port 7777
```

The probe should print `format: OK` and `rising_edge=True` when the trigger
changes from `0` to `1`.

## Simulator

To test the full bridge without SILAB hardware, start the GUI TCP host and run:

```bash
python3 tools/silab_tcp_simulator.py --host 127.0.0.1 --port 7777 --scene common --seq left_rear_vehicle_merge --read-ack --verbose
```

Useful options:

```bash
# Repeat three trigger cycles at 20 Hz
python3 tools/silab_tcp_simulator.py --host <PC_IP> --cycles 3 --rate-hz 20

# Custom fixed-rate pattern: idle 1s, active 3s, idle 1s
python3 tools/silab_tcp_simulator.py --pattern "0:1,1:3,0:1"

# Check generated packets without connecting
python3 tools/silab_tcp_simulator.py --dry-run --rate-hz 5
```
