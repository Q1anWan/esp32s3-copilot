# PC MQTT SILAB Bridge

## Target Topology

```text
SILAB --Ethernet LAN A/TCP--> Experiment PC --WiFi LAN B/MQTT--> ESP32
```

The experiment PC is the stable protocol boundary:

- Mosquitto MQTT broker listens on `0.0.0.0:1883` and is reachable from WiFi LAN B.
- `tools/silab_mqtt_bridge_gui.py` listens as the TCP host for SILAB on LAN A.
- The GUI forwards only trigger rising edges to ESP32 by MQTT.
- ESP32 only receives normal MQTT JSON commands, so SILAB protocol changes stay on the PC side.

## SILAB Packet

SILAB sends one text line per sample:

```text
trigger<TAB>scene<TAB>seq<LF>
```

Rules:

- `trigger >= 0.5` is active.
- Playback happens only on a `0 -> 1` rising edge.
- Continuous fixed-rate `trigger=1` packets are ignored until a `trigger=0` packet resets the latch.
- Numeric scene IDs map to TF-card folders using the GUI prefix. Default: `1` -> `scene001`.

Example:

```text
1	1	1
```

The GUI forwards this MQTT payload:

```json
{"type":"play","scene":"scene001","seq":1,"message_id":"silab_..."}
```

ESP32 then plays:

```text
/sdcard/audio/scene001/001.wav
```

## Run On Experiment PC

Install dependencies:

```bash
python -m pip install paho-mqtt pyserial
```

Install Mosquitto:

```bash
sudo apt install mosquitto
```

Windows: install Mosquitto from the official installer and add it to `PATH`.

Start the GUI:

```bash
python3 tools/silab_mqtt_bridge_gui.py
```

In the GUI:

1. Press `Start Broker`.
2. Set the broker host/port used by the GUI. `127.0.0.1:1883` is fine for the GUI itself.
3. Set `ESP32 USB Setup / Broker URI` to the experiment PC WiFi LAN B IP, for example `mqtt://192.168.0.10:1883`.
4. Press `Save MQTT` with ESP32 connected over USB.
5. Press `Connect` in the MQTT panel.
6. Press `Start TCP Host` and set SILAB `Destination_IP` to the experiment PC Ethernet LAN A IP.

## ESP32 MQTT Topics

Defaults:

```text
command: copilot/s3_copilot/cmd
status:  copilot/s3_copilot/status
```

Manual status query:

```json
{"type":"status","query":"all"}
```

Manual playback:

```json
{"type":"play","scene":"boot","seq":1}
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

## SILAB Debug Probe

Before connecting SILAB to the GUI, validate the actual TCP packet format on the PC:

```bash
python3 tools/silab_tcp_probe.py --host 0.0.0.0 --port 7777
```

The probe should print `format: OK` and `rising_edge=True` when the trigger changes from `0` to `1`.
