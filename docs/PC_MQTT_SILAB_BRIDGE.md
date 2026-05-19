# PC SILAB Bridge

## Target Topology

```text
SILAB --Ethernet LAN A/TCP--> Experiment PC --WiFi LAN B/direct TCP play + MQTT status--> ESP32
```

The experiment PC is the stable protocol boundary:

- Mosquitto MQTT broker listens on `0.0.0.0:1883` and is reachable from WiFi LAN B for status and fallback control.
- `tools/silab_mqtt_bridge_gui.py` listens as the TCP host for SILAB on LAN A.
- The GUI forwards only trigger rising edges to ESP32, using ESP32 direct TCP first and MQTT only as fallback.
- ESP32 still reports status over MQTT, so the GUI can discover the ESP32 IP and monitor TF/audio state.

## SILAB Packet

SILAB sends one text line per sample:

```text
trigger<TAB>scene<TAB>seq<LF>
```

Rules:

- `trigger >= 0.5` is active.
- Playback happens only on a `0 -> 1` rising edge.
- Continuous fixed-rate `trigger=1` packets are ignored until a `trigger=0` packet resets the latch.
- Numeric scene IDs first use GUI aliases, then fall back to the GUI prefix.
  The default alias is `1=boot` so the current TF-card test file
  `/sdcard/audio/boot/001.wav` works immediately. Without an alias, `1` maps
  to `scene001`.

Example:

```text
1	1	1
```

The GUI forwards this play payload over ESP32 direct TCP by default:

```json
{"type":"play","scene":"boot","seq":1,"message_id":"silab_..."}
```

ESP32 then plays:

```text
/sdcard/audio/boot/001.wav
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
6. Wait for ESP32 status; the GUI auto-fills `ESP Host` from `tcp_host` or WiFi IP.
7. Keep `Direct ESP TCP` enabled for low-latency playback.
8. Press `Start TCP Host` and set SILAB `Destination_IP` to the experiment PC Ethernet LAN A IP.
9. Keep `Aliases = 1=boot` for the current TF-card test audio. For formal
   multi-scene audio, copy files such as `/sdcard/audio/scene001/001.wav` and
   clear or edit the alias field.

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

## SILAB Simulator

To test the full bridge without SILAB hardware, start the GUI TCP host and run:

```bash
python3 tools/silab_tcp_simulator.py --host 127.0.0.1 --port 7777 --scene 1 --seq 1 --read-ack --verbose
```

The default simulation sends one pulse:

```text
0<TAB>1<TAB>1
...
1<TAB>1<TAB>1
...
0<TAB>1<TAB>1
```

Useful options:

```bash
# Repeat three trigger cycles at 20 Hz
python3 tools/silab_tcp_simulator.py --host <PC_LAN_A_IP> --cycles 3 --rate-hz 20

# Custom fixed-rate pattern: idle 1s, active 3s, idle 1s
python3 tools/silab_tcp_simulator.py --pattern "0:1,1:3,0:1"

# Check generated packets without connecting
python3 tools/silab_tcp_simulator.py --dry-run --rate-hz 5
```
