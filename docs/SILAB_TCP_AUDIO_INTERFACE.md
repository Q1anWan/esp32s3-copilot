# SILAB TCP Audio Interface

The preferred production topology makes the experiment PC the TCP host/server.
SILAB connects to the PC bridge GUI, and the PC forwards accepted trigger
events to ESP32 over direct TCP on the WiFi LAN. MQTT remains the status path
and fallback control channel. This avoids routing problems when SILAB and ESP32
are on different LANs, while keeping playback latency low.

SILAB sends one newline-delimited text packet per sample:

```text
TRIGGER;TIME_LOW;TIME_HIGH;SCENE;SEQ;SPEED<LF>
```

Example:

```text
1;235200;17792;1;1;1.2
```

Field meaning:

| Field | Meaning |
| --- | --- |
| `TRIGGER` | `1` while the scenario condition is active, `0` otherwise |
| `TIME_LOW` | Low 6 digits of Unix time |
| `TIME_HIGH` | Integer/floor of Unix time divided by `100000` |
| `SCENE` | Audio scene ID. The PC bridge can alias numeric scenes; default `1=boot` for the current TF-card test audio. Without an alias, numeric scenes map to `sceneNNN`, so `1` becomes `scene001` |
| `SEQ` | Audio sequence ID, 1..65535 |
| `SPEED` | Speed value, normally `VDyn.v_kmh`, shown in GUI/probe/HRT for observation and test-state judgment |

The bridge reconstructs Unix time as:

```text
timestamp = floor(TIME_HIGH) * 100000 + (floor(TIME_LOW) % 100000)
```

The PC bridge listens on TCP port `7777` by default. The ESP32 direct TCP host
also listens on `7777`; the GUI auto-fills the ESP32 target from MQTT status.

The firmware treats `trigger >= 0.5` as active. Playback is edge-triggered, so
fixed-rate continuous `trigger=1` traffic does not repeatedly queue audio. The
trigger must return to `0` before the next `1` can trigger playback again.

The bridge can also act as an HRT TCP device. Configure the HRT host IP and
port in the GUI. Every valid SILAB sample is forwarded to HRT as a comma
packet terminated by semicolon, matching the reference button sender style:

```text
trigger,timestamp,scene,seq,speed;
```

Example:

```text
1,1779235200,boot,1,1.2;
```

Default PC bridge playback mapping with the shipped test audio:

```text
scene = boot
seq   = 1
file  = /sdcard/audio/boot/001.wav
```

Formal multi-scene mapping without aliases:

```text
scene = scene001
seq   = 1
file  = /sdcard/audio/scene001/001.wav
```

Related files:

- PC bridge GUI: `tools/silab_mqtt_bridge_gui.py`
- SILAB reference design: `tools/silab/NOMIRobotStart_ESP32.inc`
- SILAB time base: `tools/silab/NOMITimeBase.inc`
- SILAB time-base generator: `tools/silab/prepare_nomi_time_base.py`
- PC packet validator: `tools/silab_tcp_probe.py`
- USB GUI with SILAB simulator: `tools/copilot_usb_gui.py`
- PC bridge guide: `docs/PC_MQTT_SILAB_BRIDGE.md`

Debug flow:

1. Run the packet validator on the PC:

   ```bash
   python3 tools/silab_tcp_probe.py --port 7777
   ```

2. Temporarily set `Destination_IP` in SILAB to the PC IP and run the SILAB
   scenario.

3. Confirm the validator prints `format: OK`, a reconstructed `timestamp`,
   `hrt_device_packet`, and `rising_edge=True` when the scenario trigger
   changes from 0 to 1.

4. Change `Destination_IP` to the experiment PC Ethernet LAN A IP and keep
   `Destination_Port = 7777`.

Manual stop command, sent from the GUI or any TCP/MQTT test client:

```json
{"type":"audio","action":"stop"}
```
