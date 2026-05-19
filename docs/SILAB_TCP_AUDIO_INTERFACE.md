# SILAB TCP Audio Interface

The preferred production topology makes the experiment PC the TCP host/server.
SILAB connects to the PC bridge GUI, and the PC forwards accepted trigger
events to ESP32 over MQTT. This avoids routing problems when SILAB and ESP32
are on different LANs.

SILAB sends one newline-delimited text packet per sample:

```text
trigger<TAB>scene<TAB>seq<LF>
```

Example:

```text
1	1	1
```

Field meaning:

| Field | Meaning |
| --- | --- |
| `trigger` | `1` while the scenario condition is active, `0` otherwise |
| `scene` | Audio scene ID. The PC bridge can alias numeric scenes; default `1=boot` for the current TF-card test audio. Without an alias, numeric scenes map to `sceneNNN`, so `1` becomes `scene001` |
| `seq` | Audio sequence ID, 1..65535 |

The PC bridge listens on TCP port `7777` by default. The ESP32 direct TCP host
still exists for same-LAN debugging, but the PC MQTT bridge is the normal path.

The firmware treats `trigger >= 0.5` as active. Playback is edge-triggered, so
fixed-rate continuous `trigger=1` traffic does not repeatedly queue audio. The
trigger must return to `0` before the next `1` can trigger playback again.

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

3. Confirm the validator prints `format: OK` and `rising_edge=True`
   when the scenario trigger changes from 0 to 1.

4. Change `Destination_IP` to the experiment PC Ethernet LAN A IP and keep
   `Destination_Port = 7777`.
