# Windows SILAB Bridge Deployment Guide

This guide is for non-software operators setting up the experiment PC on
Windows. Follow it in order.

## What Is Ready

The three required programs are ready in this repository:

| Purpose | File | Status |
| --- | --- | --- |
| Visual bridge and ESP32 control panel | `tools/silab_mqtt_bridge_gui.py` | OK. Receives SILAB TCP, controls ESP32 playback by direct TCP, monitors ESP32 by MQTT |
| SILAB simulator | `tools/silab_tcp_simulator.py` | OK. Sends fixed-rate SILAB-like `trigger scene seq` packets |
| SILAB packet probe | `tools/silab_tcp_probe.py` | OK. Receives real SILAB packets and prints parsed fields, trigger edge, and expected audio file |

Windows helper files are in `tools/windows/`.

Important: the probe and the bridge GUI cannot listen on port `7777` at the
same time. Use either the GUI, or the probe.

## Network Topology

```text
SILAB --Ethernet LAN A/TCP 7777--> Windows experiment PC
Windows experiment PC --WiFi LAN B/MQTT 1883 + TCP 7777--> ESP32
```

The Windows PC has two useful IP addresses:

- Ethernet LAN A IP: give this to SILAB as `Destination_IP`.
- WiFi LAN B IP: give this to ESP32 as the MQTT broker IP.

Do not use `127.0.0.1` for ESP32. `127.0.0.1` only means the Windows PC itself.

## Install Python

1. Install Python 3 from https://www.python.org/downloads/windows/.
2. During install, enable `Add python.exe to PATH` if the installer shows it.
3. Open `PowerShell` and check:

   ```powershell
   py -3 --version
   ```

4. In the project folder, double-click:

   ```text
   tools\windows\install_python_deps.bat
   ```

Expected result: it installs `paho-mqtt` and `pyserial` without red error lines.

## Install Mosquitto MQTT Broker

The bridge GUI has a `Start Dev Broker` button, but this is only for developer
tests. For the experiment PC, install Mosquitto as a normal Windows broker.

Official references:

- Mosquitto download page: https://mosquitto.org/download/
- Mosquitto 2.x remote-listener note: https://mosquitto.org/documentation/migrating-to-2-0/

### Install

1. Download the Windows x64 installer from the official Mosquitto download page.
   Use the latest stable Windows x64 installer shown on that page.
2. Run the installer.
3. Keep the default install directory:

   ```text
   C:\Program Files\mosquitto
   ```

4. If the installer asks whether to install the service, enable it.

### Configure LAN Access

Mosquitto 2.x does not automatically accept remote devices when run with the
default local-only mode. ESP32 is a remote device, so use this lab config:

1. Open Notepad as Administrator.
2. Open:

   ```text
   C:\Program Files\mosquitto\mosquitto.conf
   ```

3. Add these lines at the end, or replace the file with the same contents:

   ```conf
   listener 1883 0.0.0.0
   allow_anonymous true
   persistence false
   ```

For convenience, the same template is available here:

```text
tools\windows\mosquitto-copilot.conf
```

Security note: `allow_anonymous true` is acceptable only on an isolated lab LAN.
Do not expose this broker to the Internet.

### Open Windows Firewall

Open PowerShell as Administrator and run:

```powershell
New-NetFirewallRule -DisplayName "Copilot MQTT 1883" -Direction Inbound -Action Allow -Protocol TCP -LocalPort 1883
New-NetFirewallRule -DisplayName "Copilot SILAB TCP 7777" -Direction Inbound -Action Allow -Protocol TCP -LocalPort 7777
```

### Restart Broker

Open PowerShell as Administrator and run:

```powershell
Restart-Service mosquitto
```

If that command says the service does not exist, open Windows `Services`, find
`Mosquitto Broker` or `mosquitto`, and start/restart it there. If no service was
installed, rerun the installer and select the service option.

### Broker Self-Test

Open two PowerShell windows.

Window 1:

```powershell
& "C:\Program Files\mosquitto\mosquitto_sub.exe" -h 127.0.0.1 -t test/copilot
```

Window 2:

```powershell
& "C:\Program Files\mosquitto\mosquitto_pub.exe" -h 127.0.0.1 -t test/copilot -m hello
```

Expected result: Window 1 prints `hello`.

## Find Windows IP Addresses

Open PowerShell:

```powershell
ipconfig
```

Write down:

| Network | Where To Use It | Example |
| --- | --- | --- |
| WiFi IPv4 Address | ESP32 MQTT broker URI | `mqtt://192.168.0.10:1883` |
| Ethernet IPv4 Address | SILAB `Destination_IP` | `192.168.100.10` |

If you are not sure which is which, unplug Ethernet temporarily and run
`ipconfig` again. The disappearing adapter is Ethernet LAN A.

## Start The Bridge GUI

Double-click:

```text
tools\windows\start_bridge_gui.bat
```

In the GUI:

1. In `MQTT Broker`, set:
   - `Host`: `127.0.0.1`
   - `Port`: `1883`
2. Click `Connect`.
3. Do not click `Start Dev Broker` during formal tests.
4. In `ESP32 USB Setup`, set `Broker URI` to the Windows WiFi LAN B IP:

   ```text
   mqtt://<WINDOWS_WIFI_IP>:1883
   ```

   Example:

   ```text
   mqtt://192.168.0.10:1883
   ```

5. Connect ESP32 by USB and click `Save MQTT`.
6. If ESP32 WiFi is not already configured, fill `SSID` and `Pass`, then click
   `Save WiFi`.
7. Wait for ESP32 status. The GUI should show:
   - WiFi connected with an ESP32 IP
   - MQTT connected
   - SD mounted
   - `ESP Host` auto-filled, for example `192.168.0.56`
8. Keep `Direct ESP TCP` checked.
9. Click `Play`. ESP32 should play `/sdcard/audio/boot/001.wav`.
10. Click `Stop`. ESP32 should stop playback and return to neutral face.

## Start SILAB TCP Host

In the same Bridge GUI:

1. In `SILAB TCP Host`, set:
   - `Bind`: `0.0.0.0`
   - `Port`: `7777`
   - `Threshold`: `0.5`
   - `Aliases`: `1=boot`
2. Optional: enable `Send TCP ACK` if SILAB can read ACK lines.
3. Click `Start TCP Host`.
4. In SILAB, set:
   - `Destination_IP`: Windows Ethernet LAN A IP
   - `Destination_Port`: `7777`
5. SILAB packet format:

   ```text
   trigger<TAB>scene<TAB>seq<LF>
   ```

Example:

```text
1	1	1
```

With default alias `1=boot`, this plays:

```text
/sdcard/audio/boot/001.wav
```

Trigger rule:

- `trigger >= 0.5` means active.
- Playback happens once on the `0 -> 1` rising edge.
- Repeated fixed-rate `1` packets do not repeatedly play.
- Trigger must return to `0` before the next playback can happen.

## Use The SILAB Simulator

Use this when SILAB hardware is not connected, or before a formal run.

1. Start the Bridge GUI.
2. Click `Connect` for MQTT.
3. Click `Start TCP Host`.
4. Double-click:

   ```text
   tools\windows\start_silab_simulator.bat
   ```

5. For `Bridge PC IP`, press Enter to use `127.0.0.1` if the simulator is on
   the same Windows PC.
6. For `Bridge TCP port`, press Enter to use `7777`.

Expected GUI log:

```text
[silab] connected ...
[silab] #... play trigger=1 scene=boot seq=1
[direct] play boot/001 sent ...
[esp32] screen speaking ...
```

## Use The SILAB Packet Probe

Use this when you want to verify the real SILAB output format. The probe only
receives and prints packets. It does not control ESP32.

1. Stop the Bridge GUI TCP Host, or close the Bridge GUI.
2. Double-click:

   ```text
   tools\windows\start_silab_probe.bat
   ```

3. In SILAB, set `Destination_IP` to the Windows Ethernet LAN A IP and
   `Destination_Port` to `7777`.
4. Run the SILAB scenario.

Expected probe output:

```text
format: OK
parsed: trigger=1.0 scene=1 seq=1
bridge_audio_id: scene=boot seq=1
trigger_active=True rising_edge=True
would_play: /sdcard/audio/boot/001.wav
```

The probe also appends machine-readable logs to:

```text
silab_probe_log.jsonl
```

## Normal Experiment Startup Checklist

1. Windows connected to Ethernet LAN A and WiFi LAN B.
2. Mosquitto service running.
3. Windows firewall allows TCP `1883` and `7777`.
4. ESP32 powered on, TF card inserted.
5. Bridge GUI started.
6. GUI MQTT connected to `127.0.0.1:1883`.
7. ESP32 status shows MQTT connected and SD mounted.
8. `ESP Host` is filled and `Direct ESP TCP` is checked.
9. Manual `Play` and `Stop` work.
10. GUI `Start TCP Host` is active.
11. SILAB sends to Windows Ethernet LAN A IP, port `7777`.

## Troubleshooting

### ESP32 MQTT stays disconnected

Check:

- ESP32 Broker URI is `mqtt://<WINDOWS_WIFI_IP>:1883`, not `127.0.0.1`.
- Mosquitto service is running.
- Windows firewall allows TCP `1883`.
- ESP32 and Windows WiFi are on the same LAN B.

### GUI `Play` works but SILAB does not trigger

Check:

- GUI `Start TCP Host` is active.
- SILAB `Destination_IP` is the Windows Ethernet LAN A IP.
- Windows firewall allows TCP `7777`.
- The probe is not running at the same time as the GUI TCP Host.

### Probe cannot start

Port `7777` is already in use. Close the Bridge GUI TCP Host or any other
program using port `7777`.

PowerShell check:

```powershell
netstat -ano | findstr :7777
```

### Manual `Play` has delay or does not play

Check:

- `Direct ESP TCP` is checked.
- `ESP Host` is filled, for example `192.168.0.56`.
- ESP32 status has `tcp_host`, for example `192.168.0.56:7777`.
- TF card status is `mounted`.
- Current test file exists on TF card:

  ```text
  /sdcard/audio/boot/001.wav
  ```

### `Stop` does not stop playback

Check the ESP32 firmware version. It must include commit:

```text
5b317fe Add GUI audio stop control
```

Then restart the Bridge GUI so the new `Stop` button is visible.

## File Summary

| File | For Operator |
| --- | --- |
| `tools\windows\install_python_deps.bat` | Install Python packages |
| `tools\windows\start_bridge_gui.bat` | Start visual bridge |
| `tools\windows\start_silab_simulator.bat` | Simulate SILAB trigger |
| `tools\windows\start_silab_probe.bat` | Inspect real SILAB packets |
| `tools\windows\mosquitto-copilot.conf` | Minimal MQTT broker config |
| `tools\windows\start_mosquitto_console.bat` | Developer fallback broker console |

