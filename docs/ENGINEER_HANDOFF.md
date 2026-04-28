# Engineer Handoff

Last verified: 2026-04-28, Asia/Shanghai.

This document is the handoff note for the next engineer. Treat it as the
current operational state of the project. `README.md` contains the full command
and protocol reference; this file records the practical details needed to keep
working without rediscovering the setup.

## Project State

The firmware turns a Waveshare ESP32-S3-Touch-AMOLED-1.75 board into a slave
display/audio endpoint.

- Host controls animation state through MQTT JSON.
- Host pushes speaker audio through WebSocket binary PCM frames.
- ESP32 renders a rubber-hose style head on the round AMOLED display.
- ESP32 plays audio through ES8311 and can upload ES7210 microphone PCM.
- Animation has intentionally been reduced to two public states: `idle` and
  `speaking`.

The target product flow is:

1. Host starts MQTT broker and WebSocket audio server.
2. ESP32 joins WiFi and connects to both host services.
3. Host publishes `speaking` or `idle` to MQTT.
4. Host sends raw 16 kHz mono PCM to ESP32 over `/audio/stream`.
5. ESP32 displays the mouth-open speaking state while audio is arriving and
   plays the PCM stream through the speaker.

## Hardware And Environment

- Board: Waveshare ESP32-S3-Touch-AMOLED-1.75
- Display: 466x466 round AMOLED, LVGL
- Speaker codec: ES8311
- Microphone codec: ES7210
- IMU: QMI8658
- Serial device used in testing: `/dev/ttyACM0`
- ESP-IDF environment used in testing:

```bash
source /home/qianwan/esp/v5.5.2/esp-idf/export.sh
```

The last known working build/flash command was:

```bash
idf.py -B build_codex_restore build flash -p /dev/ttyACM0
```

## Current Firmware Configuration

Current `sdkconfig` values at handoff:

```text
CONFIG_COPILOT_WIFI_SSID="Finnox_AP"
CONFIG_COPILOT_WIFI_PASSWORD="finnox666"
CONFIG_COPILOT_MQTT_BROKER_URI="mqtt://192.168.31.12"
CONFIG_COPILOT_MQTT_TOPIC_PREFIX="copilot"
CONFIG_COPILOT_DEVICE_ID="s3_copilot"
CONFIG_COPILOT_VOICE_ENABLE=y
CONFIG_COPILOT_VOICE_SERVER_URL="http://192.168.31.12:8080"
CONFIG_COPILOT_VOICE_MODE_FULL_DUPLEX=y
CONFIG_COPILOT_VOICE_SAMPLE_RATE=16000
CONFIG_COPILOT_VOICE_TASK_STACK=8192
CONFIG_COPILOT_VOICE_TASK_CORE=1
CONFIG_COPILOT_VOICE_MIC_GAIN=24
CONFIG_COPILOT_VOICE_SPEAKER_VOLUME=80
# CONFIG_COPILOT_STARTUP_SELF_TEST is not set
CONFIG_COPILOT_LOG_AUDIO=y
CONFIG_COPILOT_LOG_MQTT=y
CONFIG_COPILOT_LOG_VOICE=y
```

Important: `192.168.31.12` is the workstation IP from the verified bench run.
Do not hard-code it for a different network. Resolve the current host IP before
flashing:

```bash
HOST_IP=$(python tools/host_ip.py)
echo "$HOST_IP"
```

If the workstation has multiple network interfaces, pass the ESP32 IP or the
WiFi gateway so the route is selected correctly:

```bash
HOST_IP=$(python tools/host_ip.py --target 192.168.0.56)
```

Then update:

```text
CONFIG_COPILOT_MQTT_BROKER_URI="mqtt://$HOST_IP:1883"
CONFIG_COPILOT_VOICE_SERVER_URL="http://$HOST_IP:8080"
```

## Main Files

- `main/main.cpp`: board startup, LVGL, optional one-shot self-test.
- `components/copilot/copilot_ui.cpp`: current rubber-hose head renderer.
- `components/copilot/copilot_app.cpp`: MQTT JSON parsing and action dispatch.
- `components/copilot/copilot_face_data.*`: public state compatibility names.
- `components/copilot/copilot_audio.cpp`: generated short tones and audio logs.
- `components/copilot_voice/copilot_voice.cpp`: voice session and PCM stream.
- `components/copilot_voice/copilot_ws_client.cpp`: WebSocket client wrapper.
- `tools/host_ip.py`: resolves the host LAN IP for firmware endpoints.
- `tools/host_loopback_test.py`: local closed-loop WebSocket plus MQTT test.
- `tools/mqtt_server.sh`: starts a local mosquitto broker.
- `README.md`: canonical protocol reference.

`docs/GUI.md` is an earlier visual design note. The current implementation is
the rubber-hose head in `components/copilot/copilot_ui.cpp`.

## Animation State

Current public states:

- `idle`: mouth closed rubber-hose head.
- `speaking`: mouth open plus elastic wobble/squash-stretch.

Compatibility names are still accepted so older tools do not break:

- `idle`, `neutral` -> `idle`
- `speaking`, `talking`, `mouth_open`, `open_mouth` -> `speaking`
- `happy`, `sad`, `angry`, `sleepy`, `dizzy` -> `idle`
- `surprised` -> `speaking`

The voice subsystem also drives the UI:

- first received WebSocket PCM frame -> voice state `SPEAKING`
- no TTS/audio for about 300 ms -> voice state returns to `LISTENING`

## MQTT Interface

Default command topic:

```text
copilot/s3_copilot/cmd
```

All payloads are compact JSON. Key commands:

```json
{"type":"emotion","name":"speaking","duration_ms":0}
{"type":"emotion","name":"idle","duration_ms":0}
{"type":"expression","id":1,"duration_ms":800,"sound":"beep_short"}
{"type":"sound","id":"chime","prelight_ms":300}
{"type":"motion","ax":0.3,"ay":-0.2,"yaw":12.0,"speed":1.0}
{"type":"ring","on":true}
{"type":"voice","action":"start"}
{"type":"voice","action":"stop"}
{"type":"voice","volume":80}
{"type":"voice","mic_gain":24}
{"type":"status","query":"voice"}
```

Known generated sound ids:

- `beep_short`: 920 Hz, 180 ms
- `beep_long`: 740 Hz, 420 ms
- `chime`: 1200 Hz, 140 ms
- `tap`: 1400 Hz, 60 ms

## WebSocket Audio Interface

ESP32 is the WebSocket client. The host must listen on:

```text
ws://<host-ip>:8080/audio/stream
```

Handshake:

```json
{"type":"start","device_id":"s3_copilot","sample_rate":16000}
```

Host reply:

```json
{"type":"started","session_id":"<any-string>"}
```

Host to ESP32 audio frames:

- raw PCM
- signed 16-bit little-endian
- mono
- 16000 Hz
- recommended frame: 20 ms = 320 samples = 640 bytes

ESP32 to host microphone frames use the same PCM format and are also normally
640 bytes per frame in full-duplex mode.

## Closed-Loop Test Procedure

Use this flow when validating another change.

1. Resolve host IP:

```bash
HOST_IP=$(python tools/host_ip.py)
echo "$HOST_IP"
```

2. Confirm `sdkconfig` points ESP32 to this host IP, rebuild/flash if needed:

```bash
source /home/qianwan/esp/v5.5.2/esp-idf/export.sh
idf.py -B build_codex_restore build flash -p /dev/ttyACM0
```

3. Start local MQTT broker:

```bash
tools/mqtt_server.sh 1883
```

4. Start local WebSocket plus MQTT loopback test:

```bash
python tools/host_loopback_test.py \
  --mqtt-host 127.0.0.1 \
  --announce-host "$HOST_IP" \
  --topic copilot/s3_copilot/cmd
```

5. Monitor ESP32:

```bash
source /home/qianwan/esp/v5.5.2/esp-idf/export.sh
idf.py -B build_codex_restore -p /dev/ttyACM0 monitor
```

The loopback tool waits for the ESP32 WebSocket connection, replies with
`started`, waits briefly for MQTT subscription stability, publishes `speaking`,
streams a 16 kHz sine PCM test tone, and publishes `idle`.

## Verified Test Evidence

The last successful bench run used:

- host IP: `192.168.31.12`
- ESP32 IP: `192.168.0.56`
- WiFi SSID: `Finnox_AP`
- MQTT broker: `mqtt://192.168.31.12:1883`
- WebSocket server: `ws://192.168.31.12:8080/audio/stream`

ESP-IDF monitor showed:

```text
Got IP: 192.168.0.56
MQTT broker: mqtt://192.168.31.12
MQTT cmd topic: copilot/s3_copilot/cmd
Initialized (server=ws://192.168.31.12:8080/audio/stream, device=s3_copilot, rate=16000)
WebSocket connected after 4 attempts
Session started: loopback-1-1530ad12
Streaming started
TTS audio received: 320 samples (first frame)
TTS playback started -> SPEAKING
MQTT data topic_len=22 payload_len=89
Queue tone id=chime freq=1200 duration=140 volume=75
Play tone start freq=1200 duration=140 volume=75
Play tone done freq=1200
MQTT data topic_len=22 payload_len=48
TTS playback finished -> LISTENING
```

The host loopback tool showed:

```text
[ws] client connected from ('192.168.0.56', ...)
[ws] rx text: {"type":"start","device_id":"s3_copilot","sample_rate":16000}
[ws] tx started session_id=loopback-1-1530ad12
[mqtt] copilot/s3_copilot/cmd <- {"type":"emotion","name":"speaking",...}
[ws] streaming 110 PCM frames (2200 ms)
[mqtt] copilot/s3_copilot/cmd <- {"type":"emotion","name":"idle","duration_ms":0}
[ws] post-stream rx binary len=640
```

The mosquitto log showed the ESP32 subscribed before the test publish:

```text
New client connected from 192.168.0.56 ... as s3_copilot
Received SUBSCRIBE from s3_copilot
copilot/s3_copilot/cmd (QoS 1)
Received PUBLISH ... 'copilot/s3_copilot/cmd' ... (89 bytes)
Sending PUBLISH to s3_copilot ...
Received PUBLISH ... 'copilot/s3_copilot/cmd' ... (48 bytes)
Sending PUBLISH to s3_copilot ...
```

## Known Behaviors And Risks

- The WiFi network is a WiFi 7 AP, but ESP32-S3 joined it successfully through
  2.4 GHz b/g/n compatibility in the verified run.
- The one-shot loopback server exits after success. ESP32 auto-reconnect is
  enabled, so later `connection reset by peer` or `Error connecting to host`
  logs are expected after the test server stops.
- `192.168.31.98` is a stale historical address. Do not use it for current
  bench testing.
- Internal RAM can get low during full-duplex audio. Watch `Min free` and
  `DMA-capable` heap in performance logs if audio becomes unstable.
- `CONFIG_COPILOT_STARTUP_SELF_TEST` should stay disabled for normal builds.
  Enable it only to verify local display/audio without WiFi/MQTT.
- Build directories such as `build_codex_restore/` and `build_codex_test/` are
  local test artifacts, not source-of-truth documentation.

## Next Engineering Steps

- Decide whether the host should always send explicit MQTT `speaking/idle`, or
  rely on WebSocket PCM arrival to drive speaking automatically.
- Consider making `tools/host_loopback_test.py` multi-session if repeated
  reconnect testing is needed.
- If the rubber-hose visual style needs further improvement, change
  `components/copilot/copilot_ui.cpp` only after preserving the two-state public
  protocol.
- If deploying beyond the bench network, replace fixed `sdkconfig` endpoints
  with a provisioning flow or generated sdkconfig overlay.
