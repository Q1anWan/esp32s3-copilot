# ESP32-S3 Copilot Slave

This project turns a Waveshare ESP32-S3-Touch-AMOLED-1.75 board into a slave
display/audio endpoint. A host can push animation state through MQTT and push
16 kHz PCM audio through WebSocket. The ESP32 renders a rubber-hose style head
on the round AMOLED screen and plays audio through the ES8311 speaker codec.

## Current Role

- ESP32-S3 is the slave device.
- Host controls short animation states with MQTT JSON messages.
- Host pushes streaming audio through the `/audio/stream` WebSocket protocol.
- Voice/TTS backend can also generate audio and forward it to the ESP32.
- ESP32 can still capture microphone audio and send it upstream when full-duplex
  voice mode is enabled.

For handoff details and the last verified bench run, see
`docs/ENGINEER_HANDOFF.md`.

## Hardware

- Board: Waveshare ESP32-S3-Touch-AMOLED-1.75
- Display: 466x466 round AMOLED, CST9217 touch
- Audio output: ES8311 speaker codec
- Microphone input: ES7210 ADC
- IMU: QMI8658

## Firmware Components

- `main/main.cpp`: board/display boot, LVGL setup, optional startup self-test.
- `components/copilot/copilot_app.cpp`: MQTT command parser and action queue.
- `components/copilot/copilot_ui.cpp`: rubber-hose head animation.
- `components/copilot/copilot_audio.cpp`: short generated tones.
- `components/copilot_voice/copilot_voice.cpp`: microphone/TTS streaming.
- `components/copilot_voice/copilot_ws_client.cpp`: ESP32 WebSocket client.
- `components/copilot_voice/copilot_audio_out.cpp`: shared speaker output path.
- `tools/voice_backend/`: Python backend for WebSocket audio, TTS, and ASR.

## Build And Flash

```bash
source /home/qianwan/esp/v5.5.2/esp-idf/export.sh
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

If the existing `build/` directory was created with another CMake generator,
use a separate build directory:

```bash
idf.py -B build_codex build
idf.py -B build_codex -p /dev/ttyACM0 flash
```

## Runtime Configuration

Configure with `idf.py menuconfig`.

Important options:

- `Copilot -> WiFi SSID/Password`
- `Copilot -> MQTT Broker URI`, for example `mqtt://<host-ip>:1883`
- `Copilot -> MQTT Topic Prefix`, default `copilot`
- `Copilot -> Device ID`, default `s3_copilot`
- `Copilot Voice -> Enable voice module`
- `Copilot Voice -> Voice backend server URL`, for example `http://<host-ip>:8080`
- `Copilot Voice -> Voice communication mode`
- `Copilot -> Run one-shot startup animation/audio self-test`, default off
- `Copilot -> Debug Logging`, useful for MQTT/audio/UI verification

The firmware converts `CONFIG_COPILOT_VOICE_SERVER_URL` to WebSocket:

- `http://host:8080` -> `ws://host:8080/audio/stream`
- `https://host:8080` -> `wss://host:8080/audio/stream`

## MQTT Control Interface

Default command topic:

```text
copilot/<device_id>/cmd
```

With default config:

```text
copilot/s3_copilot/cmd
```

All MQTT command payloads are compact JSON.

### Animation State

The animation system currently has two states:

- `idle`: mouth closed, still rubber-hose head
- `speaking`: mouth opens with elastic head wobble

Request:

```json
{"type":"emotion","name":"speaking","duration_ms":2000,"prelight_ms":300,"sound":"chime"}
```

Fields:

- `type`: `"emotion"` or `"expression"`
- `name`: `idle`, `neutral`, `speaking`, `talking`, `mouth_open`, `open_mouth`
- `id`: optional numeric state, `0=idle`, `1=speaking`
- `duration_ms`: for `speaking`, hold time in milliseconds. `0` latches speaking until idle is sent
- `prelight_ms`: show ring before applying the action
- `sound`: optional sound id, or `true` for default beep

Compatibility names:

- `happy`, `sad`, `angry`, `sleepy`, `dizzy` map to `idle`
- `surprised` maps to `speaking`

Examples:

```json
{"type":"emotion","name":"idle","duration_ms":0}
{"type":"emotion","name":"speaking","duration_ms":1500}
{"type":"expression","id":1,"duration_ms":800,"sound":"beep_short"}
```

### Motion

Motion shifts and tilts the head. Direct mode:

```json
{"type":"motion","ax":0.3,"ay":-0.2,"yaw":12.0,"speed":1.0}
```

Fields:

- `ax`: forward/back acceleration in g-like units, roughly `-1.0..1.0`
- `ay`: lateral acceleration in g-like units, roughly `-1.0..1.0`
- `yaw`: yaw angle in degrees
- `speed`: accepted, currently smoothed but not used by the character

Quaternion mode:

```json
{"type":"motion","qw":1.0,"qx":0.0,"qy":0.0,"qz":0.0,"speed":1.0}
```

The firmware converts quaternion to roll/pitch/yaw and then maps it to the same
motion fields.

### Short Sound

Short sounds are generated tones, not audio files.

```json
{"type":"sound","id":"chime","prelight_ms":300}
```

Sound ids:

- `beep_short`: 920 Hz, 180 ms
- `beep_long`: 740 Hz, 420 ms
- `chime`: 1200 Hz, 140 ms
- `tap`: 1400 Hz, 60 ms

Unknown sound ids fall back to a default 880 Hz tone.

### Ring

```json
{"type":"ring","on":true}
{"type":"ring","on":false}
```

### Voice Control

```json
{"type":"voice","action":"start"}
{"type":"voice","action":"stop"}
{"type":"voice","action":"loopback"}
{"type":"voice","volume":80}
{"type":"voice","mic_gain":24}
```

Fields:

- `action=start`: connect to the voice backend and begin WebSocket streaming
- `action=stop`: stop the voice session
- `action=loopback`: toggle local microphone-to-speaker loopback when loopback is enabled
- `volume`: speaker volume `0..100`
- `mic_gain`: microphone gain `0..36` dB

### Calibration

```json
{"type":"calibrate","target":"gyro"}
```

Starts IMU gyro zero-bias calibration. Keep the device stationary during this.

### Status Query

```json
{"type":"status","query":"all"}
{"type":"status","query":"voice"}
{"type":"status","query":"imu"}
```

Status is printed to the ESP-IDF monitor log. It is not published back to MQTT.

## WebSocket Audio Stream

This is the interface to use when the host pushes audio to the ESP32 slave.

ESP32 acts as a WebSocket client. The host must expose:

```text
ws://<host>:<port>/audio/stream
```

The ESP32 connects to the URL derived from `CONFIG_COPILOT_VOICE_SERVER_URL`.

### ESP32 -> Host Start Message

Sent by ESP32 immediately after WebSocket connect:

```json
{"type":"start","device_id":"s3_copilot","sample_rate":16000}
```

Host must reply:

```json
{"type":"started","session_id":"<uuid-or-any-string>"}
```

After this reply, ESP32 enters streaming state.

### Host -> ESP32 Audio Frames

Send binary WebSocket frames:

- Format: raw PCM
- Type: signed 16-bit little-endian
- Sample rate: 16000 Hz
- Channels: mono
- Recommended frame size: 20 ms = 320 samples = 640 bytes

Example binary frame:

```text
640 bytes = 320 int16 samples @ 16 kHz mono
```

When ESP32 receives the first binary audio frame, firmware logs TTS/audio
reception, plays it through `AUDIO_SRC_VOICE`, and switches UI to speaking.
If no audio arrives for about 300 ms, voice state returns to listening and the
UI returns to idle.

### ESP32 -> Host Microphone Frames

When voice mode is full-duplex or TX-only, ESP32 also sends binary PCM frames
to the host:

- Format: raw PCM
- Type: signed 16-bit little-endian
- Sample rate: 16000 Hz
- Channels: mono
- Frame size: 320 samples = 640 bytes

The firmware captures ES7210 stereo mic input and converts it to mono before
sending.

### Stop Message

ESP32 sends this before disconnecting:

```json
{"type":"stop"}
```

Host may also close the WebSocket. ESP32 has auto-reconnect enabled.

## Python Voice Backend APIs

Run:

```bash
cd tools/voice_backend
python main.py config.yaml
```

The backend provides these routes:

- `GET /health`
- `GET /audio/stream`: WebSocket for ESP32 audio streaming
- `GET /api/tts`: WebSocket for external clients to request TTS
- `POST /api/tts`: HTTP alternative for TTS
- `GET /api/asr`: WebSocket subscription for ASR results

### TTS WebSocket

Client sends:

```json
{"text":"你好，开始播放测试音频"}
```

Server replies:

```json
{"status":"ok","text":"你好，开始播放测试音频","sessions":1}
```

Error example:

```json
{"status":"error","message":"No ESP32 device connected"}
```

### TTS HTTP

```bash
curl -X POST http://localhost:8080/api/tts \
  -H 'Content-Type: application/json' \
  -d '{"text":"你好"}'
```

Success response:

```json
{"status":"ok","text":"你好","sessions":1}
```

### ASR WebSocket

Initial message:

```json
{"type":"connected","active_sessions":1,"sessions":[{"session_id":"...","device_id":"s3_copilot"}]}
```

ASR result:

```json
{"type":"partial","text":"你好","session_id":"...","device_id":"s3_copilot"}
{"type":"final","text":"你好世界","session_id":"...","device_id":"s3_copilot"}
```

Client ping:

```json
{"type":"ping"}
```

Server reply:

```json
{"type":"pong"}
```

## Host Push Workflow

For the target use case, run the system like this:

1. Host starts an MQTT broker and a WebSocket audio server.
2. ESP32 connects to WiFi, MQTT broker, and `/audio/stream`.
3. Host sends MQTT state:

```json
{"type":"emotion","name":"speaking","duration_ms":0}
```

4. Host sends raw 16 kHz mono PCM binary frames over WebSocket.
5. When audio ends, host sends:

```json
{"type":"emotion","name":"idle","duration_ms":0}
```

If the audio is sent through the existing voice WebSocket path, the ESP32 also
auto-enters speaking while binary audio frames are arriving, so the explicit
MQTT speaking state is optional.

### Local Bench Test

For bench testing, the host machine runs both the MQTT broker and the
WebSocket audio server. The ESP32 firmware must point to the host machine's
current LAN IP, not `localhost` and not a stale fixed address.

Resolve the host IP first:

```bash
HOST_IP=$(python tools/host_ip.py)
echo "$HOST_IP"
```

If the workstation has multiple NICs, pass the ESP32 IP or the WiFi gateway to
select the correct route:

```bash
HOST_IP=$(python tools/host_ip.py --target 192.168.0.56)
```

Then set these two firmware endpoints in `menuconfig` or `sdkconfig`:

- `CONFIG_COPILOT_MQTT_BROKER_URI="mqtt://$HOST_IP:1883"`
- `CONFIG_COPILOT_VOICE_SERVER_URL="http://$HOST_IP:8080"`

The closed-loop verification in this repo was completed with host IP
`192.168.31.12`, but the actual value must match the current workstation.

Start host services before rebooting the ESP32 so the device can subscribe to
MQTT and connect to `/audio/stream` without racing the test harness.

Start a local MQTT broker:

```bash
tools/mqtt_server.sh 1883
```

Start the minimal host loopback test server:

```bash
python tools/host_loopback_test.py \
  --mqtt-host 127.0.0.1 \
  --announce-host "$HOST_IP" \
  --topic copilot/s3_copilot/cmd
```

The script waits for the ESP32 WebSocket connection, sends the `started`
message, waits briefly for MQTT subscribe stability, publishes `speaking`,
streams 16 kHz PCM, then publishes `idle`.

Monitor the ESP32:

```bash
source /home/qianwan/esp/v5.5.2/esp-idf/export.sh
idf.py -B build_codex_restore -p /dev/ttyACM0 monitor
```

Expected successful loop:

- firmware boot shows the resolved host endpoints:
  - `MQTT broker: mqtt://<host-ip>`
  - `Initialized (server=ws://<host-ip>:8080/audio/stream, ...)`
- WebSocket session starts:
  - `WebSocket connected after ... attempts`
  - `Session started: ...`
  - `Streaming started`
- audio playback arrives and drives the speaking state:
  - `TTS audio received: 320 samples (first frame)`
  - `TTS playback started -> SPEAKING`
  - `TTS playback finished -> LISTENING`
- MQTT animation/sound commands are accepted:
  - `MQTT data topic_len=... payload_len=...`
  - `Queue tone id=chime ...`
  - `Play tone start ...`
  - `Play tone done ...`
- host-side loopback server receives mic uplink after the start handshake:
  - `[ws] post-stream rx binary len=640`

The loopback server is intentionally short-lived. If it exits after a successful
run, the ESP32 will try to auto-reconnect and may print follow-up
`connection reset by peer` or `Error connecting to host` messages. Those
messages are expected after the closed loop has already succeeded.

## Control Tools

Unified controller:

```bash
python tools/copilot_ctl.py --mqtt-host <host-ip> --voice-server ws://<host-ip>:8080
```

MQTT-only demo:

```bash
python tools/mqtt_demo.py -h <host-ip> -d s3_copilot
```

Useful interactive commands:

- `1`: idle
- `2`: speaking for 2 seconds
- `m1/m2/m3/m4/m0`: motion
- `s1/s2/s3/s4`: short tones
- `r1/r0`: ring
- `v1/v0`: voice session start/stop
- `vl`: loopback
- `vs`: voice status
- `demo`: avatar demo
- `vdemo`: voice UI demo

## Local Self-Test

For bench testing without MQTT/WiFi, enable:

```text
Copilot -> Run one-shot startup animation/audio self-test
```

On boot the firmware runs:

- speaking head state for 1800 ms
- `chime` tone

Expected monitor logs include:

```text
Startup self-test: speaking head + chime
Queue tone id=chime freq=1200 duration=140 volume=75
```

Keep this option disabled for normal builds.

## Verification Notes

Typical successful boot log includes:

```text
Audio output initialized successfully
Microphone initialized: 16000 Hz, 2 ch, 16 bit
Voice module initialized successfully
Voice-UI integration initialized
MQTT start
```

When WebSocket TTS/audio arrives:

```text
TTS audio received: 320 samples (first frame)
TTS playback started -> SPEAKING
TTS playback finished -> LISTENING
```

When MQTT sound is received with audio logging enabled:

```text
Queue tone id=chime freq=1200 duration=140 volume=75
```

If WiFi fails with `reason=201`, the device is not reaching the configured SSID
or authentication settings. Local display/audio initialization can still be
verified with the startup self-test.
