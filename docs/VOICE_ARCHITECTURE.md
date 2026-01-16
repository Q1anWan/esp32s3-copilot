# Copilot Voice Architecture

## Overview

This document describes the bidirectional voice communication system between ESP32-S3 and a Python backend server, using WebSocket for real-time audio streaming and Volcano Engine for TTS/ASR services.

## System Architecture

```
+-----------------------------------------------------------------------------------+
|                              PYTHON BACKEND (x86)                                  |
|  +-------------+     +----------------+     +---------------+     +-------------+ |
|  |  aiohttp    |<--->|  WebSocket     |<--->|  Audio        |<--->| Volcano     | |
|  |  Server     |     |  Handler       |     |  Processing   |     | Engine API  | |
|  |  (HTTP)     |     |  (ws_server)   |     |  Queue        |     | (TTS/ASR)   | |
|  +-------------+     +----------------+     +---------------+     +-------------+ |
+--------|--------------------|-----------------------------------------------------|
         | HTTP/WS            | WebSocket (PCM 16kHz mono)                          |
         v                    v                                                      |
+-----------------------------------------------------------------------------------+
|                              ESP32-S3 FIRMWARE                                     |
|  +-------------+     +-------------------+     +------------------+               |
|  |  WebSocket  |<--->|   Voice Module    |<--->|   Audio Pipeline |               |
|  |  Client     |     |   (copilot_voice) |     |   (ES7210+ES8311)|               |
|  +-------------+     +-------------------+     +------------------+               |
+-----------------------------------------------------------------------------------+

Data Flow:
  ESP32 Mic (ES7210) -> I2S DMA -> Stereo-to-Mono -> WebSocket -> Python Server
                                                                       |
                                                                       v
  ESP32 Speaker (ES8311) <- I2S DMA <- audio_out <- WebSocket <- TTS Audio
```

## Components

### ESP32-S3 Side

#### 1. Voice Module (`components/copilot_voice/`)

Main coordinator for voice streaming.

| File | Purpose |
|------|---------|
| `copilot_voice.cpp` | Main module, streaming task, state management |
| `copilot_voice.h` | Public API |
| `copilot_ws_client.cpp` | WebSocket client wrapper |
| `copilot_ws_client.h` | WebSocket client API |
| `copilot_audio_out.cpp` | Unified audio output manager (shared with notifications) |
| `copilot_audio_out.h` | Audio output API |

**Key Configuration (Kconfig):**
- `CONFIG_COPILOT_VOICE_SAMPLE_RATE`: 16000 Hz
- `CONFIG_COPILOT_VOICE_TASK_STACK`: 4096 bytes
- `CONFIG_COPILOT_VOICE_SERVER_URL`: WebSocket server URL

**Audio Frame Format:**
- Sample rate: 16000 Hz
- Channels: 1 (mono, converted from stereo mic input)
- Bit depth: 16-bit signed
- Frame size: 320 samples (20ms)
- Frame bytes: 640 bytes

#### 2. Hardware Codecs

| Codec | Role | I2C Address |
|-------|------|-------------|
| ES7210 | Microphone ADC (stereo input) | 0x40 |
| ES8311 | Speaker DAC (stereo output) | 0x18 |

**Important:** Both codecs share I2S bus. Audio output must be initialized first to set I2S to stereo mode, otherwise ES7210 mic capture fails.

### Python Backend Side (`tools/voice_backend/`)

| File | Purpose |
|------|---------|
| `main.py` | Entry point, server startup |
| `config.yaml` | Configuration (Volcano credentials, server settings) |
| `src/signaling/server.py` | HTTP/WebSocket server setup |
| `src/streaming/ws_server.py` | WebSocket handler, audio session management |
| `src/volcano/tts_client.py` | Volcano Engine TTS (Unidirectional Stream API) |
| `src/volcano/sauc_asr_client.py` | Volcano Engine ASR (SAUC BigModel API) |

## WebSocket Protocol

### Endpoint
```
ws://<server>:8080/audio/stream
```

### Message Types

**Client -> Server:**
1. **Start Session** (JSON):
   ```json
   {"type": "start", "device_id": "esp32_copilot", "sample_rate": 16000}
   ```
2. **Audio Data** (Binary): Raw PCM int16 samples, 640 bytes per frame
3. **Stop Session** (JSON):
   ```json
   {"type": "stop"}
   ```

**Server -> Client:**
1. **Session Started** (JSON):
   ```json
   {"type": "started", "session_id": "uuid"}
   ```
2. **TTS Audio** (Binary): Raw PCM int16 samples
3. **Error** (JSON):
   ```json
   {"type": "error", "message": "..."}
   ```

## SAUC ASR Streaming Notes

The backend uses Volcano SAUC BigModel streaming over WebSocket. Recommended settings
from the official guide:
- Prefer `bidirectional_async` for streaming (server only returns on changes).
- Use 100-200ms audio segments; 200ms is best for bidirectional streaming.
- Keep send interval in the same 100-200ms range.
- `nostream` mode returns results after ~15s or final packet (negative sequence).
- For faster finals, tune `end_window_size` and `force_to_speech_time`.

## WebSocket Stability Notes

The earlier `transport_poll_write(0)` drop has been mitigated by pacing mic sends
to real-time, increasing WS send timeout, and moving audio output task stack to PSRAM.
If drops return, check internal RAM pressure and WiFi quality, then consider lwIP
buffer tuning.

## Configuration Files

### ESP32 sdkconfig relevant settings
```
CONFIG_COPILOT_VOICE_ENABLE=y
CONFIG_COPILOT_VOICE_MODE_FULL_DUPLEX=y
CONFIG_COPILOT_VOICE_SAMPLE_RATE=16000
CONFIG_COPILOT_VOICE_SERVER_URL="http://192.168.31.98:8080"
```

### Python config.yaml
```yaml
server:
  host: "0.0.0.0"
  port: 8080

volcano:
  app_key: "..."
  access_key: "..."
  tts:
    voice_type: "zh_female_cancan_mars_bigtts"
    sample_rate: 16000
  asr:
    mode: "bidirectional_async"
    resource_id: "volc.bigasr.sauc.duration"
    sample_rate: 16000
    segment_duration_ms: 200
    send_interval_ms: 200
    result_type: "single"
    end_window_size: 800
    force_to_speech_time: 1000

logging:
  level: "DEBUG"

debug:
  save_audio: true
  audio_dir: "./audio_recordings"
```

## Debugging Tips

### ESP32 Side
1. Check heap stats in performance report (every 5s)
2. Look for `transport_poll_write` errors
3. Monitor `ws_client` state changes
4. Check DMA memory availability

### Server Side
1. Enable DEBUG logging in config.yaml
2. Check `audio_recordings/` for saved audio files
3. Look for SAUC `logid` and `connect_id` on session start
4. Monitor session create/destroy logs

### Useful Commands

**Run Python server:**
```bash
cd tools/voice_backend
python main.py config.yaml
```

**Monitor ESP32:**
```bash
idf.py monitor
```

**Test TTS via WebSocket:**
```python
import asyncio
import websockets
import json

async def test_tts():
    async with websockets.connect("ws://localhost:8080/api/tts") as ws:
        await ws.send(json.dumps({"text": "你好世界"}))
        response = await ws.recv()
        print(response)

asyncio.run(test_tts())
```

**Subscribe to ASR results:**
```bash
python tools/test_asr_client.py --server ws://localhost:8080 --verbose
```

## Optional Tuning

1. **Increase lwIP TCP buffers** if the network is unstable
2. **Enable WiFi/LWIP PSRAM allocation** to free internal RAM
3. **Tune `end_window_size`/`force_to_speech_time`** for faster finals
4. **Adjust `segment_duration_ms`** between 100-200ms if latency drifts

## Voice-UI Integration

The voice module integrates with the UI system to provide visual feedback during voice interactions.

### Voice-UI Bridge (`components/copilot/copilot_voice_ui.cpp`)

| File | Purpose |
|------|---------|
| `copilot_voice_ui.cpp` | Bridge between voice and UI modules |
| `copilot_voice_ui.h` | Voice-UI API declarations |

**Key APIs:**
- `copilot_voice_ui_init()` - Initialize voice-UI integration (registers state callback)
- `copilot_voice_ui_get_mouth_open()` - Get audio envelope for mouth animation (0-255)
- `copilot_voice_ui_get_state()` - Get current voice state for UI feedback

### Audio Envelope Detection

The audio output module calculates envelope for mouth animation sync:
- Peak detection runs on every audio write (samples every 4th sample for speed)
- Envelope normalized to 0-255 range (12000 peak = 255)
- Automatic decay if no audio for >50ms (smooth mouth closing)

### UI Animations During Voice States

| Voice State | Eye Shape | Mouth | Ring Light |
|-------------|-----------|-------|------------|
| IDLE/READY | Semicircle (180°) | Hidden | Off |
| LISTENING | Semicircle | Hidden | Off |
| SPEAKING | Full ellipse (360°) | Scaling ellipse | On |
| ERROR | Semicircle | Hidden | Off |

**Eye Animation:**
- Not speaking: Eyes display as semicircles (bottom half arc, 180°-360°)
- Speaking: Eyes smoothly transition to full ellipses (0°-360°)
- Transition uses exponential smoothing (α≈0.25) for natural feel

**Mouth Animation:**
- Hidden when not speaking (`mouth_open_smooth < 15`)
- Appears as scaling ellipse during speech
- Size scales 30%-100% of base size based on audio envelope
- Creates "talking" effect synchronized with audio

### Cross-Thread Communication

```
Voice Task ─────> s_voice_state (atomic uint8_t)
                              │
Audio Task ────> s_envelope   │
              (atomic uint8_t)│
                              ▼
              UI Task reads at 30 FPS
```

- Single-byte atomics are naturally thread-safe on ESP32
- No queue overhead needed
- UI polls state every 2 frames (~66ms) to reduce overhead

## Related Documentation

- [ESP-IDF WebSocket Client](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/protocols/esp_websocket_client.html)
- [Volcano Engine TTS API](https://www.volcengine.com/docs/6561/79823)
- [aiohttp WebSocket Server](https://docs.aiohttp.org/en/stable/web_quickstart.html#websockets)
