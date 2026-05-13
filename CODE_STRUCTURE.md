# Code Structure — ESP32-S3 Copilot (FactoryProgram S3)

## Project Overview

ESP32-S3 firmware for the Waveshare ESP32-S3-Touch-AMOLED-1.75 board. Renders an animated rubber-hose style face on the round AMOLED display, controlled via MQTT, with servo-driven head pan/tilt and WebSocket audio playback.

```
08_FactoryProgram_S3/
├── main/                          # Entry point
│   ├── main.cpp                   # Board init, LVGL setup, self-test
│   ├── CMakeLists.txt
│   └── dark/                      # LVGL dark theme assets (headers)
│
├── components/
│   ├── copilot/                   # Core application component
│   │   ├── copilot_app.cpp/.h     # MQTT command dispatch, action queue
│   │   ├── copilot_ui.cpp/.h      # Face animation engine (rubber-hose head, 50fps)
│   │   ├── copilot_mqtt.cpp/.h    # WiFi + MQTT connect, subscribe cmd topic
│   │   ├── copilot_audio.cpp/.h   # Tone generator (beeps, chimes)
│   │   ├── copilot_servo.cpp/.h   # Dual servo driver (LEDC PWM, GPIO17/18)
│   │   ├── copilot_axp2101.cpp/.h # AXP2101 PMIC battery monitor (I2C)
│   │   ├── copilot_imu.cpp/.h     # QMI8658 IMU driver
│   │   ├── copilot_perf.cpp/.h    # Frame timer / perf counter
│   │   ├── copilot_voice_ui.cpp/.h# Voice-state → mouth animation bridge
│   │   ├── copilot_face_data.cpp/.# Face geometry precalc
│   │   ├── CMakeLists.txt
│   │   └── Kconfig
│   │
│   ├── copilot_voice/             # WebSocket audio streaming
│   │   ├── copilot_voice.cpp/.h    # Streaming session mgmt, WS→speaker path
│   │   ├── copilot_ws_client.cpp/.h# WebSocket client (connect to dashboard)
│   │   ├── copilot_audio_out.cpp/.h# Shared audio out (ES8311, I2S DMA, ring buffer)
│   │   ├── include/copilot_voice.h
│   │   ├── CMakeLists.txt
│   │   └── Kconfig
│   │
│   ├── esp32_s3_touch_amoled_1_75/# BSP: display, I2S, codecs (ES8311/ES7210), GPIOs
│   ├── bsp_extra/                  # BSP extras (boot animation, audio init helpers)
│   ├── XPowersLib/                 # Power management IC library (AXP2101)
│   └── espressif__esp_lvgl_port/   # LVGL port to ESP (vendored)
│
├── tools/
│   ├── copilot_dashboard.py       # GUI: servo control, battery monitor, audio push
│   ├── copilot_ctl.py             # Unified CLI controller
│   ├── servo_cli.py               # Servo calibration/test CLI
│   ├── mqtt_demo.py               # Interactive MQTT demo client
│   ├── mqtt_server.py / .sh       # Local MQTT broker launcher
│   ├── mosquitto.conf             # MQTT broker config
│   ├── host_ip.py                 # Host IP detection for bench testing
│   ├── host_loopback_test.py      # Closed-loop verification (WS + MQTT)
│   ├── test_tts_client.py         # TTS test client
│   ├── test_asr_client.py         # ASR test client
│   ├── test_unidirectional_tts.py # Unidirectional TTS test
│   ├── test_tone.wav              # Test audio file (16kHz mono)
│   ├── voice_backend/             # Python voice backend (TTS/ASR server)
│   └── mqtt_protocol.md           # MQTT protocol reference
│
├── docs/
│   ├── ENGINEER_HANDOFF.md        # Handoff doc (config, pinout, servo setup)
│   ├── GPIO_PINOUT.md             # GPIO pinout reference
│   ├── GUI.md                     # GUI architecture
│   └── VOICE_ARCHITECTURE.md      # Voice system architecture
│
├── partitions.csv                 # Flash partition table (factory 9M + storage 4M)
├── sdkconfig                      # Active SDK configuration
├── sdkconfig.defaults             # Default config overrides
├── CMakeLists.txt                 # Root CMake (components: main copilot copilot_voice ...)
├── README.md
└── CODE_STRUCTURE.md              # This file
```

## Key Data Paths

### MQTT Command → Servo
```
Dashboard/MQTT → copilot/s3_copilot/cmd
  → copilot_app::copilot_handle_payload() → copilot_servo_set_target()
  → servo_task (20ms) → angle_to_pulse() → ledc_set_duty() → GPIO17/18 PWM
```

### MQTT Command → Face Animation
```
MQTT → copilot/s3_copilot/cmd
  → copilot_app → copilot_ui_set_expression() / copilot_ui_set_motion()
  → anim_timer (16ms) → copilot_apply_head() → LVGL objects
```

### Audio Playback (Dashboard → Speaker)
```
Dashboard AudioServer (ws://host:8080/audio/stream)
  → WebSocket binary frames (16kHz mono s16le, 20ms/frame)
  → ESP32 copilot_ws_client → ws_audio_callback()
  → copilot_audio_out_write() → mono→stereo → ring buffer
  → I2S DMA → ES8311 codec → speaker amp (GPIO BSP_POWER_AMP_IO)
```

### Battery Monitoring
```
AXP2101 (I2C 0x34) → copilot_axp2101_get_status()
  → MQTT status query → copilot/s3_copilot/status
```

## Build

```bash
idf.py build          # Build with current sdkconfig
idf.py -p PORT flash  # Flash to device
```

## Servo Calibration

- 180° servos: pulse 500us=-90°, 1500us=0°, 2500us=+90°
- Default soft limits: pitch ±15°, yaw ±45°
- Mechanical range (±90°) is separate from soft limits
- Set via MQTT: `{"type":"servo","calib":{"pitch":{"soft_limit_min":-15,...}}}`
