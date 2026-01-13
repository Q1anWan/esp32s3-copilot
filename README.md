# ESP32-S3 Copilot

A cute animated face companion for ESP32-S3 with AMOLED display, featuring smooth expressions, motion response, and MQTT control.

## Hardware

- **Board**: Waveshare ESP32-S3-Touch-AMOLED-1.75 (466x466 round display)
- **MCU**: ESP32-S3 with PSRAM
- **Display**: 1.75" AMOLED with CST9217 touch controller
- **Audio**: ES8311 codec for sound playback

## Features

- **Animated Face**: Eyes and mouth rendered with LVGL arcs
- **7 Expressions**: Neutral, Happy, Sad, Angry, Surprised, Sleepy, Dizzy
- **Smooth Animations**:
  - Ease-in-out transitions between expressions
  - Natural blinking with random intervals
  - Breathing animation for liveliness
- **Motion Response**: Face shifts and tilts based on accelerometer input
- **Ring Indicator**: Animated ring for notifications
- **MQTT Control**: Remote expression/sound control via JSON commands
- **Audio Playback**: Beep and chime sounds via I2S

## Build

Requires ESP-IDF v5.5+

```bash
# Set up ESP-IDF environment
source ~/esp/v5.5.2/esp-idf/export.sh

# Build
idf.py build

# Flash
idf.py flash monitor
```

## Configuration

Use `idf.py menuconfig` to configure:

- **Copilot**: WiFi credentials, MQTT broker, core affinity, animation parameters
- **Performance Profiling**: Enable FPS/RTOS/heap monitoring (disabled by default)

## MQTT Commands

Subscribe to `copilot/<device_id>/cmd` and send JSON:

```json
// Change expression
{"type":"emotion", "name":"happy", "duration_ms":400, "prelight_ms":500, "sound":"chime"}

// Motion update (Q8.8 fixed-point values)
{"type":"motion", "ax":0.3, "ay":0.0, "yaw":10.0, "speed":5.0}

// Play sound
{"type":"sound", "id":"beep_short", "prelight_ms":300}

// Toggle ring
{"type":"ring", "on":true}
```

## Project Structure

```
├── main/                   # Application entry point
├── components/
│   ├── copilot/           # Main application logic
│   │   ├── copilot_app    # MQTT handling, action queue
│   │   ├── copilot_ui     # LVGL face rendering, animations
│   │   ├── copilot_audio  # Sound playback
│   │   ├── copilot_mqtt   # WiFi/MQTT connection
│   │   ├── copilot_perf   # Performance profiling
│   │   └── copilot_face_data # Expression keyframes
│   └── esp32_s3_touch_amoled_1_75/  # BSP for display
├── spiffs/                 # Audio assets
└── sdkconfig.defaults      # Default configuration
```

## License

Apache-2.0
