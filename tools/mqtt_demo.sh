#!/usr/bin/env bash
set -euo pipefail

# Default configuration
BROKER_HOST="${MQTT_HOST:-localhost}"
TOPIC_PREFIX="${MQTT_PREFIX:-copilot}"
DEVICE_ID="${MQTT_DEVICE:-s3_copilot}"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    -h|--host)
      BROKER_HOST="$2"
      shift 2
      ;;
    -p|--prefix)
      TOPIC_PREFIX="$2"
      shift 2
      ;;
    -d|--device)
      DEVICE_ID="$2"
      shift 2
      ;;
    --help)
      echo "Usage: $0 [OPTIONS]"
      echo ""
      echo "Options:"
      echo "  -h, --host HOST      MQTT broker host (default: localhost, env: MQTT_HOST)"
      echo "  -p, --prefix PREFIX  Topic prefix (default: copilot, env: MQTT_PREFIX)"
      echo "  -d, --device ID      Device ID (default: s3_copilot, env: MQTT_DEVICE)"
      echo "  --help               Show this help message"
      echo ""
      echo "Environment variables:"
      echo "  MQTT_HOST    MQTT broker hostname"
      echo "  MQTT_PREFIX  MQTT topic prefix"
      echo "  MQTT_DEVICE  Device ID"
      exit 0
      ;;
    *)
      echo "Unknown option: $1"
      exit 1
      ;;
  esac
done

CMD_TOPIC="${TOPIC_PREFIX}/${DEVICE_ID}/cmd"

pub() {
  mosquitto_pub -h "${BROKER_HOST}" -t "${CMD_TOPIC}" -m "$1"
  echo "  -> Sent: $1"
}

show_menu() {
  echo ""
  echo "============================================"
  echo "  Copilot MQTT Demo"
  echo "  Broker: ${BROKER_HOST}"
  echo "  Topic:  ${CMD_TOPIC}"
  echo "============================================"
  echo ""
  echo "Expression Commands:"
  echo "  1) Happy       2) Sad         3) Angry"
  echo "  4) Surprised   5) Sleepy      6) Dizzy"
  echo "  7) Neutral"
  echo ""
  echo "Motion Commands:"
  echo "  m1) Motion: drift right"
  echo "  m2) Motion: drift left"
  echo "  m3) Motion: drift up"
  echo "  m4) Motion: drift down"
  echo "  m0) Motion: reset to center"
  echo ""
  echo "Sound Commands:"
  echo "  s1) beep_short   s2) beep_long"
  echo "  s3) chime        s4) tap"
  echo ""
  echo "Ring Control:"
  echo "  r1) Ring ON      r0) Ring OFF"
  echo ""
  echo "IMU Calibration:"
  echo "  cal) Start gyro calibration (keep device stationary!)"
  echo "  st)  Query IMU status"
  echo ""
  echo "Other:"
  echo "  demo) Run expression demo sequence"
  echo "  q)    Quit"
  echo ""
}

run_demo() {
  echo "Running demo sequence..."
  echo "  [1/6] happy"
  pub '{"type":"emotion","name":"happy","duration_ms":420,"prelight_ms":500,"sound":"chime"}'
  sleep 1.2
  echo "  [2/6] surprised"
  pub '{"type":"emotion","name":"surprised","duration_ms":260,"prelight_ms":400,"sound":"beep_short"}'
  sleep 1.2
  echo "  [3/6] sad"
  pub '{"type":"emotion","name":"sad","duration_ms":780,"prelight_ms":500,"sound":"beep_short"}'
  sleep 1.2
  echo "  [4/6] motion right"
  pub '{"type":"motion","ax":0.35,"ay":0.0,"yaw":8.0}'
  sleep 1.0
  echo "  [5/6] motion left+up"
  pub '{"type":"motion","ax":-0.25,"ay":0.4,"yaw":-12.0}'
  sleep 1.0
  echo "  [6/6] neutral"
  pub '{"type":"emotion","name":"neutral","duration_ms":300,"prelight_ms":0}'
  echo "Demo complete!"
}

# Check if mosquitto_pub is available
if ! command -v mosquitto_pub &> /dev/null; then
  echo "Error: mosquitto_pub not found. Install mosquitto-clients:"
  echo "  Ubuntu/Debian: sudo apt install mosquitto-clients"
  echo "  macOS: brew install mosquitto"
  exit 1
fi

# Interactive menu loop
while true; do
  show_menu
  read -rp "Enter command: " cmd

  case $cmd in
    # Expressions
    1)
      echo "Sending: happy"
      pub '{"type":"emotion","name":"happy","duration_ms":420,"prelight_ms":500,"sound":"chime"}'
      ;;
    2)
      echo "Sending: sad"
      pub '{"type":"emotion","name":"sad","duration_ms":600,"prelight_ms":500,"sound":"beep_short"}'
      ;;
    3)
      echo "Sending: angry"
      pub '{"type":"emotion","name":"angry","duration_ms":500,"prelight_ms":400}'
      ;;
    4)
      echo "Sending: surprised"
      pub '{"type":"emotion","name":"surprised","duration_ms":300,"prelight_ms":400,"sound":"beep_short"}'
      ;;
    5)
      echo "Sending: sleepy"
      pub '{"type":"emotion","name":"sleepy","duration_ms":800,"prelight_ms":500}'
      ;;
    6)
      echo "Sending: dizzy"
      pub '{"type":"emotion","name":"dizzy","duration_ms":600,"prelight_ms":400}'
      ;;
    7)
      echo "Sending: neutral"
      pub '{"type":"emotion","name":"neutral","duration_ms":300,"prelight_ms":0}'
      ;;

    # Motion
    m1)
      echo "Sending: motion drift right"
      pub '{"type":"motion","ax":0.5,"ay":0.0,"yaw":0.0}'
      ;;
    m2)
      echo "Sending: motion drift left"
      pub '{"type":"motion","ax":-0.5,"ay":0.0,"yaw":0.0}'
      ;;
    m3)
      echo "Sending: motion drift up"
      pub '{"type":"motion","ax":0.0,"ay":0.5,"yaw":0.0}'
      ;;
    m4)
      echo "Sending: motion drift down"
      pub '{"type":"motion","ax":0.0,"ay":-0.5,"yaw":0.0}'
      ;;
    m0)
      echo "Sending: motion reset"
      pub '{"type":"motion","ax":0.0,"ay":0.0,"yaw":0.0}'
      ;;

    # Sounds
    s1)
      echo "Sending: beep_short"
      pub '{"type":"sound","id":"beep_short","prelight_ms":300}'
      ;;
    s2)
      echo "Sending: beep_long"
      pub '{"type":"sound","id":"beep_long","prelight_ms":300}'
      ;;
    s3)
      echo "Sending: chime"
      pub '{"type":"sound","id":"chime","prelight_ms":300}'
      ;;
    s4)
      echo "Sending: tap"
      pub '{"type":"sound","id":"tap","prelight_ms":300}'
      ;;

    # Ring
    r1)
      echo "Sending: ring ON"
      pub '{"type":"ring","on":true}'
      ;;
    r0)
      echo "Sending: ring OFF"
      pub '{"type":"ring","on":false}'
      ;;

    # Calibration
    cal)
      echo "Starting gyro calibration..."
      echo "IMPORTANT: Keep the device STATIONARY for ~3 seconds!"
      pub '{"type":"calibrate","target":"gyro"}'
      ;;
    st)
      echo "Querying IMU status..."
      pub '{"type":"status","query":"imu"}'
      ;;

    # Demo
    demo)
      run_demo
      ;;

    # Quit
    q|Q|quit|exit)
      echo "Goodbye!"
      exit 0
      ;;

    # Custom JSON
    *)
      if [[ $cmd == "{"* ]]; then
        echo "Sending custom JSON..."
        pub "$cmd"
      else
        echo "Unknown command: $cmd"
      fi
      ;;
  esac
done
