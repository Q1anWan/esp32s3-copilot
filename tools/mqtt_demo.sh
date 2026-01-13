#!/usr/bin/env bash
set -euo pipefail

BROKER_HOST="${1:-localhost}"
TOPIC_PREFIX="${2:-copilot}"
DEVICE_ID="${3:-s3_copilot}"

CMD_TOPIC="${TOPIC_PREFIX}/${DEVICE_ID}/cmd"

pub() {
  mosquitto_pub -h "${BROKER_HOST}" -t "${CMD_TOPIC}" -m "$1"
}

echo "Publishing to ${CMD_TOPIC} via ${BROKER_HOST}"

echo "happy"
pub '{"type":"emotion","name":"happy","duration_ms":420,"prelight_ms":500,"sound":"chime"}'
sleep 0.8
echo "surprised"
pub '{"type":"emotion","name":"surprised","duration_ms":260,"prelight_ms":400,"sound":"beep_short"}'
sleep 0.8
echo "sad"
pub '{"type":"emotion","name":"sad","duration_ms":780,"prelight_ms":500, "sound":"beep_short"}'
sleep 0.8
echo "motion1"
pub '{"type":"motion","ax":0.35,"ay":0.0,"yaw":8.0,"speed":18.0}'
sleep 0.8
echo "motion2"
pub '{"type":"motion","ax":-0.25,"ay":0.4,"yaw":-12.0,"speed":8.0}'
sleep 0.8
echo "beep_long"
pub '{"type":"sound","id":"beep_long","prelight_ms":350}'
sleep 0.8
echo "neutral"
pub '{"type":"emotion","name":"neutral","duration_ms":300,"prelight_ms":0}'
