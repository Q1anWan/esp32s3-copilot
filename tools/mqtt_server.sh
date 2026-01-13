#!/usr/bin/env bash
set -euo pipefail

PORT="${1:-1883}"
BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONF_FILE="${BASE_DIR}/mosquitto.conf"

if ! command -v mosquitto >/dev/null 2>&1; then
  echo "mosquitto not found. Install it with your package manager." >&2
  exit 1
fi

mkdir -p /tmp/mosquitto_copilot

echo "Starting mosquitto on port ${PORT}"
mosquitto -v -p "${PORT}" -c "${CONF_FILE}"
