#!/usr/bin/env python3
"""
CLI tool for controlling ESP32-S3 servo motors via MQTT.

Usage:
  python tools/servo_cli.py --pitch 15 --yaw -10          # Set target angles (degrees)
  python tools/servo_cli.py --pitch -30                    # Set pitch only (yaw unchanged)
  python tools/servo_cli.py --calibrate on                 # Enter calibration mode (hold center)
  python tools/servo_cli.py --calibrate off                # Exit calibration mode
  python tools/servo_cli.py --reset                        # Reset calibration to defaults
  python tools/servo_cli.py --status                       # Query current servo state
  python tools/servo_cli.py --calib-pitch-min -60          # Set pitch angle_min
  python tools/servo_cli.py --calib-pitch-max 60           # Set pitch angle_max
  python tools/servo_cli.py --calib-yaw-min -45            # Set yaw angle_min
  python tools/servo_cli.py --calib-yaw-max 45             # Set yaw angle_max
  python tools/servo_cli.py --calib-pitch-center 1500      # Set pitch center pulse (us)
  python tools/servo_cli.py --calib-pitch-pulse-min 600    # Set pitch min pulse (us)
  python tools/servo_cli.py --calib-pitch-pulse-max 2400   # Set pitch max pulse (us)

Defaults:
  --broker   localhost (override with SERVO_MQTT_BROKER env var)
  --topic    copilot/s3_copilot/cmd (override with SERVO_MQTT_TOPIC env var)
  --device   s3_copilot (override with SERVO_DEVICE_ID env var)
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

import paho.mqtt.client as mqtt


def build_payload(args: argparse.Namespace) -> dict | None:
    """Build the JSON payload from CLI arguments. Returns None if nothing to send."""
    payload: dict = {"type": "servo"}

    # --calibrate on/off
    if args.calibrate is not None:
        payload["calibrate"] = args.calibrate
        return payload

    # --reset
    if args.reset:
        payload["reset"] = True
        return payload

    # --status
    if args.status:
        payload["query"] = "status"
        return payload

    # --pitch / --yaw target angles
    if args.pitch is not None or args.yaw is not None:
        if args.pitch is not None:
            payload["pitch"] = args.pitch
        if args.yaw is not None:
            payload["yaw"] = args.yaw
        return payload

    # Calibration parameter updates
    calib: dict = {}
    pitch: dict = {}
    yaw: dict = {}

    for attr, key, is_yaw in [
        ("calib_pitch_min", "angle_min", False),
        ("calib_pitch_max", "angle_max", False),
        ("calib_pitch_center", "pulse_center", False),
        ("calib_pitch_pulse_min", "pulse_min", False),
        ("calib_pitch_pulse_max", "pulse_max", False),
        ("calib_yaw_min", "angle_min", True),
        ("calib_yaw_max", "angle_max", True),
        ("calib_yaw_center", "pulse_center", True),
        ("calib_yaw_pulse_min", "pulse_min", True),
        ("calib_yaw_pulse_max", "pulse_max", True),
    ]:
        val = getattr(args, attr, None)
        if val is not None:
            if key in ("pulse_center", "pulse_min", "pulse_max"):
                val = int(val)
            target = yaw if is_yaw else pitch
            target[key] = val

    if pitch or yaw:
        if pitch:
            calib["pitch"] = pitch
        if yaw:
            calib["yaw"] = yaw
        payload["calib"] = calib
        return payload

    return None


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Control ESP32-S3 servos via MQTT",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )

    # MQTT connection
    parser.add_argument(
        "--broker",
        default=os.environ.get("SERVO_MQTT_BROKER", "localhost"),
        help="MQTT broker host (default: localhost, env: SERVO_MQTT_BROKER)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.environ.get("SERVO_MQTT_PORT", "1883")),
        help="MQTT broker port (default: 1883, env: SERVO_MQTT_PORT)",
    )
    parser.add_argument(
        "--topic",
        default=os.environ.get("SERVO_MQTT_TOPIC", "copilot/s3_copilot/cmd"),
        help="MQTT command topic (default: copilot/s3_copilot/cmd)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=3.0,
        help="MQTT connect timeout in seconds (default: 3)",
    )

    # Servo targets
    parser.add_argument("--pitch", type=float, help="Pitch target angle (degrees, +up/-down)")
    parser.add_argument("--yaw", type=float, help="Yaw target angle (degrees, +left/-right)")
    parser.add_argument(
        "--calibrate",
        type=str,
        choices=["on", "off"],
        help="Enable/disable calibration mode (servos hold center)",
    )
    parser.add_argument("--reset", action="store_true", help="Reset calibration to defaults")
    parser.add_argument("--status", action="store_true", help="Query servo status (check serial monitor)")

    # Calibration parameters (pitch)
    parser.add_argument("--calib-pitch-min", type=float, help="Pitch angle_min (degrees, negative)")
    parser.add_argument("--calib-pitch-max", type=float, help="Pitch angle_max (degrees, positive)")
    parser.add_argument("--calib-pitch-center", type=float, help="Pitch pulse_center_us (microseconds)")
    parser.add_argument("--calib-pitch-pulse-min", type=float, help="Pitch pulse_min_us (microseconds)")
    parser.add_argument("--calib-pitch-pulse-max", type=float, help="Pitch pulse_max_us (microseconds)")

    # Calibration parameters (yaw)
    parser.add_argument("--calib-yaw-min", type=float, help="Yaw angle_min (degrees, negative)")
    parser.add_argument("--calib-yaw-max", type=float, help="Yaw angle_max (degrees, positive)")
    parser.add_argument("--calib-yaw-center", type=float, help="Yaw pulse_center_us (microseconds)")
    parser.add_argument("--calib-yaw-pulse-min", type=float, help="Yaw pulse_min_us (microseconds)")
    parser.add_argument("--calib-yaw-pulse-max", type=float, help="Yaw pulse_max_us (microseconds)")

    args = parser.parse_args()

    payload = build_payload(args)
    if payload is None:
        parser.print_help()
        print("\nError: No servo command specified. Use --pitch/--yaw, --calibrate, --reset, or --status.")
        sys.exit(1)

    payload_str = json.dumps(payload, separators=(",", ":"))
    print(f"MQTT → {args.broker}:{args.port}  topic={args.topic}")
    print(f"Payload: {payload_str}")

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.connect(args.broker, args.port, keepalive=5)

    # Use simple publish (fire-and-forget with brief wait for delivery)
    info = client.publish(args.topic, payload_str, qos=1)
    info.wait_for_publish(timeout=args.timeout)

    if info.rc == mqtt.MQTT_ERR_SUCCESS:
        print("OK — published successfully")
    else:
        print(f"Publish failed: rc={info.rc}")
        sys.exit(1)

    client.disconnect()
    time.sleep(0.05)


if __name__ == "__main__":
    main()
