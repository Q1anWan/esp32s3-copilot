#!/usr/bin/env python3
import argparse
import json
import os
import sys
import time


def require_paho():
    try:
        import paho.mqtt.client as mqtt  # type: ignore
    except Exception:
        print("Error: paho-mqtt not installed.")
        print("Install with: python -m pip install paho-mqtt")
        sys.exit(1)
    return mqtt


def parse_args():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("-h", "--host", default=os.getenv("MQTT_HOST", "localhost"))
    parser.add_argument("--port", type=int, default=int(os.getenv("MQTT_PORT", "1883")))
    parser.add_argument("-p", "--prefix", default=os.getenv("MQTT_PREFIX", "copilot"))
    parser.add_argument("-d", "--device", default=os.getenv("MQTT_DEVICE", "s3_copilot"))
    parser.add_argument("-u", "--username", default=os.getenv("MQTT_USER", ""))
    parser.add_argument("-P", "--password", default=os.getenv("MQTT_PASS", ""))
    parser.add_argument("--help", action="store_true")
    args = parser.parse_args()

    if args.help:
        parser.print_help()
        print("")
        print("Environment variables:")
        print("  MQTT_HOST    MQTT broker hostname")
        print("  MQTT_PORT    MQTT broker port (default: 1883)")
        print("  MQTT_PREFIX  MQTT topic prefix (default: copilot)")
        print("  MQTT_DEVICE  Device ID (default: s3_copilot)")
        print("  MQTT_USER    MQTT username")
        print("  MQTT_PASS    MQTT password")
        sys.exit(0)

    return args


def show_menu(host, topic):
    print("")
    print("============================================")
    print("  Copilot MQTT Demo (Python)")
    print(f"  Broker: {host}")
    print(f"  Topic:  {topic}")
    print("============================================")
    print("")
    print("Expression Commands:")
    print("  1) Happy       2) Sad         3) Angry")
    print("  4) Surprised   5) Sleepy      6) Dizzy")
    print("  7) Neutral")
    print("")
    print("Motion Commands:")
    print("  m1) Motion: drift right")
    print("  m2) Motion: drift left")
    print("  m3) Motion: drift up")
    print("  m4) Motion: drift down")
    print("  m0) Motion: reset to center")
    print("")
    print("Sound Commands:")
    print("  s1) beep_short   s2) beep_long")
    print("  s3) chime        s4) tap")
    print("")
    print("Ring Control:")
    print("  r1) Ring ON      r0) Ring OFF")
    print("")
    print("IMU Calibration:")
    print("  cal) Start gyro calibration (keep device stationary!)")
    print("  st)  Query IMU status")
    print("")
    print("Other:")
    print("  demo) Run expression demo sequence")
    print("  q)    Quit")
    print("")


def publish_payload(client, topic, payload):
    if isinstance(payload, str):
        data = payload
    else:
        data = json.dumps(payload, separators=(",", ":"))

    result = client.publish(topic, data)
    result.wait_for_publish()
    if result.rc != 0:
        print(f"Publish failed: rc={result.rc}")
        return
    print(f"  -> Sent: {data}")


def run_demo(client, topic):
    print("Running demo sequence...")
    print("  [1/6] happy")
    publish_payload(client, topic, {
        "type": "emotion",
        "name": "happy",
        "duration_ms": 420,
        "prelight_ms": 500,
        "sound": "chime",
    })
    time.sleep(1.2)
    print("  [2/6] surprised")
    publish_payload(client, topic, {
        "type": "emotion",
        "name": "surprised",
        "duration_ms": 260,
        "prelight_ms": 400,
        "sound": "beep_short",
    })
    time.sleep(1.2)
    print("  [3/6] sad")
    publish_payload(client, topic, {
        "type": "emotion",
        "name": "sad",
        "duration_ms": 780,
        "prelight_ms": 500,
        "sound": "beep_short",
    })
    time.sleep(1.2)
    print("  [4/6] motion right")
    publish_payload(client, topic, {
        "type": "motion",
        "ax": 0.35,
        "ay": 0.0,
        "yaw": 8.0,
    })
    time.sleep(1.0)
    print("  [5/6] motion left+up")
    publish_payload(client, topic, {
        "type": "motion",
        "ax": -0.25,
        "ay": 0.4,
        "yaw": -12.0,
    })
    time.sleep(1.0)
    print("  [6/6] neutral")
    publish_payload(client, topic, {
        "type": "emotion",
        "name": "neutral",
        "duration_ms": 300,
        "prelight_ms": 0,
    })
    print("Demo complete!")


def main():
    args = parse_args()
    mqtt = require_paho()

    cmd_topic = f"{args.prefix}/{args.device}/cmd"
    client = mqtt.Client()
    if args.username:
        client.username_pw_set(args.username, args.password or None)

    try:
        client.connect(args.host, args.port, keepalive=60)
    except Exception as exc:
        print(f"Failed to connect to MQTT broker: {exc}")
        sys.exit(1)

    client.loop_start()

    commands = {
        "1": {"type": "emotion", "name": "happy", "duration_ms": 420, "prelight_ms": 500, "sound": "chime"},
        "2": {"type": "emotion", "name": "sad", "duration_ms": 600, "prelight_ms": 500, "sound": "beep_short"},
        "3": {"type": "emotion", "name": "angry", "duration_ms": 500, "prelight_ms": 400},
        "4": {"type": "emotion", "name": "surprised", "duration_ms": 300, "prelight_ms": 400, "sound": "beep_short"},
        "5": {"type": "emotion", "name": "sleepy", "duration_ms": 800, "prelight_ms": 500},
        "6": {"type": "emotion", "name": "dizzy", "duration_ms": 600, "prelight_ms": 400},
        "7": {"type": "emotion", "name": "neutral", "duration_ms": 300, "prelight_ms": 0},
        "m1": {"type": "motion", "ax": 0.5, "ay": 0.0, "yaw": 0.0},
        "m2": {"type": "motion", "ax": -0.5, "ay": 0.0, "yaw": 0.0},
        "m3": {"type": "motion", "ax": 0.0, "ay": 0.5, "yaw": 0.0},
        "m4": {"type": "motion", "ax": 0.0, "ay": -0.5, "yaw": 0.0},
        "m0": {"type": "motion", "ax": 0.0, "ay": 0.0, "yaw": 0.0},
        "s1": {"type": "sound", "id": "beep_short", "prelight_ms": 300},
        "s2": {"type": "sound", "id": "beep_long", "prelight_ms": 300},
        "s3": {"type": "sound", "id": "chime", "prelight_ms": 300},
        "s4": {"type": "sound", "id": "tap", "prelight_ms": 300},
        "r1": {"type": "ring", "on": True},
        "r0": {"type": "ring", "on": False},
        "cal": {"type": "calibrate", "target": "gyro"},
        "st": {"type": "status", "query": "imu"},
    }

    try:
        while True:
            show_menu(args.host, cmd_topic)
            cmd = input("Enter command: ").strip()

            if cmd in ("q", "Q", "quit", "exit"):
                print("Goodbye!")
                break

            if cmd == "demo":
                run_demo(client, cmd_topic)
                continue

            if cmd in commands:
                publish_payload(client, cmd_topic, commands[cmd])
                continue

            if cmd.startswith("{"):
                try:
                    json.loads(cmd)
                except Exception as exc:
                    print(f"Invalid JSON: {exc}")
                    continue
                publish_payload(client, cmd_topic, cmd)
                continue

            if cmd:
                print(f"Unknown command: {cmd}")
    except (KeyboardInterrupt, EOFError):
        print("\nGoodbye!")
    finally:
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()
