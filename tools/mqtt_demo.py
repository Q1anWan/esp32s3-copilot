#!/usr/bin/env python3
"""
Copilot MQTT Demo - Interactive control tool for ESP32-S3 Copilot

Supports:
- Expression control (happy, sad, angry, etc.)
- Motion control (drift, yaw)
- Sound playback
- Ring indicator
- Voice module control (session, loopback, volume)
- IMU calibration
"""
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
    parser = argparse.ArgumentParser(
        description="Copilot MQTT Demo - Interactive control tool",
        add_help=False,
    )
    parser.add_argument("-h", "--host", default=os.getenv("MQTT_HOST", "localhost"),
                        help="MQTT broker hostname")
    parser.add_argument("--port", type=int, default=int(os.getenv("MQTT_PORT", "1883")),
                        help="MQTT broker port")
    parser.add_argument("-p", "--prefix", default=os.getenv("MQTT_PREFIX", "copilot"),
                        help="MQTT topic prefix")
    parser.add_argument("-d", "--device", default=os.getenv("MQTT_DEVICE", "s3_copilot"),
                        help="Device ID")
    parser.add_argument("-u", "--username", default=os.getenv("MQTT_USER", ""),
                        help="MQTT username")
    parser.add_argument("-P", "--password", default=os.getenv("MQTT_PASS", ""),
                        help="MQTT password")
    parser.add_argument("--help", action="store_true", help="Show help")
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
    print("=" * 60)
    print("  Copilot MQTT Demo")
    print(f"  Broker: {host}")
    print(f"  Topic:  {topic}")
    print("=" * 60)
    print("")
    print("Expression Commands:")
    print("  1) Happy       2) Sad         3) Angry")
    print("  4) Surprised   5) Sleepy      6) Dizzy")
    print("  7) Neutral")
    print("")
    print("Motion Commands:")
    print("  m1) Drift right    m2) Drift left")
    print("  m3) Drift up       m4) Drift down")
    print("  m0) Reset center")
    print("")
    print("Sound Commands:")
    print("  s1) beep_short   s2) beep_long   s3) chime   s4) tap")
    print("")
    print("Ring Control:")
    print("  r1) Ring ON      r0) Ring OFF")
    print("")
    print("Voice Control:")
    print("  v1) Start voice session    v0) Stop voice session")
    print("  vl) Toggle loopback test   vs) Query voice status")
    print("  v+) Volume up (+10)        v-) Volume down (-10)")
    print("  g+) Mic gain up (+3dB)     g-) Mic gain down (-3dB)")
    print("")
    print("IMU Calibration:")
    print("  cal) Start gyro calibration (keep device stationary!)")
    print("  st)  Query device status (IMU + Voice)")
    print("")
    print("Demos:")
    print("  demo)  Run expression demo")
    print("  vdemo) Run voice-UI demo (shows mouth animation)")
    print("")
    print("Other:")
    print("  q) Quit   {json} Send raw JSON")
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


def run_expression_demo(client, topic):
    """Run classic expression demo sequence."""
    print("Running expression demo...")
    steps = [
        ("happy", {"type": "emotion", "name": "happy", "duration_ms": 420, "prelight_ms": 500, "sound": "chime"}),
        ("surprised", {"type": "emotion", "name": "surprised", "duration_ms": 260, "prelight_ms": 400, "sound": "beep_short"}),
        ("sad", {"type": "emotion", "name": "sad", "duration_ms": 780, "prelight_ms": 500, "sound": "beep_short"}),
        ("motion right", {"type": "motion", "ax": 0.35, "ay": 0.0, "yaw": 8.0}),
        ("motion left+up", {"type": "motion", "ax": -0.25, "ay": 0.4, "yaw": -12.0}),
        ("neutral", {"type": "emotion", "name": "neutral", "duration_ms": 300, "prelight_ms": 0}),
    ]
    for i, (name, payload) in enumerate(steps, 1):
        print(f"  [{i}/{len(steps)}] {name}")
        publish_payload(client, topic, payload)
        time.sleep(1.2 if "motion" not in name else 1.0)
    print("Demo complete!")


def run_voice_demo(client, topic):
    """Run voice-UI demo to show mouth animation."""
    print("Running voice-UI demo...")
    print("  This demo tests the voice-UI integration.")
    print("  The ring will show and mouth should animate if loopback is on.")
    print("")

    # Start loopback for mouth animation testing
    print("  [1/5] Starting voice loopback...")
    publish_payload(client, topic, {"type": "voice", "action": "loopback"})
    time.sleep(0.5)

    print("  [2/5] Setting happy expression...")
    publish_payload(client, topic, {"type": "emotion", "name": "happy", "duration_ms": 300})
    time.sleep(0.3)

    print("  [3/5] Speak into the microphone now!")
    print("        (The mouth should animate with your voice)")
    time.sleep(5.0)

    print("  [4/5] Setting neutral expression...")
    publish_payload(client, topic, {"type": "emotion", "name": "neutral", "duration_ms": 300})
    time.sleep(0.3)

    print("  [5/5] Stopping loopback...")
    publish_payload(client, topic, {"type": "voice", "action": "loopback"})

    print("Voice-UI demo complete!")


# Global state for volume/gain tracking
_volume = 80
_mic_gain = 24


def main():
    global _volume, _mic_gain

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

    # Command mapping
    commands = {
        # Expressions
        "1": {"type": "emotion", "name": "happy", "duration_ms": 420, "prelight_ms": 500, "sound": "chime"},
        "2": {"type": "emotion", "name": "sad", "duration_ms": 600, "prelight_ms": 500, "sound": "beep_short"},
        "3": {"type": "emotion", "name": "angry", "duration_ms": 500, "prelight_ms": 400},
        "4": {"type": "emotion", "name": "surprised", "duration_ms": 300, "prelight_ms": 400, "sound": "beep_short"},
        "5": {"type": "emotion", "name": "sleepy", "duration_ms": 800, "prelight_ms": 500},
        "6": {"type": "emotion", "name": "dizzy", "duration_ms": 600, "prelight_ms": 400},
        "7": {"type": "emotion", "name": "neutral", "duration_ms": 300, "prelight_ms": 0},
        # Motion
        "m1": {"type": "motion", "ax": 0.5, "ay": 0.0, "yaw": 0.0},
        "m2": {"type": "motion", "ax": -0.5, "ay": 0.0, "yaw": 0.0},
        "m3": {"type": "motion", "ax": 0.0, "ay": 0.5, "yaw": 0.0},
        "m4": {"type": "motion", "ax": 0.0, "ay": -0.5, "yaw": 0.0},
        "m0": {"type": "motion", "ax": 0.0, "ay": 0.0, "yaw": 0.0},
        # Sounds
        "s1": {"type": "sound", "id": "beep_short", "prelight_ms": 300},
        "s2": {"type": "sound", "id": "beep_long", "prelight_ms": 300},
        "s3": {"type": "sound", "id": "chime", "prelight_ms": 300},
        "s4": {"type": "sound", "id": "tap", "prelight_ms": 300},
        # Ring
        "r1": {"type": "ring", "on": True},
        "r0": {"type": "ring", "on": False},
        # Voice control
        "v1": {"type": "voice", "action": "start"},
        "v0": {"type": "voice", "action": "stop"},
        "vl": {"type": "voice", "action": "loopback"},
        "vs": {"type": "status", "query": "voice"},
        # IMU
        "cal": {"type": "calibrate", "target": "gyro"},
        "st": {"type": "status", "query": "all"},
    }

    try:
        while True:
            show_menu(args.host, cmd_topic)
            cmd = input("Enter command: ").strip().lower()

            if cmd in ("q", "quit", "exit"):
                print("Goodbye!")
                break

            if cmd == "demo":
                run_expression_demo(client, cmd_topic)
                continue

            if cmd == "vdemo":
                run_voice_demo(client, cmd_topic)
                continue

            # Volume control
            if cmd == "v+":
                _volume = min(100, _volume + 10)
                publish_payload(client, cmd_topic, {"type": "voice", "volume": _volume})
                print(f"  Volume: {_volume}%")
                continue
            if cmd == "v-":
                _volume = max(0, _volume - 10)
                publish_payload(client, cmd_topic, {"type": "voice", "volume": _volume})
                print(f"  Volume: {_volume}%")
                continue

            # Mic gain control
            if cmd == "g+":
                _mic_gain = min(36, _mic_gain + 3)
                publish_payload(client, cmd_topic, {"type": "voice", "mic_gain": _mic_gain})
                print(f"  Mic gain: {_mic_gain} dB")
                continue
            if cmd == "g-":
                _mic_gain = max(0, _mic_gain - 3)
                publish_payload(client, cmd_topic, {"type": "voice", "mic_gain": _mic_gain})
                print(f"  Mic gain: {_mic_gain} dB")
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
