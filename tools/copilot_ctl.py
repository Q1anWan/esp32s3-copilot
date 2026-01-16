#!/usr/bin/env python3
"""
Copilot Control Tool - Unified control for ESP32-S3 Copilot

Combines MQTT device control with TTS/Voice backend integration.

Features:
- Expression control (happy, sad, angry, etc.)
- Motion control (drift, yaw)
- Sound playback
- Ring indicator
- Voice module control (session, loopback, volume)
- TTS sending to voice backend
- Device status monitoring

Usage:
    python copilot_ctl.py                    # Interactive mode
    python copilot_ctl.py --mqtt-host 192.168.1.100
    python copilot_ctl.py --voice-server ws://192.168.1.100:8080
"""
import argparse
import asyncio
import json
import os
import sys
import time
import threading
from typing import Optional


def require_paho():
    try:
        import paho.mqtt.client as mqtt
        return mqtt
    except ImportError:
        print("Error: paho-mqtt not installed.")
        print("Install with: python -m pip install paho-mqtt")
        sys.exit(1)


def require_aiohttp():
    try:
        import aiohttp
        return aiohttp
    except ImportError:
        return None


class CopilotController:
    """Unified controller for Copilot device"""

    def __init__(self, mqtt_host: str, mqtt_port: int, mqtt_prefix: str,
                 device_id: str, voice_server: str):
        self.mqtt_host = mqtt_host
        self.mqtt_port = mqtt_port
        self.mqtt_prefix = mqtt_prefix
        self.device_id = device_id
        self.voice_server = voice_server

        self.cmd_topic = f"{mqtt_prefix}/{device_id}/cmd"

        self._mqtt = require_paho()
        self._client: Optional[object] = None
        self._connected = False

        # Volume/gain tracking
        self._volume = 80
        self._mic_gain = 24

    def connect(self) -> bool:
        """Connect to MQTT broker"""
        self._client = self._mqtt.Client()
        try:
            self._client.connect(self.mqtt_host, self.mqtt_port, keepalive=60)
            self._client.loop_start()
            self._connected = True
            return True
        except Exception as e:
            print(f"MQTT connection failed: {e}")
            return False

    def disconnect(self):
        """Disconnect from MQTT broker"""
        if self._client:
            self._client.loop_stop()
            self._client.disconnect()
            self._connected = False

    def send(self, payload: dict) -> bool:
        """Send MQTT command"""
        if not self._connected:
            print("Not connected to MQTT broker")
            return False

        data = json.dumps(payload, separators=(",", ":"))
        result = self._client.publish(self.cmd_topic, data)
        result.wait_for_publish()
        if result.rc != 0:
            print(f"Publish failed: rc={result.rc}")
            return False
        print(f"  -> Sent: {data}")
        return True

    # Expression commands
    def set_expression(self, name: str, duration_ms: int = 400,
                       prelight_ms: int = 500, sound: str = None):
        payload = {
            "type": "emotion",
            "name": name,
            "duration_ms": duration_ms,
            "prelight_ms": prelight_ms,
        }
        if sound:
            payload["sound"] = sound
        return self.send(payload)

    # Motion commands
    def set_motion(self, ax: float = 0.0, ay: float = 0.0, yaw: float = 0.0):
        return self.send({"type": "motion", "ax": ax, "ay": ay, "yaw": yaw})

    # Ring commands
    def ring_show(self, on: bool):
        return self.send({"type": "ring", "on": on})

    # Sound commands
    def play_sound(self, sound_id: str, prelight_ms: int = 300):
        return self.send({"type": "sound", "id": sound_id, "prelight_ms": prelight_ms})

    # Voice commands
    def voice_start(self):
        return self.send({"type": "voice", "action": "start"})

    def voice_stop(self):
        return self.send({"type": "voice", "action": "stop"})

    def voice_loopback(self):
        return self.send({"type": "voice", "action": "loopback"})

    def voice_volume(self, volume: int):
        self._volume = max(0, min(100, volume))
        return self.send({"type": "voice", "volume": self._volume})

    def voice_mic_gain(self, gain: int):
        self._mic_gain = max(0, min(36, gain))
        return self.send({"type": "voice", "mic_gain": self._mic_gain})

    # Status query
    def query_status(self, target: str = "all"):
        return self.send({"type": "status", "query": target})

    # Calibration
    def calibrate_gyro(self):
        return self.send({"type": "calibrate", "target": "gyro"})

    # TTS (requires voice backend)
    async def send_tts_async(self, text: str) -> bool:
        """Send TTS request to voice backend"""
        aiohttp = require_aiohttp()
        if aiohttp is None:
            print("Error: aiohttp not installed. Install with: pip install aiohttp")
            return False

        ws_url = self.voice_server.replace("http://", "ws://").replace("https://", "wss://")
        if not ws_url.endswith("/api/tts"):
            ws_url = ws_url.rstrip("/") + "/api/tts"

        try:
            async with aiohttp.ClientSession() as session:
                async with session.ws_connect(ws_url, timeout=5) as ws:
                    await ws.send_json({"text": text})
                    response = await ws.receive_json(timeout=10)
                    if response.get("status") == "ok":
                        print(f"  TTS sent to {response.get('sessions', 0)} device(s)")
                        return True
                    else:
                        print(f"  TTS error: {response.get('message', 'Unknown')}")
                        return False
        except Exception as e:
            print(f"  TTS failed: {e}")
            return False

    def send_tts(self, text: str) -> bool:
        """Send TTS (blocking wrapper)"""
        try:
            return asyncio.run(self.send_tts_async(text))
        except Exception as e:
            print(f"  TTS error: {e}")
            return False


def show_menu(ctl: CopilotController):
    """Display interactive menu"""
    print("")
    print("=" * 65)
    print("  Copilot Control Tool")
    print(f"  MQTT: {ctl.mqtt_host}:{ctl.mqtt_port}  Topic: {ctl.cmd_topic}")
    print(f"  Voice: {ctl.voice_server}")
    print("=" * 65)
    print("")
    print("Expression:  1)Happy 2)Sad 3)Angry 4)Surprised 5)Sleepy 6)Dizzy 7)Neutral")
    print("Motion:      m1)Right m2)Left m3)Up m4)Down m0)Reset")
    print("Sound:       s1)beep_short s2)beep_long s3)chime s4)tap")
    print("Ring:        r1)ON r0)OFF")
    print("")
    print("Voice:       v1)Start v0)Stop vl)Loopback vs)Status")
    print("             v+)Vol+ v-)Vol- g+)Gain+ g-)Gain-")
    print("")
    print("TTS:         tts <text>  (e.g., 'tts Hello World')")
    print("")
    print("Calibration: cal)Gyro  Status: st)All")
    print("Demo:        demo)Expression demo  vdemo)Voice-UI demo")
    print("")
    print("Other:       q)Quit  {json}Send raw JSON")
    print("")


def run_expression_demo(ctl: CopilotController):
    """Run expression demo sequence"""
    print("Running expression demo...")
    steps = [
        ("happy", "happy", 420, 500, "chime"),
        ("surprised", "surprised", 260, 400, "beep_short"),
        ("sad", "sad", 780, 500, "beep_short"),
        ("motion right", None, 0, 0, None),
        ("motion left+up", None, 0, 0, None),
        ("neutral", "neutral", 300, 0, None),
    ]
    for i, (name, expr, dur, pre, snd) in enumerate(steps, 1):
        print(f"  [{i}/{len(steps)}] {name}")
        if expr:
            ctl.set_expression(expr, dur, pre, snd)
        elif "right" in name:
            ctl.set_motion(ax=0.35, ay=0.0, yaw=8.0)
        elif "left" in name:
            ctl.set_motion(ax=-0.25, ay=0.4, yaw=-12.0)
        time.sleep(1.2 if expr else 1.0)
    print("Demo complete!")


def run_voice_demo(ctl: CopilotController):
    """Run voice-UI demo to test mouth animation"""
    print("Running voice-UI demo...")
    print("  This tests voice-UI integration (mouth animation).")
    print("")

    print("  [1/5] Starting voice loopback...")
    ctl.voice_loopback()
    time.sleep(0.5)

    print("  [2/5] Setting happy expression...")
    ctl.set_expression("happy", 300)
    time.sleep(0.3)

    print("  [3/5] Speak into the microphone now!")
    print("        (The mouth should animate with your voice)")
    time.sleep(5.0)

    print("  [4/5] Setting neutral expression...")
    ctl.set_expression("neutral", 300)
    time.sleep(0.3)

    print("  [5/5] Stopping loopback...")
    ctl.voice_loopback()
    print("Voice-UI demo complete!")


def main():
    parser = argparse.ArgumentParser(
        description="Copilot Control Tool - Unified control for ESP32-S3 Copilot",
        add_help=False,
    )
    parser.add_argument("--mqtt-host", default=os.getenv("MQTT_HOST", "localhost"),
                        help="MQTT broker hostname")
    parser.add_argument("--mqtt-port", type=int, default=int(os.getenv("MQTT_PORT", "1883")),
                        help="MQTT broker port")
    parser.add_argument("-p", "--prefix", default=os.getenv("MQTT_PREFIX", "copilot"),
                        help="MQTT topic prefix")
    parser.add_argument("-d", "--device", default=os.getenv("MQTT_DEVICE", "s3_copilot"),
                        help="Device ID")
    parser.add_argument("--voice-server", default=os.getenv("VOICE_SERVER", "ws://localhost:8080"),
                        help="Voice backend WebSocket URL")
    parser.add_argument("--help", action="store_true", help="Show help")
    args = parser.parse_args()

    if args.help:
        parser.print_help()
        print("")
        print("Environment variables:")
        print("  MQTT_HOST      MQTT broker hostname")
        print("  MQTT_PORT      MQTT broker port (default: 1883)")
        print("  MQTT_PREFIX    MQTT topic prefix (default: copilot)")
        print("  MQTT_DEVICE    Device ID (default: s3_copilot)")
        print("  VOICE_SERVER   Voice backend URL (default: ws://localhost:8080)")
        sys.exit(0)

    ctl = CopilotController(
        mqtt_host=args.mqtt_host,
        mqtt_port=args.mqtt_port,
        mqtt_prefix=args.prefix,
        device_id=args.device,
        voice_server=args.voice_server,
    )

    if not ctl.connect():
        print("Failed to connect to MQTT broker")
        sys.exit(1)

    # Command mapping
    commands = {
        "1": lambda: ctl.set_expression("happy", 420, 500, "chime"),
        "2": lambda: ctl.set_expression("sad", 600, 500, "beep_short"),
        "3": lambda: ctl.set_expression("angry", 500, 400),
        "4": lambda: ctl.set_expression("surprised", 300, 400, "beep_short"),
        "5": lambda: ctl.set_expression("sleepy", 800, 500),
        "6": lambda: ctl.set_expression("dizzy", 600, 400),
        "7": lambda: ctl.set_expression("neutral", 300, 0),
        "m1": lambda: ctl.set_motion(ax=0.5),
        "m2": lambda: ctl.set_motion(ax=-0.5),
        "m3": lambda: ctl.set_motion(ay=0.5),
        "m4": lambda: ctl.set_motion(ay=-0.5),
        "m0": lambda: ctl.set_motion(),
        "s1": lambda: ctl.play_sound("beep_short"),
        "s2": lambda: ctl.play_sound("beep_long"),
        "s3": lambda: ctl.play_sound("chime"),
        "s4": lambda: ctl.play_sound("tap"),
        "r1": lambda: ctl.ring_show(True),
        "r0": lambda: ctl.ring_show(False),
        "v1": lambda: ctl.voice_start(),
        "v0": lambda: ctl.voice_stop(),
        "vl": lambda: ctl.voice_loopback(),
        "vs": lambda: ctl.query_status("voice"),
        "v+": lambda: ctl.voice_volume(ctl._volume + 10),
        "v-": lambda: ctl.voice_volume(ctl._volume - 10),
        "g+": lambda: ctl.voice_mic_gain(ctl._mic_gain + 3),
        "g-": lambda: ctl.voice_mic_gain(ctl._mic_gain - 3),
        "cal": lambda: ctl.calibrate_gyro(),
        "st": lambda: ctl.query_status("all"),
    }

    try:
        while True:
            show_menu(ctl)
            cmd = input("Enter command: ").strip()

            if cmd.lower() in ("q", "quit", "exit"):
                print("Goodbye!")
                break

            if cmd == "demo":
                run_expression_demo(ctl)
                continue

            if cmd == "vdemo":
                run_voice_demo(ctl)
                continue

            # TTS command
            if cmd.lower().startswith("tts "):
                text = cmd[4:].strip()
                if text:
                    print(f"  Sending TTS: {text}")
                    ctl.send_tts(text)
                continue

            if cmd.lower() in commands:
                commands[cmd.lower()]()
                continue

            if cmd.startswith("{"):
                try:
                    payload = json.loads(cmd)
                    ctl.send(payload)
                except json.JSONDecodeError as e:
                    print(f"Invalid JSON: {e}")
                continue

            if cmd:
                print(f"Unknown command: {cmd}")

    except (KeyboardInterrupt, EOFError):
        print("\nGoodbye!")
    finally:
        ctl.disconnect()


if __name__ == "__main__":
    main()
