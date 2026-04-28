#!/usr/bin/env python3
"""
Minimal local host loopback test for ESP32 slave mode.

What it does:
1. Waits for the ESP32 WebSocket connection on /audio/stream.
2. Replies with {"type":"started",...}.
3. Publishes MQTT speaking/idle commands.
4. Pushes 16 kHz PCM mono sine-wave frames to the ESP32.

This is intended for bench verification when the ESP32 firmware points to the
host machine's IP address for both MQTT and voice streaming.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import threading
import uuid

import paho.mqtt.client as mqtt
from host_ip import detect_host_ip
from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosed


FRAME_MS = 20
SAMPLE_RATE = 16000
SAMPLES_PER_FRAME = SAMPLE_RATE * FRAME_MS // 1000
BYTES_PER_SAMPLE = 2


def build_sine_pcm(duration_ms: int, freq_hz: float = 440.0, amplitude: float = 0.28) -> list[bytes]:
    total_samples = SAMPLE_RATE * duration_ms // 1000
    frames: list[bytes] = []
    samples: list[int] = []
    for i in range(total_samples):
        v = math.sin((2.0 * math.pi * freq_hz * i) / SAMPLE_RATE)
        samples.append(int(max(-1.0, min(1.0, v * amplitude)) * 32767.0))

    for start in range(0, len(samples), SAMPLES_PER_FRAME):
        chunk = samples[start:start + SAMPLES_PER_FRAME]
        if len(chunk) < SAMPLES_PER_FRAME:
            chunk.extend([0] * (SAMPLES_PER_FRAME - len(chunk)))
        frame = bytearray()
        for sample in chunk:
            frame.extend(int(sample).to_bytes(2, "little", signed=True))
        frames.append(bytes(frame))
    return frames


class LoopbackTest:
    def __init__(
        self,
        mqtt_host: str,
        mqtt_port: int,
        topic: str,
        tone_ms: int,
        announce_host: str,
        ws_port: int,
        mqtt_settle_ms: int,
        linger_ms: int,
    ) -> None:
        self.mqtt_host = mqtt_host
        self.mqtt_port = mqtt_port
        self.topic = topic
        self.tone_ms = tone_ms
        self.announce_host = announce_host
        self.ws_port = ws_port
        self.mqtt_settle_ms = max(0, mqtt_settle_ms)
        self.linger_ms = max(0, linger_ms)
        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt_connected = threading.Event()
        self.completed_event = asyncio.Event()
        self.session_counter = 0

    def _mqtt_connect(self) -> None:
        def on_connect(client, userdata, flags, reason_code, properties):
            print(f"[mqtt] connected to {self.mqtt_host}:{self.mqtt_port}, reason={reason_code}")
            self.mqtt_connected.set()

        self.mqtt_client.on_connect = on_connect
        self.mqtt_client.connect(self.mqtt_host, self.mqtt_port, keepalive=30)
        self.mqtt_client.loop_start()

    def _mqtt_publish_json(self, payload: dict) -> None:
        encoded = json.dumps(payload, ensure_ascii=True, separators=(",", ":"))
        info = self.mqtt_client.publish(self.topic, encoded, qos=0, retain=False)
        info.wait_for_publish(timeout=3.0)
        print(f"[mqtt] {self.topic} <- {encoded}")

    async def run_server(self, host: str, port: int) -> None:
        self._mqtt_connect()
        if not self.mqtt_connected.wait(timeout=5.0):
            raise RuntimeError(f"MQTT broker {self.mqtt_host}:{self.mqtt_port} did not accept connection")

        print(f"[hint] firmware MQTT URI  : mqtt://{self.announce_host}:{self.mqtt_port}")
        print(f"[hint] firmware Voice URL : http://{self.announce_host}:{self.ws_port}")
        print(f"[hint] command topic      : {self.topic}")

        async def handler(ws):
            self.session_counter += 1
            session_id = f"loopback-{self.session_counter}-{uuid.uuid4().hex[:8]}"
            print(f"[ws] client connected from {ws.remote_address}")

            try:
                first_msg = await asyncio.wait_for(ws.recv(), timeout=8.0)
                if isinstance(first_msg, bytes):
                    raise RuntimeError("expected JSON start message, got binary")
                print(f"[ws] rx text: {first_msg}")

                await ws.send(json.dumps({
                    "type": "started",
                    "session_id": session_id,
                }, separators=(",", ":")))
                print(f"[ws] tx started session_id={session_id}")

                if self.mqtt_settle_ms > 0:
                    print(f"[mqtt] settle wait {self.mqtt_settle_ms} ms before publish")
                    await asyncio.sleep(self.mqtt_settle_ms / 1000.0)

                self._mqtt_publish_json({
                    "type": "emotion",
                    "name": "speaking",
                    "duration_ms": self.tone_ms + 500,
                    "prelight_ms": 150,
                    "sound": "chime",
                })
                await asyncio.sleep(0.15)

                frames = build_sine_pcm(self.tone_ms)
                print(f"[ws] streaming {len(frames)} PCM frames ({self.tone_ms} ms)")
                for frame in frames:
                    await ws.send(frame)
                    await asyncio.sleep(FRAME_MS / 1000.0)

                await asyncio.sleep(0.35)
                self._mqtt_publish_json({
                    "type": "emotion",
                    "name": "idle",
                    "duration_ms": 0,
                })
                self.completed_event.set()
                print("[ws] test stream completed")

                try:
                    while True:
                        msg = await asyncio.wait_for(ws.recv(), timeout=1.0)
                        kind = "binary" if isinstance(msg, bytes) else "text"
                        size = len(msg) if hasattr(msg, "__len__") else 0
                        print(f"[ws] post-stream rx {kind} len={size}")
                except asyncio.TimeoutError:
                    pass
                except ConnectionClosed:
                    pass
            finally:
                await ws.close()
                print("[ws] connection closed")

        async with serve(handler, host, port, ping_interval=20, ping_timeout=20, max_size=2**20):
            print(f"[ws] listening on ws://{host}:{port}/audio/stream")
            await self.completed_event.wait()
            if self.linger_ms > 0:
                print(f"[ws] linger {self.linger_ms} ms before shutdown")
                await asyncio.sleep(self.linger_ms / 1000.0)

        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()


def main() -> int:
    parser = argparse.ArgumentParser(description="Host loopback test for ESP32 copilot slave")
    parser.add_argument("--host", default="0.0.0.0", help="Bind host for WebSocket server")
    parser.add_argument("--port", type=int, default=8080, help="WebSocket port")
    parser.add_argument(
        "--announce-host",
        default="auto",
        help="Host/IP written into firmware endpoints. 'auto' resolves the current LAN IP.",
    )
    parser.add_argument("--mqtt-host", default="127.0.0.1", help="MQTT broker host")
    parser.add_argument("--mqtt-port", type=int, default=1883, help="MQTT broker port")
    parser.add_argument("--topic", default="copilot/s3_copilot/cmd", help="MQTT command topic")
    parser.add_argument("--tone-ms", type=int, default=2200, help="PCM tone duration in milliseconds")
    parser.add_argument(
        "--mqtt-settle-ms",
        type=int,
        default=1000,
        help="Delay after WebSocket start before publishing MQTT speaking/idle commands",
    )
    parser.add_argument(
        "--linger-ms",
        type=int,
        default=3000,
        help="Keep the server alive briefly after the test to reduce reconnect noise",
    )
    args = parser.parse_args()

    announce_host = detect_host_ip() if args.announce_host == "auto" else args.announce_host
    test = LoopbackTest(
        args.mqtt_host,
        args.mqtt_port,
        args.topic,
        args.tone_ms,
        announce_host,
        args.port,
        args.mqtt_settle_ms,
        args.linger_ms,
    )
    asyncio.run(test.run_server(args.host, args.port))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
