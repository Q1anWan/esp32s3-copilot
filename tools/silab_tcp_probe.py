#!/usr/bin/env python3
"""Validate SILAB TCP audio-ID packets for ESP32-S3 Copilot.

Expected packet, one UTF-8/ASCII line per SILAB sample:

  trigger<TAB>scene<TAB>seq<LF>

Examples:
  0<TAB>1<TAB>1
  1<TAB>1<TAB>1      -> Bridge default alias maps scene 1 to /sdcard/audio/boot/001.wav
  1<TAB>nomi<TAB>2   -> ESP32 maps scene "nomi" to /sdcard/audio/nomi/002.wav

The ESP32 plays only on the trigger rising edge. Repeated trigger=1 packets from
a fixed-rate SILAB sender are intentionally ignored until trigger returns to 0.
"""

from __future__ import annotations

import argparse
import binascii
import json
import socket
import time
from dataclasses import dataclass
from pathlib import Path

DEFAULT_SCENE_PREFIX = "scene"
DEFAULT_SCENE_ALIASES = "1=boot"


@dataclass
class AudioIdPacket:
    text: str
    trigger: float
    scene: str
    seq: int


def hex_with_spaces(data: bytes) -> str:
    text = binascii.hexlify(data).decode("ascii")
    return " ".join(text[i : i + 2] for i in range(0, len(text), 2))


def parse_integer_like(text: str) -> int | None:
    try:
        value = float(text)
    except ValueError:
        return None
    rounded = int(round(value))
    if abs(value - rounded) > 0.0001 or rounded < 0:
        return None
    return rounded


def scene_token_is_valid(scene: str) -> bool:
    if parse_integer_like(scene) is not None:
        return True
    return bool(scene) and all(c in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for c in scene)


def parse_scene_aliases(text: str) -> dict[int, str]:
    aliases: dict[int, str] = {}
    for item in text.replace(";", ",").split(","):
        item = item.strip()
        if not item or "=" not in item:
            continue
        key, value = item.split("=", 1)
        numeric = parse_integer_like(key.strip())
        value = value.strip()
        if numeric is None or numeric < 0 or not scene_token_is_valid(value):
            continue
        aliases[numeric] = value
    return aliases


def normalize_scene(scene: str, prefix: str, aliases: str) -> str:
    numeric = parse_integer_like(scene)
    if numeric is not None:
        alias = parse_scene_aliases(aliases).get(numeric)
        if alias:
            return alias
        return f"{prefix}{numeric:03d}"
    return scene


def parse_packet(data: bytes) -> AudioIdPacket | None:
    text = data.decode("utf-8", errors="ignore").strip()
    if not text:
        return None
    parts = text.replace(",", "\t").split()
    if len(parts) != 3:
        return None
    try:
        trigger = float(parts[0])
    except ValueError:
        return None
    seq = parse_integer_like(parts[2])
    if seq is None or seq < 1 or seq > 65535:
        return None
    scene = parts[1]
    if not scene_token_is_valid(scene):
        return None
    return AudioIdPacket(text=text, trigger=trigger, scene=scene, seq=seq)


def append_jsonl(path: Path, record: dict) -> None:
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")


def handle_data(data: bytes, packet_no: int, args: argparse.Namespace, state: dict) -> None:
    now = time.strftime("%Y-%m-%d %H:%M:%S")
    print("\n" + "-" * 72)
    print(f"#{packet_no} local_time={now} bytes={len(data)}")
    print("raw:", data)
    print("hex:", hex_with_spaces(data))

    packet = parse_packet(data)
    record = {
        "packet_no": packet_no,
        "local_time": now,
        "bytes": len(data),
        "raw_utf8": data.decode("utf-8", errors="replace"),
        "hex": hex_with_spaces(data),
        "valid": packet is not None,
    }

    if packet is None:
        print("format: BAD expected trigger<TAB>scene<TAB>seq")
    else:
        bridge_scene = normalize_scene(packet.scene, args.scene_prefix, args.scene_aliases)
        active = packet.trigger >= args.trigger_threshold
        rising = active and not state["latched"]
        state["latched"] = active
        record.update(
            {
                "trigger": packet.trigger,
                "scene": packet.scene,
                "bridge_scene": bridge_scene,
                "seq": packet.seq,
                "active": active,
                "rising": rising,
            }
        )
        print("format: OK")
        print(f"parsed: trigger={packet.trigger} scene={packet.scene} seq={packet.seq}")
        print(f"bridge_audio_id: scene={bridge_scene} seq={packet.seq}")
        print(f"trigger_active={active} rising_edge={rising}")
        if rising:
            print(f"would_play: /sdcard/audio/{bridge_scene}/{packet.seq:03d}.wav")

    if args.jsonl:
        append_jsonl(Path(args.jsonl), record)


def run_server(args: argparse.Namespace) -> None:
    packet_no = 0
    state = {"latched": False}
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((args.host, args.port))
        server.listen(1)
        print(f"Waiting for SILAB TCP client on {args.host}:{args.port}")
        print("Expected line: trigger<TAB>scene<TAB>seq")

        while True:
            conn, addr = server.accept()
            print(f"SILAB connected: {addr}")
            buffer = bytearray()
            with conn:
                while True:
                    data = conn.recv(1024)
                    if not data:
                        if buffer:
                            packet_no += 1
                            handle_data(bytes(buffer), packet_no, args, state)
                            buffer.clear()
                        print("connection closed")
                        break
                    for b in data:
                        if b in (10, 13):
                            if buffer:
                                packet_no += 1
                                handle_data(bytes(buffer), packet_no, args, state)
                                buffer.clear()
                        else:
                            buffer.append(b)
            if args.once:
                break


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Validate SILAB trigger/scene/seq TCP packets.")
    parser.add_argument("--host", default="0.0.0.0", help="Listen address. Default: 0.0.0.0")
    parser.add_argument("--port", type=int, default=7777, help="Listen TCP port. Default: 7777")
    parser.add_argument("--once", action="store_true", help="Exit after the first client disconnects")
    parser.add_argument("--trigger-threshold", type=float, default=0.5, help="Trigger threshold. Default: 0.5")
    parser.add_argument("--scene-prefix", default=DEFAULT_SCENE_PREFIX, help="Numeric scene prefix. Default: scene")
    parser.add_argument("--scene-aliases", default=DEFAULT_SCENE_ALIASES, help="Numeric scene aliases. Default: 1=boot")
    parser.add_argument("--jsonl", default="", help="Optional path to append machine-readable packet logs")
    return parser


def main() -> None:
    run_server(build_parser().parse_args())


if __name__ == "__main__":
    main()
