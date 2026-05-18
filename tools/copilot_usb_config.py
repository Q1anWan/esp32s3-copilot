#!/usr/bin/env python3
"""USB serial helper for ESP32-S3 Copilot.

Works on Windows and Ubuntu with pyserial:
  python tools/copilot_usb_config.py wifi --ssid MyAP --password secret
  python tools/copilot_usb_config.py status
  python tools/copilot_usb_config.py play --scene boot --seq 1
  python tools/copilot_usb_config.py monitor
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from typing import Iterable


def require_serial():
    try:
        import serial  # type: ignore
        from serial.tools import list_ports  # type: ignore
    except ImportError:
        print("pyserial is required: python -m pip install pyserial", file=sys.stderr)
        raise SystemExit(2)
    return serial, list_ports


def auto_port() -> str:
    _, list_ports = require_serial()
    ports = list(list_ports.comports())
    if not ports:
        raise SystemExit("No serial ports found. Connect USB and try again.")

    preferred = []
    for p in ports:
        text = " ".join(str(x) for x in (p.device, p.description, p.manufacturer, p.hwid) if x)
        score = 0
        for key in ("USB JTAG", "USB Serial", "CP210", "CH340", "ACM", "ESP32"):
            if key.lower() in text.lower():
                score += 1
        preferred.append((score, p.device, text))
    preferred.sort(reverse=True)
    return preferred[0][1]


def open_serial(args):
    serial, _ = require_serial()
    port = args.port if args.port != "auto" else auto_port()
    ser = serial.Serial(port, args.baud, timeout=0.05, write_timeout=1.0)
    time.sleep(args.settle)
    return ser, port


def send_line(ser, line: str) -> None:
    ser.write((line.rstrip("\r\n") + "\n").encode("utf-8"))
    ser.flush()


def read_lines(ser, seconds: float) -> list[str]:
    deadline = time.time() + seconds
    buf = bytearray()
    lines: list[str] = []
    while time.time() < deadline:
        chunk = ser.read(256)
        if not chunk:
            continue
        for b in chunk:
            if b in (10, 13):
                if buf:
                    line = buf.decode("utf-8", errors="replace")
                    lines.append(line)
                    print(line)
                    buf.clear()
            else:
                buf.append(b)
    if buf:
        line = buf.decode("utf-8", errors="replace")
        lines.append(line)
        print(line)
    return lines


def extract_status(lines: Iterable[str]) -> dict | None:
    for line in reversed(list(lines)):
        if line.startswith("COPILOT_STATUS "):
            payload = line[len("COPILOT_STATUS ") :]
            try:
                return json.loads(payload)
            except json.JSONDecodeError:
                return None
    return None


def cmd_wifi(args) -> None:
    ser, port = open_serial(args)
    print(f"[usb] opened {port} @ {args.baud}")
    payload = {"type": "wifi", "ssid": args.ssid, "password": args.password or ""}
    send_line(ser, json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
    read_lines(ser, args.wait)
    send_line(ser, "status")
    lines = read_lines(ser, args.wait)
    status = extract_status(lines)
    if status:
        wifi = status.get("wifi", {})
        print(f"[status] connected={wifi.get('connected')} ip={wifi.get('ip')} tcp={status.get('tcp_host')}")


def cmd_status(args) -> None:
    ser, port = open_serial(args)
    print(f"[usb] opened {port} @ {args.baud}")
    send_line(ser, "status")
    lines = read_lines(ser, args.wait)
    status = extract_status(lines)
    if status:
        print(json.dumps(status, ensure_ascii=False, indent=2))


def cmd_play(args) -> None:
    ser, port = open_serial(args)
    print(f"[usb] opened {port} @ {args.baud}")
    send_line(ser, f"play {args.scene} {args.seq}")
    read_lines(ser, args.wait)


def cmd_debug(args) -> None:
    ser, port = open_serial(args)
    print(f"[usb] opened {port} @ {args.baud}")
    send_line(ser, f"debug {args.state}")
    read_lines(ser, args.wait)


def cmd_monitor(args) -> None:
    ser, port = open_serial(args)
    print(f"[usb] opened {port} @ {args.baud}; Ctrl+C to exit")
    try:
        while True:
            read_lines(ser, 0.25)
    except KeyboardInterrupt:
        print()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Configure and debug ESP32-S3 Copilot over USB serial.")
    parser.add_argument("--port", default="auto", help="Serial port, for example COM7 or /dev/ttyACM0. Default: auto")
    parser.add_argument("--baud", type=int, default=2_000_000, help="Serial baud rate. Default: 2000000")
    parser.add_argument("--settle", type=float, default=0.2, help="Delay after opening port. Default: 0.2s")
    parser.add_argument("--wait", type=float, default=2.0, help="How long to read responses. Default: 2s")

    sub = parser.add_subparsers(dest="cmd", required=True)

    wifi = sub.add_parser("wifi", help="Save WiFi credentials to NVS and reconnect")
    wifi.add_argument("--ssid", required=True)
    wifi.add_argument("--password", default="")
    wifi.set_defaults(func=cmd_wifi)

    status = sub.add_parser("status", help="Print system status JSON")
    status.set_defaults(func=cmd_status)

    play = sub.add_parser("play", help="Play /sdcard/audio/<scene>/<seq>.wav")
    play.add_argument("--scene", required=True)
    play.add_argument("--seq", required=True, type=int)
    play.set_defaults(func=cmd_play)

    debug = sub.add_parser("debug", help="Turn selected firmware debug logs on/off")
    debug.add_argument("state", choices=["on", "off"])
    debug.set_defaults(func=cmd_debug)

    monitor = sub.add_parser("monitor", help="Print serial logs")
    monitor.set_defaults(func=cmd_monitor)

    return parser


def main() -> None:
    args = build_parser().parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
