#!/usr/bin/env python3
"""USB serial helper for ESP32-S3 Copilot.

Works on Windows and Ubuntu with pyserial:
  python tools/copilot_usb_config.py wifi --ssid MyAP --password secret
  python tools/copilot_usb_config.py mqtt --broker mqtt://192.168.0.10:1883
  python tools/copilot_usb_config.py status
  python tools/copilot_usb_config.py play --scene boot --seq 1
  python tools/copilot_usb_config.py play --scene common --seq left_rear_vehicle_merge
  python tools/copilot_usb_config.py monitor
"""

from __future__ import annotations

import argparse
import json
import os
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
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = args.baud
    ser.timeout = 0.05
    ser.write_timeout = 1.0
    ser.dtr = False
    ser.rts = False
    ser.open()
    disable_hangup_on_close(ser)
    ser.dtr = False
    ser.rts = False
    if args.reset:
        ser.dtr = True
        ser.rts = False
        time.sleep(0.05)
        ser.dtr = False
        ser.rts = False
    time.sleep(args.settle)
    return ser, port


def disable_hangup_on_close(ser) -> None:
    if os.name != "posix":
        return
    try:
        import termios

        attrs = termios.tcgetattr(ser.fileno())
        attrs[2] &= ~termios.HUPCL
        termios.tcsetattr(ser.fileno(), termios.TCSANOW, attrs)
    except Exception:
        pass


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


def wait_ready(ser, seconds: float) -> list[str]:
    if seconds <= 0:
        return []
    deadline = time.time() + seconds
    started = time.time()
    seen: list[str] = []
    while time.time() < deadline:
        lines = read_lines(ser, min(0.25, max(0.0, deadline - time.time())))
        seen.extend(lines)
        if any("COPILOT_SERIAL ready" in line for line in lines):
            break
        if not seen and time.time() - started >= 1.0:
            break
    return seen


def extract_status(lines: Iterable[str]) -> dict | None:
    for line in reversed(list(lines)):
        if line.startswith("COPILOT_STATUS "):
            payload = line[len("COPILOT_STATUS ") :]
            try:
                return json.loads(payload)
            except json.JSONDecodeError:
                return None
    return None


def request_status(ser, wait: float) -> tuple[dict | None, list[str]]:
    send_line(ser, "status")
    lines = read_lines(ser, wait)
    return extract_status(lines), lines


def wait_status_response(ser, wait: float, timeout: float) -> dict | None:
    deadline = time.time() + max(0.0, timeout)
    last_status: dict | None = None
    while time.time() < deadline:
        status, _ = request_status(ser, wait)
        if status:
            return status
        time.sleep(0.25)
    return last_status


def command_needs_json(ssid: str, password: str) -> bool:
    return any(ch.isspace() for ch in ssid) or any(ch.isspace() for ch in password)


def wifi_command_line(ssid: str, password: str) -> str:
    if command_needs_json(ssid, password):
        payload = {"type": "wifi", "ssid": ssid, "password": password}
        return json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
    return f"wifi {ssid} {password}".rstrip()


def command_accepted(lines: Iterable[str], command: str) -> bool:
    if command.startswith("{"):
        return any(line.startswith("COPILOT_OK json_accepted") for line in lines)
    return any(line.startswith("COPILOT_OK wifi ") for line in lines)


def send_wifi_with_retry(ser, ssid: str, password: str, wait: float, attempts: int = 3) -> bool:
    command = wifi_command_line(ssid, password)
    safe = command.replace(password, "***") if password else command
    for idx in range(1, attempts + 1):
        print(f"[usb] wifi attempt {idx}: {safe}")
        send_line(ser, command)
        lines = read_lines(ser, wait)
        if command_accepted(lines, command):
            return True
        time.sleep(0.4)
    return False


def status_is_ready(status: dict | None, wait_wifi: bool) -> bool:
    if not status:
        return False
    if not wait_wifi:
        return True
    wifi = status.get("wifi", {})
    return bool(wifi.get("connected") and wifi.get("ip") and status.get("tcp_host"))


def poll_status(ser, wait: float, timeout: float, wait_wifi: bool) -> dict | None:
    deadline = time.time() + max(0.0, timeout)
    last_status: dict | None = None
    while True:
        status, _ = request_status(ser, wait)
        if status:
            last_status = status
        if status_is_ready(status, wait_wifi) or time.time() >= deadline:
            return last_status
        time.sleep(0.4)


def cmd_wifi(args) -> None:
    ser, port = open_serial(args)
    print(f"[usb] opened {port} @ {args.baud}")
    wait_ready(ser, args.ready_timeout)
    wait_status_response(ser, args.wait, args.ready_timeout)
    accepted = send_wifi_with_retry(ser, args.ssid, args.password or "", max(args.wait, 3.0))
    if accepted:
        print("[usb] WiFi credentials accepted; NVS/Flash save requested")
    else:
        print("[usb] WARNING: no WiFi ACK received before timeout", file=sys.stderr)
    status = poll_status(ser, args.wait, args.timeout, True)
    if status:
        wifi = status.get("wifi", {})
        print(
            f"[status] saved={wifi.get('saved')} ssid={wifi.get('ssid')} "
            f"saved_ssid={wifi.get('saved_ssid')} connected={wifi.get('connected')} "
            f"ip={wifi.get('ip')} tcp={status.get('tcp_host')}"
        )


def cmd_mqtt(args) -> None:
    ser, port = open_serial(args)
    print(f"[usb] opened {port} @ {args.baud}")
    wait_ready(ser, args.ready_timeout)
    broker = args.broker.strip()
    if not broker.startswith(("mqtt://", "mqtts://")):
        broker = "mqtt://" + broker
    send_line(ser, f"mqtt {broker}")
    read_lines(ser, args.wait)
    status = poll_status(ser, args.wait, args.timeout, False)
    if status:
        mqtt_status = status.get("mqtt", {})
        print(
            f"[status] mqtt_started={mqtt_status.get('started')} "
            f"mqtt_connected={mqtt_status.get('connected')} "
            f"broker={mqtt_status.get('broker')}"
        )


def cmd_status(args) -> None:
    ser, port = open_serial(args)
    print(f"[usb] opened {port} @ {args.baud}")
    wait_ready(ser, args.ready_timeout)
    status = poll_status(ser, args.wait, args.timeout, not args.no_wait_wifi)
    if status:
        print(json.dumps(status, ensure_ascii=False, indent=2))


def cmd_play(args) -> None:
    ser, port = open_serial(args)
    print(f"[usb] opened {port} @ {args.baud}")
    wait_ready(ser, args.ready_timeout)
    send_line(ser, f"play {args.scene} {args.seq}")
    read_lines(ser, args.wait)


def cmd_debug(args) -> None:
    ser, port = open_serial(args)
    print(f"[usb] opened {port} @ {args.baud}")
    wait_ready(ser, args.ready_timeout)
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
    parser.add_argument("--baud", type=int, default=115_200, help="Serial baud rate. Default: 115200")
    parser.add_argument("--settle", type=float, default=0.2, help="Delay after opening port. Default: 0.2s")
    parser.add_argument("--ready-timeout", type=float, default=8.0, help="Wait for COPILOT_SERIAL ready before commands. Default: 8s")
    parser.add_argument("--reset", action="store_true", help="Reset the ESP32 after opening the serial port before sending commands")
    parser.add_argument("--wait", type=float, default=2.0, help="How long to read responses. Default: 2s")

    sub = parser.add_subparsers(dest="cmd", required=True)

    wifi = sub.add_parser("wifi", help="Save WiFi credentials to NVS and reconnect")
    wifi.add_argument("--ssid", required=True)
    wifi.add_argument("--password", default="")
    wifi.add_argument("--timeout", type=float, default=15.0, help="Seconds to wait for WiFi/TCP after configuring. Default: 15s")
    wifi.set_defaults(func=cmd_wifi)

    mqtt = sub.add_parser("mqtt", help="Save MQTT broker URI to NVS and reconnect MQTT")
    mqtt.add_argument("--broker", required=True, help="Broker URI or host[:port], for example mqtt://192.168.0.10:1883")
    mqtt.add_argument("--timeout", type=float, default=8.0, help="Seconds to wait for status after configuring. Default: 8s")
    mqtt.set_defaults(func=cmd_mqtt)

    status = sub.add_parser("status", help="Print system status JSON")
    status.add_argument("--timeout", type=float, default=15.0, help="Seconds to wait for WiFi/TCP readiness. Default: 15s")
    status.add_argument("--no-wait-wifi", action="store_true", help="Return the first status response without waiting for WiFi/TCP")
    status.set_defaults(func=cmd_status)

    play = sub.add_parser("play", help="Play /sdcard/audio/<scene>/<seq>.wav")
    play.add_argument("--scene", required=True)
    play.add_argument("--seq", required=True, help="File stem under the scene folder. Numeric 1 also resolves to 001.wav.")
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
