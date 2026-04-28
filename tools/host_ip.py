#!/usr/bin/env python3
"""
Resolve the host IPv4 address that the ESP32 should use on the current LAN.

Examples:
    python tools/host_ip.py
    python tools/host_ip.py --target 192.168.0.56
    HOST_IP=$(python tools/host_ip.py)
"""

from __future__ import annotations

import argparse
import re
import socket
import subprocess
import sys


def _ip_from_udp_route(target: str) -> str | None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect((target, 9))
        ip = sock.getsockname()[0]
        if ip and not ip.startswith("127."):
            return ip
        return None
    except OSError:
        return None
    finally:
        sock.close()


def _default_gateway() -> str | None:
    try:
        proc = subprocess.run(
            ["ip", "route", "show", "default"],
            capture_output=True,
            text=True,
            check=False,
        )
    except FileNotFoundError:
        return None

    match = re.search(r"\bvia\s+(\d+\.\d+\.\d+\.\d+)\b", proc.stdout)
    return match.group(1) if match else None


def _first_global_ipv4() -> str | None:
    try:
        proc = subprocess.run(
            ["ip", "-4", "-o", "addr", "show", "scope", "global"],
            capture_output=True,
            text=True,
            check=False,
        )
    except FileNotFoundError:
        return None

    for line in proc.stdout.splitlines():
        match = re.search(r"\binet\s+(\d+\.\d+\.\d+\.\d+)/", line)
        if match:
            return match.group(1)
    return None


def detect_host_ip(target: str | None = None) -> str:
    candidates: list[str] = []
    if target:
        candidates.append(target)

    gateway = _default_gateway()
    if gateway:
        candidates.append(gateway)

    candidates.extend(["1.1.1.1", "8.8.8.8"])

    seen: set[str] = set()
    for candidate in candidates:
        if candidate in seen:
            continue
        seen.add(candidate)
        ip = _ip_from_udp_route(candidate)
        if ip:
            return ip

    fallback = _first_global_ipv4()
    if fallback:
        return fallback

    raise RuntimeError("unable to determine a non-loopback IPv4 address")


def main() -> int:
    parser = argparse.ArgumentParser(description="Print the current host LAN IPv4 address")
    parser.add_argument(
        "--target",
        default="",
        help="Optional ESP32 IP or gateway IP used to select the correct network interface",
    )
    args = parser.parse_args()

    try:
        print(detect_host_ip(args.target or None))
        return 0
    except RuntimeError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
