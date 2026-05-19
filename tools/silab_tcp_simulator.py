#!/usr/bin/env python3
"""Simulate a SILAB TCP sender for Copilot audio trigger tests.

The simulator acts as the SILAB TCP client and connects to the experiment PC
bridge GUI:

  simulator --TCP--> tools/silab_mqtt_bridge_gui.py --direct TCP--> ESP32

MQTT is still used by the GUI for ESP32 status and as a playback fallback.

Default packet format, one line per fixed-rate sample:

  trigger<TAB>scene<TAB>seq<LF>

The bridge should play only when trigger rises from 0 to 1. Repeated trigger=1
samples are intentionally sent so the edge latch can be tested.
"""

from __future__ import annotations

import argparse
import json
import select
import socket
import sys
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class Sample:
    trigger: float
    scene: str
    seq: int

    def line(self, delimiter: str) -> str:
        return f"{self.trigger:g}{delimiter}{self.scene}{delimiter}{self.seq}\n"


def parse_pattern(text: str, scene: str, seq: int) -> list[tuple[float, Sample]]:
    """Parse a compact pattern string.

    Format:
      trigger:seconds[,trigger:seconds...]

    Example:
      0:1,1:2,0:1,1:0.5,0:1
    """

    steps: list[tuple[float, Sample]] = []
    for item in text.split(","):
        item = item.strip()
        if not item:
            continue
        if ":" not in item:
            raise ValueError(f"Bad pattern item {item!r}; expected trigger:seconds")
        trigger_text, seconds_text = item.split(":", 1)
        trigger = float(trigger_text.strip())
        seconds = float(seconds_text.strip())
        if seconds < 0:
            raise ValueError("Pattern seconds must be >= 0")
        steps.append((seconds, Sample(trigger=trigger, scene=scene, seq=seq)))
    if not steps:
        raise ValueError("Pattern is empty")
    return steps


def default_steps(args: argparse.Namespace) -> list[tuple[float, Sample]]:
    idle = Sample(trigger=args.idle_trigger, scene=args.scene, seq=args.seq)
    active = Sample(trigger=args.active_trigger, scene=args.scene, seq=args.seq)
    return [
        (args.pre_idle, idle),
        (args.hold, active),
        (args.post_idle, idle),
    ]


def expand_steps(
    steps: list[tuple[float, Sample]],
    rate_hz: float,
) -> list[Sample]:
    if rate_hz <= 0:
        raise ValueError("rate-hz must be > 0")
    samples: list[Sample] = []
    for seconds, sample in steps:
        count = max(1, int(round(seconds * rate_hz))) if seconds > 0 else 0
        samples.extend(sample for _ in range(count))
    return samples


def drain_acks(sock: socket.socket, buffer: bytearray, timeout_s: float) -> list[str]:
    deadline = time.monotonic() + timeout_s
    lines: list[str] = []
    while time.monotonic() < deadline:
        remaining = max(0.0, deadline - time.monotonic())
        ready, _, _ = select.select([sock], [], [], min(0.05, remaining))
        if not ready:
            continue
        data = sock.recv(4096)
        if not data:
            break
        for byte in data:
            if byte in (10, 13):
                if buffer:
                    lines.append(buffer.decode("utf-8", errors="replace"))
                    buffer.clear()
            else:
                buffer.append(byte)
    return lines


def send_samples(sock: socket.socket, args: argparse.Namespace, samples: list[Sample], cycle_no: int) -> None:
    period_s = 1.0 / args.rate_hz
    ack_buffer = bytearray()
    for index, sample in enumerate(samples, start=1):
        line = sample.line(args.delimiter)
        raw = line.encode("utf-8")
        send_t = time.monotonic()
        sock.sendall(raw)
        if args.verbose:
            printable = line.rstrip("\n").replace("\t", "\\t")
            print(f"[send] cycle={cycle_no} sample={index}/{len(samples)} {printable}", flush=True)
        if args.read_ack:
            for ack in drain_acks(sock, ack_buffer, args.ack_window):
                print_ack(ack)
        elapsed = time.monotonic() - send_t
        sleep_s = period_s - elapsed
        if sleep_s > 0:
            time.sleep(sleep_s)
    if args.read_ack:
        for ack in drain_acks(sock, ack_buffer, args.ack_window):
            print_ack(ack)


def print_ack(text: str) -> None:
    try:
        payload = json.loads(text)
        print("[ack]", json.dumps(payload, ensure_ascii=False, separators=(",", ":")), flush=True)
    except json.JSONDecodeError:
        print("[ack]", text, flush=True)


def run(args: argparse.Namespace) -> None:
    delimiter = args.delimiter
    if delimiter == r"\t":
        args.delimiter = "\t"
    elif delimiter == "tab":
        args.delimiter = "\t"
    elif delimiter == "space":
        args.delimiter = " "
    elif delimiter == "comma":
        args.delimiter = ","

    if args.seq < 1 or args.seq > 65535:
        raise ValueError("seq must be 1..65535")

    steps = parse_pattern(args.pattern, args.scene, args.seq) if args.pattern else default_steps(args)
    samples = expand_steps(steps, args.rate_hz)
    if args.dry_run:
        print(f"[dry-run] {len(samples)} samples at {args.rate_hz:g} Hz")
        for index, sample in enumerate(samples, start=1):
            print(f"{index:04d}: {sample.line(args.delimiter).rstrip()}")
        return

    print(f"[connect] {args.host}:{args.port}", flush=True)
    with socket.create_connection((args.host, args.port), timeout=args.timeout) as sock:
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        print(
            f"[connected] rate={args.rate_hz:g}Hz scene={args.scene} seq={args.seq} "
            f"samples/cycle={len(samples)} cycles={args.cycles}",
            flush=True,
        )
        for cycle_no in range(1, args.cycles + 1):
            if cycle_no > 1 and args.cycle_gap > 0:
                time.sleep(args.cycle_gap)
            send_samples(sock, args, samples, cycle_no)
        print("[done]", flush=True)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Simulate SILAB fixed-rate TCP audio trigger packets.")
    parser.add_argument("--host", default="127.0.0.1", help="Bridge TCP host IP. Default: 127.0.0.1")
    parser.add_argument("--port", type=int, default=7777, help="Bridge TCP port. Default: 7777")
    parser.add_argument("--scene", default="1", help="Scene ID sent by SILAB. Default: 1")
    parser.add_argument("--seq", type=int, default=1, help="Sequence ID sent by SILAB. Default: 1")
    parser.add_argument("--rate-hz", type=float, default=10.0, help="Fixed send rate. Default: 10")
    parser.add_argument("--pre-idle", type=float, default=1.0, help="Seconds of trigger=0 before pulse. Default: 1")
    parser.add_argument("--hold", type=float, default=2.0, help="Seconds of trigger=1 during pulse. Default: 2")
    parser.add_argument("--post-idle", type=float, default=1.0, help="Seconds of trigger=0 after pulse. Default: 1")
    parser.add_argument("--cycles", type=int, default=1, help="Number of pulse cycles. Default: 1")
    parser.add_argument("--cycle-gap", type=float, default=0.0, help="Seconds between cycles. Default: 0")
    parser.add_argument("--idle-trigger", type=float, default=0.0, help="Idle trigger value. Default: 0")
    parser.add_argument("--active-trigger", type=float, default=1.0, help="Active trigger value. Default: 1")
    parser.add_argument(
        "--pattern",
        default="",
        help="Override timing with trigger:seconds steps, e.g. '0:1,1:2,0:1,1:0.5,0:1'",
    )
    parser.add_argument(
        "--delimiter",
        default=r"\t",
        help=r"Field delimiter: \t, tab, space, or comma. Default: \t",
    )
    parser.add_argument("--timeout", type=float, default=5.0, help="TCP connect timeout seconds. Default: 5")
    parser.add_argument("--read-ack", action="store_true", help="Print JSON ACK lines if GUI Send TCP ACK is enabled")
    parser.add_argument("--ack-window", type=float, default=0.03, help="ACK read window after each send. Default: 0.03")
    parser.add_argument("--dry-run", action="store_true", help="Print generated samples without connecting")
    parser.add_argument("--verbose", action="store_true", help="Print every sent sample")
    return parser


def main() -> int:
    try:
        run(build_parser().parse_args())
    except KeyboardInterrupt:
        print("\n[stopped]", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"[error] {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
