#!/usr/bin/env python3
"""Simulate the SILAB-side logic decision program.

The real logic program will read SILAB state, decide trigger/scene/seq, then
send the same decision to the small robot Bridge Host and HRT Host.

Robot/Bridge frame:
    TRIGGER;TIMESTAMP;SCENE;SEQ;SPEED<LF>

HRT frame:
    TRIGGER,TIMESTAMP,SCENE,SEQ,SPEED;
"""

from __future__ import annotations

import argparse
import socket
import sys
import time
from dataclasses import dataclass
from pathlib import Path

from voice_package import VoiceEntry, VoicePackage, load_voice_package


@dataclass(frozen=True)
class PatternStep:
    trigger: float
    seconds: float


class TcpSink:
    def __init__(self, name: str, host: str, port: int, timeout: float) -> None:
        self.name = name
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock: socket.socket | None = None

    def connect(self) -> None:
        if self.sock is not None:
            return
        self.sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        print(f"[{self.name}] connected {self.host}:{self.port}", flush=True)

    def send(self, payload: str) -> None:
        if self.sock is None:
            self.connect()
        assert self.sock is not None
        self.sock.sendall(payload.encode("utf-8"))

    def close(self) -> None:
        if self.sock is None:
            return
        try:
            self.sock.close()
        finally:
            self.sock = None


def parse_pattern(text: str) -> list[PatternStep]:
    steps: list[PatternStep] = []
    for item in text.split(","):
        item = item.strip()
        if not item:
            continue
        if ":" not in item:
            raise ValueError(f"bad pattern item {item!r}; expected trigger:seconds")
        trigger_text, seconds_text = item.split(":", 1)
        trigger = float(trigger_text.strip())
        seconds = float(seconds_text.strip())
        if seconds < 0:
            raise ValueError("pattern seconds must be >= 0")
        steps.append(PatternStep(trigger=trigger, seconds=seconds))
    if not steps:
        raise ValueError("pattern is empty")
    return steps


def select_entries(package: VoicePackage, args: argparse.Namespace) -> list[VoiceEntry]:
    if args.entry:
        selected: list[VoiceEntry] = []
        for item in args.entry:
            if "/" not in item:
                raise ValueError(f"--entry must be SCENE/SEQ: {item}")
            scene, seq = item.split("/", 1)
            entry = package.find(scene, seq)
            if entry is None:
                raise ValueError(f"voice entry not found: {item}")
            selected.append(entry)
        return selected

    if args.event:
        event_entries = package.entries_for_event(args.event)
        if not event_entries:
            raise ValueError(f"event not found in voice package: {args.event}")
        if args.all_event:
            return event_entries
        index = max(1, args.index)
        if index > len(event_entries):
            raise ValueError(f"--index {index} out of range for {args.event} ({len(event_entries)} entries)")
        return [event_entries[index - 1]]

    if args.common:
        common_entries = [entry for entry in package.by_audio_id.values() if entry.scene == "common"]
        if not common_entries:
            raise ValueError("no common entries found")
        return common_entries if args.all_event else [common_entries[0]]

    return [package.entries[0]]


def frame_timestamp() -> str:
    return f"{time.time():.3f}".rstrip("0").rstrip(".")


def format_number(value: float) -> str:
    if abs(value - round(value)) < 0.0001:
        return str(int(round(value)))
    return f"{value:.6f}".rstrip("0").rstrip(".")


def bridge_frame(entry: VoiceEntry, trigger: float, timestamp: str, speed: float) -> str:
    return f"{format_number(trigger)};{timestamp};{entry.scene};{entry.seq};{format_number(speed)}\n"


def hrt_frame(entry: VoiceEntry, trigger: float, timestamp: str, speed: float) -> str:
    return f"{format_number(trigger)},{timestamp},{entry.scene},{entry.seq},{format_number(speed)};"


def run(args: argparse.Namespace) -> None:
    package = load_voice_package(
        package_dir=args.voice_package,
        source_md=args.voice_md,
        audio_root=args.audio_root,
    )
    entries = select_entries(package, args)
    pattern = parse_pattern(args.pattern)
    print(
        f"[voice] loaded entries={len(package.entries)} selected={len(entries)} source={package.source_md}",
        flush=True,
    )

    bridge = None if args.no_bridge else TcpSink("robot", args.bridge_host, args.bridge_port, args.timeout)
    hrt = None if args.no_hrt else TcpSink("hrt", args.hrt_host, args.hrt_port, args.timeout)
    period_s = 1.0 / args.rate_hz

    try:
        if bridge and not args.dry_run:
            bridge.connect()
        if hrt and not args.dry_run:
            hrt.connect()

        sample_no = 0
        for cycle in range(1, args.cycles + 1):
            for entry in entries:
                print(
                    f"[decision] cycle={cycle} target={entry.audio_id} "
                    f"refer_time={entry.refer_time or '-'} text={entry.text}",
                    flush=True,
                )
                for step in pattern:
                    count = max(1, int(round(step.seconds * args.rate_hz))) if step.seconds > 0 else 0
                    for _ in range(count):
                        sample_no += 1
                        ts = frame_timestamp()
                        speed = args.active_speed if step.trigger >= args.threshold else args.idle_speed
                        b_frame = bridge_frame(entry, step.trigger, ts, speed)
                        h_frame = hrt_frame(entry, step.trigger, ts, speed)
                        if args.verbose or args.dry_run:
                            print(
                                f"[sample {sample_no:04d}] bridge={b_frame.rstrip()} hrt={h_frame}",
                                flush=True,
                            )
                        start = time.monotonic()
                        if not args.dry_run:
                            if bridge:
                                bridge.send(b_frame)
                            if hrt:
                                hrt.send(h_frame)
                        elapsed = time.monotonic() - start
                        sleep_s = period_s - elapsed
                        if sleep_s > 0 and not args.dry_run:
                            time.sleep(sleep_s)
                if args.entry_gap > 0 and not args.dry_run:
                    time.sleep(args.entry_gap)
            if cycle < args.cycles and args.cycle_gap > 0 and not args.dry_run:
                time.sleep(args.cycle_gap)
        print(f"[done] samples={sample_no}", flush=True)
    finally:
        if bridge:
            bridge.close()
        if hrt:
            hrt.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Simulate the SILAB logic decision TCP client.")
    parser.add_argument("--bridge-host", default="127.0.0.1", help="Robot/Bridge Host IP. Default: 127.0.0.1")
    parser.add_argument("--bridge-port", type=int, default=7777, help="Robot/Bridge Host port. Default: 7777")
    parser.add_argument("--hrt-host", default="127.0.0.1", help="HRT Host IP. Default: 127.0.0.1")
    parser.add_argument("--hrt-port", type=int, default=9001, help="HRT Host port. Default: 9001")
    parser.add_argument("--voice-package", type=Path, help="Path to common_voice_test_assets_YYYYMMDD package")
    parser.add_argument("--voice-md", type=Path, help="Path to playback Markdown if no package directory is used")
    parser.add_argument("--audio-root", type=Path, help="Path to package audio directory")
    parser.add_argument("--entry", action="append", default=[], help="Audio id to trigger, e.g. common/left_rear_vehicle_merge")
    parser.add_argument("--event", default="", help="Original event id, e.g. 01_visible_day")
    parser.add_argument("--index", type=int, default=1, help="1-based entry index within --event. Default: 1")
    parser.add_argument("--all-event", action="store_true", help="Send every entry in --event, or every common entry with --common")
    parser.add_argument("--common", action="store_true", help="Select the first common entry when --entry/--event are omitted")
    parser.add_argument("--pattern", default="0:0.5,1:0.4,0:0.5", help="Fixed-rate trigger pattern. Default: 0:0.5,1:0.4,0:0.5")
    parser.add_argument("--rate-hz", type=float, default=10.0, help="Sample send rate. Default: 10")
    parser.add_argument("--cycles", type=int, default=1, help="Number of cycles. Default: 1")
    parser.add_argument("--cycle-gap", type=float, default=0.5, help="Seconds between cycles. Default: 0.5")
    parser.add_argument("--entry-gap", type=float, default=0.25, help="Seconds between selected entries. Default: 0.25")
    parser.add_argument("--threshold", type=float, default=0.5, help="Trigger active threshold for speed selection. Default: 0.5")
    parser.add_argument("--idle-speed", type=float, default=0.0, help="Speed sent while trigger is idle. Default: 0")
    parser.add_argument("--active-speed", type=float, default=1.2, help="Speed sent while trigger is active. Default: 1.2")
    parser.add_argument("--timeout", type=float, default=3.0, help="TCP connect timeout. Default: 3")
    parser.add_argument("--no-bridge", action="store_true", help="Do not send robot/Bridge frames")
    parser.add_argument("--no-hrt", action="store_true", help="Do not send HRT frames")
    parser.add_argument("--dry-run", action="store_true", help="Print frames without connecting")
    parser.add_argument("--verbose", action="store_true", help="Print every generated frame")
    return parser


def main() -> int:
    try:
        args = build_parser().parse_args()
        if args.rate_hz <= 0:
            raise ValueError("--rate-hz must be > 0")
        if args.cycles < 1:
            raise ValueError("--cycles must be >= 1")
        run(args)
    except KeyboardInterrupt:
        print("\n[stopped]", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"[error] {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

