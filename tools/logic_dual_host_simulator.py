#!/usr/bin/env python3
"""Run two TCP hosts that emulate the project robot/Bridge and HRT endpoints.

Robot/Bridge host protocol:
    TRIGGER;TIMESTAMP;SCENE;SEQ;SPEED<LF>

HRT host protocol:
    trigger,timestamp,scene,seq,speed;

The script uses only the Python standard library so it can run on Windows
or Ubuntu after installing Python 3.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import socket
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable

from voice_package import VoicePackage, load_voice_package


BRIDGE_DEFAULT_PORT = 7777
HRT_DEFAULT_PORT = 9001


class AudioPlayer:
    def __init__(self, enabled: bool = True, command: str = "auto") -> None:
        self.enabled = enabled
        self.command = command
        self._lock = threading.Lock()
        self._proc: subprocess.Popen | None = None

    def play(self, path: Path) -> bool:
        if not self.enabled:
            return False
        if not path.exists():
            log(f"[AUDIO] missing wav {path}")
            return False
        if os.name == "nt":
            threading.Thread(target=self._play_windows, args=(path,), daemon=True).start()
            return True

        command = self._select_command()
        if not command:
            log("[AUDIO] no local player found; install pipewire-utils, pulseaudio-utils, alsa-utils, or ffmpeg")
            return False

        args = self._command_args(command, path)
        with self._lock:
            self._stop_locked()
            try:
                self._proc = subprocess.Popen(
                    args,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            except OSError as exc:
                log(f"[AUDIO] player failed {command}: {exc}")
                return False
        log(f"[AUDIO] playing {path.name} via {command}")
        return True

    def stop(self) -> None:
        with self._lock:
            self._stop_locked()

    def _stop_locked(self) -> None:
        if self._proc is None:
            return
        if self._proc.poll() is None:
            self._proc.terminate()
        self._proc = None

    def _select_command(self) -> str:
        if self.command and self.command != "auto":
            return self.command if shutil.which(self.command) else ""
        for candidate in ("pw-play", "paplay", "aplay", "ffplay", "afplay"):
            if shutil.which(candidate):
                return candidate
        return ""

    @staticmethod
    def _command_args(command: str, path: Path) -> list[str]:
        path_text = str(path)
        name = Path(command).name
        if name == "aplay":
            return [command, "-q", path_text]
        if name == "ffplay":
            return [command, "-nodisp", "-autoexit", "-loglevel", "quiet", path_text]
        return [command, path_text]

    @staticmethod
    def _play_windows(path: Path) -> None:
        try:
            import winsound

            winsound.PlaySound(str(path), winsound.SND_FILENAME | winsound.SND_ASYNC)
            log(f"[AUDIO] playing {path.name} via winsound")
        except Exception as exc:
            log(f"[AUDIO] winsound failed: {exc}")


@dataclass(frozen=True)
class ParsedFrame:
    trigger: float
    timestamp: str
    scene: str
    seq: str
    speed: float

    @property
    def active(self) -> bool:
        return self.trigger >= 0.5


def now_text() -> str:
    return datetime.now().strftime("%H:%M:%S")


def log(message: str) -> None:
    print(f"{now_text()} {message}", flush=True)


def parse_bridge_frame(frame: str) -> ParsedFrame:
    fields = [part.strip() for part in frame.strip().split(";")]
    if len(fields) != 5:
        raise ValueError(f"Bridge frame needs 5 fields, got {len(fields)}")
    return ParsedFrame(
        trigger=float(fields[0]),
        timestamp=fields[1],
        scene=_require_text(fields[2], "scene"),
        seq=_require_text(fields[3], "seq"),
        speed=float(fields[4]),
    )


def parse_hrt_frame(frame: str) -> ParsedFrame:
    fields = [part.strip() for part in frame.strip().split(",")]
    if len(fields) != 5:
        raise ValueError(f"HRT frame needs 5 fields, got {len(fields)}")
    return ParsedFrame(
        trigger=float(fields[0]),
        timestamp=fields[1],
        scene=_require_text(fields[2], "scene"),
        seq=_require_text(fields[3], "seq"),
        speed=float(fields[4]),
    )


def _require_text(value: str, name: str) -> str:
    if not value:
        raise ValueError(f"{name} is empty")
    return value


def extract_line_frame(buffer: bytearray) -> bytes | None:
    positions = [pos for pos in (buffer.find(b"\n"), buffer.find(b"\r")) if pos >= 0]
    if not positions:
        return None
    idx = min(positions)
    frame = bytes(buffer[:idx])
    del buffer[: idx + 1]
    while buffer and buffer[0] in (10, 13):
        del buffer[0]
    return frame


def extract_semicolon_frame(buffer: bytearray) -> bytes | None:
    idx = buffer.find(b";")
    if idx < 0:
        return None
    frame = bytes(buffer[:idx])
    del buffer[: idx + 1]
    return frame


@dataclass(frozen=True)
class HostConfig:
    name: str
    bind_host: str
    port: int
    parser: Callable[[str], ParsedFrame]
    extractor: Callable[[bytearray], bytes | None]
    ack: bool


class TcpHost(threading.Thread):
    def __init__(
        self,
        config: HostConfig,
        stop_event: threading.Event,
        voice_package: VoicePackage | None = None,
        audio_player: AudioPlayer | None = None,
    ) -> None:
        super().__init__(daemon=True)
        self.config = config
        self.stop_event = stop_event
        self.voice_package = voice_package
        self.audio_player = audio_player
        self._server: socket.socket | None = None
        self._last_active = False
        self._lock = threading.Lock()

    def run(self) -> None:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
                self._server = server
                server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                server.bind((self.config.bind_host, self.config.port))
                server.listen(8)
                server.settimeout(0.5)
                log(f"[{self.config.name}] listening on {self.config.bind_host}:{self.config.port}")
                while not self.stop_event.is_set():
                    try:
                        conn, addr = server.accept()
                    except socket.timeout:
                        continue
                    except OSError:
                        break
                    threading.Thread(
                        target=self._handle_client,
                        args=(conn, addr),
                        daemon=True,
                    ).start()
        except OSError as exc:
            log(f"[{self.config.name}] host failed: {exc}")
            self.stop_event.set()

    def stop(self) -> None:
        if self._server is not None:
            try:
                self._server.close()
            except OSError:
                pass

    def _handle_client(self, conn: socket.socket, addr: tuple[str, int]) -> None:
        peer = f"{addr[0]}:{addr[1]}"
        log(f"[{self.config.name}] client connected {peer}")
        buffer = bytearray()
        with conn:
            conn.settimeout(0.5)
            while not self.stop_event.is_set():
                try:
                    data = conn.recv(4096)
                except socket.timeout:
                    continue
                except OSError as exc:
                    log(f"[{self.config.name}] recv error from {peer}: {exc}")
                    break
                if not data:
                    break
                buffer.extend(data)
                while True:
                    raw_frame = self.config.extractor(buffer)
                    if raw_frame is None:
                        break
                    if not raw_frame.strip():
                        continue
                    self._process_frame(conn, peer, raw_frame)
        log(f"[{self.config.name}] client disconnected {peer}")

    def _process_frame(self, conn: socket.socket, peer: str, raw_frame: bytes) -> None:
        text = raw_frame.decode("utf-8", errors="replace")
        try:
            parsed = self.config.parser(text)
        except ValueError as exc:
            log(f"[{self.config.name}] BAD from {peer}: {text!r} reason={exc}")
            if self.config.ack:
                self._send_ack(conn, ok=False, reason=str(exc))
            return

        with self._lock:
            rising = parsed.active and not self._last_active
            self._last_active = parsed.active

        state = "RISING" if rising else ("ACTIVE" if parsed.active else "IDLE")
        log(
            f"[{self.config.name}] {state} trigger={parsed.trigger:g} "
            f"ts={parsed.timestamp} scene={parsed.scene} seq={parsed.seq} "
            f"speed={parsed.speed:g} raw={text!r}"
        )
        if rising and self.voice_package is not None:
            self._log_voice_lookup(parsed)
        if self.config.ack:
            self._send_ack(conn, ok=True, rising=rising)

    def _log_voice_lookup(self, parsed: ParsedFrame) -> None:
        if self.voice_package is None:
            return
        entry = self.voice_package.find(parsed.scene, parsed.seq)
        if entry is None:
            log(f"[{self.config.name}] robot voice MISS {parsed.scene}/{parsed.seq}")
            return
        wav = str(entry.wav_path) if entry.wav_path is not None else "-"
        exists = bool(entry.wav_path and entry.wav_path.exists())
        log(
            f"[{self.config.name}] robot would_play {entry.audio_id}.wav "
            f"exists={str(exists).lower()} refer_time={entry.refer_time or '-'} "
            f"text={entry.text}"
        )
        log(f"[{self.config.name}] robot wav_path {wav}")
        if exists and self.audio_player is not None and entry.wav_path is not None:
            played = self.audio_player.play(entry.wav_path)
            if not played:
                log(f"[{self.config.name}] robot local playback skipped")

    def _send_ack(self, conn: socket.socket, **payload: object) -> None:
        body = {"host": self.config.name, **payload}
        try:
            conn.sendall((json.dumps(body, separators=(",", ":")) + "\n").encode("utf-8"))
        except OSError:
            pass


def send_sample(host: str, port: int, payload: bytes) -> None:
    with socket.create_connection((host, port), timeout=2.0) as sock:
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.sendall(payload)


def find_free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Start two TCP hosts that simulate Bridge and HRT."
    )
    parser.add_argument("--bridge-host", default="0.0.0.0", help="Bridge bind host. Default: 0.0.0.0")
    parser.add_argument("--bridge-port", type=int, default=BRIDGE_DEFAULT_PORT, help="Bridge bind port. Default: 7777")
    parser.add_argument("--hrt-host", default="0.0.0.0", help="HRT bind host. Default: 0.0.0.0")
    parser.add_argument("--hrt-port", type=int, default=HRT_DEFAULT_PORT, help="HRT bind port. Default: 9001")
    parser.add_argument("--voice-package", type=Path, help="Path to common_voice_test_assets_YYYYMMDD package")
    parser.add_argument("--voice-md", type=Path, help="Path to playback Markdown if not using a package directory")
    parser.add_argument("--audio-root", type=Path, help="Path to package audio directory")
    parser.add_argument("--no-voice-package", action="store_true", help="Disable robot voice-package lookup")
    parser.add_argument("--no-play-audio", action="store_true", help="Only log would_play; do not play WAV on this PC")
    parser.add_argument("--audio-player", default="auto", help="Local WAV player command: auto, pw-play, paplay, aplay, ffplay, afplay. Default: auto")
    parser.add_argument("--ack", action="store_true", help="Send JSON ACK lines back to clients.")
    parser.add_argument("--self-test", action="store_true", help="Run local sample clients and exit.")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    stop_event = threading.Event()

    if args.self_test:
        args.bridge_host = "127.0.0.1"
        args.hrt_host = "127.0.0.1"
        args.bridge_port = find_free_port()
        args.hrt_port = find_free_port()

    voice_package = None
    if not args.no_voice_package:
        try:
            voice_package = load_voice_package(
                package_dir=args.voice_package,
                source_md=args.voice_md,
                audio_root=args.audio_root,
            )
            log(
                f"[ROBOT_HOST] voice package loaded entries={len(voice_package.entries)} "
                f"source={voice_package.source_md}"
            )
        except Exception as exc:
            log(f"[ROBOT_HOST] voice package unavailable: {exc}")

    audio_player = AudioPlayer(enabled=not args.no_play_audio, command=args.audio_player)
    if args.no_play_audio:
        log("[ROBOT_HOST] local WAV playback disabled")

    hosts = [
        TcpHost(
            HostConfig(
                name="ROBOT_HOST",
                bind_host=args.bridge_host,
                port=args.bridge_port,
                parser=parse_bridge_frame,
                extractor=extract_line_frame,
                ack=args.ack,
            ),
            stop_event,
            voice_package,
            audio_player,
        ),
        TcpHost(
            HostConfig(
                name="HRT_HOST",
                bind_host=args.hrt_host,
                port=args.hrt_port,
                parser=parse_hrt_frame,
                extractor=extract_semicolon_frame,
                ack=args.ack,
            ),
            stop_event,
        ),
    ]

    for host in hosts:
        host.start()

    log("")
    log("Logic program should connect to:")
    log(f"  Robot/Bridge host: {display_target(args.bridge_host, args.bridge_port)}")
    log(f"  HRT host:          {display_target(args.hrt_host, args.hrt_port)}")
    log("Example Robot payload: 1;1779235200;common;left_rear_vehicle_merge;1.2<LF>")
    log("Example HRT payload:    1,1779235200,boot,1,1.2;")
    log("")

    try:
        if args.self_test:
            time.sleep(0.3)
            scene, seq = "boot", "1"
            if voice_package is not None:
                sample_entry = voice_package.entries[0]
                scene, seq = sample_entry.scene, sample_entry.seq
            send_sample(args.bridge_host, args.bridge_port, f"0;1779235199;{scene};{seq};0\n".encode())
            send_sample(args.bridge_host, args.bridge_port, f"1;1779235200;{scene};{seq};1.2\n".encode())
            send_sample(args.hrt_host, args.hrt_port, f"1,1779235200,{scene},{seq},1.2;".encode())
            time.sleep(0.5)
            log("self-test finished")
            return 0

        while not stop_event.is_set():
            time.sleep(0.2)
    except KeyboardInterrupt:
        log("stopping...")
    finally:
        stop_event.set()
        audio_player.stop()
        for host in hosts:
            host.stop()
        for host in hosts:
            host.join(timeout=1.0)
    return 0


def display_target(bind_host: str, port: int) -> str:
    if bind_host == "0.0.0.0":
        return f"<this-PC-LAN-IP>:{port} (host listens on 0.0.0.0)"
    return f"{bind_host}:{port}"


if __name__ == "__main__":
    raise SystemExit(main())
