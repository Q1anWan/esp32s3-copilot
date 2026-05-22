#!/usr/bin/env python3
"""Load the shared TTS voice package for PC-side simulation tools.

The package is produced by the sibling silent_failure_tts repository. This
module intentionally uses only the Python standard library so the Windows
experiment PC does not need extra dependencies for simulation.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path


DEFAULT_PACKAGE_NAME = "common_voice_test_assets_20260522"
COMMON_MD_NAME = "播放文本_compact_tts_scene_seq_common.md"


@dataclass(frozen=True)
class VoiceEntry:
    event_id: str
    scene: str
    seq: str
    refer_time: str
    text: str
    wav_path: Path | None = None

    @property
    def audio_id(self) -> str:
        return f"{self.scene}/{self.seq}"


@dataclass(frozen=True)
class VoicePackage:
    package_dir: Path
    source_md: Path
    audio_root: Path | None
    entries: list[VoiceEntry]

    @property
    def by_audio_id(self) -> dict[str, VoiceEntry]:
        result: dict[str, VoiceEntry] = {}
        for entry in self.entries:
            result.setdefault(entry.audio_id, entry)
        return result

    def find(self, scene: str, seq: str) -> VoiceEntry | None:
        lookup = self.by_audio_id
        key = f"{scene}/{seq}"
        if key in lookup:
            return lookup[key]
        if seq.isdigit():
            for alt in (str(int(seq)), f"{int(seq):03d}"):
                key = f"{scene}/{alt}"
                if key in lookup:
                    return lookup[key]
        return None

    def entries_for_event(self, event_id: str) -> list[VoiceEntry]:
        return [entry for entry in self.entries if entry.event_id == event_id]


def find_default_package_dir() -> Path | None:
    here = Path(__file__).resolve()
    candidates = [
        here.parents[1] / "dist" / DEFAULT_PACKAGE_NAME,
        here.parents[2] / "silent_failure_tts" / "dist" / DEFAULT_PACKAGE_NAME,
        Path.cwd() / "dist" / DEFAULT_PACKAGE_NAME,
    ]
    for path in candidates:
        if (path / "source" / COMMON_MD_NAME).exists():
            return path
    return None


def load_voice_package(
    package_dir: Path | None = None,
    source_md: Path | None = None,
    audio_root: Path | None = None,
) -> VoicePackage:
    if package_dir is None:
        package_dir = find_default_package_dir()
    if package_dir is None and source_md is None:
        raise FileNotFoundError(
            "voice package not found; pass --voice-package or --voice-md"
        )

    if package_dir is None:
        package_dir = source_md.parent.parent if source_md else Path(".")
    package_dir = package_dir.expanduser().resolve()

    if source_md is None:
        source_md = package_dir / "source" / COMMON_MD_NAME
    source_md = source_md.expanduser().resolve()
    if not source_md.exists():
        raise FileNotFoundError(source_md)

    if audio_root is None:
        candidate = package_dir / "audio"
        audio_root = candidate if candidate.exists() else None
    elif audio_root:
        audio_root = audio_root.expanduser().resolve()

    manifest_paths = []
    if audio_root is not None:
        manifest_paths.append(audio_root / "manifest.json")
    manifest_paths.append(package_dir / "audio" / "manifest.json")
    manifest_wavs = load_manifest_wavs(manifest_paths, audio_root)

    entries = []
    for entry in parse_voice_markdown(source_md):
        wav_path = manifest_wavs.get(entry.audio_id)
        if wav_path is None and audio_root is not None:
            candidate = audio_root / entry.scene / f"{entry.seq}.wav"
            wav_path = candidate if candidate.exists() else candidate
        entries.append(
            VoiceEntry(
                event_id=entry.event_id,
                scene=entry.scene,
                seq=entry.seq,
                refer_time=entry.refer_time,
                text=entry.text,
                wav_path=wav_path,
            )
        )

    return VoicePackage(
        package_dir=package_dir,
        source_md=source_md,
        audio_root=audio_root,
        entries=entries,
    )


def load_manifest_wavs(paths: list[Path], audio_root: Path | None) -> dict[str, Path]:
    for path in paths:
        if not path.exists():
            continue
        data = json.loads(path.read_text(encoding="utf-8"))
        root = audio_root if audio_root is not None else path.parent
        result: dict[str, Path] = {}
        for item in data.get("entries", []):
            entry_id = str(item.get("entry_id") or "")
            wav_path = str(item.get("wav_path") or "")
            if entry_id and wav_path:
                result[entry_id] = root / wav_path
        return result
    return {}


def parse_voice_markdown(path: Path) -> list[VoiceEntry]:
    entries: list[VoiceEntry] = []
    current_event = ""
    in_fence = False
    current: dict[str, str] | None = None

    def finish_current() -> None:
        nonlocal current
        if not current:
            return
        if current.get("SCENE") and current.get("SEQ") and current.get("text"):
            entries.append(
                VoiceEntry(
                    event_id=current_event,
                    scene=current["SCENE"],
                    seq=current["SEQ"],
                    refer_time=current.get("refer_time", ""),
                    text=current["text"],
                )
            )
        current = None

    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.rstrip()
        event_match = re.match(r"^##\s+event:\s*(.+?)\s*$", line)
        if event_match and not in_fence:
            current_event = event_match.group(1).strip()
            continue
        if in_fence and line.strip() == "```":
            finish_current()
            in_fence = False
            continue
        if not in_fence and re.match(r"^```\s*(yaml|yml)?\s*$", line, re.I):
            in_fence = True
            current = None
            continue
        if not in_fence:
            continue

        stripped = line.strip()
        if stripped.startswith("- "):
            finish_current()
            current = {}
            stripped = stripped[2:].strip()
            if stripped:
                parse_key_value(stripped, current)
            continue
        if current is not None:
            parse_key_value(stripped, current)

    finish_current()
    if not entries:
        raise ValueError(f"{path}: no voice entries found")
    return entries


def parse_key_value(text: str, target: dict[str, str]) -> None:
    if ":" not in text:
        return
    key, value = text.split(":", 1)
    key = key.strip()
    if key not in {"SCENE", "SEQ", "refer_time", "text"}:
        return
    target[key] = parse_scalar(value.strip())


def parse_scalar(value: str) -> str:
    if value in {"", "null", "Null", "NULL"}:
        return ""
    if len(value) >= 2 and value[0] == value[-1] == "'":
        return value[1:-1].replace("''", "'")
    if len(value) >= 2 and value[0] == value[-1] == '"':
        return bytes(value[1:-1], "utf-8").decode("unicode_escape")
    return value
