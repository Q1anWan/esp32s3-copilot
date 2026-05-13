#!/usr/bin/env python3
"""
ESP32-S3 Copilot Dashboard — servo control, battery monitor, audio playback.

Dependencies: paho-mqtt, numpy, scipy, websockets
System: ffmpeg (for audio file conversion)

Usage:
  python tools/copilot_dashboard.py
  python tools/copilot_dashboard.py --broker 192.168.31.12
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import queue
import struct
import subprocess
import sys
import tempfile
import threading
import time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import numpy as np
import paho.mqtt.client as mqtt
from scipy import signal as scipy_signal
from websockets.asyncio.server import serve as ws_serve
from websockets.exceptions import ConnectionClosed


# ── Defaults ────────────────────────────────────────────────────────────────

DEF_BROKER = os.environ.get("COPILOT_MQTT_BROKER", "localhost")
DEF_PORT   = int(os.environ.get("COPILOT_MQTT_PORT", "1883"))
DEF_TOPIC  = os.environ.get("COPILOT_MQTT_TOPIC", "copilot/s3_copilot/cmd")
DEF_STATUS_TOPIC = os.environ.get("COPILOT_STATUS_TOPIC", "copilot/s3_copilot/status")
DEF_WS_PORT = int(os.environ.get("COPILOT_WS_PORT", "8080"))

SAMPLE_RATE = 16000  # ESP32 expects 16kHz mono s16le PCM
FRAME_MS   = 20
FRAME_SAMPLES = int(SAMPLE_RATE * FRAME_MS / 1000)  # 320 samples per frame
FRAME_BYTES   = FRAME_SAMPLES * 2  # 640 bytes per frame


# ── Audio conversion ────────────────────────────────────────────────────────

def convert_to_pcm16(file_path: str) -> bytes:
    """Convert any audio file to 16kHz mono s16le PCM via ffmpeg. Returns raw PCM bytes."""
    args = [
        "ffmpeg", "-y", "-loglevel", "error",
        "-i", file_path,
        "-f", "s16le",
        "-acodec", "pcm_s16le",
        "-ar", str(SAMPLE_RATE),
        "-ac", "1",
        "pipe:1",
    ]
    proc = subprocess.run(args, capture_output=True)
    if proc.returncode != 0:
        err = proc.stderr.decode(errors="replace")
        raise RuntimeError(f"ffmpeg conversion failed: {err}")
    return proc.stdout


def resample_pcm(pcm: bytes, orig_rate: int) -> bytes:
    """Resample raw s16le mono PCM to 16kHz using scipy."""
    samples = np.frombuffer(pcm, dtype=np.int16).astype(np.float64)
    target_len = int(len(samples) * SAMPLE_RATE / orig_rate)
    resampled = scipy_signal.resample(samples, target_len)
    return resampled.astype(np.int16).tobytes()


def read_wav_pcm(file_path: str) -> bytes:
    """Read a WAV file, resample to 16kHz mono s16le if needed."""
    import wave
    with wave.open(file_path, "rb") as wf:
        nchannels = wf.getnchannels()
        sampwidth = wf.getsampwidth()
        framerate = wf.getframerate()
        nframes   = wf.getnframes()
        raw = wf.readframes(nframes)

    # Convert to int16 numpy
    if sampwidth == 2:
        samples = np.frombuffer(raw, dtype=np.int16)
    elif sampwidth == 1:
        samples = (np.frombuffer(raw, dtype=np.uint8).astype(np.float64) - 128) * 256
        samples = samples.astype(np.int16)
    elif sampwidth == 3:
        # 24-bit: pad to 32-bit then truncate
        padded = np.frombuffer(raw, dtype=np.uint8).astype(np.int32)
        arr = np.zeros(len(raw) // 3, dtype=np.int32)
        arr = padded[0::3] | (padded[1::3].astype(np.int32) << 8) | (padded[2::3].astype(np.int32) << 16)
        arr = np.where(arr >= 0x800000, arr - 0x1000000, arr)
        samples = (arr >> 8).astype(np.int16)
    else:
        raise ValueError(f"Unsupported sample width: {sampwidth}")

    # Convert to mono
    if nchannels == 2:
        samples = (samples[0::2].astype(np.float64) + samples[1::2].astype(np.float64)) / 2.0
        samples = samples.astype(np.int16)
    elif nchannels > 2:
        raise ValueError(f"Too many channels: {nchannels}")

    # Resample if needed
    if framerate != SAMPLE_RATE:
        samples_f = samples.astype(np.float64)
        target_len = int(len(samples_f) * SAMPLE_RATE / framerate)
        samples_f = scipy_signal.resample(samples_f, target_len)
        samples = samples_f.astype(np.int16)

    return samples.tobytes()


# ── WebSocket audio server ──────────────────────────────────────────────────

class AudioServer:
    """Runs a WebSocket server for ESP32 audio streaming."""

    def __init__(self, host: str = "0.0.0.0", port: int = DEF_WS_PORT):
        self.host = host
        self.port = port
        self.pcm_data: bytes = b""
        self.playing = False
        self.stop_requested = False
        self.server_task: asyncio.Task | None = None
        self._lock = threading.Lock()

    def load_file(self, file_path: str) -> float:
        """Load and convert an audio file. Returns duration in seconds."""
        ext = os.path.splitext(file_path)[1].lower()
        if ext in (".wav", ".wave"):
            pcm = read_wav_pcm(file_path)
        else:
            pcm = convert_to_pcm16(file_path)
        with self._lock:
            self.pcm_data = pcm
            self.playing = False
            self.stop_requested = False
        return len(pcm) / (2 * SAMPLE_RATE)  # duration in seconds

    def play(self):
        with self._lock:
            self.playing = True
            self.stop_requested = False

    def stop(self):
        with self._lock:
            self.stop_requested = True

    @property
    def is_playing(self) -> bool:
        return self.playing and not self.stop_requested

    async def _handler(self, ws):
        """Handle a single ESP32 WebSocket connection."""
        print(f"[ws] ESP32 connected from {ws.remote_address}")

        # Read handshake
        msg = await ws.recv()
        try:
            hs = json.loads(msg)
            device_id = hs.get("device_id", "unknown")
            print(f"[ws] Handshake: device={device_id}")
        except Exception:
            device_id = "unknown"

        session_id = f"dashboard-{int(time.time() * 1000)}"
        await ws.send(json.dumps({"type": "started", "session_id": session_id}))
        print(f"[ws] Session started: {session_id}")

        try:
            while True:
                with self._lock:
                    pcm = self.pcm_data
                    playing = self.playing
                    stop = self.stop_requested

                if not playing or len(pcm) == 0:
                    await asyncio.sleep(0.1)
                    continue

                if stop:
                    await ws.send(b"")  # Signal stop
                    with self._lock:
                        self.playing = False
                        self.stop_requested = False
                    break

                # Stream in frames
                offset = 0
                while offset < len(pcm):
                    with self._lock:
                        if self.stop_requested:
                            break
                    chunk = pcm[offset:offset + FRAME_BYTES]
                    if len(chunk) == 0:
                        break
                    await ws.send(chunk)
                    offset += len(chunk)
                    await asyncio.sleep(FRAME_MS / 1000.0)

                with self._lock:
                    self.playing = False
                print(f"[ws] Stream finished ({len(pcm)} bytes)")
                await ws.send(b"")  # Signal end

        except ConnectionClosed:
            print("[ws] ESP32 disconnected")

    async def _run_server(self):
        print(f"[ws] Audio server listening on ws://{self.host}:{self.port}/audio/stream")
        async with ws_serve(self._handler, self.host, self.port) as server:
            await server.serve_forever()

    def start(self):
        def _run():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self._run_server())
        t = threading.Thread(target=_run, daemon=True)
        t.start()


# ── MQTT Client wrapper ─────────────────────────────────────────────────────

class MqttClient:
    def __init__(self, broker: str, port: int, topic: str, status_topic: str):
        self.broker = broker
        self.port = port
        self.topic = topic
        self.status_topic = status_topic
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_message = self._on_message
        self._status_queue: queue.Queue[dict] = queue.Queue()
        self._connected = False
        self._status_callback = None  # callable(dict) for servo_status etc.

    def set_status_callback(self, cb):
        self._status_callback = cb

    def connect(self) -> bool:
        try:
            self.client.connect(self.broker, self.port, keepalive=30)
            self.client.loop_start()
            self.client.subscribe(self.status_topic, qos=1)
            self._connected = True
            return True
        except Exception as e:
            print(f"MQTT connect error: {e}")
            self._connected = False
            return False

    def disconnect(self):
        self.client.loop_stop()
        try:
            self.client.disconnect()
        except Exception:
            pass
        self._connected = False

    def publish(self, payload: dict):
        s = json.dumps(payload, separators=(",", ":"))
        info = self.client.publish(self.topic, s, qos=1)
        info.wait_for_publish(timeout=3)

    def query_status(self) -> dict | None:
        """Query power status and wait for response."""
        self.publish({"type": "status", "query": "power"})
        try:
            return self._status_queue.get(timeout=5)
        except queue.Empty:
            return None

    def _on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            self._status_queue.put(payload)
            if self._status_callback:
                self._status_callback(payload)
        except Exception:
            pass

    @property
    def connected(self) -> bool:
        return self._connected


# ── GUI Dashboard ───────────────────────────────────────────────────────────

class Dashboard(tk.Tk):
    def __init__(self, broker: str, port: int, topic: str, ws_port: int):
        super().__init__()
        self.title("ESP32-S3 Copilot Dashboard")
        self.geometry("680x620")
        self.resizable(False, False)
        self.configure(bg="#1a1a2e")

        # Style
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("TFrame", background="#1a1a2e")
        style.configure("TLabelframe", background="#1a1a2e", foreground="#cccccc",
                        bordercolor="#333355", relief="groove")
        style.configure("TLabelframe.Label", background="#1a1a2e", foreground="#cccccc",
                        font=("Segoe UI", 10, "bold"))
        style.configure("TLabel", background="#1a1a2e", foreground="#cccccc", font=("Segoe UI", 10))
        style.configure("TButton", background="#2a2a4e", foreground="#ffffff",
                        font=("Segoe UI", 10, "bold"), borderwidth=1, padding=(6, 4))
        style.map("TButton", background=[("active", "#3a3a6e")])
        style.configure("TScale", background="#1a1a2e")
        style.configure("TCheckbutton", background="#1a1a2e", foreground="#cccccc")
        style.configure("TEntry", fieldbackground="#2a2a4e", foreground="#ffffff")

        # MQTT client
        self.mqtt = MqttClient(broker, port, topic, DEF_STATUS_TOPIC)
        self.mqtt.set_status_callback(self._on_status)

        # Audio server
        self.audio_server = AudioServer(port=ws_port)
        self.audio_server.start()

        # Variables
        self.pitch_var = tk.DoubleVar(value=0.0)
        self.yaw_var = tk.DoubleVar(value=0.0)
        self.calib_var = tk.BooleanVar(value=False)
        self.manual_var = tk.BooleanVar(value=False)
        self.servo_status_var = tk.StringVar(value="Servo: --")
        self.bat_pct_var = tk.StringVar(value="--")
        self.bat_mv_var = tk.StringVar(value="-- mV")
        self.vbus_var = tk.StringVar(value="-- mV")
        self.sys_var = tk.StringVar(value="-- mV")
        self.temp_var = tk.StringVar(value="-- °C")
        self.chg_var = tk.StringVar(value="--")
        self.conn_var = tk.StringVar(value="Disconnected")
        self.status_var = tk.StringVar(value="Ready")
        self.audio_file_var = tk.StringVar(value="")
        self.audio_dur_var = tk.StringVar(value="")
        self.play_var = tk.StringVar(value="Play")

        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self):
        main_frame = ttk.Frame(self, padding=10)
        main_frame.pack(fill="both", expand=True)

        # ── Row 0: Connection bar ──
        conn_frame = ttk.Frame(main_frame)
        conn_frame.pack(fill="x", pady=(0, 8))

        ttk.Label(conn_frame, text=f"MQTT: {self.mqtt.broker}:{self.mqtt.port} | "
                  f"Topic: {self.mqtt.topic}",
                  font=("Segoe UI", 9)).pack(side="left")

        self.conn_var.set("Disconnected")
        conn_label = ttk.Label(conn_frame, textvariable=self.conn_var,
                               foreground="#ff6666", font=("Segoe UI", 9, "bold"))
        conn_label.pack(side="left", padx=(10, 0))

        self.conn_btn = ttk.Button(conn_frame, text="Connect MQTT", command=self._toggle_mqtt)
        self.conn_btn.pack(side="right")

        # ── Row 1: Servo + Battery ──
        top_row = ttk.Frame(main_frame)
        top_row.pack(fill="x", pady=(0, 8))

        # Servo control frame
        servo_frame = ttk.Labelframe(top_row, text="Servo Control", padding=10)
        servo_frame.pack(side="left", fill="both", expand=True)

        # Pitch
        self.pitch_range_label = ttk.Label(servo_frame, text="Pitch")
        self.pitch_range_label.pack(anchor="w")
        self.pitch_scale = ttk.Scale(servo_frame, from_=15, to=-15,
                                     variable=self.pitch_var,
                                     command=self._on_servo_change)
        self.pitch_scale.pack(fill="x")
        self.pitch_val_label = ttk.Label(servo_frame, textvariable=self.pitch_var,
                                         font=("Segoe UI", 9), anchor="center")
        self.pitch_val_label.pack(fill="x")

        # Yaw
        self.yaw_range_label = ttk.Label(servo_frame, text="Yaw",
                                         font=("Segoe UI", 10))
        self.yaw_range_label.pack(anchor="w", pady=(8, 0))
        self.yaw_scale = ttk.Scale(servo_frame, from_=45, to=-45,
                                   variable=self.yaw_var,
                                   command=self._on_servo_change)
        self.yaw_scale.pack(fill="x")
        self.yaw_val_label = ttk.Label(servo_frame, textvariable=self.yaw_var,
                                        font=("Segoe UI", 9), anchor="center")
        self.yaw_val_label.pack(fill="x")

        # Servo buttons
        servo_btn_frame = ttk.Frame(servo_frame)
        servo_btn_frame.pack(fill="x", pady=(8, 0))
        ttk.Button(servo_btn_frame, text="Center", width=8,
                   command=lambda: self._set_servo(0, 0)).pack(side="left", padx=(0, 4))
        self.calib_btn = ttk.Button(servo_btn_frame, text="Calib: OFF", width=10,
                                    command=self._toggle_calibration)
        self.calib_btn.pack(side="left", padx=4)
        ttk.Button(servo_btn_frame, text="Reset Calib", width=10,
                   command=self._reset_calib).pack(side="left", padx=4)
        self.mode_btn = ttk.Button(servo_btn_frame, text="Mode: Auto", width=11,
                                   command=self._toggle_mode)
        self.mode_btn.pack(side="left", padx=4)

        # Servo status line
        servo_status_frame = ttk.Frame(servo_frame)
        servo_status_frame.pack(fill="x", pady=(6, 0))
        ttk.Label(servo_status_frame, textvariable=self.servo_status_var,
                  font=("Segoe UI", 8), foreground="#888888").pack(side="left")
        ttk.Button(servo_status_frame, text="Query", width=6,
                   command=self._query_servo).pack(side="right")

        # Battery frame
        bat_frame = ttk.Labelframe(top_row, text="Battery Status", padding=10)
        bat_frame.pack(side="right", fill="both", padx=(10, 0))

        # Battery percentage (large display)
        pct_label = ttk.Label(bat_frame, textvariable=self.bat_pct_var,
                              font=("Segoe UI", 36, "bold"),
                              foreground="#66ff66", background="#1a1a2e")
        pct_label.pack()

        # Battery progress bar
        self.bat_bar = ttk.Progressbar(bat_frame, orient="horizontal", length=200,
                                       mode="determinate", maximum=100, value=0)
        self.bat_bar.pack(fill="x", pady=(4, 8))

        # Details grid
        info_frame = ttk.Frame(bat_frame)
        info_frame.pack(fill="x")
        for row, (label, var) in enumerate([
            ("Battery:", self.bat_mv_var),
            ("VBUS:", self.vbus_var),
            ("System:", self.sys_var),
            ("Temp:", self.temp_var),
            ("Status:", self.chg_var),
        ]):
            ttk.Label(info_frame, text=label, font=("Segoe UI", 9),
                      foreground="#888888").grid(row=row, column=0, sticky="e", padx=(0, 6))
            ttk.Label(info_frame, textvariable=var, font=("Segoe UI", 9)).grid(
                row=row, column=1, sticky="w")

        ttk.Button(bat_frame, text="Refresh", command=self._refresh_battery).pack(pady=(8, 0))

        # ── Row 2: Audio playback ──
        audio_frame = ttk.Labelframe(main_frame, text="Audio Playback (WebSocket)", padding=10)
        audio_frame.pack(fill="x", pady=(0, 8))

        file_frame = ttk.Frame(audio_frame)
        file_frame.pack(fill="x")
        ttk.Button(file_frame, text="Select File", command=self._select_audio_file).pack(side="left")
        ttk.Label(file_frame, textvariable=self.audio_file_var,
                  background="#2a2a4e", width=50, anchor="w",
                  font=("Segoe UI", 9)).pack(side="left", fill="x", expand=True, padx=(6, 0))
        ttk.Label(file_frame, textvariable=self.audio_dur_var,
                  font=("Segoe UI", 9), foreground="#888888").pack(side="left", padx=(6, 0))

        ctrl_frame = ttk.Frame(audio_frame)
        ctrl_frame.pack(fill="x", pady=(6, 0))
        self.play_btn = ttk.Button(ctrl_frame, text="Play", command=self._toggle_play, width=10)
        self.play_btn.pack(side="left", padx=(0, 6))
        ttk.Label(ctrl_frame,
                  text=f"WS: :{self.audio_server.port}  |  Format: 16kHz mono s16le PCM  |  Frame: {FRAME_MS}ms/{FRAME_SAMPLES}samples",
                  font=("Segoe UI", 8), foreground="#666666").pack(side="left")

        # ── Row 3: Status bar ──
        status_frame = ttk.Frame(main_frame)
        status_frame.pack(fill="x", side="bottom")
        ttk.Label(status_frame, textvariable=self.status_var,
                  font=("Segoe UI", 9), foreground="#888888").pack(side="left")

        # Quick presets
        preset_frame = ttk.Labelframe(main_frame, text="Quick Presets", padding=8)
        preset_frame.pack(fill="x")
        presets = [
            ("Forward", 0, 0), ("Left", 0, -45), ("Right", 0, 45),
            ("Up", 15, 0), ("Down", -15, 0), ("Look Left", 12, -40),
            ("Look Right", 12, 40), ("Nod Down", -12, 0),
        ]
        for label, pitch, yaw in presets:
            ttk.Button(preset_frame, text=label, width=9,
                       command=lambda p=pitch, y=yaw: self._set_servo(p, y)).pack(
                side="left", padx=3)

    # ── Actions ──

    def _toggle_mqtt(self):
        if self.mqtt.connected:
            self.mqtt.disconnect()
            self.conn_var.set("Disconnected")
            self.conn_btn.configure(text="Connect MQTT")
            self.status_var.set("MQTT disconnected")
        else:
            if self.mqtt.connect():
                self.conn_var.set("Connected")
                self.conn_btn.configure(text="Disconnect MQTT")
                self.status_var.set("MQTT connected")
                # Query initial battery
                self._refresh_battery()
                # Sync calibration state and query servo status
                self.mqtt.publish({"type": "servo", "calibrate": False})
                self.mqtt.publish({"type": "servo", "query": "status"})
            else:
                messagebox.showwarning("MQTT", "Failed to connect to MQTT broker")

    def _on_servo_change(self, *_):
        """Debounced slider callback — MQTT is sent only after the slider stops moving."""
        if not self.mqtt.connected:
            return
        pitch = self.pitch_var.get()
        yaw = self.yaw_var.get()
        self._ensure_manual_mode()
        self.status_var.set(f"Servo → pitch={pitch:.1f} yaw={yaw:.1f}")
        # Cancel any pending send and reschedule
        if hasattr(self, '_servo_debounce_id'):
            self.after_cancel(self._servo_debounce_id)
        self._servo_debounce_id = self.after(80, self._send_servo_debounced)

    def _send_servo_debounced(self):
        """Send the current slider values to the servo via MQTT."""
        pitch = self.pitch_var.get()
        yaw = self.yaw_var.get()
        self.mqtt.publish({"type": "servo", "pitch": round(pitch, 1), "yaw": round(yaw, 1)})

    def _set_servo(self, pitch: float, yaw: float):
        """Preset button — sets slider and sends MQTT immediately (no debounce)."""
        # Cancel any pending debounced send since we're sending now
        if hasattr(self, '_servo_debounce_id'):
            self.after_cancel(self._servo_debounce_id)
        self.pitch_var.set(pitch)
        self.yaw_var.set(yaw)
        if self.mqtt.connected:
            self._ensure_manual_mode()
            self.mqtt.publish({"type": "servo", "pitch": round(pitch, 1), "yaw": round(yaw, 1)})
            self.status_var.set(f"Servo → pitch={pitch:.1f} yaw={yaw:.1f}")

    def _ensure_manual_mode(self):
        """Switch to manual mode locally and notify firmware."""
        if not self.manual_var.get():
            self.manual_var.set(True)
            self._update_mode_button()
            self.mqtt.publish({"type": "servo", "mode": "manual"})

    def _toggle_mode(self):
        new_mode = not self.manual_var.get()
        self.manual_var.set(new_mode)
        if self.mqtt.connected:
            mode_str = "manual" if new_mode else "auto"
            self.mqtt.publish({"type": "servo", "mode": mode_str})
            self.status_var.set(f"Servo mode: {'MANUAL' if new_mode else 'AUTO'}")
        self._update_mode_button()

    def _update_mode_button(self):
        label = "Mode: Manual" if self.manual_var.get() else "Mode: Auto"
        self.mode_btn.configure(text=label)

    def _on_status(self, payload: dict):
        """Handle incoming status messages from firmware."""
        if payload.get("type") == "servo_status":
            pitch = payload.get("pitch_deg", 0)
            yaw = payload.get("yaw_deg", 0)
            manual = payload.get("manual", False)
            calib = payload.get("calib", False)
            pitch_us = payload.get("pitch_us", 0)
            yaw_us = payload.get("yaw_us", 0)
            limits_p = payload.get("limits_pitch", [-15, 15])
            limits_y = payload.get("limits_yaw", [-45, 45])

            # Update slider ranges from firmware limits
            p_min, p_max = float(limits_p[0]), float(limits_p[1])
            y_min, y_max = float(limits_y[0]), float(limits_y[1])
            self.pitch_scale.configure(from_=p_max, to=p_min)
            self.yaw_scale.configure(from_=y_max, to=y_min)
            self.pitch_range_label.configure(text=f"Pitch ({p_min:+.0f}°~{p_max:+.0f}°)")
            self.yaw_range_label.configure(text=f"Yaw ({y_min:+.0f}°~{y_max:+.0f}°)")

            self.servo_status_var.set(
                f"pitch={pitch:.1f}° ({pitch_us}us)  yaw={yaw:.1f}° ({yaw_us}us)  "
                f"{'MANUAL' if manual else 'AUTO'}{' +CALIB' if calib else ''}")
            if self.manual_var.get() != manual:
                self.manual_var.set(manual)
                self._update_mode_button()

    def _toggle_calibration(self):
        new_state = not self.calib_var.get()
        self.calib_var.set(new_state)
        if self.mqtt.connected:
            self.mqtt.publish({"type": "servo", "calibrate": new_state})
        label = "Calib: ON" if new_state else "Calib: OFF"
        self.calib_btn.configure(text=label)
        self.status_var.set(f"Calibration mode: {'ON' if new_state else 'OFF'}")

    def _reset_calib(self):
        if self.mqtt.connected:
            self.mqtt.publish({"type": "servo", "reset": True})
            self.status_var.set("Calibration reset to defaults")

    def _query_servo(self):
        if self.mqtt.connected:
            self.mqtt.publish({"type": "servo", "query": "status"})
            self.status_var.set("Querying servo status...")

    def _refresh_battery(self):
        if not self.mqtt.connected:
            return
        result = self.mqtt.query_status()
        if result and "power" in result:
            p = result["power"]
            pct = p.get("battery_pct", -1)
            self.bat_pct_var.set(f"{pct}%" if pct >= 0 else "N/A")
            self.bat_bar["value"] = max(0, min(100, pct))
            self.bat_mv_var.set(f"{p.get('batt_mv', 0)} mV")
            self.vbus_var.set(f"{p.get('vbus_mv', 0)} mV")
            self.sys_var.set(f"{p.get('sys_mv', 0)} mV")
            self.temp_var.set(f"{p.get('temp_c', 0):.1f} °C")
            if p.get("charging"):
                self.chg_var.set("Charging")
            elif p.get("vbus_in"):
                self.chg_var.set("USB (full)")
            elif p.get("batt_conn"):
                self.chg_var.set("Discharging")
            else:
                self.chg_var.set("No battery")
            self.status_var.set("Battery status refreshed")
        else:
            self.status_var.set("No battery response (PMU not ready?)")

    def _select_audio_file(self):
        path = filedialog.askopenfilename(
            title="Select Audio File",
            filetypes=[("Audio files", "*.wav *.mp3 *.ogg *.flac *.m4a *.aac *.opus"),
                       ("All files", "*.*")])
        if not path:
            return
        try:
            dur = self.audio_server.load_file(path)
            fname = os.path.basename(path)
            self.audio_file_var.set(fname)
            self.audio_dur_var.set(f"{dur:.1f}s")
            self.status_var.set(f"Loaded: {fname} ({dur:.1f}s)")
        except Exception as e:
            messagebox.showerror("Audio Error", str(e))

    def _toggle_play(self):
        if self.audio_server.is_playing:
            self.audio_server.stop()
            self.play_var.set("Play")
            self.play_btn.configure(text="Play")
            if self.mqtt.connected:
                self.mqtt.publish({"type": "emotion", "name": "idle"})
            self.status_var.set("Playback stopped")
        else:
            if len(self.audio_server.pcm_data) == 0:
                messagebox.showinfo("No Audio", "Please select an audio file first.")
                return
            self.audio_server.play()
            self.play_var.set("Stop")
            self.play_btn.configure(text="Stop")
            if self.mqtt.connected:
                # Send speaking emotion to animate mouth
                self.mqtt.publish({"type": "emotion", "name": "speaking", "duration_ms": 0})
            self.status_var.set("Playing (ESP32 will animate mouth)")

    def _on_close(self):
        if self.mqtt.connected:
            self.mqtt.disconnect()
        self.destroy()

    def run_auto_refresh(self):
        """Periodically refresh battery status."""
        def _loop():
            while True:
                time.sleep(10)
                if self.mqtt.connected:
                    self._refresh_battery()
        t = threading.Thread(target=_loop, daemon=True)
        t.start()


def main():
    parser = argparse.ArgumentParser(description="ESP32-S3 Copilot Dashboard")
    parser.add_argument("--broker", default=DEF_BROKER, help="MQTT broker host")
    parser.add_argument("--port", type=int, default=DEF_PORT, help="MQTT broker port")
    parser.add_argument("--topic", default=DEF_TOPIC, help="MQTT command topic")
    parser.add_argument("--ws-port", type=int, default=DEF_WS_PORT, help="WebSocket audio server port")
    parser.add_argument("--no-gui", action="store_true", help="Headless mode (audio server only)")
    args = parser.parse_args()

    if args.no_gui:
        srv = AudioServer(port=args.ws_port)
        srv.start()
        print(f"Audio server on ws://0.0.0.0:{args.ws_port}/audio/stream")
        print("Press Ctrl+C to stop")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        return

    app = Dashboard(args.broker, args.port, args.topic, args.ws_port)
    app.run_auto_refresh()
    app.mainloop()


if __name__ == "__main__":
    main()
