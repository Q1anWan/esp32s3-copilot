#!/usr/bin/env python3
"""Tkinter USB control panel for ESP32-S3 Copilot.

The GUI keeps one USB serial session open so status polling, WiFi setup, and
TF-card audio playback can be tested without repeatedly reopening the port.

Usage:
  python3 tools/copilot_usb_gui.py
  python3 tools/copilot_usb_gui.py --port /dev/ttyACM2
  python tools/copilot_usb_gui.py --port COM7
"""

from __future__ import annotations

import argparse
import json
import os
import queue
import socket
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk


DEFAULT_BAUD = 115_200
DEFAULT_TCP_PORT = 7777
DEFAULT_SCENES = ("boot", "hachimi", "default")


def require_serial():
    try:
        import serial  # type: ignore
        from serial.tools import list_ports  # type: ignore
    except ImportError as exc:
        raise RuntimeError("pyserial is required: python -m pip install pyserial") from exc
    return serial, list_ports


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


def list_serial_ports() -> list[tuple[str, str]]:
    _, list_ports = require_serial()
    ports = []
    for port in list(list_ports.comports()):
        desc = port.description or ""
        hwid = port.hwid or ""
        label = f"{port.device} - {desc}".strip()
        if hwid:
            label = f"{label} [{hwid}]"
        ports.append((port.device, label))
    return ports


def auto_select_port() -> str:
    ports = list_serial_ports()
    if not ports:
        raise RuntimeError("No serial ports found. Connect ESP32 USB and refresh.")

    scored: list[tuple[int, str]] = []
    for device, label in ports:
        text = f"{device} {label}".lower()
        score = 0
        for key in ("usb jtag", "usb serial", "esp32", "acm", "cp210", "ch340"):
            if key in text:
                score += 1
        scored.append((score, device))
    scored.sort(reverse=True)
    return scored[0][1]


def open_esp_serial(port: str, baud: int, reset: bool):
    serial, _ = require_serial()
    if port == "auto":
        port = auto_select_port()

    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 0.05
    ser.write_timeout = 1.0
    ser.dtr = False
    ser.rts = False
    ser.open()
    disable_hangup_on_close(ser)
    ser.dtr = False
    ser.rts = False
    if reset:
        ser.dtr = True
        ser.rts = False
        time.sleep(0.05)
        ser.dtr = False
        ser.rts = False
    time.sleep(0.2)
    return ser, port


class SerialWorker:
    def __init__(self, line_queue: "queue.Queue[tuple[str, object]]") -> None:
        self.line_queue = line_queue
        self.ser = None
        self.port = ""
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    @property
    def connected(self) -> bool:
        return bool(self.ser and self.ser.is_open)

    def connect(self, port: str, baud: int, reset: bool) -> str:
        self.disconnect()
        ser, actual_port = open_esp_serial(port, baud, reset)
        self.ser = ser
        self.port = actual_port
        self._stop.clear()
        self._thread = threading.Thread(target=self._read_loop, name="copilot-usb-reader", daemon=True)
        self._thread.start()
        return actual_port

    def disconnect(self) -> None:
        self._stop.set()
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.port = ""

    def write_line(self, line: str) -> None:
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("USB serial is not connected")
        data = (line.rstrip("\r\n") + "\n").encode("utf-8")
        with self._lock:
            self.ser.write(data)
            self.ser.flush()

    def _emit_line(self, text: str) -> None:
        self.line_queue.put(("line", text))

    def _read_loop(self) -> None:
        buf = bytearray()
        while not self._stop.is_set():
            ser = self.ser
            if not ser:
                break
            try:
                chunk = ser.read(256)
            except Exception as exc:
                self.line_queue.put(("error", f"serial read failed: {exc}"))
                break
            if not chunk:
                continue
            for b in chunk:
                if b in (10, 13):
                    if buf:
                        self._emit_line(buf.decode("utf-8", errors="replace"))
                        buf.clear()
                else:
                    buf.append(b)
                    if len(buf) > 2048:
                        self._emit_line(buf.decode("utf-8", errors="replace"))
                        buf.clear()
        if buf:
            self._emit_line(buf.decode("utf-8", errors="replace"))


class CopilotUsbGui(tk.Tk):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__()
        self.title("Copilot USB Audio Test")
        self.minsize(980, 640)

        self.args = args
        self.queue: "queue.Queue[tuple[str, object]]" = queue.Queue()
        self.worker = SerialWorker(self.queue)
        self.port_map: dict[str, str] = {}
        self.last_status: dict | None = None
        self.auto_poll_after: str | None = None
        self.wifi_poll_until = 0.0

        self._build_style()
        self._build_ui()
        self.refresh_ports(initial=True)
        self.after(60, self._drain_queue)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        if args.autoconnect:
            self.after(200, self.connect)

    def _build_style(self) -> None:
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("Title.TLabel", font=("", 14, "bold"))
        style.configure("Status.TLabel", font=("", 10))
        style.configure("Good.TLabel", foreground="#147a39")
        style.configure("Warn.TLabel", foreground="#a15c00")
        style.configure("Bad.TLabel", foreground="#b02020")

    def _build_ui(self) -> None:
        self.columnconfigure(0, weight=0)
        self.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)

        left = ttk.Frame(self, padding=12)
        left.grid(row=0, column=0, sticky="ns")
        right = ttk.Frame(self, padding=(0, 12, 12, 12))
        right.grid(row=0, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)
        right.rowconfigure(2, weight=1)

        self._build_connection(left)
        self._build_wifi(left)
        self._build_playback(left)
        self._build_silab_sim(left)
        self._build_tools(left)
        self._build_status(right)
        self._build_log(right)

    def _build_connection(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="USB")
        box.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        box.columnconfigure(1, weight=1)

        ttk.Label(box, text="Port").grid(row=0, column=0, sticky="w", padx=8, pady=(8, 4))
        self.port_var = tk.StringVar(value=self.args.port)
        self.port_combo = ttk.Combobox(box, textvariable=self.port_var, width=32, state="readonly")
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=8, pady=(8, 4))

        ttk.Button(box, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=(0, 8), pady=(8, 4))

        ttk.Label(box, text="Baud").grid(row=1, column=0, sticky="w", padx=8, pady=4)
        self.baud_var = tk.IntVar(value=self.args.baud)
        ttk.Entry(box, textvariable=self.baud_var, width=12).grid(row=1, column=1, sticky="w", padx=8, pady=4)

        self.reset_var = tk.BooleanVar(value=self.args.reset)
        ttk.Checkbutton(box, text="Reset on connect", variable=self.reset_var).grid(
            row=2, column=0, columnspan=3, sticky="w", padx=8, pady=4
        )

        self.connect_btn = ttk.Button(box, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=3, column=0, columnspan=3, sticky="ew", padx=8, pady=(4, 8))

    def _build_wifi(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="WiFi")
        box.grid(row=1, column=0, sticky="ew", pady=(0, 10))
        box.columnconfigure(1, weight=1)

        ttk.Label(box, text="SSID").grid(row=0, column=0, sticky="w", padx=8, pady=(8, 4))
        self.ssid_var = tk.StringVar(value=self.args.ssid)
        ttk.Entry(box, textvariable=self.ssid_var, width=28).grid(row=0, column=1, sticky="ew", padx=8, pady=(8, 4))

        ttk.Label(box, text="Password").grid(row=1, column=0, sticky="w", padx=8, pady=4)
        self.password_var = tk.StringVar(value=self.args.password)
        self.password_entry = ttk.Entry(box, textvariable=self.password_var, width=28, show="*")
        self.password_entry.grid(row=1, column=1, sticky="ew", padx=8, pady=4)

        self.show_password_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(box, text="Show password", variable=self.show_password_var, command=self._toggle_password).grid(
            row=2, column=0, columnspan=2, sticky="w", padx=8, pady=4
        )

        ttk.Button(box, text="Save WiFi", command=self.configure_wifi).grid(
            row=3, column=0, columnspan=2, sticky="ew", padx=8, pady=(4, 8)
        )

    def _build_playback(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="TF Audio")
        box.grid(row=2, column=0, sticky="ew", pady=(0, 10))
        box.columnconfigure(1, weight=1)

        ttk.Label(box, text="Scene").grid(row=0, column=0, sticky="w", padx=8, pady=(8, 4))
        self.scene_var = tk.StringVar(value="boot")
        self.scene_combo = ttk.Combobox(box, textvariable=self.scene_var, values=DEFAULT_SCENES, width=20)
        self.scene_combo.grid(row=0, column=1, sticky="ew", padx=8, pady=(8, 4))

        ttk.Label(box, text="Seq").grid(row=1, column=0, sticky="w", padx=8, pady=4)
        self.seq_var = tk.IntVar(value=1)
        ttk.Spinbox(box, from_=1, to=65535, textvariable=self.seq_var, width=10).grid(
            row=1, column=1, sticky="w", padx=8, pady=4
        )

        ttk.Button(box, text="Play", command=self.play_selected).grid(
            row=2, column=0, columnspan=2, sticky="ew", padx=8, pady=(4, 8)
        )

        quick = ttk.Frame(box)
        quick.grid(row=3, column=0, columnspan=2, sticky="ew", padx=8, pady=(0, 8))
        for col in range(3):
            quick.columnconfigure(col, weight=1)
        ttk.Button(quick, text="boot/001", command=lambda: self.play("boot", 1)).grid(row=0, column=0, sticky="ew")
        ttk.Button(quick, text="hachimi/001", command=lambda: self.play("hachimi", 1)).grid(row=0, column=1, sticky="ew", padx=4)
        ttk.Button(quick, text="default/001", command=lambda: self.play("default", 1)).grid(row=0, column=2, sticky="ew")

    def _build_tools(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="Tools")
        box.grid(row=4, column=0, sticky="ew")
        for col in range(2):
            box.columnconfigure(col, weight=1)

        ttk.Button(box, text="Status", command=self.request_status).grid(row=0, column=0, sticky="ew", padx=8, pady=(8, 4))
        ttk.Button(box, text="Clear Log", command=self.clear_log).grid(row=0, column=1, sticky="ew", padx=(0, 8), pady=(8, 4))

        self.auto_poll_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(box, text="Auto status", variable=self.auto_poll_var, command=self._schedule_auto_poll).grid(
            row=1, column=0, columnspan=2, sticky="w", padx=8, pady=4
        )

        ttk.Button(box, text="Debug On", command=lambda: self.send_command("debug on")).grid(
            row=2, column=0, sticky="ew", padx=8, pady=4
        )
        ttk.Button(box, text="Debug Off", command=lambda: self.send_command("debug off")).grid(
            row=2, column=1, sticky="ew", padx=(0, 8), pady=4
        )
        ttk.Button(box, text="Reboot", command=self.reboot).grid(row=3, column=0, columnspan=2, sticky="ew", padx=8, pady=(4, 8))

    def _build_silab_sim(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="TCP Audio ID Sim")
        box.grid(row=3, column=0, sticky="ew", pady=(0, 10))
        box.columnconfigure(1, weight=1)

        ttk.Label(box, text="Host").grid(row=0, column=0, sticky="w", padx=8, pady=(8, 4))
        self.tcp_host_var = tk.StringVar(value=self.args.tcp_host)
        ttk.Entry(box, textvariable=self.tcp_host_var, width=24).grid(row=0, column=1, sticky="ew", padx=8, pady=(8, 4))

        ttk.Label(box, text="Port").grid(row=1, column=0, sticky="w", padx=8, pady=4)
        self.tcp_port_var = tk.IntVar(value=self.args.tcp_port)
        ttk.Entry(box, textvariable=self.tcp_port_var, width=10).grid(row=1, column=1, sticky="w", padx=8, pady=4)

        ttk.Label(box, text="Trigger").grid(row=2, column=0, sticky="w", padx=8, pady=4)
        self.audio_id_trigger_var = tk.DoubleVar(value=1.0)
        ttk.Entry(box, textvariable=self.audio_id_trigger_var, width=10).grid(row=2, column=1, sticky="w", padx=8, pady=4)

        ttk.Label(box, text="Scene").grid(row=3, column=0, sticky="w", padx=8, pady=4)
        self.audio_id_scene_var = tk.StringVar(value="1")
        ttk.Entry(box, textvariable=self.audio_id_scene_var, width=10).grid(row=3, column=1, sticky="w", padx=8, pady=4)

        ttk.Label(box, text="Seq").grid(row=4, column=0, sticky="w", padx=8, pady=4)
        self.audio_id_seq_var = tk.IntVar(value=1)
        ttk.Entry(box, textvariable=self.audio_id_seq_var, width=10).grid(row=4, column=1, sticky="w", padx=8, pady=4)

        buttons = ttk.Frame(box)
        buttons.grid(row=5, column=0, columnspan=2, sticky="ew", padx=8, pady=(4, 8))
        for col in range(3):
            buttons.columnconfigure(col, weight=1)
        ttk.Button(buttons, text="Send", command=self.send_audio_id_packet).grid(row=0, column=0, sticky="ew")
        ttk.Button(buttons, text="Reset 0", command=lambda: self.send_audio_id_packet(0.0)).grid(row=0, column=1, sticky="ew", padx=4)
        ttk.Button(buttons, text="0 -> 1", command=self.send_audio_id_demo).grid(row=0, column=2, sticky="ew")

    def _build_status(self, parent: ttk.Frame) -> None:
        top = ttk.Frame(parent)
        top.grid(row=0, column=0, sticky="ew")
        top.columnconfigure(0, weight=1)

        ttk.Label(top, text="Device Status", style="Title.TLabel").grid(row=0, column=0, sticky="w")
        self.connection_var = tk.StringVar(value="Disconnected")
        self.connection_label = ttk.Label(top, textvariable=self.connection_var, style="Bad.TLabel")
        self.connection_label.grid(row=0, column=1, sticky="e")

        status = ttk.LabelFrame(parent, text="Parsed COPILOT_STATUS")
        status.grid(row=1, column=0, sticky="ew", pady=(8, 10))
        for col in range(4):
            status.columnconfigure(col, weight=1)

        self.status_vars: dict[str, tk.StringVar] = {}
        fields = [
            ("WiFi", "wifi"),
            ("IP", "ip"),
            ("TCP", "tcp"),
            ("MQTT", "mqtt"),
            ("Audio", "audio"),
            ("SD", "sd"),
            ("Files", "files"),
            ("Heap", "heap"),
        ]
        for index, (label, key) in enumerate(fields):
            row = index // 4
            col = index % 4
            cell = ttk.Frame(status, padding=6)
            cell.grid(row=row, column=col, sticky="ew")
            ttk.Label(cell, text=label, style="Status.TLabel").grid(row=0, column=0, sticky="w")
            var = tk.StringVar(value="-")
            self.status_vars[key] = var
            ttk.Label(cell, textvariable=var).grid(row=1, column=0, sticky="w")

    def _build_log(self, parent: ttk.Frame) -> None:
        log_frame = ttk.LabelFrame(parent, text="Serial Log")
        log_frame.grid(row=2, column=0, sticky="nsew")
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)

        self.log_text = tk.Text(log_frame, wrap="word", height=22, font=("Consolas", 10))
        self.log_text.grid(row=0, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        self.log_text.configure(yscrollcommand=scroll.set)

    def refresh_ports(self, initial: bool = False) -> None:
        try:
            ports = list_serial_ports()
        except Exception as exc:
            if not initial:
                messagebox.showerror("Serial", str(exc))
            ports = []

        self.port_map = {"auto": "auto"}
        values = ["auto"]
        for device, label in ports:
            values.append(label)
            self.port_map[label] = device
        self.port_combo.configure(values=values)

        requested = self.args.port if initial else self.port_var.get()
        if requested != "auto":
            for label, device in self.port_map.items():
                if device == requested or label.startswith(f"{requested} "):
                    self.port_var.set(label)
                    return
        self.port_var.set("auto" if "auto" in values else (values[0] if values else ""))

    def toggle_connection(self) -> None:
        if self.worker.connected:
            self.disconnect()
        else:
            self.connect()

    def connect(self) -> None:
        label = self.port_var.get() or "auto"
        port = self.port_map.get(label, label)
        try:
            actual = self.worker.connect(port, int(self.baud_var.get()), self.reset_var.get())
        except Exception as exc:
            messagebox.showerror("Connect failed", str(exc))
            self._set_connection(False)
            return
        self._set_connection(True, actual)
        self._log(f"[gui] connected {actual} @ {self.baud_var.get()}")
        self._schedule_auto_poll()

    def disconnect(self) -> None:
        self.worker.disconnect()
        self._set_connection(False)
        self._log("[gui] disconnected")

    def _set_connection(self, connected: bool, port: str = "") -> None:
        if connected:
            self.connection_var.set(f"Connected: {port}")
            self.connection_label.configure(style="Good.TLabel")
            self.connect_btn.configure(text="Disconnect")
        else:
            self.connection_var.set("Disconnected")
            self.connection_label.configure(style="Bad.TLabel")
            self.connect_btn.configure(text="Connect")

    def _toggle_password(self) -> None:
        self.password_entry.configure(show="" if self.show_password_var.get() else "*")

    def send_command(self, line: str, log_line: str | None = None) -> bool:
        try:
            self.worker.write_line(line)
        except Exception as exc:
            messagebox.showerror("USB command failed", str(exc))
            return False
        self._log(log_line if log_line is not None else f"> {line}")
        return True

    def _tcp_target(self) -> tuple[str, int]:
        host = self.tcp_host_var.get().strip()
        if not host and self.last_status:
            wifi = self.last_status.get("wifi", {})
            host = str(wifi.get("ip") or "")
        if not host:
            raise RuntimeError("TCP host is empty. Run USB Status first or type the ESP32 IP.")
        return host, int(self.tcp_port_var.get())

    def _format_audio_id_line(self, trigger: float | None = None, scene: str | None = None, seq: int | None = None) -> str:
        trigger_value = float(self.audio_id_trigger_var.get() if trigger is None else trigger)
        scene_value = self.audio_id_scene_var.get().strip() if scene is None else str(scene).strip()
        seq_value = int(self.audio_id_seq_var.get() if seq is None else seq)
        if not scene_value:
            raise RuntimeError("Scene is empty")
        if seq_value < 1 or seq_value > 65535:
            raise RuntimeError("Seq must be 1..65535")
        return f"{trigger_value:g}\t{scene_value}\t{seq_value}\n"

    def send_audio_id_packet(self, trigger: float | None = None, scene: str | None = None, seq: int | None = None) -> None:
        try:
            host, port = self._tcp_target()
            line = self._format_audio_id_line(trigger, scene, seq)
        except Exception as exc:
            messagebox.showerror("TCP Audio ID", str(exc))
            return
        self._log(f"> tcp {host}:{port} {line.rstrip()!r}")
        threading.Thread(
            target=self._send_audio_id_lines_worker,
            args=(host, port, [line]),
            name="audio-id-tcp-send",
            daemon=True,
        ).start()

    def send_audio_id_demo(self) -> None:
        try:
            host, port = self._tcp_target()
            reset_line = self._format_audio_id_line(0.0)
            trigger_line = self._format_audio_id_line(1.0)
        except Exception as exc:
            messagebox.showerror("TCP Audio ID", str(exc))
            return
        lines = [reset_line, trigger_line, trigger_line]
        self._log(f"> tcp {host}:{port} audio ID 0 -> 1 demo")
        threading.Thread(
            target=self._send_audio_id_lines_worker,
            args=(host, port, lines),
            name="audio-id-tcp-demo",
            daemon=True,
        ).start()

    def _send_audio_id_lines_worker(self, host: str, port: int, lines: list[str]) -> None:
        try:
            with socket.create_connection((host, port), timeout=3.0) as sock:
                sock.settimeout(0.5)
                try:
                    greeting = sock.recv(512)
                except OSError:
                    greeting = b""
                if greeting:
                    self.queue.put(("gui", f"[tcp] {greeting.decode('utf-8', errors='replace').strip()}"))
                for line in lines:
                    sock.sendall(line.encode("utf-8"))
                    time.sleep(0.25)
                    try:
                        ack = sock.recv(512)
                    except OSError:
                        ack = b""
                    if ack:
                        self.queue.put(("gui", f"[tcp] {ack.decode('utf-8', errors='replace').strip()}"))
                try:
                    sock.shutdown(socket.SHUT_WR)
                except OSError:
                    pass
            self.queue.put(("gui", "[tcp] audio ID packet sent"))
        except Exception as exc:
            self.queue.put(("gui", f"[tcp] send failed: {exc}"))

    def configure_wifi(self) -> None:
        ssid = self.ssid_var.get().strip()
        if not ssid:
            messagebox.showwarning("WiFi", "SSID is empty")
            return
        payload = {"type": "wifi", "ssid": ssid, "password": self.password_var.get()}
        line = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        if self.send_command(line, f'> {{"type":"wifi","ssid":"{ssid}","password":"***"}}'):
            self.wifi_poll_until = time.time() + 25.0
            self.after(600, self._poll_wifi_status)

    def _poll_wifi_status(self) -> None:
        if not self.worker.connected:
            return
        self.request_status()
        if time.time() < self.wifi_poll_until:
            self.after(1500, self._poll_wifi_status)

    def request_status(self) -> None:
        self.send_command("status")

    def play_selected(self) -> None:
        try:
            seq = int(self.seq_var.get())
        except (TypeError, ValueError):
            messagebox.showwarning("Audio", "Seq must be a number")
            return
        self.play(self.scene_var.get().strip(), seq)

    def play(self, scene: str, seq: int) -> None:
        if not scene:
            messagebox.showwarning("Audio", "Scene is empty")
            return
        if seq < 1 or seq > 65535:
            messagebox.showwarning("Audio", "Seq must be 1..65535")
            return
        self.scene_var.set(scene)
        self.seq_var.set(seq)
        self.send_command(f"play {scene} {seq}")
        self.after(1200, self.request_status)

    def reboot(self) -> None:
        if messagebox.askyesno("Reboot", "Reboot ESP32 now?"):
            self.send_command("reboot")

    def _schedule_auto_poll(self) -> None:
        if self.auto_poll_after:
            self.after_cancel(self.auto_poll_after)
            self.auto_poll_after = None
        if self.auto_poll_var.get() and self.worker.connected:
            self.auto_poll_after = self.after(3000, self._auto_poll)

    def _auto_poll(self) -> None:
        self.auto_poll_after = None
        if self.auto_poll_var.get() and self.worker.connected:
            self.request_status()
            self.auto_poll_after = self.after(3000, self._auto_poll)

    def _drain_queue(self) -> None:
        try:
            while True:
                kind, payload = self.queue.get_nowait()
                if kind == "line":
                    line = str(payload)
                    self._log(line)
                    self._handle_line(line)
                elif kind == "gui":
                    self._log(str(payload))
                elif kind == "error":
                    self._log(f"[gui] {payload}")
                    self._set_connection(False)
        except queue.Empty:
            pass
        self.after(60, self._drain_queue)

    def _handle_line(self, line: str) -> None:
        if line.startswith("COPILOT_STATUS "):
            data = line[len("COPILOT_STATUS ") :]
            try:
                self.last_status = json.loads(data)
            except json.JSONDecodeError:
                return
            self._update_status(self.last_status)
        elif line.startswith("COPILOT_SERIAL ready"):
            self.after(250, self.request_status)
        elif line.startswith("COPILOT_OK play"):
            self.after(800, self.request_status)
        elif line.startswith("COPILOT_OK wifi"):
            self.wifi_poll_until = time.time() + 25.0
            self.after(600, self._poll_wifi_status)

    def _update_status(self, status: dict) -> None:
        wifi = status.get("wifi", {})
        mqtt = status.get("mqtt", {})
        audio = status.get("audio", {})
        heap = status.get("heap", {})

        wifi_text = "connected" if wifi.get("connected") else "offline"
        if wifi.get("ssid"):
            wifi_text = f"{wifi_text} / {wifi.get('ssid')}"
        self.status_vars["wifi"].set(wifi_text)
        self.status_vars["ip"].set(str(wifi.get("ip") or "-"))
        self.status_vars["tcp"].set(str(status.get("tcp_host") or "-"))
        self.status_vars["mqtt"].set("connected" if mqtt.get("connected") else "off")
        self._maybe_fill_tcp_target(status)

        audio_bits = []
        audio_bits.append("ready" if audio.get("ready") else "not ready")
        if audio.get("playing_file"):
            audio_bits.append("playing")
        if audio.get("last_error"):
            audio_bits.append(f"err={audio.get('last_error')}")
        self.status_vars["audio"].set(", ".join(audio_bits))
        self.status_vars["sd"].set("mounted" if audio.get("sd_mounted") else "not mounted")
        self.status_vars["files"].set(str(audio.get("files_played", 0)))
        self.status_vars["heap"].set(f"{heap.get('internal_free', '-')}/{heap.get('psram_free', '-')}")

    def _maybe_fill_tcp_target(self, status: dict) -> None:
        tcp_host = str(status.get("tcp_host") or "")
        if tcp_host and ":" in tcp_host:
            host, port_text = tcp_host.rsplit(":", 1)
            if not self.tcp_host_var.get().strip():
                self.tcp_host_var.set(host)
            try:
                if int(self.tcp_port_var.get()) == DEFAULT_TCP_PORT:
                    self.tcp_port_var.set(int(port_text))
            except (TypeError, ValueError, tk.TclError):
                pass
            return

        wifi = status.get("wifi", {})
        ip = str(wifi.get("ip") or "")
        if ip and not self.tcp_host_var.get().strip():
            self.tcp_host_var.set(ip)

    def _log(self, text: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self.log_text.insert("end", f"{stamp} {text}\n")
        line_count = int(self.log_text.index("end-1c").split(".")[0])
        if line_count > 1200:
            self.log_text.delete("1.0", "200.0")
        self.log_text.see("end")

    def clear_log(self) -> None:
        self.log_text.delete("1.0", "end")

    def _on_close(self) -> None:
        if self.auto_poll_after:
            self.after_cancel(self.auto_poll_after)
            self.auto_poll_after = None
        self.worker.disconnect()
        self.destroy()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="USB GUI for ESP32-S3 Copilot WiFi setup and TF audio playback.")
    parser.add_argument("--port", default="auto", help="Serial port, for example /dev/ttyACM2 or COM7. Default: auto")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"Serial baud. Default: {DEFAULT_BAUD}")
    parser.add_argument("--ssid", default="", help="Pre-fill WiFi SSID")
    parser.add_argument("--password", default="", help="Pre-fill WiFi password")
    parser.add_argument("--tcp-host", default="", help="Pre-fill ESP32 TCP host IP for SILAB simulation")
    parser.add_argument("--tcp-port", type=int, default=DEFAULT_TCP_PORT, help=f"ESP32 TCP host port. Default: {DEFAULT_TCP_PORT}")
    parser.add_argument("--reset", action="store_true", help="Reset ESP32 when connecting")
    parser.add_argument("--autoconnect", action="store_true", help="Connect when the GUI starts")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    try:
        app = CopilotUsbGui(args)
        app.mainloop()
    except RuntimeError as exc:
        messagebox.showerror("Copilot USB GUI", str(exc))


if __name__ == "__main__":
    main()
