#!/usr/bin/env python3
"""PC-side SILAB TCP to ESP32 MQTT bridge GUI.

Topology:
  SILAB --LAN A/TCP--> experiment PC --LAN B/MQTT--> ESP32

The PC listens for fixed-rate SILAB audio-ID packets:
  trigger<TAB>scene<TAB>seq<LF>

Only trigger rising edges are forwarded to ESP32 as MQTT play commands.
"""

from __future__ import annotations

import argparse
import json
import os
import queue
import socket
import subprocess
import sys
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import messagebox, ttk


DEFAULT_MQTT_HOST = os.environ.get("COPILOT_MQTT_BROKER", "127.0.0.1")
DEFAULT_MQTT_PORT = int(os.environ.get("COPILOT_MQTT_PORT", "1883"))
DEFAULT_CMD_TOPIC = os.environ.get("COPILOT_MQTT_TOPIC", "copilot/s3_copilot/cmd")
DEFAULT_STATUS_TOPIC = os.environ.get("COPILOT_STATUS_TOPIC", "copilot/s3_copilot/status")
DEFAULT_TCP_HOST = os.environ.get("SILAB_TCP_HOST", "0.0.0.0")
DEFAULT_TCP_PORT = int(os.environ.get("SILAB_TCP_PORT", "7777"))
DEFAULT_SCENE_PREFIX = os.environ.get("SILAB_SCENE_PREFIX", "scene")


def require_mqtt():
    try:
        import paho.mqtt.client as mqtt  # type: ignore
    except ImportError as exc:
        raise RuntimeError("paho-mqtt is required: python -m pip install paho-mqtt") from exc
    return mqtt


def require_serial():
    try:
        import serial  # type: ignore
        from serial.tools import list_ports  # type: ignore
    except ImportError as exc:
        raise RuntimeError("pyserial is required: python -m pip install pyserial") from exc
    return serial, list_ports


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


def send_usb_line(port: str, baud: int, line: str, read_seconds: float = 2.0) -> list[str]:
    serial, _ = require_serial()
    actual_port = auto_select_port() if port == "auto" else port
    ser = serial.Serial()
    ser.port = actual_port
    ser.baudrate = baud
    ser.timeout = 0.05
    ser.write_timeout = 1.0
    ser.dtr = False
    ser.rts = False
    ser.open()
    try:
        disable_hangup_on_close(ser)
        ser.dtr = False
        ser.rts = False
        time.sleep(0.2)
        ser.write((line.rstrip("\r\n") + "\n").encode("utf-8"))
        ser.flush()
        deadline = time.time() + read_seconds
        buf = bytearray()
        lines: list[str] = []
        while time.time() < deadline:
            chunk = ser.read(256)
            if not chunk:
                continue
            for b in chunk:
                if b in (10, 13):
                    if buf:
                        lines.append(buf.decode("utf-8", errors="replace"))
                        buf.clear()
                else:
                    buf.append(b)
        if buf:
            lines.append(buf.decode("utf-8", errors="replace"))
        return lines
    finally:
        ser.close()


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
    allowed = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-"
    return bool(scene) and all(c in allowed for c in scene)


def normalize_scene(scene: str, prefix: str) -> str:
    numeric = parse_integer_like(scene)
    if numeric is not None:
        return f"{prefix}{numeric:03d}"
    return scene


@dataclass
class AudioIdPacket:
    raw_text: str
    trigger: float
    scene: str
    seq: int


def parse_audio_id_line(line: str) -> AudioIdPacket | None:
    text = line.strip()
    if not text:
        return None
    parts = text.replace(",", "\t").split()
    if len(parts) != 3:
        return None
    try:
        trigger = float(parts[0])
    except ValueError:
        return None
    scene = parts[1]
    if not scene_token_is_valid(scene):
        return None
    seq = parse_integer_like(parts[2])
    if seq is None or seq < 1 or seq > 65535:
        return None
    return AudioIdPacket(raw_text=text, trigger=trigger, scene=scene, seq=seq)


class MqttController:
    def __init__(self, events: "queue.Queue[tuple[str, object]]") -> None:
        self.events = events
        self.client = None
        self.connected = False
        self.host = ""
        self.port = DEFAULT_MQTT_PORT
        self.cmd_topic = DEFAULT_CMD_TOPIC
        self.status_topic = DEFAULT_STATUS_TOPIC

    def connect(self, host: str, port: int, cmd_topic: str, status_topic: str) -> None:
        mqtt = require_mqtt()
        self.disconnect()
        self.host = host
        self.port = port
        self.cmd_topic = cmd_topic
        self.status_topic = status_topic
        client_id = f"copilot_bridge_gui_{int(time.time() * 1000)}"
        try:
            self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
        except Exception:
            self.client = mqtt.Client(client_id=client_id)

        def on_connect(client, userdata, flags, reason_code, properties=None):
            rc_value = getattr(reason_code, "value", reason_code)
            ok = rc_value == 0 or str(reason_code) == "Success"
            self.connected = ok
            if ok:
                client.subscribe(self.status_topic, qos=1)
                self.events.put(("log", f"[mqtt] connected {self.host}:{self.port}; sub {self.status_topic}"))
            else:
                self.events.put(("log", f"[mqtt] connect failed rc={reason_code}"))

        def on_disconnect(client, userdata, flags=None, reason_code=None, properties=None):
            self.connected = False
            self.events.put(("log", "[mqtt] disconnected"))

        def on_message(client, userdata, msg):
            text = msg.payload.decode("utf-8", errors="replace")
            self.events.put(("log", f"[mqtt] {msg.topic} <- {text}"))
            try:
                payload = json.loads(text)
            except json.JSONDecodeError:
                return
            self.events.put(("status", payload))

        self.client.on_connect = on_connect
        self.client.on_disconnect = on_disconnect
        self.client.on_message = on_message
        self.client.connect(host, port, keepalive=30)
        self.client.loop_start()

    def disconnect(self) -> None:
        if self.client:
            try:
                self.client.loop_stop()
                self.client.disconnect()
            except Exception:
                pass
        self.client = None
        self.connected = False

    def publish(self, payload: dict) -> bool:
        if not self.client or not self.connected:
            self.events.put(("log", "[mqtt] publish skipped: not connected"))
            return False
        data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        info = self.client.publish(self.cmd_topic, data, qos=1)
        info.wait_for_publish(timeout=3)
        ok = info.rc == 0
        self.events.put(("log", f"[mqtt] {self.cmd_topic} -> {data} rc={info.rc}"))
        return ok


class SilabTcpBridge:
    def __init__(
        self,
        events: "queue.Queue[tuple[str, object]]",
        publish_cb,
        scene_prefix_cb,
        threshold_cb,
        ack_cb,
    ) -> None:
        self.events = events
        self.publish_cb = publish_cb
        self.scene_prefix_cb = scene_prefix_cb
        self.threshold_cb = threshold_cb
        self.ack_cb = ack_cb
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._listen_socket: socket.socket | None = None
        self._lock = threading.Lock()
        self._latched = False
        self._packet_count = 0
        self._trigger_count = 0

    @property
    def running(self) -> bool:
        return bool(self._thread and self._thread.is_alive())

    def start(self, host: str, port: int) -> None:
        if self.running:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._server_loop, args=(host, port), name="silab-tcp-host", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        sock = self._listen_socket
        if sock:
            try:
                sock.close()
            except OSError:
                pass

    def _server_loop(self, host: str, port: int) -> None:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
                self._listen_socket = server
                server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                server.bind((host, port))
                server.listen(4)
                server.settimeout(0.5)
                self.events.put(("log", f"[silab] TCP host listening on {host}:{port}"))
                self.events.put(("tcp_running", True))
                while not self._stop.is_set():
                    try:
                        conn, addr = server.accept()
                    except socket.timeout:
                        continue
                    except OSError:
                        break
                    threading.Thread(target=self._client_loop, args=(conn, addr), daemon=True).start()
        except Exception as exc:
            self.events.put(("log", f"[silab] host failed: {exc}"))
        finally:
            self._listen_socket = None
            self.events.put(("tcp_running", False))

    def _client_loop(self, conn: socket.socket, addr) -> None:
        peer = f"{addr[0]}:{addr[1]}"
        self.events.put(("log", f"[silab] connected {peer}"))
        buffer = bytearray()
        with conn:
            conn.settimeout(0.5)
            while not self._stop.is_set():
                try:
                    data = conn.recv(1024)
                except socket.timeout:
                    continue
                except OSError:
                    break
                if not data:
                    break
                for b in data:
                    if b in (10, 13):
                        if buffer:
                            self._handle_line(conn, bytes(buffer).decode("utf-8", errors="replace"), peer)
                            buffer.clear()
                    else:
                        buffer.append(b)
                        if len(buffer) > 256:
                            self.events.put(("log", "[silab] drop overlong line"))
                            buffer.clear()
            if buffer:
                self._handle_line(conn, bytes(buffer).decode("utf-8", errors="replace"), peer)
        self.events.put(("log", f"[silab] disconnected {peer}"))

    def _handle_line(self, conn: socket.socket, line: str, peer: str) -> None:
        packet = parse_audio_id_line(line)
        with self._lock:
            self._packet_count += 1
            packet_no = self._packet_count
        if packet is None:
            self.events.put(("log", f"[silab] #{packet_no} BAD {line.strip()!r}"))
            self._send_ack(conn, False, False, "", 0, "bad_format")
            return

        active = packet.trigger >= float(self.threshold_cb())
        with self._lock:
            rising = active and not self._latched
            self._latched = active
            if rising:
                self._trigger_count += 1
            trigger_count = self._trigger_count

        scene = normalize_scene(packet.scene, self.scene_prefix_cb())
        accepted = False
        if rising:
            payload = {
                "type": "play",
                "scene": scene,
                "seq": packet.seq,
                "message_id": f"silab_{packet_no}_{trigger_count}_{int(time.time() * 1000)}",
            }
            accepted = bool(self.publish_cb(payload))

        state = "play" if rising else ("held" if active else "idle")
        self.events.put(
            (
                "silab_packet",
                {
                    "packet_no": packet_no,
                    "peer": peer,
                    "trigger": packet.trigger,
                    "scene": scene,
                    "seq": packet.seq,
                    "state": state,
                    "accepted": accepted,
                    "trigger_count": trigger_count,
                },
            )
        )
        self.events.put(("log", f"[silab] #{packet_no} {state} trigger={packet.trigger:g} scene={scene} seq={packet.seq}"))
        self._send_ack(conn, True, accepted, scene, packet.seq, state)

    def _send_ack(self, conn: socket.socket, ok: bool, accepted: bool, scene: str, seq: int, state: str) -> None:
        if not self.ack_cb():
            return
        payload = {"ok": ok, "accepted": accepted, "scene": scene, "seq": seq, "state": state}
        try:
            conn.sendall((json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8"))
        except OSError:
            pass


class BridgeGui(tk.Tk):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__()
        self.title("Copilot SILAB MQTT Bridge")
        self.minsize(1120, 720)
        self.args = args
        self.events: "queue.Queue[tuple[str, object]]" = queue.Queue()
        self.mqtt = MqttController(self.events)
        self.tcp_bridge = SilabTcpBridge(
            self.events,
            publish_cb=self._publish_mqtt,
            scene_prefix_cb=lambda: self.scene_prefix_var.get().strip() or DEFAULT_SCENE_PREFIX,
            threshold_cb=lambda: self.threshold_var.get(),
            ack_cb=lambda: self.tcp_ack_var.get(),
        )
        self.port_map: dict[str, str] = {}
        self.broker_proc: subprocess.Popen | None = None
        self.auto_status_after: str | None = None
        self._build_style()
        self._build_ui()
        self.refresh_ports(initial=True)
        self.after(80, self._drain_events)
        self.after(1000, self._auto_status)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_style(self) -> None:
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("Title.TLabel", font=("", 14, "bold"))
        style.configure("Good.TLabel", foreground="#147a39")
        style.configure("Warn.TLabel", foreground="#a15c00")
        style.configure("Bad.TLabel", foreground="#b02020")

    def _build_ui(self) -> None:
        self.columnconfigure(0, weight=0)
        self.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)
        left = ttk.Frame(self, padding=12)
        right = ttk.Frame(self, padding=(0, 12, 12, 12))
        left.grid(row=0, column=0, sticky="ns")
        right.grid(row=0, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)
        right.rowconfigure(2, weight=1)
        self._build_mqtt(left)
        self._build_usb(left)
        self._build_tcp(left)
        self._build_play(left)
        self._build_status(right)
        self._build_log(right)

    def _build_mqtt(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="MQTT Broker")
        box.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        box.columnconfigure(1, weight=1)
        self.mqtt_host_var = tk.StringVar(value=self.args.broker)
        self.mqtt_port_var = tk.IntVar(value=self.args.mqtt_port)
        self.cmd_topic_var = tk.StringVar(value=self.args.topic)
        self.status_topic_var = tk.StringVar(value=self.args.status_topic)
        ttk.Label(box, text="Host").grid(row=0, column=0, sticky="w", padx=8, pady=(8, 4))
        ttk.Entry(box, textvariable=self.mqtt_host_var, width=28).grid(row=0, column=1, sticky="ew", padx=8, pady=(8, 4))
        ttk.Label(box, text="Port").grid(row=1, column=0, sticky="w", padx=8, pady=4)
        ttk.Entry(box, textvariable=self.mqtt_port_var, width=10).grid(row=1, column=1, sticky="w", padx=8, pady=4)
        ttk.Label(box, text="Cmd").grid(row=2, column=0, sticky="w", padx=8, pady=4)
        ttk.Entry(box, textvariable=self.cmd_topic_var, width=28).grid(row=2, column=1, sticky="ew", padx=8, pady=4)
        ttk.Label(box, text="Status").grid(row=3, column=0, sticky="w", padx=8, pady=4)
        ttk.Entry(box, textvariable=self.status_topic_var, width=28).grid(row=3, column=1, sticky="ew", padx=8, pady=4)
        row = ttk.Frame(box)
        row.grid(row=4, column=0, columnspan=2, sticky="ew", padx=8, pady=(4, 8))
        for col in range(3):
            row.columnconfigure(col, weight=1)
        ttk.Button(row, text="Start Broker", command=self.start_broker).grid(row=0, column=0, sticky="ew")
        self.mqtt_btn = ttk.Button(row, text="Connect", command=self.toggle_mqtt)
        self.mqtt_btn.grid(row=0, column=1, sticky="ew", padx=4)
        ttk.Button(row, text="Query", command=self.query_status).grid(row=0, column=2, sticky="ew")

    def _build_usb(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="ESP32 USB Setup")
        box.grid(row=1, column=0, sticky="ew", pady=(0, 10))
        box.columnconfigure(1, weight=1)
        self.port_var = tk.StringVar(value=self.args.port)
        self.baud_var = tk.IntVar(value=self.args.baud)
        self.usb_broker_var = tk.StringVar(value=self.args.esp32_broker_uri)
        self.ssid_var = tk.StringVar(value=self.args.ssid)
        self.password_var = tk.StringVar(value=self.args.password)
        ttk.Label(box, text="Port").grid(row=0, column=0, sticky="w", padx=8, pady=(8, 4))
        self.port_combo = ttk.Combobox(box, textvariable=self.port_var, width=28, state="readonly")
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=8, pady=(8, 4))
        ttk.Button(box, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=(0, 8), pady=(8, 4))
        ttk.Label(box, text="Broker URI").grid(row=1, column=0, sticky="w", padx=8, pady=4)
        ttk.Entry(box, textvariable=self.usb_broker_var, width=28).grid(row=1, column=1, columnspan=2, sticky="ew", padx=8, pady=4)
        ttk.Label(box, text="SSID").grid(row=2, column=0, sticky="w", padx=8, pady=4)
        ttk.Entry(box, textvariable=self.ssid_var, width=28).grid(row=2, column=1, columnspan=2, sticky="ew", padx=8, pady=4)
        ttk.Label(box, text="Pass").grid(row=3, column=0, sticky="w", padx=8, pady=4)
        ttk.Entry(box, textvariable=self.password_var, width=28, show="*").grid(row=3, column=1, columnspan=2, sticky="ew", padx=8, pady=4)
        row = ttk.Frame(box)
        row.grid(row=4, column=0, columnspan=3, sticky="ew", padx=8, pady=(4, 8))
        for col in range(3):
            row.columnconfigure(col, weight=1)
        ttk.Button(row, text="Use URI", command=self.fill_usb_broker_uri).grid(row=0, column=0, sticky="ew")
        ttk.Button(row, text="Save MQTT", command=self.configure_usb_mqtt).grid(row=0, column=1, sticky="ew", padx=4)
        ttk.Button(row, text="Save WiFi", command=self.configure_usb_wifi).grid(row=0, column=2, sticky="ew")

    def _build_tcp(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="SILAB TCP Host")
        box.grid(row=2, column=0, sticky="ew", pady=(0, 10))
        box.columnconfigure(1, weight=1)
        self.tcp_host_var = tk.StringVar(value=self.args.tcp_host)
        self.tcp_port_var = tk.IntVar(value=self.args.tcp_port)
        self.threshold_var = tk.DoubleVar(value=self.args.trigger_threshold)
        self.scene_prefix_var = tk.StringVar(value=self.args.scene_prefix)
        self.tcp_ack_var = tk.BooleanVar(value=self.args.tcp_ack)
        ttk.Label(box, text="Bind").grid(row=0, column=0, sticky="w", padx=8, pady=(8, 4))
        ttk.Entry(box, textvariable=self.tcp_host_var, width=18).grid(row=0, column=1, sticky="ew", padx=8, pady=(8, 4))
        ttk.Label(box, text="Port").grid(row=1, column=0, sticky="w", padx=8, pady=4)
        ttk.Entry(box, textvariable=self.tcp_port_var, width=10).grid(row=1, column=1, sticky="w", padx=8, pady=4)
        ttk.Label(box, text="Threshold").grid(row=2, column=0, sticky="w", padx=8, pady=4)
        ttk.Entry(box, textvariable=self.threshold_var, width=10).grid(row=2, column=1, sticky="w", padx=8, pady=4)
        ttk.Label(box, text="Prefix").grid(row=3, column=0, sticky="w", padx=8, pady=4)
        ttk.Entry(box, textvariable=self.scene_prefix_var, width=10).grid(row=3, column=1, sticky="w", padx=8, pady=4)
        ttk.Checkbutton(box, text="Send TCP ACK", variable=self.tcp_ack_var).grid(row=4, column=0, columnspan=2, sticky="w", padx=8, pady=4)
        self.tcp_btn = ttk.Button(box, text="Start TCP Host", command=self.toggle_tcp)
        self.tcp_btn.grid(row=5, column=0, columnspan=2, sticky="ew", padx=8, pady=(4, 8))

    def _build_play(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="Manual MQTT Play")
        box.grid(row=3, column=0, sticky="ew", pady=(0, 10))
        box.columnconfigure(1, weight=1)
        self.play_scene_var = tk.StringVar(value="boot")
        self.play_seq_var = tk.IntVar(value=1)
        ttk.Label(box, text="Scene").grid(row=0, column=0, sticky="w", padx=8, pady=(8, 4))
        ttk.Entry(box, textvariable=self.play_scene_var, width=18).grid(row=0, column=1, sticky="ew", padx=8, pady=(8, 4))
        ttk.Label(box, text="Seq").grid(row=1, column=0, sticky="w", padx=8, pady=4)
        ttk.Entry(box, textvariable=self.play_seq_var, width=10).grid(row=1, column=1, sticky="w", padx=8, pady=4)
        ttk.Button(box, text="Play via MQTT", command=self.play_manual).grid(row=2, column=0, columnspan=2, sticky="ew", padx=8, pady=(4, 8))

    def _build_status(self, parent: ttk.Frame) -> None:
        top = ttk.Frame(parent)
        top.grid(row=0, column=0, sticky="ew")
        top.columnconfigure(0, weight=1)
        ttk.Label(top, text="Bridge Status", style="Title.TLabel").grid(row=0, column=0, sticky="w")
        self.mqtt_state_var = tk.StringVar(value="MQTT: disconnected")
        self.tcp_state_var = tk.StringVar(value="SILAB: stopped")
        ttk.Label(top, textvariable=self.mqtt_state_var).grid(row=0, column=1, sticky="e")
        ttk.Label(top, textvariable=self.tcp_state_var).grid(row=1, column=1, sticky="e")

        status = ttk.LabelFrame(parent, text="ESP32 / SILAB")
        status.grid(row=1, column=0, sticky="ew", pady=(8, 10))
        for col in range(4):
            status.columnconfigure(col, weight=1)
        self.status_vars: dict[str, tk.StringVar] = {}
        fields = [
            ("WiFi", "wifi"),
            ("MQTT", "mqtt"),
            ("Audio", "audio"),
            ("SD", "sd"),
            ("Path", "path"),
            ("SILAB", "silab"),
            ("Trigger", "trigger"),
            ("Files", "files"),
        ]
        for index, (label, key) in enumerate(fields):
            row = index // 4
            col = index % 4
            cell = ttk.Frame(status, padding=6)
            cell.grid(row=row, column=col, sticky="ew")
            ttk.Label(cell, text=label).grid(row=0, column=0, sticky="w")
            var = tk.StringVar(value="-")
            self.status_vars[key] = var
            ttk.Label(cell, textvariable=var, wraplength=220).grid(row=1, column=0, sticky="w")

    def _build_log(self, parent: ttk.Frame) -> None:
        log_frame = ttk.LabelFrame(parent, text="Log")
        log_frame.grid(row=2, column=0, sticky="nsew")
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        self.log_text = tk.Text(log_frame, wrap="word", height=24, font=("Consolas", 10))
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
        self.port_var.set("auto")

    def start_broker(self) -> None:
        if self.broker_proc and self.broker_proc.poll() is None:
            self._log("[broker] already running from GUI")
            return
        script = Path(__file__).with_name("mqtt_server.py")
        port = str(int(self.mqtt_port_var.get()))
        try:
            self.broker_proc = subprocess.Popen(
                [sys.executable, str(script), "--port", port],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
        except Exception as exc:
            messagebox.showerror("Broker", str(exc))
            return
        threading.Thread(target=self._read_broker_output, daemon=True).start()

    def _read_broker_output(self) -> None:
        proc = self.broker_proc
        if not proc or not proc.stdout:
            return
        for line in proc.stdout:
            self.events.put(("log", "[broker] " + line.rstrip()))
        self.events.put(("log", f"[broker] exited rc={proc.poll()}"))

    def toggle_mqtt(self) -> None:
        if self.mqtt.connected:
            self.mqtt.disconnect()
            self.mqtt_btn.configure(text="Connect")
            self.mqtt_state_var.set("MQTT: disconnected")
            return
        try:
            self.mqtt.connect(
                self.mqtt_host_var.get().strip(),
                int(self.mqtt_port_var.get()),
                self.cmd_topic_var.get().strip(),
                self.status_topic_var.get().strip(),
            )
        except Exception as exc:
            messagebox.showerror("MQTT", str(exc))

    def toggle_tcp(self) -> None:
        if self.tcp_bridge.running:
            self.tcp_bridge.stop()
            return
        try:
            self.tcp_bridge.start(self.tcp_host_var.get().strip(), int(self.tcp_port_var.get()))
        except Exception as exc:
            messagebox.showerror("SILAB TCP", str(exc))

    def fill_usb_broker_uri(self) -> None:
        host = self.mqtt_host_var.get().strip()
        port = int(self.mqtt_port_var.get())
        if host in ("", "0.0.0.0", "127.0.0.1", "localhost"):
            host = socket.gethostbyname(socket.gethostname())
        self.usb_broker_var.set(f"mqtt://{host}:{port}")

    def _selected_port(self) -> str:
        label = self.port_var.get() or "auto"
        return self.port_map.get(label, label)

    def configure_usb_mqtt(self) -> None:
        broker = self.usb_broker_var.get().strip()
        if not broker:
            messagebox.showwarning("USB MQTT", "Broker URI is empty")
            return
        threading.Thread(target=self._usb_send_worker, args=(f"mqtt {broker}",), daemon=True).start()

    def configure_usb_wifi(self) -> None:
        ssid = self.ssid_var.get().strip()
        if not ssid:
            messagebox.showwarning("USB WiFi", "SSID is empty")
            return
        password = self.password_var.get()
        payload = json.dumps({"type": "wifi", "ssid": ssid, "password": password}, separators=(",", ":"))
        threading.Thread(target=self._usb_send_worker, args=(payload,), daemon=True).start()

    def _usb_send_worker(self, line: str) -> None:
        password = self.password_var.get()
        safe_line = line
        if line.startswith("{") and password:
            safe_line = line.replace(password, "***")
        self.events.put(("log", f"[usb] > {safe_line}"))
        try:
            lines = send_usb_line(self._selected_port(), int(self.baud_var.get()), line)
        except Exception as exc:
            self.events.put(("log", f"[usb] failed: {exc}"))
            return
        for item in lines:
            self.events.put(("log", f"[usb] {item}"))

    def _publish_mqtt(self, payload: dict) -> bool:
        return self.mqtt.publish(payload)

    def play_manual(self) -> None:
        scene = self.play_scene_var.get().strip()
        seq = int(self.play_seq_var.get())
        if not scene or seq < 1 or seq > 65535:
            messagebox.showwarning("Play", "Scene must be non-empty and seq must be 1..65535")
            return
        scene = normalize_scene(scene, self.scene_prefix_var.get().strip() or DEFAULT_SCENE_PREFIX)
        self.mqtt.publish({"type": "play", "scene": scene, "seq": seq, "message_id": f"manual_{int(time.time() * 1000)}"})

    def query_status(self) -> None:
        self.mqtt.publish({"type": "status", "query": "all"})

    def _auto_status(self) -> None:
        if self.mqtt.connected:
            self.query_status()
        self.auto_status_after = self.after(3000, self._auto_status)

    def _drain_events(self) -> None:
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "log":
                    self._log(str(payload))
                elif kind == "status":
                    self._update_status(payload if isinstance(payload, dict) else {})
                elif kind == "silab_packet":
                    self._update_silab(payload if isinstance(payload, dict) else {})
                elif kind == "tcp_running":
                    running = bool(payload)
                    self.tcp_btn.configure(text="Stop TCP Host" if running else "Start TCP Host")
                    self.tcp_state_var.set("SILAB: listening" if running else "SILAB: stopped")
        except queue.Empty:
            pass
        self.mqtt_state_var.set("MQTT: connected" if self.mqtt.connected else "MQTT: disconnected")
        self.mqtt_btn.configure(text="Disconnect" if self.mqtt.connected else "Connect")
        self.after(80, self._drain_events)

    def _update_status(self, payload: dict) -> None:
        if payload.get("type") != "status":
            return
        wifi = payload.get("wifi", {})
        mqtt_status = payload.get("mqtt", {})
        audio = payload.get("audio", {})
        wifi_text = "connected" if wifi.get("connected") else "offline"
        if wifi.get("ip"):
            wifi_text += f" {wifi.get('ip')}"
        self.status_vars["wifi"].set(wifi_text)
        self.status_vars["mqtt"].set("connected" if mqtt_status.get("connected") else "off")
        audio_text = "ready" if audio.get("ready") else "not ready"
        if audio.get("last_error"):
            audio_text += f" err={audio.get('last_error')}"
        self.status_vars["audio"].set(audio_text)
        self.status_vars["sd"].set("mounted" if audio.get("sd_mounted") else "not mounted")
        self.status_vars["path"].set(str(audio.get("current_path") or "-"))
        self.status_vars["files"].set(str(audio.get("files_played", 0)))

    def _update_silab(self, packet: dict) -> None:
        self.status_vars["silab"].set(
            f"#{packet.get('packet_no')} {packet.get('state')} {packet.get('scene')}/{int(packet.get('seq', 0)):03d}"
        )
        self.status_vars["trigger"].set(
            f"{packet.get('trigger')} count={packet.get('trigger_count')} ok={packet.get('accepted')}"
        )

    def _log(self, text: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self.log_text.insert("end", f"{stamp} {text}\n")
        line_count = int(self.log_text.index("end-1c").split(".")[0])
        if line_count > 1500:
            self.log_text.delete("1.0", "250.0")
        self.log_text.see("end")

    def _on_close(self) -> None:
        if self.auto_status_after:
            self.after_cancel(self.auto_status_after)
        self.tcp_bridge.stop()
        self.mqtt.disconnect()
        if self.broker_proc and self.broker_proc.poll() is None:
            self.broker_proc.terminate()
            try:
                self.broker_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.broker_proc.kill()
        self.destroy()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="SILAB TCP to ESP32 MQTT bridge GUI.")
    parser.add_argument("--broker", default=DEFAULT_MQTT_HOST, help="MQTT broker host for this GUI. Default: 127.0.0.1")
    parser.add_argument("--mqtt-port", type=int, default=DEFAULT_MQTT_PORT, help="MQTT broker port. Default: 1883")
    parser.add_argument("--topic", default=DEFAULT_CMD_TOPIC, help="ESP32 command topic")
    parser.add_argument("--status-topic", default=DEFAULT_STATUS_TOPIC, help="ESP32 status topic")
    parser.add_argument("--tcp-host", default=DEFAULT_TCP_HOST, help="SILAB TCP bind host. Default: 0.0.0.0")
    parser.add_argument("--tcp-port", type=int, default=DEFAULT_TCP_PORT, help="SILAB TCP port. Default: 7777")
    parser.add_argument("--trigger-threshold", type=float, default=0.5, help="Trigger active threshold. Default: 0.5")
    parser.add_argument("--scene-prefix", default=DEFAULT_SCENE_PREFIX, help="Numeric scene prefix. Default: scene")
    parser.add_argument("--tcp-ack", action="store_true", help="Send JSON ACK lines back to SILAB TCP client")
    parser.add_argument("--port", default="auto", help="ESP32 USB serial port for setup. Default: auto")
    parser.add_argument("--baud", type=int, default=115_200, help="ESP32 USB serial baud. Default: 115200")
    parser.add_argument("--ssid", default="", help="Pre-fill ESP32 WiFi SSID")
    parser.add_argument("--password", default="", help="Pre-fill ESP32 WiFi password")
    parser.add_argument("--esp32-broker-uri", default="", help="Pre-fill ESP32 broker URI, for example mqtt://192.168.0.10:1883")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    app = BridgeGui(args)
    app.mainloop()


if __name__ == "__main__":
    main()
