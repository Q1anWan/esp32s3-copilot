#!/usr/bin/env python3
"""PC-side logic-decision TCP to ESP32 bridge GUI.

Topology:
  Logic decision program --LAN/TCP--> experiment PC bridge GUI
                                      └─ direct TCP/MQTT --> ESP32

MQTT is still used for ESP32 status, and as a play-command fallback when the
direct TCP host is unavailable.

The PC listens for fixed-rate logic-program audio-ID packets:
  TRIGGER;TIMESTAMP;SCENE;SEQ;SPEED<LF>

Only trigger rising edges are forwarded to ESP32 as MQTT play commands.
SCENE is the TF-card audio folder, and SEQ is the file name without extension.
"""

from __future__ import annotations

import argparse
import json
import math
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
DEFAULT_TCP_HOST = os.environ.get("COPILOT_LOGIC_TCP_HOST", os.environ.get("SILAB_TCP_HOST", "0.0.0.0"))
DEFAULT_TCP_PORT = int(os.environ.get("COPILOT_LOGIC_TCP_PORT", os.environ.get("SILAB_TCP_PORT", "7777")))
DEFAULT_ESP_TCP_HOST = os.environ.get("COPILOT_ESP_TCP_HOST", "")
DEFAULT_ESP_TCP_PORT = int(os.environ.get("COPILOT_ESP_TCP_PORT", "7777"))


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


def open_usb_serial(port: str, baud: int):
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
    disable_hangup_on_close(ser)
    ser.dtr = False
    ser.rts = False
    return ser


def read_usb_lines(ser, read_seconds: float) -> list[str]:
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


def send_usb_line(port: str, baud: int, line: str, read_seconds: float = 2.0) -> list[str]:
    ser = open_usb_serial(port, baud)
    try:
        time.sleep(0.2)
        ser.write((line.rstrip("\r\n") + "\n").encode("utf-8"))
        ser.flush()
        return read_usb_lines(ser, read_seconds)
    finally:
        ser.close()


def extract_usb_status(lines: list[str]) -> dict | None:
    for line in reversed(lines):
        if not line.startswith("COPILOT_STATUS "):
            continue
        try:
            return json.loads(line[len("COPILOT_STATUS ") :])
        except json.JSONDecodeError:
            return None
    return None


def usb_wifi_command_line(ssid: str, password: str) -> str:
    if any(ch.isspace() for ch in ssid) or any(ch.isspace() for ch in password):
        return json.dumps({"type": "wifi", "ssid": ssid, "password": password}, separators=(",", ":"))
    return f"wifi {ssid} {password}".rstrip()


def usb_wait_for_status(ser, lines: list[str], timeout: float = 8.0) -> dict | None:
    deadline = time.time() + timeout
    last_status = extract_usb_status(lines)
    if last_status:
        return last_status
    while time.time() < deadline:
        ser.write(b"status\n")
        ser.flush()
        chunk = read_usb_lines(ser, 1.0)
        lines.extend(chunk)
        status = extract_usb_status(chunk)
        if status:
            return status
        time.sleep(0.25)
    return None


def read_usb_config(port: str, baud: int, timeout: float = 10.0) -> dict:
    ser = open_usb_serial(port, baud)
    lines: list[str] = []
    try:
        started = time.time()
        deadline = time.time() + min(timeout, 8.0)
        while time.time() < deadline:
            chunk = read_usb_lines(ser, 0.25)
            lines.extend(chunk)
            if any("COPILOT_SERIAL ready" in line for line in chunk):
                break
            if not lines and time.time() - started >= 1.0:
                break

        status = usb_wait_for_status(ser, lines, timeout=timeout)
        return {"status": status, "lines": lines}
    finally:
        ser.close()


def configure_usb_wifi_flash(port: str, baud: int, ssid: str, password: str, wait_seconds: float = 30.0) -> dict:
    ser = open_usb_serial(port, baud)
    lines: list[str] = []
    statuses: list[dict] = []
    accepted = False
    command = usb_wifi_command_line(ssid, password)
    try:
        started = time.time()
        deadline = time.time() + 8.0
        while time.time() < deadline:
            chunk = read_usb_lines(ser, 0.25)
            lines.extend(chunk)
            if any("COPILOT_SERIAL ready" in line for line in chunk):
                break
            if not lines and time.time() - started >= 1.0:
                break

        status = usb_wait_for_status(ser, lines, timeout=8.0)
        if status:
            statuses.append(status)

        for _ in range(3):
            ser.write((command + "\n").encode("utf-8"))
            ser.flush()
            chunk = read_usb_lines(ser, 3.0)
            lines.extend(chunk)
            if command.startswith("{"):
                accepted = any(line.startswith("COPILOT_OK json_accepted") for line in chunk)
            else:
                accepted = any(line.startswith("COPILOT_OK wifi ") for line in chunk)
            status = extract_usb_status(chunk)
            if status:
                statuses.append(status)
            if accepted:
                break
            time.sleep(0.4)

        poll_deadline = time.time() + wait_seconds
        while time.time() < poll_deadline:
            ser.write(b"status\n")
            ser.flush()
            chunk = read_usb_lines(ser, 1.2)
            lines.extend(chunk)
            status = extract_usb_status(chunk)
            if status:
                statuses.append(status)
                wifi = status.get("wifi") or {}
                current = str(wifi.get("ssid") or "")
                saved = str(wifi.get("saved_ssid") or "")
                if current == ssid and (wifi.get("connected") or saved == ssid or wifi.get("saved")):
                    if wifi.get("connected"):
                        break
            time.sleep(0.8)
    finally:
        ser.close()
    return {"accepted": accepted, "lines": lines, "statuses": statuses, "command": command}


def parse_integer_like(text: str) -> int | None:
    try:
        value = float(text)
    except ValueError:
        return None
    rounded = int(round(value))
    if abs(value - rounded) > 0.0001 or rounded < 0:
        return None
    return rounded


def parse_finite_float(text: str) -> float | None:
    try:
        value = float(text)
    except ValueError:
        return None
    return value if math.isfinite(value) else None


def scene_token_is_valid(scene: str) -> bool:
    if parse_integer_like(scene) is not None:
        return True
    allowed = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-"
    return bool(scene) and all(c in allowed for c in scene)


def seq_token_is_valid(seq: str) -> bool:
    allowed = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-"
    return bool(seq) and all(c in allowed for c in seq)


def format_number_for_wire(value: float) -> str:
    if math.isfinite(value) and abs(value - round(value)) < 0.0001:
        return str(int(round(value)))
    return f"{value:.6f}".rstrip("0").rstrip(".")


def split_silab_fields(text: str) -> list[str]:
    normalized = text.replace(";", " ").replace(",", " ").replace("\t", " ")
    return [item for item in normalized.split() if item]


def reconstruct_silab_timestamp(time_low_text: str, time_high_text: str) -> tuple[float, str] | None:
    try:
        low = float(time_low_text)
        high = float(time_high_text)
    except ValueError:
        return None
    if not (math.isfinite(low) and math.isfinite(high)):
        return None
    high_int = int(math.floor(high))
    low_int = int(math.floor(low))
    frac = low - low_int
    if high_int < 0 or low_int < 0:
        return None
    # TIME_LOW carries the low 6 digits, while TIME_HIGH already covers the
    # 100000-second digit. Keep only the low 5 digits during reconstruction.
    return (float(high_int * 100000 + (low_int % 100000)) + frac, "trigger_low6_high100000")


def reconstruct_split_timestamp(part_a: str, part_b: str) -> tuple[float, str] | None:
    """Reconstruct Unix seconds from two SILAB-friendly timestamp parts.

    Kept for backward compatibility with older test packets.

    Supported forms:
    - high + low:         17792, 35200   -> 1779235200
    - div10000 + low:     177923, 35200  -> 1779235200
    - base + low:         1779200000, 35200 -> 1779235200
    - negative offset + low: -1779200000, 35200 -> 1779235200
    """

    try:
        a = float(part_a)
        b = float(part_b)
    except ValueError:
        return None
    if not (math.isfinite(a) and math.isfinite(b)):
        return None

    a_int = int(math.floor(a))
    b_int = int(math.floor(b))
    frac = b - b_int

    if a_int < 0:
        return (-float(a_int) + b, "offset_low")
    if a_int >= 1_000_000_000:
        return (float(a_int) + b, "base_low")
    if a_int >= 100_000:
        return (float(a_int * 10_000 + (b_int % 10_000)) + frac, "div10000_low")
    if a_int >= 1:
        return (float(a_int * 100_000 + (b_int % 100_000)) + frac, "high_low")
    return None


@dataclass
class AudioIdPacket:
    raw_text: str
    timestamp: float
    timestamp_source: str
    trigger: float
    scene: str
    seq: str
    speed: float | None = None


def parse_audio_id_line(line: str) -> AudioIdPacket | None:
    text = line.strip().strip(";")
    if not text:
        return None
    parts = split_silab_fields(text)
    timestamp = time.time()
    timestamp_source = "pc_local"
    speed: float | None = None
    if len(parts) == 3:
        trigger_text, scene, seq_text = parts
    elif len(parts) == 4:
        try:
            timestamp = float(parts[0])
        except ValueError:
            return None
        if not math.isfinite(timestamp):
            return None
        timestamp_source = "full"
        trigger_text, scene, seq_text = parts[1], parts[2], parts[3]
    elif len(parts) == 5:
        logic_trigger = parse_finite_float(parts[0])
        logic_timestamp = parse_finite_float(parts[1])
        logic_speed = parse_finite_float(parts[4])
        if (
            logic_trigger is not None
            and logic_timestamp is not None
            and logic_speed is not None
            and scene_token_is_valid(parts[2])
            and seq_token_is_valid(parts[3])
        ):
            timestamp = logic_timestamp
            timestamp_source = "logic_full"
            trigger_text, scene, seq_text = parts[0], parts[2], parts[3]
            speed = logic_speed
        else:
            reconstructed = reconstruct_split_timestamp(parts[0], parts[1])
            if reconstructed is None:
                return None
            timestamp, timestamp_source = reconstructed
            trigger_text, scene, seq_text = parts[2], parts[3], parts[4]
    elif len(parts) == 6:
        reconstructed = reconstruct_silab_timestamp(parts[1], parts[2])
        if reconstructed is None:
            return None
        timestamp, timestamp_source = reconstructed
        trigger_text, scene, seq_text, speed_text = parts[0], parts[3], parts[4], parts[5]
        try:
            speed = float(speed_text)
        except ValueError:
            return None
        if not math.isfinite(speed):
            return None
    else:
        return None
    trigger = parse_finite_float(trigger_text)
    if trigger is None:
        return None
    if not scene_token_is_valid(scene):
        return None
    if not seq_token_is_valid(seq_text):
        return None
    return AudioIdPacket(
        raw_text=text,
        timestamp=timestamp,
        timestamp_source=timestamp_source,
        trigger=trigger,
        scene=scene,
        seq=seq_text,
        speed=speed,
    )


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
                self.events.put(("log", f"[mqtt] connected {self.host}:{self.port}"))
            else:
                self.events.put(("log", f"[mqtt] connect failed rc={reason_code}"))

        def on_disconnect(client, userdata, flags=None, reason_code=None, properties=None):
            self.connected = False
            self.events.put(("log", "[mqtt] disconnected"))

        def on_message(client, userdata, msg):
            text = msg.payload.decode("utf-8", errors="replace")
            try:
                payload = json.loads(text)
            except json.JSONDecodeError:
                self.events.put(("log", f"[mqtt] ignored non-JSON status on {msg.topic}"))
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

    def publish(self, payload: dict, quiet: bool = False) -> bool:
        if not self.client or not self.connected:
            self.events.put(("log", "[mqtt] publish skipped: not connected"))
            return False
        data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        info = self.client.publish(self.cmd_topic, data, qos=1)
        info.wait_for_publish(timeout=2)
        ok = info.rc == 0
        if not quiet:
            self.events.put(("mqtt_publish", {"payload": dict(payload), "ok": ok, "rc": info.rc}))
        return ok


class LogicTcpBridge:
    def __init__(
        self,
        events: "queue.Queue[tuple[str, object]]",
        publish_cb,
        threshold_cb,
        ack_cb,
    ) -> None:
        self.events = events
        self.publish_cb = publish_cb
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
        self._thread = threading.Thread(target=self._server_loop, args=(host, port), name="logic-tcp-host", daemon=True)
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
                self.events.put(("log", f"[logic] TCP host listening on {host}:{port}"))
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
            self.events.put(("log", f"[logic] host failed: {exc}"))
        finally:
            self._listen_socket = None
            self.events.put(("tcp_running", False))

    def _client_loop(self, conn: socket.socket, addr) -> None:
        peer = f"{addr[0]}:{addr[1]}"
        self.events.put(("log", f"[logic] connected {peer}"))
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
                            self.events.put(("log", "[logic] drop overlong line"))
                            buffer.clear()
            if buffer:
                self._handle_line(conn, bytes(buffer).decode("utf-8", errors="replace"), peer)
        self.events.put(("log", f"[logic] disconnected {peer}"))

    def _handle_line(self, conn: socket.socket, line: str, peer: str) -> None:
        packet = parse_audio_id_line(line)
        with self._lock:
            self._packet_count += 1
            packet_no = self._packet_count
        if packet is None:
            self.events.put(("log", f"[logic] #{packet_no} BAD {line.strip()!r}"))
            self._send_ack(conn, False, False, "", "", "bad_format")
            return

        active = packet.trigger >= float(self.threshold_cb())
        with self._lock:
            rising = active and not self._latched
            self._latched = active
            if rising:
                self._trigger_count += 1
            trigger_count = self._trigger_count

        scene = packet.scene
        accepted = False
        if rising:
            payload = {
                "type": "play",
                "scene": scene,
                "seq": packet.seq,
                "message_id": f"logic_{packet_no}_{trigger_count}_{int(time.time() * 1000)}",
            }
            accepted = bool(self.publish_cb(payload))

        state = "play" if rising else ("held" if active else "idle")
        self.events.put(
            (
                "logic_packet",
                {
                    "packet_no": packet_no,
                    "peer": peer,
                    "timestamp": packet.timestamp,
                    "timestamp_source": packet.timestamp_source,
                    "trigger": packet.trigger,
                    "scene": scene,
                    "seq": packet.seq,
                    "speed": packet.speed,
                    "state": state,
                    "accepted": accepted,
                    "trigger_count": trigger_count,
                },
            )
        )
        self.events.put(
            (
                "log",
                f"[logic] #{packet_no} {state} ts={format_number_for_wire(packet.timestamp)} "
                f"trigger={packet.trigger:g} scene={scene} seq={packet.seq} "
                f"speed={format_number_for_wire(packet.speed) if packet.speed is not None else '-'}",
            )
        )
        self._send_ack(conn, True, accepted, scene, packet.seq, state, packet.timestamp, packet.speed)

    def _send_ack(
        self,
        conn: socket.socket,
        ok: bool,
        accepted: bool,
        scene: str,
        seq: str,
        state: str,
        timestamp: float | None = None,
        speed: float | None = None,
    ) -> None:
        if not self.ack_cb():
            return
        payload = {"ok": ok, "accepted": accepted, "scene": scene, "seq": seq, "state": state}
        if timestamp is not None:
            payload["timestamp"] = timestamp
        if speed is not None:
            payload["speed"] = speed
        try:
            conn.sendall((json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8"))
        except OSError:
            pass


class BridgeGui(tk.Tk):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__()
        self.title("Copilot ESP32 Bridge")
        self.minsize(1120, 720)
        self.args = args
        self.events: "queue.Queue[tuple[str, object]]" = queue.Queue()
        self.mqtt = MqttController(self.events)
        self._trigger_threshold = float(args.trigger_threshold)
        self._tcp_ack = bool(args.tcp_ack)
        self._direct_tcp_enabled = not args.no_direct_tcp
        self._esp_tcp_host = args.esp_tcp_host.strip()
        self._esp_tcp_port = int(args.esp_tcp_port)
        self.tcp_bridge = LogicTcpBridge(
            self.events,
            publish_cb=self._send_play_command,
            threshold_cb=lambda: self._trigger_threshold,
            ack_cb=lambda: self._tcp_ack,
        )
        self.port_map: dict[str, str] = {}
        self.broker_proc: subprocess.Popen | None = None
        self.auto_status_after: str | None = None
        self._last_files_played: int | None = None
        self._last_audio_error = ""
        self._last_audio_path = ""
        self._last_wifi_connected: bool | None = None
        self._last_mqtt_connected: bool | None = None
        self._last_sd_mounted: bool | None = None
        self._last_esp_status_at = 0.0
        self._last_direct_host = ""
        self._pending_plays: dict[str, dict] = {}
        self._played_event_keys: set[tuple[int, str]] = set()
        self._build_style()
        self._build_ui()
        self._sync_runtime_config()
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
        ttk.Button(row, text="Start Dev Broker", command=self.start_broker).grid(row=0, column=0, sticky="ew")
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
        for col in range(4):
            row.columnconfigure(col, weight=1)
        ttk.Button(row, text="Read Config", command=self.read_usb_config).grid(row=0, column=0, sticky="ew")
        ttk.Button(row, text="Use URI", command=self.fill_usb_broker_uri).grid(row=0, column=1, sticky="ew", padx=(4, 0))
        ttk.Button(row, text="Save MQTT", command=self.configure_usb_mqtt).grid(row=0, column=2, sticky="ew", padx=4)
        ttk.Button(row, text="Save WiFi", command=self.configure_usb_wifi).grid(row=0, column=3, sticky="ew")

    def _build_tcp(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="Logic TCP Host")
        box.grid(row=2, column=0, sticky="ew", pady=(0, 10))
        box.columnconfigure(1, weight=1)
        self.tcp_host_var = tk.StringVar(value=self.args.tcp_host)
        self.tcp_port_var = tk.IntVar(value=self.args.tcp_port)
        self.threshold_var = tk.DoubleVar(value=self.args.trigger_threshold)
        self.tcp_ack_var = tk.BooleanVar(value=self.args.tcp_ack)
        ttk.Label(box, text="Bind").grid(row=0, column=0, sticky="w", padx=8, pady=(8, 4))
        ttk.Entry(box, textvariable=self.tcp_host_var, width=18).grid(row=0, column=1, sticky="ew", padx=8, pady=(8, 4))
        ttk.Label(box, text="Port").grid(row=1, column=0, sticky="w", padx=8, pady=4)
        ttk.Entry(box, textvariable=self.tcp_port_var, width=10).grid(row=1, column=1, sticky="w", padx=8, pady=4)
        ttk.Label(box, text="Threshold").grid(row=2, column=0, sticky="w", padx=8, pady=4)
        ttk.Entry(box, textvariable=self.threshold_var, width=10).grid(row=2, column=1, sticky="w", padx=8, pady=4)
        ttk.Label(box, text="Frame").grid(row=3, column=0, sticky="w", padx=8, pady=4)
        ttk.Label(box, text="TRIGGER;TIMESTAMP;SCENE;SEQ;SPEED").grid(
            row=3, column=1, sticky="w", padx=8, pady=4
        )
        ttk.Checkbutton(box, text="Send TCP ACK", variable=self.tcp_ack_var).grid(row=4, column=0, columnspan=2, sticky="w", padx=8, pady=4)
        self.tcp_btn = ttk.Button(box, text="Start TCP Host", command=self.toggle_tcp)
        self.tcp_btn.grid(row=5, column=0, columnspan=2, sticky="ew", padx=8, pady=(4, 8))

    def _build_play(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="Manual Play")
        box.grid(row=3, column=0, sticky="ew", pady=(0, 10))
        box.columnconfigure(1, weight=1)
        self.play_scene_var = tk.StringVar(value="boot")
        self.play_seq_var = tk.StringVar(value="1")
        self.direct_tcp_var = tk.BooleanVar(value=not self.args.no_direct_tcp)
        self.esp_tcp_host_var = tk.StringVar(value=self.args.esp_tcp_host)
        self.esp_tcp_port_var = tk.IntVar(value=self.args.esp_tcp_port)
        ttk.Label(box, text="Scene").grid(row=0, column=0, sticky="w", padx=8, pady=(8, 4))
        ttk.Entry(box, textvariable=self.play_scene_var, width=18).grid(row=0, column=1, sticky="ew", padx=8, pady=(8, 4))
        ttk.Label(box, text="Seq").grid(row=1, column=0, sticky="w", padx=8, pady=4)
        ttk.Entry(box, textvariable=self.play_seq_var, width=10).grid(row=1, column=1, sticky="w", padx=8, pady=4)
        ttk.Checkbutton(box, text="Direct ESP TCP", variable=self.direct_tcp_var).grid(
            row=2, column=0, columnspan=2, sticky="w", padx=8, pady=4
        )
        ttk.Label(box, text="ESP Host").grid(row=3, column=0, sticky="w", padx=8, pady=4)
        ttk.Entry(box, textvariable=self.esp_tcp_host_var, width=18).grid(row=3, column=1, sticky="ew", padx=8, pady=4)
        ttk.Label(box, text="ESP Port").grid(row=4, column=0, sticky="w", padx=8, pady=4)
        ttk.Entry(box, textvariable=self.esp_tcp_port_var, width=10).grid(row=4, column=1, sticky="w", padx=8, pady=4)
        row = ttk.Frame(box)
        row.grid(row=5, column=0, columnspan=2, sticky="ew", padx=8, pady=(4, 8))
        row.columnconfigure(0, weight=1)
        row.columnconfigure(1, weight=1)
        ttk.Button(row, text="Play", command=self.play_manual).grid(row=0, column=0, sticky="ew")
        ttk.Button(row, text="Stop", command=self.stop_manual).grid(row=0, column=1, sticky="ew", padx=(4, 0))

    def _build_status(self, parent: ttk.Frame) -> None:
        top = ttk.Frame(parent)
        top.grid(row=0, column=0, sticky="ew")
        top.columnconfigure(0, weight=1)
        ttk.Label(top, text="Bridge Status", style="Title.TLabel").grid(row=0, column=0, sticky="w")
        self.mqtt_state_var = tk.StringVar(value="MQTT: disconnected")
        self.tcp_state_var = tk.StringVar(value="Logic: stopped")
        ttk.Label(top, textvariable=self.mqtt_state_var).grid(row=0, column=1, sticky="e")
        ttk.Label(top, textvariable=self.tcp_state_var).grid(row=1, column=1, sticky="e")

        status = ttk.LabelFrame(parent, text="ESP32 / Logic")
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
            ("Logic", "logic"),
            ("Trigger", "trigger"),
            ("Speed", "speed"),
            ("Timestamp", "timestamp"),
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
        self._sync_runtime_config()
        try:
            self.tcp_bridge.start(self.tcp_host_var.get().strip(), int(self.tcp_port_var.get()))
        except Exception as exc:
            messagebox.showerror("Logic TCP", str(exc))

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

    def read_usb_config(self) -> None:
        threading.Thread(target=self._usb_read_config_worker, daemon=True).start()

    def _usb_read_config_worker(self) -> None:
        self.events.put(("log", "[usb] reading ESP32 config/status"))
        try:
            result = read_usb_config(self._selected_port(), int(self.baud_var.get()))
        except Exception as exc:
            self.events.put(("log", f"[usb] read config failed: {exc}"))
            return

        lines = result.get("lines") if isinstance(result.get("lines"), list) else []
        for item in lines:
            if isinstance(item, str) and item.startswith("COPILOT_SERIAL ready"):
                self.events.put(("log", "[usb] ESP32 serial ready"))
                break

        status = result.get("status")
        if not isinstance(status, dict):
            self.events.put(("log", "[usb] no ESP32 status returned"))
            return

        self.events.put(("status", status))
        mqtt_status = status.get("mqtt") or {}
        wifi = status.get("wifi") or {}
        broker = str(mqtt_status.get("broker") or "")
        ssid = str(wifi.get("saved_ssid") or wifi.get("ssid") or "")
        self.events.put(("usb_config", {"broker": broker, "ssid": ssid}))

        reason = wifi.get("last_reason") or 0
        self.events.put(
            (
                "log",
                f"[usb] config broker={broker or '-'} ssid={ssid or '-'} "
                f"saved={bool(wifi.get('saved'))} connected={bool(wifi.get('connected'))} "
                f"reason={reason} ip={wifi.get('ip') or '-'}",
            )
        )

    def configure_usb_wifi(self) -> None:
        ssid = self.ssid_var.get().strip()
        if not ssid:
            messagebox.showwarning("USB WiFi", "SSID is empty")
            return
        password = self.password_var.get()
        threading.Thread(target=self._usb_wifi_worker, args=(ssid, password), daemon=True).start()

    def _usb_wifi_worker(self, ssid: str, password: str) -> None:
        self.events.put(("log", f"[usb] saving WiFi SSID={ssid} to ESP32 Flash"))
        try:
            result = configure_usb_wifi_flash(self._selected_port(), int(self.baud_var.get()), ssid, password)
        except Exception as exc:
            self.events.put(("log", f"[usb] WiFi failed: {exc}"))
            return

        accepted = bool(result.get("accepted"))
        lines = result.get("lines") if isinstance(result.get("lines"), list) else []
        statuses = result.get("statuses") if isinstance(result.get("statuses"), list) else []

        logged = 0
        for item in lines:
            if not isinstance(item, str):
                continue
            if item.startswith("COPILOT_STATUS "):
                status = extract_usb_status([item])
                if status:
                    self.events.put(("status", status))
                continue
            if item.startswith("COPILOT_OK") or item.startswith("COPILOT_ERROR") or "WiFi config command" in item:
                safe = item.replace(password, "***") if password else item
                self.events.put(("log", f"[usb] {safe}"))
                logged += 1
                if logged >= 8:
                    break

        if accepted:
            self.events.put(("log", "[usb] WiFi command accepted; credentials saved to NVS/Flash"))
        else:
            self.events.put(("log", "[usb] WARNING: ESP32 did not ACK the WiFi command"))

        status = statuses[-1] if statuses else None
        if isinstance(status, dict):
            self.events.put(("status", status))
            wifi = status.get("wifi") or {}
            current = str(wifi.get("ssid") or "")
            saved = str(wifi.get("saved_ssid") or "")
            connected = bool(wifi.get("connected"))
            ip = str(wifi.get("ip") or "")
            reason = wifi.get("last_reason")
            flash_ok = bool(wifi.get("saved")) and (not saved or saved == ssid)
            self.events.put(
                (
                    "log",
                    f"[usb] WiFi status ssid={current or '-'} saved_ssid={saved or '-'} "
                    f"flash={'ok' if flash_ok else 'unknown'} connected={connected} "
                    f"reason={reason or 0} ip={ip or '-'}",
                )
            )
            if current == ssid and connected:
                self.events.put(("log", f"[usb] ESP32 connected to {ssid}"))
            elif flash_ok or current == ssid:
                self.events.put(("log", "[usb] Flash write is visible, but ESP32 is not connected yet"))
        else:
            self.events.put(("log", "[usb] no status returned after WiFi command"))

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

    def _sync_runtime_config(self) -> None:
        try:
            self._trigger_threshold = float(self.threshold_var.get())
        except (tk.TclError, ValueError):
            self._trigger_threshold = 0.5
        self._tcp_ack = bool(self.tcp_ack_var.get())
        self._direct_tcp_enabled = bool(self.direct_tcp_var.get())
        self._esp_tcp_host = self.esp_tcp_host_var.get().strip()
        try:
            self._esp_tcp_port = int(self.esp_tcp_port_var.get())
        except (tk.TclError, ValueError):
            self._esp_tcp_port = DEFAULT_ESP_TCP_PORT

    def _direct_tcp_target(self) -> tuple[str, int] | None:
        host = self._esp_tcp_host
        if not host:
            return None
        port = self._esp_tcp_port
        if port < 1 or port > 65535:
            return None
        return host, port

    def _send_direct_tcp(self, payload: dict) -> tuple[bool, str]:
        target = self._direct_tcp_target()
        if target is None:
            return False, "no ESP TCP target"
        host, port = target
        started = time.monotonic()
        data = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8") + b"\n"
        try:
            with socket.create_connection((host, port), timeout=0.35) as sock:
                sock.settimeout(0.12)
                try:
                    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                except OSError:
                    pass
                sock.sendall(data)
        except OSError as exc:
            return False, str(exc)
        elapsed_ms = int((time.monotonic() - started) * 1000)
        msg_type = payload.get("type", "cmd")
        action = payload.get("action", "")
        if msg_type == "play" or (msg_type in ("audio", "sound") and action != "stop"):
            scene = payload.get("scene", "-")
            seq = str(payload.get("seq", "-") or "-")
            summary = f"play {scene}/{seq}"
        elif action == "stop" or msg_type == "stop":
            summary = "stop"
        else:
            summary = str(msg_type)
        self.events.put(("log", f"[direct] {summary} sent {host}:{port} {elapsed_ms}ms"))
        return True, "sent"

    def _track_play_request(self, payload: dict) -> None:
        message_id = str(payload.get("message_id") or f"play_{int(time.time() * 1000)}")
        self._pending_plays[message_id] = {
            "scene": str(payload.get("scene") or "-"),
            "seq": str(payload.get("seq", "-") or "-"),
            "files_before": self._last_files_played,
            "t0": time.time(),
        }
        self.after(900, lambda: self.query_status(quiet=True))
        self.after(2200, lambda: self.query_status(quiet=True))

    def _publish_mqtt(self, payload: dict) -> bool:
        ok = self.mqtt.publish(payload)
        if ok and payload.get("type") == "play":
            self.events.put(("track_play", dict(payload)))
        return ok

    def _send_control_command(self, payload: dict, track_play: bool = False) -> bool:
        if self._direct_tcp_enabled:
            ok, reason = self._send_direct_tcp(payload)
            if ok:
                if track_play:
                    self.events.put(("track_play", dict(payload)))
                return True
            self.events.put(("log", f"[direct] failed {reason}; fallback MQTT"))
        return self._publish_mqtt(payload)

    def _send_play_command(self, payload: dict) -> bool:
        return self._send_control_command(payload, track_play=True)

    def play_manual(self) -> None:
        self._sync_runtime_config()
        scene = self.play_scene_var.get().strip()
        seq = self.play_seq_var.get().strip()
        if not scene or not seq_token_is_valid(seq):
            messagebox.showwarning("Play", "Scene must be non-empty and seq must use letters, digits, _ or -")
            return
        if not scene_token_is_valid(scene):
            messagebox.showwarning("Play", "Scene must use letters, digits, _ or -")
            return
        message_id = f"manual_{int(time.time() * 1000)}"
        payload = {"type": "play", "scene": scene, "seq": seq, "message_id": message_id}
        if self._send_play_command(payload):
            self._log(f"[manual] play requested {scene}/{seq}")

    def stop_manual(self) -> None:
        self._sync_runtime_config()
        message_id = f"stop_{int(time.time() * 1000)}"
        payload = {"type": "audio", "action": "stop", "message_id": message_id}
        if self._send_control_command(payload):
            self._pending_plays.clear()
            self._log("[manual] stop requested")
            self.after(300, lambda: self.query_status(quiet=True))
            self.after(1000, lambda: self.query_status(quiet=True))

    def query_status(self, quiet: bool = False) -> None:
        self.mqtt.publish({"type": "status", "query": "status"}, quiet=quiet)

    def _auto_status(self) -> None:
        now = time.time()
        play_recent = any(now - item.get("t0", 0) < 4.0 for item in self._pending_plays.values())
        if self.mqtt.connected and not play_recent:
            self.query_status(quiet=True)
        self.auto_status_after = self.after(8000 if not play_recent else 1000, self._auto_status)

    def _maybe_update_direct_target(self, host: str, port: int | None = None) -> None:
        host = host.strip()
        if not host:
            return
        current = self.esp_tcp_host_var.get().strip()
        if current and current != self._last_direct_host:
            return
        self.esp_tcp_host_var.set(host)
        self._last_direct_host = host
        self._esp_tcp_host = host
        if port is not None and 1 <= port <= 65535:
            self.esp_tcp_port_var.set(port)
            self._esp_tcp_port = port

    def _drain_events(self) -> None:
        self._sync_runtime_config()
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "log":
                    self._log(str(payload))
                elif kind == "mqtt_publish":
                    self._log_publish(payload if isinstance(payload, dict) else {})
                elif kind == "status":
                    self._update_status(payload if isinstance(payload, dict) else {})
                elif kind == "usb_config":
                    self._apply_usb_config(payload if isinstance(payload, dict) else {})
                elif kind == "logic_packet":
                    self._update_logic(payload if isinstance(payload, dict) else {})
                elif kind == "track_play":
                    self._track_play_request(payload if isinstance(payload, dict) else {})
                elif kind == "tcp_running":
                    running = bool(payload)
                    self.tcp_btn.configure(text="Stop TCP Host" if running else "Start TCP Host")
                    self.tcp_state_var.set("Logic: listening" if running else "Logic: stopped")
        except queue.Empty:
            pass
        self.mqtt_state_var.set("MQTT: connected" if self.mqtt.connected else "MQTT: disconnected")
        self.mqtt_btn.configure(text="Disconnect" if self.mqtt.connected else "Connect")
        self.after(80, self._drain_events)

    def _apply_usb_config(self, payload: dict) -> None:
        broker = str(payload.get("broker") or "")
        ssid = str(payload.get("ssid") or "")
        if broker:
            self.usb_broker_var.set(broker)
        if ssid:
            self.ssid_var.set(ssid)

    def _update_status(self, payload: dict) -> None:
        if payload.get("type") != "status":
            if payload.get("type") == "screen_event":
                state = payload.get("screen_state", "-")
                message_id = payload.get("message_id") or ""
                suffix = f" message={message_id}" if message_id else ""
                self._log(f"[esp32] screen {state}{suffix}")
            return
        self._last_esp_status_at = time.time()

        if "wifi" in payload:
            wifi = payload.get("wifi") or {}
            wifi_connected = bool(wifi.get("connected"))
            wifi_text = "connected" if wifi_connected else "offline"
            ssid = str(wifi.get("ssid") or "")
            saved_ssid = str(wifi.get("saved_ssid") or "")
            if wifi.get("ip"):
                wifi_text += f" {wifi.get('ip')}"
                self._maybe_update_direct_target(str(wifi.get("ip")), DEFAULT_ESP_TCP_PORT)
            if ssid:
                wifi_text += f" ssid={ssid}"
            if saved_ssid and saved_ssid != ssid:
                wifi_text += f" saved={saved_ssid}"
            elif wifi.get("saved"):
                wifi_text += " saved"
            reason = wifi.get("last_reason")
            if reason not in (None, 0, "0"):
                wifi_text += f" reason={reason}"
            self.status_vars["wifi"].set(wifi_text)
            if self._last_wifi_connected is not None and self._last_wifi_connected != wifi_connected:
                self._log(f"[esp32] WiFi {'connected' if wifi_connected else 'offline'}")
            self._last_wifi_connected = wifi_connected

        tcp_host = str(payload.get("tcp_host") or "")
        if tcp_host:
            host, port = tcp_host, None
            if ":" in tcp_host:
                host_part, port_text = tcp_host.rsplit(":", 1)
                try:
                    port = int(port_text)
                    host = host_part
                except ValueError:
                    port = None
            self._maybe_update_direct_target(host, port)

        if "mqtt" in payload:
            mqtt_status = payload.get("mqtt") or {}
            mqtt_connected = bool(mqtt_status.get("connected"))
            self.status_vars["mqtt"].set("connected" if mqtt_connected else "off")
            if self._last_mqtt_connected is not None and self._last_mqtt_connected != mqtt_connected:
                self._log(f"[esp32] MQTT {'connected' if mqtt_connected else 'offline'}")
            self._last_mqtt_connected = mqtt_connected

        if "audio" in payload:
            audio = payload.get("audio") or {}
            audio_text = "ready" if audio.get("ready") else "not ready"
            last_error = str(audio.get("last_error") or "")
            if last_error:
                audio_text += f" err={last_error}"
            self.status_vars["audio"].set(audio_text)

            sd_mounted = bool(audio.get("sd_mounted"))
            self.status_vars["sd"].set("mounted" if sd_mounted else "not mounted")
            if self._last_sd_mounted is not None and self._last_sd_mounted != sd_mounted:
                self._log(f"[esp32] TF card {'mounted' if sd_mounted else 'not mounted'}")
            self._last_sd_mounted = sd_mounted

            path = str(audio.get("current_path") or "-")
            files_played = int(audio.get("files_played", 0) or 0)
            self.status_vars["path"].set(path)
            self.status_vars["files"].set(str(files_played))

            if self._last_files_played is not None and files_played < self._last_files_played:
                self._played_event_keys.clear()
            if self._last_files_played is not None and files_played > self._last_files_played:
                event_key = (files_played, path)
                if event_key not in self._played_event_keys:
                    self._played_event_keys.add(event_key)
                    self._log(f"[audio] played {path} files={files_played}")
                self._pending_plays.clear()
            if last_error and last_error != self._last_audio_error:
                self._log(f"[audio] error {last_error} path={path}")
            if path != self._last_audio_path and path != "-":
                self._last_audio_path = path
            self._last_audio_error = last_error
            self._last_files_played = files_played

    def _update_logic(self, packet: dict) -> None:
        self.status_vars["logic"].set(
            f"#{packet.get('packet_no')} {packet.get('state')} {packet.get('scene')}/{packet.get('seq')}"
        )
        self.status_vars["trigger"].set(
            f"{packet.get('trigger')} count={packet.get('trigger_count')} ok={packet.get('accepted')}"
        )
        speed = packet.get("speed")
        self.status_vars["speed"].set(
            format_number_for_wire(float(speed)) if isinstance(speed, (int, float)) else "-"
        )
        timestamp = packet.get("timestamp")
        ts_text = format_number_for_wire(float(timestamp)) if isinstance(timestamp, (int, float)) else "-"
        self.status_vars["timestamp"].set(ts_text)

    def _log_publish(self, event: dict) -> None:
        payload = event.get("payload") or {}
        ok = bool(event.get("ok"))
        rc = event.get("rc")
        msg_type = payload.get("type")
        if msg_type == "play":
            scene = payload.get("scene", "-")
            seq = str(payload.get("seq", "-") or "-")
            self._log(f"[mqtt] play {'sent' if ok else 'failed'} {scene}/{seq} rc={rc}")
        elif msg_type in ("audio", "stop") and payload.get("action", "stop") == "stop":
            self._log(f"[mqtt] stop {'sent' if ok else 'failed'} rc={rc}")
        elif msg_type == "status":
            self._log(f"[mqtt] status query {'sent' if ok else 'failed'} rc={rc}")
        elif msg_type:
            self._log(f"[mqtt] {msg_type} {'sent' if ok else 'failed'} rc={rc}")

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
    parser = argparse.ArgumentParser(description="Logic-decision TCP to ESP32 bridge GUI.")
    parser.add_argument("--broker", default=DEFAULT_MQTT_HOST, help="MQTT broker host for this GUI. Default: 127.0.0.1")
    parser.add_argument("--mqtt-port", type=int, default=DEFAULT_MQTT_PORT, help="MQTT broker port. Default: 1883")
    parser.add_argument("--topic", default=DEFAULT_CMD_TOPIC, help="ESP32 command topic")
    parser.add_argument("--status-topic", default=DEFAULT_STATUS_TOPIC, help="ESP32 status topic")
    parser.add_argument("--tcp-host", default=DEFAULT_TCP_HOST, help="Logic TCP bind host. Default: 0.0.0.0")
    parser.add_argument("--tcp-port", type=int, default=DEFAULT_TCP_PORT, help="Logic TCP port. Default: 7777")
    parser.add_argument("--esp-tcp-host", default=DEFAULT_ESP_TCP_HOST, help="ESP32 direct TCP host. Auto-filled from status when empty")
    parser.add_argument("--esp-tcp-port", type=int, default=DEFAULT_ESP_TCP_PORT, help="ESP32 direct TCP port. Default: 7777")
    parser.add_argument("--no-direct-tcp", action="store_true", help="Disable direct TCP play and use MQTT for play commands")
    parser.add_argument("--trigger-threshold", type=float, default=0.5, help="Trigger active threshold. Default: 0.5")
    parser.add_argument("--tcp-ack", action="store_true", help="Send JSON ACK lines back to logic TCP client")
    parser.add_argument("--hrt-host", default="", help=argparse.SUPPRESS)
    parser.add_argument("--hrt-port", type=int, default=9001, help=argparse.SUPPRESS)
    parser.add_argument("--enable-hrt", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--scene-prefix", default="", help=argparse.SUPPRESS)
    parser.add_argument("--scene-aliases", default="", help=argparse.SUPPRESS)
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
