#!/usr/bin/env python3
import argparse
import os
import shutil
import subprocess
import sys
import tempfile


def find_mosquitto(custom_path):
    if custom_path:
        if os.path.isfile(custom_path):
            return custom_path
        return None

    for name in ("mosquitto", "mosquitto.exe"):
        path = shutil.which(name)
        if path:
            return path

    if os.name == "nt":
        candidates = [
            r"C:\Program Files\mosquitto\mosquitto.exe",
            r"C:\Program Files (x86)\mosquitto\mosquitto.exe",
        ]
        for path in candidates:
            if os.path.isfile(path):
                return path

    return None


def write_default_conf(port):
    temp_dir = tempfile.gettempdir()
    persist_dir = os.path.join(temp_dir, "mosquitto_copilot")
    os.makedirs(persist_dir, exist_ok=True)
    persist_path = persist_dir.replace("\\", "/")

    conf_text = "\n".join([
        f"listener {port} 0.0.0.0",
        "allow_anonymous true",
        "persistence true",
        f"persistence_location {persist_path}/",
        "log_type all",
        "",
    ])

    conf_path = os.path.join(temp_dir, "mosquitto_copilot.conf")
    with open(conf_path, "w", encoding="utf-8") as handle:
        handle.write(conf_text)
    return conf_path


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--port", type=int, default=int(os.getenv("MQTT_PORT", "1883")))
    parser.add_argument("-c", "--conf", default="")
    parser.add_argument("--mosquitto", default="")
    return parser.parse_args()


def main():
    args = parse_args()
    mosquitto_path = find_mosquitto(args.mosquitto)
    if not mosquitto_path:
        print("mosquitto not found.")
        print("Install mosquitto and ensure it is in PATH.")
        print("  Ubuntu/Debian: sudo apt install mosquitto")
        print("  Windows: https://mosquitto.org/download/")
        sys.exit(1)

    conf_path = args.conf.strip() if args.conf else write_default_conf(args.port)
    print(f"Starting mosquitto: {mosquitto_path}")
    print(f"Config: {conf_path}")

    proc = subprocess.Popen([mosquitto_path, "-v", "-c", conf_path])
    try:
        proc.wait()
    except KeyboardInterrupt:
        print("\nStopping mosquitto...")
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()

    return proc.returncode or 0


if __name__ == "__main__":
    raise SystemExit(main())
