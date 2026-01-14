#!/usr/bin/env python3
"""
Copilot Voice Backend - Main Entry Point

WebRTC voice backend for bidirectional audio communication with ESP32-S3.
Integrates with Volcano Engine for TTS and ASR.
"""

import asyncio
import logging
import signal
import sys
from pathlib import Path

import yaml

from src.signaling import SignalingServer


def load_config(config_path: str = "config.yaml") -> dict:
    """Load configuration from YAML file."""
    path = Path(config_path)
    if not path.exists():
        print(f"Config file not found: {config_path}")
        print("Using default configuration.")
        return {
            "server": {"host": "0.0.0.0", "port": 8080},
            "volcano": {"app_id": "", "access_token": ""},
            "logging": {"level": "INFO"},
        }

    with open(path, "r") as f:
        return yaml.safe_load(f)


def setup_logging(config: dict) -> None:
    """Configure logging from config."""
    log_config = config.get("logging", {})
    level = getattr(logging, log_config.get("level", "INFO").upper())
    log_format = log_config.get(
        "format", "%(asctime)s [%(levelname)s] %(name)s: %(message)s"
    )

    logging.basicConfig(level=level, format=log_format)

    # Reduce noise from third-party libraries
    logging.getLogger("aiohttp").setLevel(logging.WARNING)
    logging.getLogger("aiortc").setLevel(logging.WARNING)
    logging.getLogger("av").setLevel(logging.WARNING)


async def main_async(config: dict) -> None:
    """Main async entry point."""
    logger = logging.getLogger("main")

    # Create signaling server
    server_config = config.get("server", {})
    host = server_config.get("host", "0.0.0.0")
    port = server_config.get("port", 8080)

    server = SignalingServer(config)

    # Setup graceful shutdown
    loop = asyncio.get_running_loop()
    stop_event = asyncio.Event()

    def signal_handler():
        logger.info("Shutdown signal received")
        stop_event.set()

    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, signal_handler)

    # Start server
    logger.info(f"Starting voice backend on {host}:{port}")
    await server.start(host, port)

    # Wait for shutdown signal
    await stop_event.wait()

    # Cleanup
    logger.info("Shutting down...")
    await server.stop()
    logger.info("Shutdown complete")


def main() -> None:
    """Main entry point."""
    # Load configuration
    config_path = sys.argv[1] if len(sys.argv) > 1 else "config.yaml"
    config = load_config(config_path)

    # Setup logging
    setup_logging(config)

    logger = logging.getLogger("main")
    logger.info("Copilot Voice Backend starting...")

    # Check for Volcano credentials
    volcano_config = config.get("volcano", {})
    if not volcano_config.get("app_id") or not volcano_config.get("access_token"):
        logger.warning(
            "Volcano Engine credentials not configured. "
            "TTS/ASR will not work. Configure in config.yaml."
        )

    # Run async main
    try:
        asyncio.run(main_async(config))
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    except Exception as e:
        logger.exception(f"Fatal error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
