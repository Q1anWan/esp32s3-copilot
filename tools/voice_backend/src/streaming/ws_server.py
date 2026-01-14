"""
WebSocket Audio Streaming Server

Simplified audio streaming protocol for ESP32 clients.
More practical than full WebRTC for embedded systems.

Protocol:
- Binary frames: Raw PCM audio (int16, 16kHz, mono)
- Text frames: JSON control messages

Control Messages:
- {"type": "start", "device_id": "xxx"} - Start session
- {"type": "stop"} - Stop session
- {"type": "config", "sample_rate": 16000} - Configure audio
"""

import asyncio
import json
import logging
from typing import Callable, Dict, Optional
from uuid import uuid4

from aiohttp import web, WSMsgType
import numpy as np

from ..volcano import VolcanoTTSClient, VolcanoASRClient


logger = logging.getLogger(__name__)


class AudioStreamSession:
    """Manages a single audio streaming session."""

    def __init__(
        self,
        session_id: str,
        device_id: str,
        ws: web.WebSocketResponse,
        config: dict,
    ):
        self.session_id = session_id
        self.device_id = device_id
        self.ws = ws
        self.config = config

        self._tts: Optional[VolcanoTTSClient] = None
        self._asr: Optional[VolcanoASRClient] = None
        self._running = False

        # Stats
        self.frames_received = 0
        self.frames_sent = 0

    async def start(self) -> None:
        """Start the audio processing session."""
        self._running = True

        # Initialize Volcano clients
        self._tts = VolcanoTTSClient(self.config)
        self._asr = VolcanoASRClient(self.config)

        # Start ASR session
        try:
            await self._asr.start_session(
                on_partial=self._on_asr_partial,
                on_final=self._on_asr_final,
            )
        except Exception as e:
            logger.warning(f"ASR start failed: {e}")

        logger.info(f"Session {self.session_id} started for device {self.device_id}")

    async def stop(self) -> None:
        """Stop the session."""
        self._running = False

        if self._asr:
            await self._asr.stop_session()
            self._asr = None

        self._tts = None
        logger.info(f"Session {self.session_id} stopped")

    async def process_audio(self, pcm_data: bytes) -> None:
        """Process incoming PCM audio from ESP32."""
        self.frames_received += 1

        # Convert to numpy array
        samples = np.frombuffer(pcm_data, dtype=np.int16)

        # Feed to ASR
        if self._asr and self._asr.is_active:
            self._asr.feed_audio(samples)

    async def send_audio(self, samples: np.ndarray) -> None:
        """Send PCM audio to ESP32."""
        if not self._running or self.ws.closed:
            return

        # Convert to bytes and send
        pcm_data = samples.astype(np.int16).tobytes()

        try:
            await self.ws.send_bytes(pcm_data)
            self.frames_sent += 1
        except Exception as e:
            logger.error(f"Failed to send audio: {e}")

    def _on_asr_partial(self, text: str) -> None:
        """Handle partial ASR result."""
        logger.debug(f"ASR partial: {text}")

    def _on_asr_final(self, text: str) -> None:
        """Handle final ASR result and generate TTS response."""
        logger.info(f"ASR final: {text}")

        if text.strip():
            asyncio.create_task(self._generate_response(text))

    async def _generate_response(self, user_text: str) -> None:
        """Generate and send TTS response."""
        # Simple echo response (replace with actual AI logic)
        response_text = f"你说的是：{user_text}"

        logger.info(f"TTS response: {response_text}")

        try:
            if self._tts:
                # Synthesize and stream
                async for chunk in self._tts.synthesize(response_text):
                    samples = np.frombuffer(chunk, dtype=np.int16)
                    await self.send_audio(samples)

        except Exception as e:
            logger.error(f"TTS failed: {e}")


class AudioStreamServer:
    """WebSocket server for audio streaming with ESP32 clients."""

    def __init__(self, config: dict):
        self.config = config
        self.sessions: Dict[str, AudioStreamSession] = {}

    async def handle_websocket(self, request: web.Request) -> web.WebSocketResponse:
        """Handle WebSocket connection for audio streaming."""
        ws = web.WebSocketResponse()
        await ws.prepare(request)

        session: Optional[AudioStreamSession] = None
        session_id = str(uuid4())

        logger.info(f"WebSocket connection from {request.remote}")

        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    # Control message (JSON)
                    try:
                        data = json.loads(msg.data)
                        msg_type = data.get("type")

                        if msg_type == "start":
                            device_id = data.get("device_id", "unknown")
                            session = AudioStreamSession(
                                session_id=session_id,
                                device_id=device_id,
                                ws=ws,
                                config=self.config,
                            )
                            self.sessions[session_id] = session
                            await session.start()

                            # Send confirmation
                            await ws.send_json({
                                "type": "started",
                                "session_id": session_id,
                            })

                        elif msg_type == "stop":
                            if session:
                                await session.stop()
                                self.sessions.pop(session_id, None)
                            break

                    except json.JSONDecodeError:
                        logger.warning(f"Invalid JSON: {msg.data}")

                elif msg.type == WSMsgType.BINARY:
                    # Audio data (PCM)
                    if session:
                        await session.process_audio(msg.data)

                elif msg.type == WSMsgType.ERROR:
                    logger.error(f"WebSocket error: {ws.exception()}")
                    break

        except Exception as e:
            logger.exception(f"WebSocket handler error: {e}")

        finally:
            if session:
                await session.stop()
                self.sessions.pop(session_id, None)

            logger.info(f"WebSocket connection closed: {session_id}")

        return ws

    def setup_routes(self, app: web.Application) -> None:
        """Add WebSocket routes to the application."""
        app.router.add_get("/audio/stream", self.handle_websocket)
