"""
WebSocket Audio Streaming Server

Handles bidirectional audio streaming with ESP32 and provides APIs for external clients.

Endpoints:
- /audio/stream - WebSocket for ESP32 (bidirectional audio)
- /api/tts - WebSocket for external clients to send TTS text
- /api/asr - WebSocket for external clients to receive ASR results

Protocol (ESP32):
- Binary frames: Raw PCM audio (int16, 16kHz, mono)
- Text frames: JSON control messages

Protocol (External clients):
- /api/tts: Send {"text": "..."} to synthesize and play on ESP32
- /api/asr: Receive {"type": "partial|final", "text": "..."} for ASR results
"""

import asyncio
import json
import logging
import os
import wave
from datetime import datetime
from typing import Callable, Dict, Optional, Set
from uuid import uuid4

from aiohttp import web, WSMsgType
import numpy as np

from ..volcano import UnidirectionalTTSClient
from ..volcano.sauc_asr_client import SaucAsrClient


logger = logging.getLogger(__name__)

VOICE_SAMPLE_RATE = 16000
VOICE_FRAME_MS = 20
VOICE_FRAME_SAMPLES = int(VOICE_SAMPLE_RATE * VOICE_FRAME_MS / 1000)
VOICE_FRAME_BYTES = VOICE_FRAME_SAMPLES * 2


class AudioStreamSession:
    """Manages a single ESP32 audio streaming session."""

    def __init__(
        self,
        session_id: str,
        device_id: str,
        ws: web.WebSocketResponse,
        config: dict,
        asr_subscribers: Set[web.WebSocketResponse],
    ):
        self.session_id = session_id
        self.device_id = device_id
        self.ws = ws
        self.config = config
        self._asr_subscribers = asr_subscribers

        self._tts: Optional[UnidirectionalTTSClient] = None
        self._asr: Optional[SaucAsrClient] = None
        self._running = False
        self._tts_task: Optional[asyncio.Task] = None
        self._audio_processor_task: Optional[asyncio.Task] = None

        # Stats
        self.frames_received = 0
        self.frames_sent = 0

        # TTS queue for external requests
        self._tts_queue: asyncio.Queue = asyncio.Queue()

        # Audio RX queue for non-blocking processing
        self._rx_queue: asyncio.Queue = asyncio.Queue(maxsize=500)

        # Audio saving for debugging
        self._save_audio = config.get("debug", {}).get("save_audio", False)
        self._audio_dir = config.get("debug", {}).get("audio_dir", "./audio_recordings")
        self._rx_wav: Optional[wave.Wave_write] = None
        self._tx_wav: Optional[wave.Wave_write] = None

    async def start(self) -> None:
        """Start the audio processing session."""
        self._running = True

        # Initialize audio recording if enabled
        if self._save_audio:
            try:
                os.makedirs(self._audio_dir, exist_ok=True)
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                sample_rate = 16000

                # RX: Audio received from ESP32 (microphone)
                rx_path = os.path.join(self._audio_dir, f"{timestamp}_{self.session_id[:8]}_rx.wav")
                self._rx_wav = wave.open(rx_path, 'wb')
                self._rx_wav.setnchannels(1)  # Mono
                self._rx_wav.setsampwidth(2)  # 16-bit
                self._rx_wav.setframerate(sample_rate)

                # TX: Audio sent to ESP32 (TTS)
                tx_path = os.path.join(self._audio_dir, f"{timestamp}_{self.session_id[:8]}_tx.wav")
                self._tx_wav = wave.open(tx_path, 'wb')
                self._tx_wav.setnchannels(1)  # Mono
                self._tx_wav.setsampwidth(2)  # 16-bit
                self._tx_wav.setframerate(sample_rate)

                logger.info(f"Audio recording enabled: RX={rx_path}, TX={tx_path}")
            except Exception as e:
                logger.warning(f"Failed to initialize audio recording: {e}")
                self._save_audio = False

        # Initialize Volcano clients (using new unidirectional stream TTS API)
        self._tts = UnidirectionalTTSClient(self.config)
        self._asr = SaucAsrClient(self.config)

        # Start ASR session with callbacks
        try:
            await self._asr.start_session(
                on_partial=self._on_asr_partial,
                on_final=self._on_asr_final,
            )
        except Exception as e:
            logger.warning(f"ASR start failed: {e}")

        # Start TTS processing task
        self._tts_task = asyncio.create_task(self._tts_processor())

        # Start audio processor task (processes RX queue in background)
        self._audio_processor_task = asyncio.create_task(self._audio_processor())

        logger.info(f"Session {self.session_id} started for device {self.device_id}")

    async def stop(self) -> None:
        """Stop the session."""
        self._running = False

        # Cancel TTS task
        if self._tts_task and not self._tts_task.done():
            self._tts_task.cancel()
            try:
                await self._tts_task
            except asyncio.CancelledError:
                pass
            self._tts_task = None

        # Cancel audio processor task
        if self._audio_processor_task and not self._audio_processor_task.done():
            self._audio_processor_task.cancel()
            try:
                await self._audio_processor_task
            except asyncio.CancelledError:
                pass
            self._audio_processor_task = None

        if self._asr:
            try:
                await self._asr.stop_session()
            except Exception as e:
                logger.warning(f"Error stopping ASR: {e}")
            self._asr = None

        self._tts = None

        # Close audio recording files
        if self._rx_wav:
            try:
                self._rx_wav.close()
                logger.info(f"RX audio saved: {self.frames_received} frames")
            except Exception as e:
                logger.warning(f"Error closing RX wav: {e}")
            self._rx_wav = None
        if self._tx_wav:
            try:
                self._tx_wav.close()
                logger.info(f"TX audio saved: {self.frames_sent} frames")
            except Exception as e:
                logger.warning(f"Error closing TX wav: {e}")
            self._tx_wav = None

        logger.info(f"Session {self.session_id} stopped")

    def queue_audio(self, pcm_data: bytes) -> None:
        """Queue incoming PCM audio for background processing (non-blocking)."""
        self.frames_received += 1

        # Log progress periodically
        if self.frames_received % 250 == 0:  # Every 5 seconds (250 * 20ms)
            logger.info(f"Session {self.session_id[:8]}: received {self.frames_received} frames")

        try:
            self._rx_queue.put_nowait(pcm_data)
        except asyncio.QueueFull:
            if self.frames_received % 100 == 0:
                logger.warning(f"RX queue full, dropping frame {self.frames_received}")

    async def _audio_processor(self) -> None:
        """Background task to process queued audio frames."""
        logger.debug(f"Audio processor started for session {self.session_id[:8]}")
        try:
            while self._running:
                try:
                    pcm_data = await asyncio.wait_for(self._rx_queue.get(), timeout=0.5)
                except asyncio.TimeoutError:
                    continue

                try:
                    # Convert to numpy array
                    samples = np.frombuffer(pcm_data, dtype=np.int16)

                    # Save to file for debugging
                    if self._rx_wav:
                        try:
                            self._rx_wav.writeframes(pcm_data)
                        except Exception as e:
                            logger.warning(f"Error writing RX audio: {e}")

                    # Feed to ASR (non-blocking)
                    if self._asr and self._asr.is_active:
                        try:
                            self._asr.feed_audio(samples)
                        except Exception as e:
                            logger.warning(f"ASR feed error: {e}")

                except Exception as e:
                    logger.warning(f"Error processing audio: {e}")

        except asyncio.CancelledError:
            logger.debug(f"Audio processor cancelled for session {self.session_id[:8]}")

    async def send_audio(self, samples: np.ndarray) -> None:
        """Send PCM audio to ESP32."""
        if not self._running or self.ws.closed:
            return

        # Convert to bytes and send
        pcm_data = samples.astype(np.int16).tobytes()

        # Save to file for debugging
        if self._tx_wav:
            try:
                self._tx_wav.writeframes(pcm_data)
            except Exception as e:
                logger.warning(f"Error writing TX audio: {e}")

        try:
            await self.ws.send_bytes(pcm_data)
            self.frames_sent += 1
        except Exception as e:
            logger.error(f"Failed to send audio: {e}")

    async def queue_tts(self, text: str) -> None:
        """Queue text for TTS synthesis."""
        await self._tts_queue.put(text)

    async def _tts_processor(self) -> None:
        """Background task to process TTS requests."""
        try:
            while self._running:
                try:
                    text = await asyncio.wait_for(self._tts_queue.get(), timeout=0.5)
                    await self._synthesize_and_send(text)
                except asyncio.TimeoutError:
                    continue
                except asyncio.CancelledError:
                    raise
                except Exception as e:
                    logger.error(f"TTS processor error: {e}")
        except asyncio.CancelledError:
            logger.debug("TTS processor cancelled")

    async def _synthesize_and_send(self, text: str) -> None:
        """Synthesize text and send audio to ESP32."""
        if not self._tts or not text.strip():
            return

        logger.info(f"TTS: {text}")

        try:
            # Send a short silence before audio to "warm up" the DAC
            # This prevents audio popping on first chunk
            silence = np.zeros(VOICE_FRAME_SAMPLES, dtype=np.int16)
            await self.send_audio(silence)

            is_first_chunk = True
            pcm_buffer = bytearray()
            async for chunk in self._tts.synthesize(text):
                if not chunk:
                    continue
                pcm_buffer.extend(chunk)

                while len(pcm_buffer) >= VOICE_FRAME_BYTES:
                    frame = bytes(pcm_buffer[:VOICE_FRAME_BYTES])
                    del pcm_buffer[:VOICE_FRAME_BYTES]
                    samples = np.frombuffer(frame, dtype=np.int16)

                    # Apply fade-in to first chunk to smooth the transition
                    if is_first_chunk and len(samples) > 0:
                        is_first_chunk = False
                        fade_samples = min(160, len(samples))  # 10ms fade-in
                        fade_curve = np.linspace(0, 1, fade_samples, dtype=np.float32)
                        samples = samples.copy()  # Make writable
                        samples[:fade_samples] = (samples[:fade_samples] * fade_curve).astype(np.int16)

                    await self.send_audio(samples)
                    await asyncio.sleep(VOICE_FRAME_MS / 1000.0)

            if pcm_buffer:
                # Send any remaining samples without padding
                samples = np.frombuffer(pcm_buffer, dtype=np.int16)
                await self.send_audio(samples)
        except Exception as e:
            logger.error(f"TTS failed: {e}")

    def _on_asr_partial(self, text: str) -> None:
        """Handle partial ASR result."""
        asyncio.create_task(self._broadcast_asr("partial", text))

    def _on_asr_final(self, text: str) -> None:
        """Handle final ASR result."""
        asyncio.create_task(self._broadcast_asr("final", text))

    async def _broadcast_asr(self, result_type: str, text: str) -> None:
        """Broadcast ASR result to all subscribers."""
        message = {
            "type": result_type,
            "text": text,
            "session_id": self.session_id,
            "device_id": self.device_id,
        }

        dead_subscribers = set()

        for ws in self._asr_subscribers:
            try:
                if ws.closed:
                    dead_subscribers.add(ws)
                else:
                    await ws.send_json(message)
            except Exception as e:
                logger.warning(f"Failed to send ASR to subscriber: {e}")
                dead_subscribers.add(ws)

        # Remove dead subscribers
        self._asr_subscribers -= dead_subscribers


class AudioStreamServer:
    """WebSocket server for audio streaming with ESP32 and external clients."""

    def __init__(self, config: dict):
        self.config = config
        self.sessions: Dict[str, AudioStreamSession] = {}

        # External client subscribers
        self._asr_subscribers: Set[web.WebSocketResponse] = set()

    async def stop_all_sessions(self) -> None:
        """Stop all active sessions (for shutdown)."""
        logger.info(f"Stopping {len(self.sessions)} active session(s)...")
        for session_id, session in list(self.sessions.items()):
            try:
                await session.stop()
            except Exception as e:
                logger.warning(f"Error stopping session {session_id}: {e}")
        self.sessions.clear()

        # Close all ASR subscribers
        for ws in list(self._asr_subscribers):
            try:
                await ws.close()
            except Exception:
                pass
        self._asr_subscribers.clear()

    async def handle_esp32_websocket(self, request: web.Request) -> web.WebSocketResponse:
        """Handle WebSocket connection from ESP32 for audio streaming."""
        # Note: ESP32 handles its own pings (ping_interval_sec=5), so we just need autoping
        # to respond to those pings. Don't set server heartbeat to avoid conflicts.
        ws = web.WebSocketResponse(autoping=True)
        await ws.prepare(request)

        session: Optional[AudioStreamSession] = None
        session_id = str(uuid4())

        logger.info(f"ESP32 WebSocket connection from {request.remote}")

        try:
            msg_count = 0
            # Use explicit receive with timeout instead of async for to detect dead connections
            while True:
                try:
                    # Wait for message with 5-second timeout
                    # This ensures we detect dead connections quickly
                    msg = await asyncio.wait_for(ws.receive(), timeout=5.0)
                except asyncio.TimeoutError:
                    # No message received for 5 seconds - connection might be dead
                    if ws.closed:
                        logger.info(f"WebSocket closed during timeout wait")
                        break
                    # Try to send a ping to check if connection is still alive
                    try:
                        await ws.ping()
                        logger.debug(f"No message for 5s, sent ping (ws.closed={ws.closed})")
                    except Exception as e:
                        logger.info(f"Connection dead - ping failed: {e}")
                        break
                    continue

                msg_count += 1
                # Debug: log first few messages
                if msg_count <= 5:
                    logger.debug(f"WS msg #{msg_count}: type={msg.type}, len={len(msg.data) if msg.data else 0}")

                if msg.type == WSMsgType.TEXT:
                    # Control message (JSON)
                    try:
                        data = json.loads(msg.data)
                        msg_type = data.get("type")

                        if msg_type == "start":
                            device_id = data.get("device_id", "unknown")
                            sample_rate = data.get("sample_rate", 16000)

                            session = AudioStreamSession(
                                session_id=session_id,
                                device_id=device_id,
                                ws=ws,
                                config=self.config,
                                asr_subscribers=self._asr_subscribers,
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
                    # Audio data (PCM) - queue for background processing (non-blocking)
                    if session:
                        # Debug: log first binary frame
                        if session.frames_received == 0:
                            logger.info(f"Received first binary frame: {len(msg.data)} bytes")
                        session.queue_audio(msg.data)  # Non-blocking queue
                    else:
                        logger.warning(f"Received binary data but no session (len={len(msg.data)})")

                elif msg.type == WSMsgType.ERROR:
                    logger.error(f"WebSocket error: {ws.exception()}")
                    break

                elif msg.type == WSMsgType.CLOSE:
                    logger.info(f"ESP32 WebSocket CLOSE received (code={msg.data}, reason={msg.extra})")
                    break

                elif msg.type == WSMsgType.CLOSED:
                    logger.info(f"ESP32 WebSocket CLOSED signal")
                    break

                elif msg.type == WSMsgType.CLOSING:
                    logger.info(f"ESP32 WebSocket CLOSING signal")
                    # Don't break, wait for CLOSED

                elif msg.type == WSMsgType.PING:
                    logger.debug(f"Received PING from ESP32")

                elif msg.type == WSMsgType.PONG:
                    logger.debug(f"Received PONG from ESP32")

                else:
                    logger.warning(f"Unknown WS message type: {msg.type}")

            # Log when loop exits
            logger.info(f"ESP32 WebSocket loop ended (received {msg_count} messages, ws.closed={ws.closed})")

        except asyncio.CancelledError:
            logger.info(f"ESP32 WebSocket handler cancelled")

        except Exception as e:
            logger.exception(f"ESP32 WebSocket error: {e}")

        finally:
            if session:
                await session.stop()
                self.sessions.pop(session_id, None)

            logger.info(f"ESP32 WebSocket closed: {session_id}")

        return ws

    async def handle_tts_websocket(self, request: web.Request) -> web.WebSocketResponse:
        """
        Handle WebSocket connection for TTS requests from external clients.

        Protocol:
        - Client sends: {"text": "要合成的文本"}
        - Server sends: {"status": "ok"} or {"status": "error", "message": "..."}
        """
        ws = web.WebSocketResponse()
        await ws.prepare(request)

        client_id = str(uuid4())[:8]
        logger.info(f"TTS client connected: {client_id}")

        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    try:
                        data = json.loads(msg.data)
                        text = data.get("text", "")

                        if not text:
                            await ws.send_json({
                                "status": "error",
                                "message": "Missing 'text' field",
                            })
                            continue

                        # Send to all active ESP32 sessions
                        if not self.sessions:
                            await ws.send_json({
                                "status": "error",
                                "message": "No ESP32 device connected",
                            })
                            continue

                        # Queue TTS for all sessions
                        for session in self.sessions.values():
                            await session.queue_tts(text)

                        await ws.send_json({
                            "status": "ok",
                            "text": text,
                            "sessions": len(self.sessions),
                        })

                        logger.info(f"TTS queued from {client_id}: {text[:50]}...")

                    except json.JSONDecodeError:
                        await ws.send_json({
                            "status": "error",
                            "message": "Invalid JSON",
                        })

                elif msg.type == WSMsgType.ERROR:
                    break

        except Exception as e:
            logger.error(f"TTS WebSocket error: {e}")

        finally:
            logger.info(f"TTS client disconnected: {client_id}")

        return ws

    async def handle_asr_websocket(self, request: web.Request) -> web.WebSocketResponse:
        """
        Handle WebSocket connection for ASR result subscription.

        Protocol:
        - Server sends: {"type": "partial|final", "text": "...", "session_id": "...", "device_id": "..."}
        """
        ws = web.WebSocketResponse()
        await ws.prepare(request)

        client_id = str(uuid4())[:8]
        logger.info(f"ASR subscriber connected: {client_id}")

        # Add to subscribers
        self._asr_subscribers.add(ws)

        # Send current status
        await ws.send_json({
            "type": "connected",
            "active_sessions": len(self.sessions),
            "sessions": [
                {"session_id": s.session_id, "device_id": s.device_id}
                for s in self.sessions.values()
            ],
        })

        try:
            # Keep connection alive and handle pings
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    # Client can send ping
                    try:
                        data = json.loads(msg.data)
                        if data.get("type") == "ping":
                            await ws.send_json({"type": "pong"})
                    except json.JSONDecodeError:
                        pass

                elif msg.type == WSMsgType.ERROR:
                    break

        except Exception as e:
            logger.error(f"ASR subscriber error: {e}")

        finally:
            self._asr_subscribers.discard(ws)
            logger.info(f"ASR subscriber disconnected: {client_id}")

        return ws

    async def handle_tts_post(self, request: web.Request) -> web.Response:
        """
        Handle POST request for TTS (alternative to WebSocket).

        POST /api/tts
        Body: {"text": "要合成的文本"}
        """
        try:
            data = await request.json()
        except json.JSONDecodeError:
            return web.json_response({"error": "Invalid JSON"}, status=400)

        text = data.get("text", "")
        if not text:
            return web.json_response({"error": "Missing 'text' field"}, status=400)

        if not self.sessions:
            return web.json_response(
                {"error": "No ESP32 device connected"}, status=503
            )

        # Queue TTS for all sessions
        for session in self.sessions.values():
            await session.queue_tts(text)

        return web.json_response({
            "status": "ok",
            "text": text,
            "sessions": len(self.sessions),
        })

    def setup_routes(self, app: web.Application) -> None:
        """Add WebSocket and HTTP routes to the application."""
        # ESP32 audio streaming
        app.router.add_get("/audio/stream", self.handle_esp32_websocket)

        # External client APIs
        app.router.add_get("/api/tts", self.handle_tts_websocket)
        app.router.add_post("/api/tts", self.handle_tts_post)
        app.router.add_get("/api/asr", self.handle_asr_websocket)
