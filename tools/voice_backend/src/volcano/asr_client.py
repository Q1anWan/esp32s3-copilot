"""
Volcano Engine ASR Client

Automatic Speech Recognition client using Volcano Engine streaming ASR API.
"""

import asyncio
import json
import logging
from typing import AsyncIterator, Callable, Optional
from uuid import uuid4

import websockets
import numpy as np


logger = logging.getLogger(__name__)


class VolcanoASRClient:
    """Volcano Engine streaming ASR WebSocket client."""

    # Note: Use the actual Volcano Engine ASR endpoint
    ASR_ENDPOINT = "wss://openspeech.bytedance.com/api/v2/asr"

    def __init__(self, config: dict):
        volcano_config = config.get("volcano", {})
        self.app_id = volcano_config.get("app_id", "")
        self.access_token = volcano_config.get("access_token", "")

        asr_config = volcano_config.get("asr", {})
        self.language = asr_config.get("language", "zh-CN")
        self.format = asr_config.get("format", "pcm")
        self.sample_rate = asr_config.get("sample_rate", 16000)

        self._ws: Optional[websockets.WebSocketClientProtocol] = None
        self._running = False
        self._audio_queue: asyncio.Queue = asyncio.Queue()

    def _build_start_params(self) -> dict:
        """Build ASR session start parameters."""
        return {
            "app": {
                "appid": self.app_id,
                "cluster": "volcengine_streaming_common",
            },
            "user": {
                "uid": str(uuid4()),
            },
            "request": {
                "reqid": str(uuid4()),
                "workflow": "audio_in,resample,partition,vad,fe,decode",
                "sequence": 1,
                "nbest": 1,
            },
            "audio": {
                "format": self.format,
                "rate": self.sample_rate,
                "language": self.language,
                "bits": 16,
                "channel": 1,
            },
        }

    async def start_session(
        self,
        on_partial: Optional[Callable[[str], None]] = None,
        on_final: Optional[Callable[[str], None]] = None,
    ) -> None:
        """
        Start ASR streaming session.

        Args:
            on_partial: Callback for partial recognition results
            on_final: Callback for final recognition results
        """
        if not self.app_id or not self.access_token:
            raise ValueError("Volcano Engine credentials not configured")

        logger.info("Starting ASR session")

        headers = {
            "Authorization": f"Bearer;{self.access_token}",
        }

        try:
            self._ws = await websockets.connect(
                self.ASR_ENDPOINT,
                extra_headers=headers,
            )
            self._running = True

            # Send start parameters
            start_params = self._build_start_params()
            await self._ws.send(json.dumps(start_params))

            # Start receive loop in background
            asyncio.create_task(
                self._receive_loop(on_partial, on_final)
            )

            # Start send loop in background
            asyncio.create_task(self._send_loop())

            logger.info("ASR session started")

        except Exception as e:
            logger.error(f"Failed to start ASR session: {e}")
            self._running = False
            raise

    async def _receive_loop(
        self,
        on_partial: Optional[Callable[[str], None]],
        on_final: Optional[Callable[[str], None]],
    ) -> None:
        """Background task to receive ASR results."""
        try:
            while self._running and self._ws:
                try:
                    message = await asyncio.wait_for(
                        self._ws.recv(), timeout=1.0
                    )
                except asyncio.TimeoutError:
                    continue

                try:
                    result = json.loads(message)
                except json.JSONDecodeError:
                    logger.warning(f"Invalid JSON from ASR: {message}")
                    continue

                # Parse result
                if "result" in result:
                    text = result["result"].get("text", "")
                    is_final = result.get("is_final", False)

                    if is_final:
                        logger.info(f"ASR final: {text}")
                        if on_final:
                            on_final(text)
                    else:
                        logger.debug(f"ASR partial: {text}")
                        if on_partial:
                            on_partial(text)

                elif "error" in result:
                    logger.error(f"ASR error: {result['error']}")

        except websockets.exceptions.ConnectionClosed:
            logger.info("ASR connection closed")
        except Exception as e:
            logger.error(f"ASR receive error: {e}")
        finally:
            self._running = False

    async def _send_loop(self) -> None:
        """Background task to send audio data."""
        try:
            while self._running and self._ws:
                try:
                    audio_data = await asyncio.wait_for(
                        self._audio_queue.get(), timeout=0.1
                    )
                    await self._ws.send(audio_data)
                except asyncio.TimeoutError:
                    continue
                except asyncio.QueueEmpty:
                    continue

        except websockets.exceptions.ConnectionClosed:
            pass
        except Exception as e:
            logger.error(f"ASR send error: {e}")

    def feed_audio(self, samples: np.ndarray) -> None:
        """
        Feed audio samples to ASR.

        Args:
            samples: numpy array of int16 PCM samples
        """
        if not self._running:
            return

        # Convert to bytes
        if samples.dtype != np.int16:
            samples = (samples * 32767).astype(np.int16)

        audio_bytes = samples.tobytes()

        try:
            self._audio_queue.put_nowait(audio_bytes)
        except asyncio.QueueFull:
            logger.warning("ASR audio queue full")

    async def stop_session(self) -> None:
        """Stop the ASR session."""
        self._running = False

        if self._ws:
            try:
                # Send end-of-stream marker
                await self._ws.send(json.dumps({"signal": "end"}))
                await self._ws.close()
            except Exception as e:
                logger.warning(f"Error closing ASR connection: {e}")
            finally:
                self._ws = None

        logger.info("ASR session stopped")

    @property
    def is_active(self) -> bool:
        """Check if ASR session is active."""
        return self._running and self._ws is not None
