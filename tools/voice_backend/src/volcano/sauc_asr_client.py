"""
Volcano Engine SAUC ASR Client

Streaming ASR using SAUC binary protocol (BigModel API).
Based on the official SDK protocol.
"""

import asyncio
import gzip
import json
import logging
import struct
from typing import Callable, Optional, List
from uuid import uuid4

import aiohttp
import numpy as np

logger = logging.getLogger(__name__)


class ProtocolVersion:
    V1 = 0b0001


class MessageType:
    CLIENT_FULL_REQUEST = 0b0001
    CLIENT_AUDIO_ONLY_REQUEST = 0b0010
    SERVER_FULL_RESPONSE = 0b1001
    SERVER_ERROR_RESPONSE = 0b1111


class MessageTypeSpecificFlags:
    NO_SEQUENCE = 0b0000
    POS_SEQUENCE = 0b0001
    NEG_SEQUENCE = 0b0010
    NEG_WITH_SEQUENCE = 0b0011


class SerializationType:
    NO_SERIALIZATION = 0b0000
    JSON = 0b0001


class CompressionType:
    GZIP = 0b0001


class SaucAsrClient:
    """Volcano Engine SAUC ASR client using binary protocol."""

    ASR_ENDPOINTS = {
        "bidirectional": "wss://openspeech.bytedance.com/api/v3/sauc/bigmodel",
        "bidirectional_async": "wss://openspeech.bytedance.com/api/v3/sauc/bigmodel_async",
        "nostream": "wss://openspeech.bytedance.com/api/v3/sauc/bigmodel_nostream",
    }

    def __init__(self, config: dict):
        volcano_config = config.get("volcano", {})
        self.app_key = volcano_config.get("app_key", "")
        self.access_key = volcano_config.get("access_key", "")

        asr_config = volcano_config.get("asr", {})
        self.mode = asr_config.get("mode", "bidirectional_async")
        self.endpoint = asr_config.get("endpoint") or self.ASR_ENDPOINTS.get(
            self.mode, self.ASR_ENDPOINTS["bidirectional_async"]
        )
        self.resource_id = asr_config.get("resource_id", "volc.bigasr.sauc.duration")
        self.language = asr_config.get("language", "zh-CN")
        self.audio_format = asr_config.get("format", "pcm")
        self.audio_codec = asr_config.get("codec", "raw")
        self.sample_rate = asr_config.get("sample_rate", 16000)
        self.segment_duration_ms = int(asr_config.get("segment_duration_ms", 200))
        if self.segment_duration_ms <= 0:
            self.segment_duration_ms = 200
        self.send_interval_ms = int(
            asr_config.get("send_interval_ms", self.segment_duration_ms)
        )
        if self.send_interval_ms < 0:
            self.send_interval_ms = 0
        self.result_type = asr_config.get("result_type")
        self.enable_itn = asr_config.get("enable_itn", True)
        self.enable_punc = asr_config.get("enable_punc", True)
        self.enable_ddc = asr_config.get("enable_ddc", True)
        self.show_utterances = asr_config.get("show_utterances", True)
        self.enable_nonstream = asr_config.get("enable_nonstream", False)
        self.end_window_size = asr_config.get("end_window_size")
        self.force_to_speech_time = asr_config.get("force_to_speech_time")
        self.vad_segment_duration = asr_config.get("vad_segment_duration")
        self.enable_accelerate_text = asr_config.get("enable_accelerate_text")
        self.accelerate_score = asr_config.get("accelerate_score")

        self._session: Optional[aiohttp.ClientSession] = None
        self._ws: Optional[aiohttp.ClientWebSocketResponse] = None
        self._running = False
        self._seq = 1
        self._audio_queue: asyncio.Queue = asyncio.Queue(maxsize=100)
        self._next_send_time: Optional[float] = None
        self._last_partial_text = ""
        self._last_final_text = ""
        self._connect_id = ""

        # Callbacks
        self._on_partial: Optional[Callable[[str], None]] = None
        self._on_final: Optional[Callable[[str], None]] = None

        # Buffer for accumulating audio
        self._audio_buffer = bytearray()
        self._segment_size = int(self.sample_rate * 2 * self.segment_duration_ms / 1000)

        if self.mode in ("bidirectional", "bidirectional_async"):
            if not (100 <= self.segment_duration_ms <= 200):
                logger.warning(
                    "SAUC segment_duration_ms=%d outside recommended 100-200ms",
                    self.segment_duration_ms,
                )

    def _build_auth_headers(self) -> dict:
        """Build authentication headers."""
        self._connect_id = str(uuid4())
        return {
            "X-Api-Resource-Id": self.resource_id,
            "X-Api-Request-Id": str(uuid4()),
            "X-Api-Access-Key": self.access_key,
            "X-Api-App-Key": self.app_key,
            "X-Api-Connect-Id": self._connect_id,
        }

    def _build_header(
        self,
        message_type: int = MessageType.CLIENT_FULL_REQUEST,
        flags: int = MessageTypeSpecificFlags.POS_SEQUENCE,
        serialization: int = SerializationType.JSON,
        compression: int = CompressionType.GZIP,
    ) -> bytes:
        """Build SAUC protocol header."""
        header = bytearray()
        header.append((ProtocolVersion.V1 << 4) | 1)
        header.append((message_type << 4) | flags)
        header.append((serialization << 4) | compression)
        header.append(0x00)  # Reserved
        return bytes(header)

    def _build_full_request(self) -> bytes:
        """Build full client request with configuration."""
        header = self._build_header(
            MessageType.CLIENT_FULL_REQUEST,
            MessageTypeSpecificFlags.POS_SEQUENCE,
        )

        payload = {
            "user": {"uid": "esp32_copilot"},
            "audio": {
                "format": self.audio_format,
                "codec": self.audio_codec,
                "rate": self.sample_rate,
                "bits": 16,
                "channel": 1,
            },
            "request": {
                "model_name": "bigmodel",
                "enable_itn": self.enable_itn,
                "enable_punc": self.enable_punc,
                "enable_ddc": self.enable_ddc,
                "show_utterances": self.show_utterances,
            },
        }
        if self.mode == "nostream" and self.language:
            payload["audio"]["language"] = self.language

        if self.result_type:
            payload["request"]["result_type"] = self.result_type
        if self.enable_nonstream and self.mode == "bidirectional_async":
            payload["request"]["enable_nonstream"] = True
        if self.end_window_size is not None:
            payload["request"]["end_window_size"] = int(self.end_window_size)
        if self.force_to_speech_time is not None:
            payload["request"]["force_to_speech_time"] = int(self.force_to_speech_time)
        if self.vad_segment_duration is not None and self.end_window_size is None:
            payload["request"]["vad_segment_duration"] = int(self.vad_segment_duration)
        if self.enable_accelerate_text is not None:
            payload["request"]["enable_accelerate_text"] = bool(self.enable_accelerate_text)
        if self.accelerate_score is not None:
            payload["request"]["accelerate_score"] = int(self.accelerate_score)

        payload_bytes = json.dumps(payload).encode("utf-8")
        compressed = gzip.compress(payload_bytes)

        request = bytearray()
        request.extend(header)
        request.extend(struct.pack(">i", self._seq))
        request.extend(struct.pack(">I", len(compressed)))
        request.extend(compressed)

        self._seq += 1
        return bytes(request)

    def _build_audio_request(self, audio_data: bytes, is_last: bool = False) -> bytes:
        """Build audio-only request."""
        if is_last:
            flags = MessageTypeSpecificFlags.NEG_WITH_SEQUENCE
            seq = -self._seq
        else:
            flags = MessageTypeSpecificFlags.POS_SEQUENCE
            seq = self._seq
            self._seq += 1

        header = self._build_header(
            MessageType.CLIENT_AUDIO_ONLY_REQUEST,
            flags,
            serialization=SerializationType.NO_SERIALIZATION,
            compression=CompressionType.GZIP,
        )

        compressed = gzip.compress(audio_data)

        request = bytearray()
        request.extend(header)
        request.extend(struct.pack(">i", seq))
        request.extend(struct.pack(">I", len(compressed)))
        request.extend(compressed)

        return bytes(request)

    def _parse_response(self, data: bytes) -> dict:
        """Parse SAUC protocol response."""
        result = {
            "code": 0,
            "is_last": False,
            "sequence": 0,
            "partial_text": "",
            "final_texts": [],
            "utterances": [],
        }

        if len(data) < 4:
            return result

        header_size = data[0] & 0x0F
        message_type = data[1] >> 4
        flags = data[1] & 0x0F
        compression = data[2] & 0x0F

        payload = data[header_size * 4 :]

        # Parse flags
        if flags & 0x01:  # Has sequence
            result["sequence"] = struct.unpack(">i", payload[:4])[0]
            payload = payload[4:]
        if flags & 0x02:  # Is last
            result["is_last"] = True

        # Parse message type
        if message_type == MessageType.SERVER_FULL_RESPONSE:
            payload_size = struct.unpack(">I", payload[:4])[0]
            payload = payload[4:]
        elif message_type == MessageType.SERVER_ERROR_RESPONSE:
            result["code"] = struct.unpack(">i", payload[:4])[0]
            payload_size = struct.unpack(">I", payload[4:8])[0]
            payload = payload[8:]

        if not payload:
            return result

        # Decompress
        if compression == CompressionType.GZIP:
            try:
                payload = gzip.decompress(payload)
            except Exception as e:
                logger.warning(f"Decompression failed: {e}")
                return result

        # Parse JSON payload
        try:
            msg = json.loads(payload.decode("utf-8"))
            result["payload"] = msg

            # Extract text from result payload
            if "result" in msg:
                result_payload = msg["result"]
                result["partial_text"] = result_payload.get("text", "")
                utterances = result_payload.get("utterances", [])
                result["utterances"] = utterances

                # Collect definite utterances for streaming final segments
                for utt in utterances:
                    if utt.get("definite", False):
                        text = utt.get("text", "")
                        if text:
                            result["final_texts"].append(text)

        except Exception as e:
            logger.warning(f"JSON parse failed: {e}")

        return result

    async def start_session(
        self,
        on_partial: Optional[Callable[[str], None]] = None,
        on_final: Optional[Callable[[str], None]] = None,
    ) -> None:
        """Start ASR streaming session."""
        if not self.app_key or not self.access_key:
            logger.warning("Volcano credentials not configured, ASR disabled")
            return

        self._on_partial = on_partial
        self._on_final = on_final
        self._seq = 1
        self._audio_buffer.clear()
        self._next_send_time = None
        self._last_partial_text = ""
        self._last_final_text = ""

        logger.info("Starting SAUC ASR session (mode=%s, endpoint=%s)", self.mode, self.endpoint)

        try:
            self._session = aiohttp.ClientSession()
            headers = self._build_auth_headers()

            self._ws = await self._session.ws_connect(
                self.endpoint,
                headers=headers,
            )
            self._running = True
            logid = ""
            if getattr(self._ws, "_response", None):
                logid = self._ws._response.headers.get("X-Tt-Logid", "")
            if logid:
                logger.info("SAUC logid=%s connect_id=%s", logid, self._connect_id)

            # Send initial full request
            init_request = self._build_full_request()
            await self._ws.send_bytes(init_request)
            logger.debug(f"Sent full request, seq={self._seq - 1}")

            # Wait for initial response
            msg = await self._ws.receive()
            if msg.type == aiohttp.WSMsgType.BINARY:
                response = self._parse_response(msg.data)
                logger.debug(f"Init response: code={response['code']}")

            # Start background tasks
            asyncio.create_task(self._receive_loop())
            asyncio.create_task(self._send_loop())

            logger.info("SAUC ASR session started")

        except Exception as e:
            logger.error(f"Failed to start ASR: {e}")
            self._running = False
            await self._cleanup()
            raise

    async def _receive_loop(self) -> None:
        """Background task to receive ASR results."""
        try:
            while self._running and self._ws:
                try:
                    msg = await asyncio.wait_for(self._ws.receive(), timeout=1.0)
                except asyncio.TimeoutError:
                    continue

                if msg.type == aiohttp.WSMsgType.BINARY:
                    response = self._parse_response(msg.data)

                    if response["code"] != 0:
                        logger.error(f"ASR error: {response['code']}")
                        continue

                    final_texts = response.get("final_texts", [])
                    partial_text = response.get("partial_text", "")

                    if final_texts:
                        for text in final_texts:
                            if text and text != self._last_final_text:
                                self._last_final_text = text
                                logger.info(f"ASR final: {text}")
                                if self._on_final:
                                    self._on_final(text)
                        self._last_partial_text = ""
                    elif partial_text and partial_text != self._last_partial_text:
                        self._last_partial_text = partial_text
                        logger.debug(f"ASR partial: {partial_text}")
                        if self._on_partial:
                            self._on_partial(partial_text)

                    if response["is_last"]:
                        if partial_text and partial_text != self._last_final_text:
                            self._last_final_text = partial_text
                            logger.info(f"ASR final: {partial_text}")
                            if self._on_final:
                                self._on_final(partial_text)
                        # Reset for next utterance
                        self._seq = 1

                elif msg.type in (aiohttp.WSMsgType.CLOSE, aiohttp.WSMsgType.CLOSED):
                    logger.info("ASR WebSocket closed")
                    break

                elif msg.type == aiohttp.WSMsgType.ERROR:
                    logger.error(f"ASR WebSocket error")
                    break

        except Exception as e:
            logger.error(f"ASR receive error: {e}")
        finally:
            self._running = False

    async def _send_loop(self) -> None:
        """Background task to send audio segments."""
        try:
            while self._running and self._ws:
                try:
                    audio_data = await asyncio.wait_for(
                        self._audio_queue.get(), timeout=0.1
                    )
                    loop = asyncio.get_running_loop()
                    if self.send_interval_ms > 0:
                        now = loop.time()
                        if self._next_send_time is None:
                            self._next_send_time = now
                        if now < self._next_send_time:
                            await asyncio.sleep(self._next_send_time - now)
                    request = self._build_audio_request(audio_data, is_last=False)
                    await self._ws.send_bytes(request)
                    if self.send_interval_ms > 0:
                        interval_s = self.send_interval_ms / 1000.0
                        self._next_send_time = max(
                            (self._next_send_time or 0) + interval_s, loop.time()
                        )

                except asyncio.TimeoutError:
                    continue

        except Exception as e:
            logger.error(f"ASR send error: {e}")

    def feed_audio(self, samples: np.ndarray) -> None:
        """Feed audio samples to ASR."""
        if not self._running:
            return

        # Convert to bytes
        if samples.dtype != np.int16:
            samples = (samples * 32767).astype(np.int16)

        self._audio_buffer.extend(samples.tobytes())

        # Send segments when buffer is large enough
        while len(self._audio_buffer) >= self._segment_size:
            segment = bytes(self._audio_buffer[: self._segment_size])
            self._audio_buffer = self._audio_buffer[self._segment_size :]

            try:
                self._audio_queue.put_nowait(segment)
            except asyncio.QueueFull:
                logger.warning("ASR audio queue full")

    async def _cleanup(self) -> None:
        """Cleanup resources."""
        if self._ws and not self._ws.closed:
            await self._ws.close()
        self._ws = None

        if self._session and not self._session.closed:
            await self._session.close()
        self._session = None

    async def stop_session(self) -> None:
        """Stop the ASR session."""
        self._running = False

        # Send remaining audio as last segment
        if self._audio_buffer and self._ws and not self._ws.closed:
            try:
                request = self._build_audio_request(
                    bytes(self._audio_buffer), is_last=True
                )
                await self._ws.send_bytes(request)
            except Exception as e:
                logger.warning(f"Error sending final segment: {e}")

        await self._cleanup()
        logger.info("SAUC ASR session stopped")

    @property
    def is_active(self) -> bool:
        """Check if ASR is active."""
        return self._running and self._ws is not None
