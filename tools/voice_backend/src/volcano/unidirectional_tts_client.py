"""
Volcano Engine Unidirectional Stream TTS Client

Text-to-Speech client using the new Volcano Engine v3 unidirectional stream API.
Reference: https://www.volcengine.com/docs/6561/1257544

Protocol:
- Endpoint: wss://openspeech.bytedance.com/api/v3/tts/unidirectional/stream
- Authentication via HTTP headers (X-Api-App-Key, X-Api-Access-Key, X-Api-Resource-Id)
- Binary protocol with Message framing
- Streaming audio response with events: TTSSentenceStart, TTSResponse, TTSSentenceEnd, SessionFinished
"""

import io
import json
import logging
import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import AsyncIterator, Optional
from uuid import uuid4

import websockets
import numpy as np


logger = logging.getLogger(__name__)


# Protocol constants
class MsgType(IntEnum):
    """Message type enumeration"""
    Invalid = 0
    FullClientRequest = 0b0001
    AudioOnlyClient = 0b0010
    FullServerResponse = 0b1001
    AudioOnlyServer = 0b1011
    FrontEndResultServer = 0b1100
    Error = 0b1111


class MsgTypeFlagBits(IntEnum):
    """Message type flag bits"""
    NoSeq = 0
    PositiveSeq = 0b0001
    LastNoSeq = 0b0010
    NegativeSeq = 0b0011
    WithEvent = 0b0100


class EventType(IntEnum):
    """Event type enumeration"""
    None_ = 0
    StartConnection = 1
    FinishConnection = 2
    ConnectionStarted = 50
    ConnectionFailed = 51
    ConnectionFinished = 52
    StartSession = 100
    SessionFinished = 152
    TaskRequest = 200
    TTSSentenceStart = 350
    TTSSentenceEnd = 351
    TTSResponse = 352


@dataclass
class Message:
    """Binary protocol message"""
    version: int = 1
    header_size: int = 1  # in 4-byte units
    type: MsgType = MsgType.Invalid
    flag: MsgTypeFlagBits = MsgTypeFlagBits.NoSeq
    serialization: int = 1  # JSON
    compression: int = 0  # None

    event: EventType = EventType.None_
    session_id: str = ""
    connect_id: str = ""
    sequence: int = 0
    error_code: int = 0

    payload: bytes = b""

    def marshal(self) -> bytes:
        """Serialize message to bytes"""
        buffer = io.BytesIO()

        # Write 4-byte header
        header = bytes([
            (self.version << 4) | self.header_size,
            (self.type << 4) | self.flag,
            (self.serialization << 4) | self.compression,
            0,  # Reserved
        ])
        buffer.write(header)

        # Write payload size and payload
        buffer.write(struct.pack(">I", len(self.payload)))
        buffer.write(self.payload)

        return buffer.getvalue()

    @classmethod
    def from_bytes(cls, data: bytes) -> "Message":
        """Deserialize message from bytes"""
        if len(data) < 4:
            raise ValueError(f"Data too short: {len(data)}")

        byte0, byte1, byte2, byte3 = data[:4]

        msg = cls(
            version=byte0 >> 4,
            header_size=byte0 & 0x0F,
            type=MsgType(byte1 >> 4),
            flag=MsgTypeFlagBits(byte1 & 0x0F),
            serialization=byte2 >> 4,
            compression=byte2 & 0x0F,
        )

        # Parse rest based on message type and flags
        offset = msg.header_size * 4

        # Handle sequence for audio messages
        if msg.type in [MsgType.AudioOnlyServer, MsgType.AudioOnlyClient]:
            if msg.flag in [MsgTypeFlagBits.PositiveSeq, MsgTypeFlagBits.NegativeSeq]:
                if len(data) >= offset + 4:
                    msg.sequence = struct.unpack(">i", data[offset:offset + 4])[0]
                    offset += 4

        # Handle error code
        if msg.type == MsgType.Error:
            if len(data) >= offset + 4:
                msg.error_code = struct.unpack(">I", data[offset:offset + 4])[0]
                offset += 4

        # Handle event and session_id for WithEvent flag
        if msg.flag == MsgTypeFlagBits.WithEvent:
            if len(data) >= offset + 4:
                msg.event = EventType(struct.unpack(">i", data[offset:offset + 4])[0])
                offset += 4

            # Session ID (skip for connection events)
            if msg.event not in [EventType.StartConnection, EventType.FinishConnection,
                                 EventType.ConnectionStarted, EventType.ConnectionFailed,
                                 EventType.ConnectionFinished]:
                if len(data) >= offset + 4:
                    session_len = struct.unpack(">I", data[offset:offset + 4])[0]
                    offset += 4
                    if session_len > 0 and len(data) >= offset + session_len:
                        msg.session_id = data[offset:offset + session_len].decode("utf-8")
                        offset += session_len
            else:
                # Connection ID for connection events
                if len(data) >= offset + 4:
                    connect_len = struct.unpack(">I", data[offset:offset + 4])[0]
                    offset += 4
                    if connect_len > 0 and len(data) >= offset + connect_len:
                        msg.connect_id = data[offset:offset + connect_len].decode("utf-8")
                        offset += connect_len

        # Payload
        if len(data) >= offset + 4:
            payload_len = struct.unpack(">I", data[offset:offset + 4])[0]
            offset += 4
            if payload_len > 0 and len(data) >= offset + payload_len:
                msg.payload = data[offset:offset + payload_len]

        return msg


class UnidirectionalTTSClient:
    """Volcano Engine Unidirectional Stream TTS client."""

    TTS_ENDPOINT = "wss://openspeech.bytedance.com/api/v3/tts/unidirectional/stream"

    def __init__(self, config: dict):
        volcano_config = config.get("volcano", {})
        self.app_id = volcano_config.get("app_id", "")
        self.access_token = volcano_config.get("access_token", "")

        tts_config = volcano_config.get("tts", {})
        self.voice_type = tts_config.get("voice_type", "zh_female_cancan_mars_bigtts")
        self.encoding = tts_config.get("encoding", "pcm")
        self.sample_rate = tts_config.get("sample_rate", 16000)
        self.resource_id = tts_config.get("resource_id", "")

        self._ws: Optional[websockets.WebSocketClientProtocol] = None

    def _get_resource_id(self) -> str:
        """Determine resource ID based on voice type."""
        if self.resource_id:
            return self.resource_id

        # Auto-detect based on voice type prefix
        voice = self.voice_type
        if voice.startswith("S_") or voice.startswith("saturn_"):
            return "volc.megatts.default"  # ICL voices
        return "volc.service_type.10029"  # Default BigTTS voices (seed-tts-1.0)

    def _build_request_payload(self, text: str) -> dict:
        """Build TTS request payload for unidirectional stream API."""
        return {
            "user": {
                "uid": str(uuid4()),
            },
            "req_params": {
                "speaker": self.voice_type,
                "audio_params": {
                    "format": self.encoding,
                    "sample_rate": self.sample_rate,
                },
                "text": text,
                "additions": json.dumps({
                    "disable_markdown_filter": False,
                }),
            },
        }

    async def synthesize(self, text: str) -> AsyncIterator[bytes]:
        """
        Synthesize text to speech using unidirectional stream API.

        Yields PCM audio chunks as they arrive.
        """
        if not self.app_id or not self.access_token:
            raise ValueError("Volcano Engine credentials not configured")

        logger.info(f"TTS request: '{text[:50]}...' voice={self.voice_type}")

        # Build authentication headers
        connect_id = str(uuid4())
        headers = {
            "X-Api-App-Key": self.app_id,
            "X-Api-Access-Key": self.access_token,
            "X-Api-Resource-Id": self._get_resource_id(),
            "X-Api-Connect-Id": connect_id,
        }

        try:
            async with websockets.connect(
                self.TTS_ENDPOINT,
                additional_headers=headers,
                max_size=10 * 1024 * 1024,  # 10MB
            ) as ws:
                logger.debug(f"Connected to TTS endpoint, connect_id={connect_id}")

                # Send TTS request
                payload = self._build_request_payload(text)
                request_msg = Message(
                    type=MsgType.FullClientRequest,
                    flag=MsgTypeFlagBits.NoSeq,
                    payload=json.dumps(payload).encode("utf-8"),
                )
                await ws.send(request_msg.marshal())
                logger.debug("Sent TTS request")

                # Receive audio chunks
                while True:
                    response = await ws.recv()
                    if isinstance(response, str):
                        logger.warning(f"Unexpected text response: {response}")
                        continue

                    msg = Message.from_bytes(response)

                    if msg.type == MsgType.Error:
                        error_msg = msg.payload.decode("utf-8") if msg.payload else f"Error code: {msg.error_code}"
                        raise RuntimeError(f"TTS error: {error_msg}")

                    if msg.type == MsgType.FullServerResponse:
                        # Handle events
                        if msg.event == EventType.SessionFinished:
                            logger.info("TTS synthesis complete")
                            break
                        elif msg.event == EventType.TTSSentenceStart:
                            logger.debug(f"Sentence start: {msg.payload.decode('utf-8', 'ignore')}")
                        elif msg.event == EventType.TTSSentenceEnd:
                            logger.debug(f"Sentence end: {msg.payload.decode('utf-8', 'ignore')}")

                    elif msg.type == MsgType.AudioOnlyServer:
                        # Audio data
                        if msg.payload:
                            yield msg.payload

        except websockets.exceptions.WebSocketException as e:
            logger.error(f"WebSocket error: {e}")
            raise
        except Exception as e:
            logger.error(f"TTS synthesis error: {e}")
            raise

    async def synthesize_to_array(self, text: str) -> np.ndarray:
        """
        Synthesize text and return as numpy array.

        Returns:
            numpy array of int16 PCM samples
        """
        chunks = []
        async for chunk in self.synthesize(text):
            chunks.append(chunk)

        if not chunks:
            return np.array([], dtype=np.int16)

        # Concatenate all chunks
        audio_data = b"".join(chunks)

        # Convert to numpy array
        samples = np.frombuffer(audio_data, dtype=np.int16)
        logger.info(f"TTS result: {len(samples)} samples ({len(samples) / self.sample_rate:.2f}s)")

        return samples


# Alias for compatibility
VolcanoUnidirectionalTTSClient = UnidirectionalTTSClient
