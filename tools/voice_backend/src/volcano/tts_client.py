"""
Volcano Engine TTS Client

Text-to-Speech client using Volcano Engine WebSocket binary protocol.
Based on volcengine_binary_demo protocol implementation.
"""

import asyncio
import json
import logging
import struct
from typing import AsyncIterator, Optional
from uuid import uuid4

import websockets
import numpy as np


logger = logging.getLogger(__name__)


# Protocol constants (from volcengine_binary_demo)
class MsgType:
    FULL_CLIENT_REQUEST = 0b0001
    AUDIO_ONLY_CLIENT = 0b0010
    FULL_SERVER_RESPONSE = 0b1001
    AUDIO_ONLY_SERVER = 0b1011
    FRONTEND_RESULT = 0b1100
    ERROR = 0b1111


class MsgTypeFlags:
    NO_SEQ = 0b0000
    POSITIVE_SEQ = 0b0001
    LAST_NO_SEQ = 0b0010
    NEGATIVE_SEQ = 0b0011
    WITH_EVENT = 0b0100


class VolcanoTTSClient:
    """Volcano Engine TTS WebSocket client."""

    TTS_ENDPOINT = "wss://openspeech.bytedance.com/api/v1/tts/ws_binary"

    def __init__(self, config: dict):
        volcano_config = config.get("volcano", {})
        self.app_id = volcano_config.get("app_id", "")
        self.access_token = volcano_config.get("access_token", "")

        tts_config = volcano_config.get("tts", {})
        self.voice_type = tts_config.get("voice_type", "BV001_streaming")
        self.encoding = tts_config.get("encoding", "pcm")
        self.sample_rate = tts_config.get("sample_rate", 16000)

        self._ws: Optional[websockets.WebSocketClientProtocol] = None

    def _build_request_payload(self, text: str) -> dict:
        """Build TTS request payload."""
        # Determine cluster based on voice type
        cluster = "volcano_icl" if self.voice_type.startswith("S_") else "volcano_tts"

        return {
            "app": {
                "appid": self.app_id,
                "token": "access_token",  # Placeholder, actual token in header
                "cluster": cluster,
            },
            "user": {
                "uid": str(uuid4()),
            },
            "audio": {
                "voice_type": self.voice_type,
                "encoding": self.encoding,
                "sample_rate": self.sample_rate,
            },
            "request": {
                "reqid": str(uuid4()),
                "text": text,
                "operation": "submit",
            },
        }

    def _encode_message(self, payload: dict) -> bytes:
        """Encode message using binary protocol."""
        # Header format:
        # Byte 0: [Version (4 bits) | Header Size (4 bits)]
        # Byte 1: [Msg Type (4 bits) | Flags (4 bits)]
        # Byte 2: [Serialization (4 bits) | Compression (4 bits)]
        # Byte 3: Reserved

        version = 1
        header_size = 1  # 4 bytes = 1 unit
        msg_type = MsgType.FULL_CLIENT_REQUEST
        flags = MsgTypeFlags.NO_SEQ
        serialization = 1  # JSON
        compression = 0  # None

        header = bytes([
            (version << 4) | header_size,
            (msg_type << 4) | flags,
            (serialization << 4) | compression,
            0,  # Reserved
        ])

        body = json.dumps(payload).encode("utf-8")

        return header + body

    def _decode_message(self, data: bytes) -> tuple:
        """Decode binary message."""
        if len(data) < 4:
            raise ValueError("Message too short")

        # Parse header
        byte0, byte1, byte2, byte3 = data[:4]

        version = byte0 >> 4
        header_size = (byte0 & 0x0F) * 4
        msg_type = byte1 >> 4
        flags = byte1 & 0x0F
        serialization = byte2 >> 4
        compression = byte2 & 0x0F

        # Parse body
        body = data[header_size:] if header_size > 4 else data[4:]

        # Handle sequence number
        sequence = None
        if flags in (MsgTypeFlags.POSITIVE_SEQ, MsgTypeFlags.NEGATIVE_SEQ):
            if len(body) >= 4:
                sequence = struct.unpack(">i", body[:4])[0]
                body = body[4:]

        return msg_type, flags, sequence, body

    async def synthesize(self, text: str) -> AsyncIterator[bytes]:
        """
        Synthesize text to speech.

        Yields PCM audio chunks as they arrive.
        """
        if not self.app_id or not self.access_token:
            raise ValueError("Volcano Engine credentials not configured")

        logger.info(f"TTS request: '{text[:50]}...' voice={self.voice_type}")

        # Connect to TTS endpoint
        headers = {
            "Authorization": f"Bearer;{self.access_token}",
        }

        try:
            async with websockets.connect(
                self.TTS_ENDPOINT,
                extra_headers=headers,
                max_size=10 * 1024 * 1024,  # 10MB
            ) as ws:
                # Send TTS request
                payload = self._build_request_payload(text)
                message = self._encode_message(payload)
                await ws.send(message)

                # Receive audio chunks
                while True:
                    response = await ws.recv()
                    msg_type, flags, sequence, body = self._decode_message(response)

                    if msg_type == MsgType.ERROR:
                        error_msg = body.decode("utf-8") if body else "Unknown error"
                        raise RuntimeError(f"TTS error: {error_msg}")

                    if msg_type == MsgType.FRONTEND_RESULT:
                        # Metadata, skip
                        continue

                    if msg_type == MsgType.AUDIO_ONLY_SERVER:
                        # Audio data
                        if body:
                            yield body

                        # Check if this is the last message
                        if sequence is not None and sequence < 0:
                            logger.info("TTS synthesis complete")
                            break

        except websockets.exceptions.WebSocketException as e:
            logger.error(f"WebSocket error: {e}")
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
