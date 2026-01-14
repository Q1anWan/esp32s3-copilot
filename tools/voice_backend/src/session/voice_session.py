"""
Voice Session Manager

Orchestrates the full voice conversation flow:
ESP32 audio -> ASR -> AI/Logic -> TTS -> ESP32 audio
"""

import asyncio
import logging
from enum import Enum
from typing import Dict, List, Optional, Tuple

import numpy as np
from av import AudioFrame

from ..webrtc import WebRTCPeer
from ..volcano import VolcanoTTSClient, VolcanoASRClient


logger = logging.getLogger(__name__)


class SessionState(str, Enum):
    """Voice session states."""
    CREATED = "created"
    CONNECTING = "connecting"
    LISTENING = "listening"
    PROCESSING = "processing"
    SPEAKING = "speaking"
    STOPPED = "stopped"
    ERROR = "error"


class VoiceSession:
    """
    Manages a single voice conversation session with an ESP32 device.

    Flow:
    1. ESP32 connects via WebRTC
    2. Audio from ESP32 is sent to ASR
    3. ASR results trigger response generation
    4. TTS audio is sent back to ESP32
    """

    def __init__(
        self,
        session_id: str,
        device_id: str,
        config: dict,
    ):
        self.session_id = session_id
        self.device_id = device_id
        self.config = config

        self._state = SessionState.CREATED
        self._webrtc: Optional[WebRTCPeer] = None
        self._tts: Optional[VolcanoTTSClient] = None
        self._asr: Optional[VolcanoASRClient] = None

        # Stats
        self._audio_frames_received = 0
        self._audio_frames_sent = 0
        self._asr_results = []

        # Audio buffer for ASR
        self._audio_buffer: List[np.ndarray] = []
        self._buffer_lock = asyncio.Lock()

        logger.info(f"Session created: {session_id} for device {device_id}")

    @property
    def state(self) -> str:
        return self._state.value

    async def handle_offer(self, sdp_offer: str) -> Tuple[str, List[dict]]:
        """
        Handle WebRTC SDP offer from ESP32.

        Args:
            sdp_offer: SDP offer string

        Returns:
            Tuple of (SDP answer, ICE candidates)
        """
        self._state = SessionState.CONNECTING

        # Create WebRTC peer
        self._webrtc = WebRTCPeer(
            config=self.config,
            on_audio_frame=self._on_audio_frame,
        )

        try:
            # Process offer and get answer
            sdp_answer, ice_candidates = await self._webrtc.create_answer(sdp_offer)

            # Initialize Volcano clients
            self._tts = VolcanoTTSClient(self.config)
            self._asr = VolcanoASRClient(self.config)

            # Start ASR session
            await self._start_asr()

            self._state = SessionState.LISTENING
            logger.info(f"Session {self.session_id} connected")

            return sdp_answer, ice_candidates

        except Exception as e:
            self._state = SessionState.ERROR
            logger.exception(f"Failed to handle offer: {e}")
            raise

    async def _start_asr(self) -> None:
        """Start ASR streaming session."""
        if not self._asr:
            return

        try:
            await self._asr.start_session(
                on_partial=self._on_asr_partial,
                on_final=self._on_asr_final,
            )
        except Exception as e:
            logger.warning(f"ASR start failed (credentials may not be configured): {e}")

    def _on_audio_frame(self, frame: AudioFrame) -> None:
        """
        Callback for audio frames received from ESP32.

        Extracts PCM samples and feeds to ASR.
        """
        self._audio_frames_received += 1

        try:
            # Extract samples from frame
            samples = frame.to_ndarray()

            # If stereo, convert to mono
            if len(samples.shape) > 1 and samples.shape[0] > 1:
                samples = samples.mean(axis=0)

            # Flatten and convert to int16
            samples = samples.flatten()
            if samples.dtype != np.int16:
                # Assume float32 in range [-1, 1]
                samples = (samples * 32767).astype(np.int16)

            # Feed to ASR
            if self._asr and self._asr.is_active:
                self._asr.feed_audio(samples)

        except Exception as e:
            logger.error(f"Error processing audio frame: {e}")

    def _on_asr_partial(self, text: str) -> None:
        """Callback for partial ASR results."""
        logger.debug(f"ASR partial: {text}")

    def _on_asr_final(self, text: str) -> None:
        """
        Callback for final ASR results.

        Triggers TTS response generation.
        """
        logger.info(f"ASR final: {text}")
        self._asr_results.append(text)

        if text.strip():
            # Generate response (for now, just echo back)
            asyncio.create_task(self._generate_response(text))

    async def _generate_response(self, user_text: str) -> None:
        """
        Generate and speak a response to user input.

        This is where AI/dialog logic would go.
        For now, simple echo.
        """
        self._state = SessionState.PROCESSING

        # Simple response logic (replace with actual AI)
        response_text = f"You said: {user_text}"

        logger.info(f"Generating response: {response_text}")

        self._state = SessionState.SPEAKING

        try:
            if self._tts and self._webrtc:
                # Synthesize response
                samples = await self._tts.synthesize_to_array(response_text)

                # Send audio to ESP32
                await self._send_audio(samples)

        except Exception as e:
            logger.error(f"Response generation failed: {e}")

        self._state = SessionState.LISTENING

    async def _send_audio(self, samples: np.ndarray) -> None:
        """Send audio samples to ESP32 via WebRTC."""
        if not self._webrtc or not samples.size:
            return

        # Split into frames (20ms at 16kHz = 320 samples)
        sample_rate = self.config.get("webrtc", {}).get("opus", {}).get("sample_rate", 16000)
        frame_samples = sample_rate // 50  # 20ms

        for i in range(0, len(samples), frame_samples):
            frame = samples[i:i + frame_samples]
            if len(frame) < frame_samples:
                # Pad last frame
                frame = np.pad(frame, (0, frame_samples - len(frame)))

            self._webrtc.send_audio(frame)
            self._audio_frames_sent += 1

            # Small delay to maintain timing
            await asyncio.sleep(0.018)  # Slightly less than 20ms

    async def stop(self) -> None:
        """Stop the voice session."""
        logger.info(f"Stopping session {self.session_id}")
        self._state = SessionState.STOPPED

        if self._asr:
            await self._asr.stop_session()
            self._asr = None

        if self._webrtc:
            await self._webrtc.close()
            self._webrtc = None

        self._tts = None

    def get_stats(self) -> Dict:
        """Get session statistics."""
        return {
            "audio_frames_received": self._audio_frames_received,
            "audio_frames_sent": self._audio_frames_sent,
            "asr_results_count": len(self._asr_results),
            "webrtc_connected": self._webrtc.is_connected if self._webrtc else False,
        }
