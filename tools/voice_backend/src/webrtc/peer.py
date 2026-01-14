"""
WebRTC Peer Connection Manager

Handles aiortc WebRTC peer connection for audio streaming.
"""

import asyncio
import logging
from typing import Callable, Optional, Tuple, List

from aiortc import RTCPeerConnection, RTCSessionDescription, RTCIceCandidate
from aiortc.contrib.media import MediaRelay
from av import AudioFrame
import numpy as np


logger = logging.getLogger(__name__)


class AudioTrackSink:
    """Receives audio frames from WebRTC track."""

    def __init__(self, on_frame: Callable[[AudioFrame], None]):
        self.on_frame = on_frame
        self._running = False

    async def start(self, track):
        """Start receiving frames from track."""
        self._running = True
        logger.info(f"AudioTrackSink started for track: {track.kind}")

        try:
            while self._running:
                frame = await track.recv()
                if self.on_frame:
                    self.on_frame(frame)
        except Exception as e:
            if self._running:
                logger.error(f"Error receiving audio frame: {e}")
        finally:
            self._running = False

    def stop(self):
        """Stop receiving frames."""
        self._running = False


class AudioTrackSource:
    """Sends audio frames to WebRTC track."""

    def __init__(self, sample_rate: int = 16000, channels: int = 1):
        self.sample_rate = sample_rate
        self.channels = channels
        self._queue: asyncio.Queue[AudioFrame] = asyncio.Queue(maxsize=50)
        self._running = False
        self._pts = 0

    def push_frame(self, samples: np.ndarray):
        """Push audio samples to be sent."""
        if not self._running:
            return

        # Create AudioFrame from numpy array
        frame = AudioFrame(format="s16", layout="mono" if self.channels == 1 else "stereo")
        frame.rate = self.sample_rate
        frame.pts = self._pts
        self._pts += len(samples)

        # Convert samples to proper format
        if samples.dtype != np.int16:
            samples = (samples * 32767).astype(np.int16)

        frame.planes[0].update(samples.tobytes())

        try:
            self._queue.put_nowait(frame)
        except asyncio.QueueFull:
            logger.warning("Audio output queue full, dropping frame")

    async def recv(self) -> AudioFrame:
        """Called by aiortc to get next frame to send."""
        if not self._running:
            raise StopAsyncIteration

        try:
            return await asyncio.wait_for(self._queue.get(), timeout=1.0)
        except asyncio.TimeoutError:
            # Return silence if no audio available
            return self._generate_silence()

    def _generate_silence(self) -> AudioFrame:
        """Generate a silent audio frame."""
        samples_per_frame = self.sample_rate // 50  # 20ms frame
        silence = np.zeros(samples_per_frame, dtype=np.int16)

        frame = AudioFrame(format="s16", layout="mono")
        frame.rate = self.sample_rate
        frame.pts = self._pts
        self._pts += samples_per_frame
        frame.planes[0].update(silence.tobytes())

        return frame

    def start(self):
        """Start the source."""
        self._running = True
        self._pts = 0

    def stop(self):
        """Stop the source."""
        self._running = False


class WebRTCPeer:
    """Manages WebRTC peer connection for audio streaming."""

    def __init__(
        self,
        config: dict,
        on_audio_frame: Optional[Callable[[AudioFrame], None]] = None,
    ):
        self.config = config
        self.on_audio_frame = on_audio_frame

        self.pc: Optional[RTCPeerConnection] = None
        self.audio_sink: Optional[AudioTrackSink] = None
        self.audio_source: Optional[AudioTrackSource] = None
        self._ice_candidates: List[dict] = []
        self._sink_task: Optional[asyncio.Task] = None

    async def create_answer(self, sdp_offer: str) -> Tuple[str, List[dict]]:
        """
        Process SDP offer and create answer.

        Returns:
            Tuple of (SDP answer string, list of ICE candidates)
        """
        # Create peer connection
        self.pc = RTCPeerConnection()
        self._ice_candidates = []

        # Setup event handlers
        @self.pc.on("icecandidate")
        def on_ice_candidate(candidate: RTCIceCandidate):
            if candidate:
                self._ice_candidates.append({
                    "candidate": candidate.candidate,
                    "sdpMid": candidate.sdpMid,
                    "sdpMLineIndex": candidate.sdpMLineIndex,
                })

        @self.pc.on("track")
        async def on_track(track):
            logger.info(f"Received track: {track.kind}")
            if track.kind == "audio":
                # Start receiving audio from remote
                self.audio_sink = AudioTrackSink(self.on_audio_frame)
                self._sink_task = asyncio.create_task(
                    self.audio_sink.start(track)
                )

        @self.pc.on("connectionstatechange")
        async def on_connection_state_change():
            logger.info(f"Connection state: {self.pc.connectionState}")
            if self.pc.connectionState == "failed":
                await self.close()

        # Setup audio output track (for TTS playback to ESP32)
        webrtc_config = self.config.get("webrtc", {})
        opus_config = webrtc_config.get("opus", {})
        sample_rate = opus_config.get("sample_rate", 16000)
        channels = opus_config.get("channels", 1)

        self.audio_source = AudioTrackSource(sample_rate, channels)
        self.audio_source.start()

        # Add audio track for sending
        # Note: aiortc will automatically negotiate OPUS codec
        self.pc.addTrack(self.audio_source)

        # Set remote description (offer)
        offer = RTCSessionDescription(sdp=sdp_offer, type="offer")
        await self.pc.setRemoteDescription(offer)

        # Create and set local description (answer)
        answer = await self.pc.createAnswer()
        await self.pc.setLocalDescription(answer)

        logger.info("Created WebRTC answer")

        # Wait a moment for ICE gathering
        await asyncio.sleep(0.5)

        return self.pc.localDescription.sdp, self._ice_candidates

    def send_audio(self, samples: np.ndarray):
        """Send audio samples to remote peer."""
        if self.audio_source:
            self.audio_source.push_frame(samples)

    async def close(self):
        """Close the peer connection."""
        if self.audio_sink:
            self.audio_sink.stop()

        if self._sink_task:
            self._sink_task.cancel()
            try:
                await self._sink_task
            except asyncio.CancelledError:
                pass

        if self.audio_source:
            self.audio_source.stop()

        if self.pc:
            await self.pc.close()
            self.pc = None

        logger.info("WebRTC peer connection closed")

    @property
    def is_connected(self) -> bool:
        """Check if peer connection is established."""
        return (
            self.pc is not None
            and self.pc.connectionState == "connected"
        )
