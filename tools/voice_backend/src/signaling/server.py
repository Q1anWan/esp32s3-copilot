"""
HTTP Signaling Server for WebRTC

Handles SDP offer/answer exchange and session management for ESP32 clients.
"""

import asyncio
import json
import logging
from typing import Dict, Optional
from uuid import uuid4

from aiohttp import web

from ..session import VoiceSession


logger = logging.getLogger(__name__)


class SignalingServer:
    """HTTP server for WebRTC signaling with ESP32 clients."""

    def __init__(self, config: dict):
        self.config = config
        self.app = web.Application()
        self.runner: Optional[web.AppRunner] = None
        self.sessions: Dict[str, VoiceSession] = {}

        self._setup_routes()

    def _setup_routes(self):
        """Setup HTTP routes."""
        self.app.router.add_get("/", self._handle_health)
        self.app.router.add_get("/health", self._handle_health)
        self.app.router.add_post("/voice/session", self._handle_create_session)
        self.app.router.add_post("/voice/offer", self._handle_offer)
        self.app.router.add_post("/voice/stop", self._handle_stop_session)
        self.app.router.add_get("/voice/status/{session_id}", self._handle_status)

    async def start(self, host: str, port: int):
        """Start the HTTP server."""
        self.runner = web.AppRunner(self.app)
        await self.runner.setup()
        site = web.TCPSite(self.runner, host, port)
        await site.start()
        logger.info(f"Signaling server listening on http://{host}:{port}")

    async def stop(self):
        """Stop the HTTP server and cleanup sessions."""
        # Stop all active sessions
        for session_id, session in list(self.sessions.items()):
            try:
                await session.stop()
            except Exception as e:
                logger.warning(f"Error stopping session {session_id}: {e}")
        self.sessions.clear()

        # Stop HTTP server
        if self.runner:
            await self.runner.cleanup()
            self.runner = None

    async def _handle_health(self, request: web.Request) -> web.Response:
        """Health check endpoint."""
        return web.json_response({
            "status": "ok",
            "service": "copilot-voice-backend",
            "active_sessions": len(self.sessions),
        })

    async def _handle_create_session(self, request: web.Request) -> web.Response:
        """
        Create a new voice session.

        Request body:
        {
            "device_id": "s3_copilot",
            "capabilities": {
                "sample_rate": 16000,
                "codec": "opus"
            }
        }

        Response:
        {
            "session_id": "uuid",
            "status": "created"
        }
        """
        try:
            data = await request.json()
        except json.JSONDecodeError:
            return web.json_response(
                {"error": "Invalid JSON"}, status=400
            )

        device_id = data.get("device_id", "unknown")
        capabilities = data.get("capabilities", {})

        # Generate session ID
        session_id = str(uuid4())

        # Create voice session
        try:
            session = VoiceSession(
                session_id=session_id,
                device_id=device_id,
                config=self.config,
            )
            self.sessions[session_id] = session

            logger.info(
                f"Created session {session_id} for device {device_id} "
                f"with capabilities: {capabilities}"
            )

            return web.json_response({
                "session_id": session_id,
                "status": "created",
            })

        except Exception as e:
            logger.exception(f"Failed to create session: {e}")
            return web.json_response(
                {"error": f"Failed to create session: {e}"}, status=500
            )

    async def _handle_offer(self, request: web.Request) -> web.Response:
        """
        Handle WebRTC SDP offer from ESP32.

        Request body:
        {
            "session_id": "uuid",
            "sdp": "v=0\r\n..."
        }

        Response:
        {
            "sdp": "v=0\r\n...",
            "ice_candidates": [...]
        }
        """
        try:
            data = await request.json()
        except json.JSONDecodeError:
            return web.json_response(
                {"error": "Invalid JSON"}, status=400
            )

        session_id = data.get("session_id")
        sdp_offer = data.get("sdp")

        if not session_id or not sdp_offer:
            return web.json_response(
                {"error": "Missing session_id or sdp"}, status=400
            )

        session = self.sessions.get(session_id)
        if not session:
            return web.json_response(
                {"error": f"Session not found: {session_id}"}, status=404
            )

        try:
            # Process offer and generate answer
            sdp_answer, ice_candidates = await session.handle_offer(sdp_offer)

            logger.info(f"Processed offer for session {session_id}")

            return web.json_response({
                "sdp": sdp_answer,
                "ice_candidates": ice_candidates,
            })

        except Exception as e:
            logger.exception(f"Failed to process offer: {e}")
            return web.json_response(
                {"error": f"Failed to process offer: {e}"}, status=500
            )

    async def _handle_stop_session(self, request: web.Request) -> web.Response:
        """
        Stop a voice session.

        Request body:
        {
            "session_id": "uuid"
        }
        """
        try:
            data = await request.json()
        except json.JSONDecodeError:
            return web.json_response(
                {"error": "Invalid JSON"}, status=400
            )

        session_id = data.get("session_id")
        if not session_id:
            return web.json_response(
                {"error": "Missing session_id"}, status=400
            )

        session = self.sessions.pop(session_id, None)
        if not session:
            return web.json_response(
                {"error": f"Session not found: {session_id}"}, status=404
            )

        try:
            await session.stop()
            logger.info(f"Stopped session {session_id}")
            return web.json_response({"status": "stopped"})

        except Exception as e:
            logger.exception(f"Error stopping session: {e}")
            return web.json_response(
                {"error": f"Error stopping session: {e}"}, status=500
            )

    async def _handle_status(self, request: web.Request) -> web.Response:
        """Get session status."""
        session_id = request.match_info.get("session_id")
        session = self.sessions.get(session_id)

        if not session:
            return web.json_response(
                {"error": f"Session not found: {session_id}"}, status=404
            )

        return web.json_response({
            "session_id": session_id,
            "device_id": session.device_id,
            "state": session.state,
            "stats": session.get_stats(),
        })
