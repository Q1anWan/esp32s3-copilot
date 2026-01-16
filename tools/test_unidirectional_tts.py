#!/usr/bin/env python3
"""
Test script for Volcano Engine Unidirectional Stream TTS API.

Usage:
    python test_unidirectional_tts.py --text "你好世界"
    python test_unidirectional_tts.py --text "Hello world" --voice zh_male_zhongyuan_speech_mars_bigtts
"""

import argparse
import asyncio
import logging
import sys
from pathlib import Path

# Add voice_backend to path
sys.path.insert(0, str(Path(__file__).parent / "voice_backend"))

import yaml
from src.volcano.unidirectional_tts_client import UnidirectionalTTSClient

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger(__name__)


async def test_tts(config: dict, text: str, output_file: str):
    """Test TTS synthesis."""
    client = UnidirectionalTTSClient(config)

    logger.info(f"Synthesizing: '{text}'")
    logger.info(f"Voice: {client.voice_type}")
    logger.info(f"Sample rate: {client.sample_rate} Hz")
    logger.info(f"Resource ID: {client._get_resource_id()}")

    # Synthesize
    audio_chunks = []
    async for chunk in client.synthesize(text):
        audio_chunks.append(chunk)
        logger.info(f"  Received chunk: {len(chunk)} bytes")

    if not audio_chunks:
        logger.error("No audio data received")
        return False

    # Save to file
    audio_data = b"".join(audio_chunks)
    with open(output_file, "wb") as f:
        f.write(audio_data)

    duration = len(audio_data) / 2 / client.sample_rate  # int16 = 2 bytes
    logger.info(f"✓ Saved to {output_file}: {len(audio_data)} bytes ({duration:.2f}s)")
    logger.info(f"  Play with: aplay -r {client.sample_rate} -f S16_LE {output_file}")

    return True


def main():
    parser = argparse.ArgumentParser(
        description="Test Volcano Engine Unidirectional Stream TTS API",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Default voice (zh_female_cancan_mars_bigtts)
  python test_unidirectional_tts.py --text "你好，我是火山引擎的语音合成服务。"

  # Male voice
  python test_unidirectional_tts.py --text "你好世界" --voice zh_male_zhongyuan_speech_mars_bigtts

  # Save to specific file
  python test_unidirectional_tts.py --text "测试" --output my_audio.pcm
        """,
    )
    parser.add_argument(
        "--text",
        default="你好，我是火山引擎的语音合成服务。这是一个测试。",
        help="Text to synthesize",
    )
    parser.add_argument(
        "--voice",
        help="Voice type (overrides config)",
    )
    parser.add_argument(
        "--output",
        default="tts_output.pcm",
        help="Output PCM file (default: tts_output.pcm)",
    )
    parser.add_argument(
        "--config",
        default="voice_backend/config.yaml",
        help="Config file path (default: voice_backend/config.yaml)",
    )

    args = parser.parse_args()

    # Load config
    config_path = Path(__file__).parent / args.config
    if not config_path.exists():
        logger.error(f"Config file not found: {config_path}")
        sys.exit(1)

    with open(config_path) as f:
        config = yaml.safe_load(f)

    # Override voice if specified
    if args.voice:
        config["volcano"]["tts"]["voice_type"] = args.voice

    # Run test
    try:
        success = asyncio.run(test_tts(config, args.text, args.output))
        sys.exit(0 if success else 1)
    except Exception as e:
        logger.exception(f"TTS test failed: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
