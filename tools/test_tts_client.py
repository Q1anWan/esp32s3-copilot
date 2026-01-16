#!/usr/bin/env python3
"""
TTS Test Client

发送文本到服务器，服务器合成语音后发送给 ESP32 播放。

用法:
    python test_tts_client.py --server ws://192.168.31.98:8080
    python test_tts_client.py --server ws://localhost:8080 --text "你好世界"
    python test_tts_client.py --server http://localhost:8080 --http --text "你好"
"""

import argparse
import asyncio
import json
import logging
import sys

import aiohttp

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
logger = logging.getLogger(__name__)


async def send_tts_websocket(server_url: str, text: str = None):
    """通过 WebSocket 发送 TTS 请求（交互式模式）"""
    ws_url = server_url.replace("http://", "ws://").replace("https://", "wss://")
    if not ws_url.endswith("/api/tts"):
        ws_url = ws_url.rstrip("/") + "/api/tts"

    logger.info(f"连接到 TTS 服务: {ws_url}")

    async with aiohttp.ClientSession() as session:
        try:
            async with session.ws_connect(ws_url) as ws:
                logger.info("已连接，输入要合成的文本（输入 'quit' 退出）")

                if text:
                    # 单次发送模式
                    await ws.send_json({"text": text})
                    response = await ws.receive_json()
                    logger.info(f"响应: {response}")
                else:
                    # 交互模式
                    while True:
                        try:
                            text_input = input("\n请输入文本 > ").strip()
                            if text_input.lower() in ("quit", "exit", "q"):
                                break
                            if not text_input:
                                continue

                            await ws.send_json({"text": text_input})
                            response = await ws.receive_json()

                            if response.get("status") == "ok":
                                logger.info(f"✓ 已发送到 {response.get('sessions', 0)} 个设备")
                            else:
                                logger.error(f"✗ 错误: {response.get('message', 'Unknown error')}")

                        except EOFError:
                            break
                        except KeyboardInterrupt:
                            break

        except aiohttp.ClientError as e:
            logger.error(f"连接失败: {e}")
            sys.exit(1)


async def send_tts_http(server_url: str, text: str):
    """通过 HTTP POST 发送 TTS 请求"""
    if not server_url.startswith("http"):
        server_url = "http://" + server_url

    api_url = server_url.rstrip("/") + "/api/tts"
    logger.info(f"发送 TTS 请求到: {api_url}")

    async with aiohttp.ClientSession() as session:
        try:
            async with session.post(api_url, json={"text": text}) as resp:
                response = await resp.json()

                if resp.status == 200:
                    logger.info(f"✓ 成功: {response}")
                else:
                    logger.error(f"✗ 错误 ({resp.status}): {response}")

        except aiohttp.ClientError as e:
            logger.error(f"请求失败: {e}")
            sys.exit(1)


def main():
    parser = argparse.ArgumentParser(
        description="TTS 测试客户端 - 发送文本到服务器合成语音",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # WebSocket 交互模式
  python test_tts_client.py --server ws://192.168.31.98:8080

  # WebSocket 单次发送
  python test_tts_client.py --server ws://localhost:8080 --text "你好世界"

  # HTTP POST 模式
  python test_tts_client.py --server http://localhost:8080 --http --text "你好"
        """,
    )
    parser.add_argument(
        "--server",
        default="ws://localhost:8080",
        help="服务器地址 (默认: ws://localhost:8080)",
    )
    parser.add_argument(
        "--text",
        help="要合成的文本（不指定则进入交互模式）",
    )
    parser.add_argument(
        "--http",
        action="store_true",
        help="使用 HTTP POST 而不是 WebSocket",
    )

    args = parser.parse_args()

    if args.http:
        if not args.text:
            logger.error("HTTP 模式必须指定 --text 参数")
            sys.exit(1)
        asyncio.run(send_tts_http(args.server, args.text))
    else:
        asyncio.run(send_tts_websocket(args.server, args.text))


if __name__ == "__main__":
    main()
