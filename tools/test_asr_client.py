#!/usr/bin/env python3
"""
ASR Test Client

订阅 ASR 结果，接收 ESP32 麦克风识别的文本。

用法:
    python test_asr_client.py --server ws://192.168.31.98:8080
    python test_asr_client.py --server ws://localhost:8080 --verbose
"""

import argparse
import asyncio
import json
import logging
import sys
from datetime import datetime

import aiohttp

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
logger = logging.getLogger(__name__)


class ASRSubscriber:
    """ASR 结果订阅客户端"""

    def __init__(self, server_url: str, verbose: bool = False):
        self.server_url = server_url
        self.verbose = verbose
        self._running = False

    async def run(self):
        """连接并接收 ASR 结果"""
        ws_url = self.server_url.replace("http://", "ws://").replace(
            "https://", "wss://"
        )
        if not ws_url.endswith("/api/asr"):
            ws_url = ws_url.rstrip("/") + "/api/asr"

        logger.info(f"连接到 ASR 服务: {ws_url}")

        self._running = True

        async with aiohttp.ClientSession() as session:
            while self._running:
                try:
                    async with session.ws_connect(ws_url) as ws:
                        logger.info("已连接，等待 ASR 结果...")
                        logger.info("按 Ctrl+C 退出\n")

                        async for msg in ws:
                            if msg.type == aiohttp.WSMsgType.TEXT:
                                await self._handle_message(msg.data)

                            elif msg.type == aiohttp.WSMsgType.ERROR:
                                logger.error(f"WebSocket 错误: {ws.exception()}")
                                break

                            elif msg.type == aiohttp.WSMsgType.CLOSED:
                                logger.info("连接已关闭")
                                break

                except aiohttp.ClientError as e:
                    logger.error(f"连接失败: {e}")
                    if self._running:
                        logger.info("5秒后重连...")
                        await asyncio.sleep(5)

                except asyncio.CancelledError:
                    break

    async def _handle_message(self, data: str):
        """处理收到的消息"""
        try:
            msg = json.loads(data)
        except json.JSONDecodeError:
            logger.warning(f"无效 JSON: {data}")
            return

        msg_type = msg.get("type")
        timestamp = datetime.now().strftime("%H:%M:%S")

        if msg_type == "connected":
            # 连接成功消息
            sessions = msg.get("active_sessions", 0)
            logger.info(f"当前活动会话数: {sessions}")
            if sessions > 0:
                for s in msg.get("sessions", []):
                    logger.info(f"  - 设备: {s.get('device_id')} ({s.get('session_id')[:8]}...)")

        elif msg_type == "partial":
            # 部分识别结果
            text = msg.get("text", "")
            device = msg.get("device_id", "unknown")

            if self.verbose:
                print(f"\r[{timestamp}] [{device}] (识别中) {text}...", end="", flush=True)

        elif msg_type == "final":
            # 最终识别结果
            text = msg.get("text", "")
            device = msg.get("device_id", "unknown")

            # 清除部分结果行
            print("\r" + " " * 80 + "\r", end="")
            print(f"[{timestamp}] [{device}] ✓ {text}")

        elif msg_type == "pong":
            # 心跳响应
            if self.verbose:
                logger.debug("收到心跳响应")

        else:
            if self.verbose:
                logger.debug(f"未知消息类型: {msg}")

    def stop(self):
        """停止订阅"""
        self._running = False


async def main(args):
    subscriber = ASRSubscriber(args.server, args.verbose)

    try:
        await subscriber.run()
    except KeyboardInterrupt:
        logger.info("\n正在退出...")
        subscriber.stop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="ASR 测试客户端 - 接收语音识别结果",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 连接到服务器接收 ASR 结果
  python test_asr_client.py --server ws://192.168.31.98:8080

  # 详细模式（显示部分识别结果）
  python test_asr_client.py --server ws://localhost:8080 --verbose
        """,
    )
    parser.add_argument(
        "--server",
        default="ws://localhost:8080",
        help="服务器地址 (默认: ws://localhost:8080)",
    )
    parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="详细模式（显示部分识别结果）",
    )

    args = parser.parse_args()
    asyncio.run(main(args))
