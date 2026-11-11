# Copyright 2025-2026 Dimensional Inc. Licensed under the Apache License, Version 2.0.
"""Communication-only extraction of dimos_simple's wired Unitree connection.

Keeps the pinned SDK, direct Ethernet ICE policy, mode selection and joystick
mapping. No video, replay, LCM, reactive streams or DimOS message conversion.
"""

import asyncio
import logging
import threading
from concurrent.futures import Future
from pathlib import Path

from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD

from dimos.robot.unitree.go2.wired_link import WiredLink
from dimos.robot.unitree.go2.wired_webrtc import WiredWebRTC

log = logging.getLogger(__name__)


class UnitreeWebRTCConnection:
    def __init__(self, ip, mode="ai", aes_128_key=None):
        self.link = WiredLink.check(ip)
        self.conn = WiredWebRTC(self.link, aes_128_key)
        self.mode = mode
        self.loop = asyncio.new_event_loop()
        self.thread = None
        self.task = None
        self.closed = False

    def start(self):
        startup = Future()

        async def connect():
            try:
                await asyncio.wait_for(self.conn.connect(), timeout=20)
                await self.conn.datachannel.disableTrafficSaving(True)
                self.conn.datachannel.set_decoder(decoder_type="native")
                await asyncio.wait_for(
                    self.conn.datachannel.pub_sub.publish_request_new(
                        RTC_TOPIC["MOTION_SWITCHER"],
                        {"api_id": 1002, "parameter": {"name": self.mode}},
                    ),
                    timeout=10,
                )
                startup.set_result(None)
                carrier = Path("/sys/class/net") / self.link.interface / "carrier"
                while True:
                    await asyncio.sleep(0.25)
                    if (
                        carrier.read_text().strip() != "1"
                        or self.conn.pc is None
                        or self.conn.pc.connectionState in ("failed", "closed")
                    ):
                        raise RuntimeError("wired Go2 connection lost")
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                if not startup.done():
                    startup.set_exception(exc)
                self.failure = exc
                log.error("Go2 connection: %s", exc)

        def worker():
            asyncio.set_event_loop(self.loop)
            self.task = self.loop.create_task(connect())
            self.loop.run_forever()
            pending = asyncio.all_tasks(self.loop)
            for task in pending:
                task.cancel()
            self.loop.run_until_complete(
                asyncio.gather(*pending, return_exceptions=True)
            )
            self.loop.close()

        self.failure = None
        self.thread = threading.Thread(target=worker, daemon=True)
        self.thread.start()
        try:
            startup.result(timeout=35)
        except Exception:
            self.stop()
            raise

    def check(self):
        if self.failure:
            raise RuntimeError(str(self.failure))

    def subscribe(self, topic, callback):
        self.loop.call_soon_threadsafe(
            self.conn.datachannel.pub_sub.subscribe, topic, callback
        )

    def move(self, x, y, yaw):
        async def send():
            self.conn.datachannel.pub_sub.publish_without_callback(
                RTC_TOPIC["WIRELESS_CONTROLLER"],
                data={"lx": -y, "ly": x, "rx": -yaw, "ry": 0},
            )

        asyncio.run_coroutine_threadsafe(send(), self.loop).result(timeout=2)

    def posture(self, name):
        api = {
            "standup": "StandUp",
            "liedown": "StandDown",
            "balance_stand": "BalanceStand",
        }[name]
        future = asyncio.run_coroutine_threadsafe(
            self.conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD[api]}
            ),
            self.loop,
        )
        return bool(future.result(timeout=10))

    def stop(self):
        if self.closed:
            return
        self.closed = True
        if self.loop.is_running():

            async def disconnect():
                try:
                    self.conn.datachannel.pub_sub.publish_without_callback(
                        RTC_TOPIC["WIRELESS_CONTROLLER"],
                        data={"lx": 0, "ly": 0, "rx": 0, "ry": 0},
                    )
                    await self.conn.disconnect()
                except Exception:
                    log.exception("Go2 disconnect failed")

            try:
                asyncio.run_coroutine_threadsafe(disconnect(), self.loop).result(
                    timeout=3
                )
            except Exception:
                log.exception("Go2 disconnect timed out")
            self.loop.call_soon_threadsafe(self.loop.stop)
        if self.thread:
            self.thread.join(timeout=4)
