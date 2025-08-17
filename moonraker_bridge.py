#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# moonraker_bridge.py
import os
import asyncio
import json
import threading
import time
from typing import Any, Dict

import aiohttp
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String

MOONRAKER_BASE = os.environ.get("MOONRAKER_BASE", "http://127.0.0.1:7125")
WS_URL = MOONRAKER_BASE.replace("http", "ws") + "/websocket"

SUB_OBJECTS = {
    "gcode_move": ["is_gcode_moving"],
    "manual_stepper m1": ["position"],
    "manual_stepper m2": ["position"],
    "manual_stepper m3": ["position"],
}

class MoonrakerBridge(Node):
    def __init__(self):
        super().__init__("moonraker_bridge")
        # publishers
        self.pub_pos = {
            "manual_stepper m1": self.create_publisher(Float32, "/m1/pos", 10),
            "manual_stepper m2": self.create_publisher(Float32, "/m2/pos", 10),
            "manual_stepper m3": self.create_publisher(Float32, "/m3/pos", 10),
        }
        self.pub_moving = self.create_publisher(Bool, "/klipper/moving", 10)

        # subscriber: gcode
        self.create_subscription(String, "/gcode/send", self.on_gcode, 10)

        # async runner
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()

    # ---------- HTTP helpers ----------
    async def _http_post(self, path: str, json_body: Dict[str, Any]):
        url = f"{MOONRAKER_BASE}{path}"
        async with aiohttp.ClientSession() as sess:
            async with sess.post(url, json=json_body, timeout=10) as r:
                r.raise_for_status()
                return await r.json()

    async def _http_get(self, path: str):
        url = f"{MOONRAKER_BASE}{path}"
        async with aiohttp.ClientSession() as sess:
            async with sess.get(url, timeout=10) as r:
                r.raise_for_status()
                return await r.json()

    def on_gcode(self, msg: String):
        # POST /printer/gcode/script
        fut = asyncio.run_coroutine_threadsafe(
            self._http_post("/printer/gcode/script", {"script": msg.data}), self.loop
        )
        try:
            fut.result(timeout=5)
        except Exception as e:
            self.get_logger().warn(f"G-code post failed: {e}")

    # ---------- Async loop ----------
    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.create_task(self._main_async())
        self.loop.run_forever()

    async def _initial_query(self):
        q = await self._http_get(
            "/printer/objects/query?"
            "gcode_move=is_gcode_moving&"
            "manual_stepper%20m1=position&"
            "manual_stepper%20m2=position&"
            "manual_stepper%20m3=position"
        )
        st = (q.get("result") or {}).get("status") or {}

        gm = st.get("gcode_move") or {}
        self.pub_moving.publish(Bool(data=bool(gm.get("is_gcode_moving", False))))

        for k in self.pub_pos:
            pos = (st.get(k) or {}).get("position")
            if isinstance(pos, (int, float)):
                self.pub_pos[k].publish(Float32(data=float(pos)))

    async def _ws_loop(self):
        sub_msg = {
            "jsonrpc": "2.0",
            "method": "printer.objects.subscribe",
            "params": {"objects": SUB_OBJECTS},
            "id": 1,
        }
        while rclpy.ok():
            try:
                async with aiohttp.ClientSession() as sess:
                    async with sess.ws_connect(WS_URL, heartbeat=20) as ws:
                        await ws.send_str(json.dumps(sub_msg))
                        async for msg in ws:
                            if msg.type != aiohttp.WSMsgType.TEXT:
                                continue
                            try:
                                data = json.loads(msg.data)
                            except Exception:
                                continue

                            if data.get("method") == "notify_status_update":
                                payload = (data.get("params") or [None])[0] or {}

                                gm = payload.get("gcode_move")
                                if gm and "is_gcode_moving" in gm:
                                    self.pub_moving.publish(Bool(data=bool(gm["is_gcode_moving"])))

                                for k in self.pub_pos:
                                    v = payload.get(k)
                                    if v and "position" in v:
                                        pos = v["position"]
                                        if isinstance(pos, (int, float)):
                                            self.pub_pos[k].publish(Float32(data=float(pos)))
            except Exception as e:
                self.get_logger().warn(f"Moonraker WS error: {e}; retry in 2s")
                await asyncio.sleep(2.0)

    async def _main_async(self):
        try:
            await self._initial_query()
        except Exception as e:
            self.get_logger().warn(f"initial query failed: {e}")
        await self._ws_loop()


def main():
    rclpy.init()
    node = MoonrakerBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()