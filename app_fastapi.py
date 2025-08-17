#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# app_fastapi.py
import json
import threading
import time
from typing import Any, Dict, Optional

from fastapi import FastAPI, Body
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String, Int64, Int16

# -----------------------
# ROS Node (백그라운드)
# -----------------------
class BridgeState:
    def __init__(self):
        self.lock = threading.Lock()
        self.joints = {
            "j1": {"speed": None, "pos": None, "moving": False},
            "j2": {"speed": None, "pos": None, "moving": False},
        }
        self.m_pos = {"m1": None, "m2": None, "m3": None}

    def set_j(self, j: str, key: str, val: Any):
        with self.lock:
            if j in self.joints:
                self.joints[j][key] = val

    def set_mpos(self, m: str, val: Optional[float]):
        with self.lock:
            self.m_pos[m] = val

    def snapshot(self) -> Dict[str, Any]:
        with self.lock:
            return {
                "joints": {
                    "j1": dict(self.joints["j1"]),
                    "j2": dict(self.joints["j2"]),
                },
                "m1": {"pos": self.m_pos["m1"]},
                "m2": {"pos": self.m_pos["m2"]},
                "m3": {"pos": self.m_pos["m3"]},
            }

STATE = BridgeState()

class FastAPIBridgeNode(Node):
    def __init__(self):
        super().__init__("fastapi_bridge_node")

        # --- Subscribers: manual_stepper positions from Moonraker bridge ---
        self.create_subscription(Float32, "/m1/pos", self._on_m1, 10)
        self.create_subscription(Float32, "/m2/pos", self._on_m2, 10)
        self.create_subscription(Float32, "/m3/pos", self._on_m3, 10)

        # --- Subscribers: J1/J2 상태 (String JSON or split topics) ---
        self.create_subscription(String, "/j1/state", self._on_j1_json, 10)
        self.create_subscription(String, "/j2/state", self._on_j2_json, 10)

        # Optional split topics if CAN57_bridge publishes them:
        self.create_subscription(Int16, "/j1/speed", lambda m: STATE.set_j("j1", "speed", int(m.data)), 10)
        self.create_subscription(Int16, "/j2/speed", lambda m: STATE.set_j("j2", "speed", int(m.data)), 10)
        self.create_subscription(Int64, "/j1/pos",   lambda m: STATE.set_j("j1", "pos",   int(m.data)), 10)
        self.create_subscription(Int64, "/j2/pos",   lambda m: STATE.set_j("j2", "pos",   int(m.data)), 10)
        self.create_subscription(Bool,  "/j1/moving",lambda m: STATE.set_j("j1", "moving", bool(m.data)), 10)
        self.create_subscription(Bool,  "/j2/moving",lambda m: STATE.set_j("j2", "moving", bool(m.data)), 10)

        # --- Publishers: commands to ROS bridges ---
        self.pub_gcode = self.create_publisher(String, "/gcode/send", 10)
        self.pub_can_j1 = self.create_publisher(String, "/can/j1/cmd", 10)
        self.pub_can_j2 = self.create_publisher(String, "/can/j2/cmd", 10)

    # --------- Callbacks ---------
    def _on_m1(self, msg: Float32):
        STATE.set_mpos("m1", float(msg.data))

    def _on_m2(self, msg: Float32):
        STATE.set_mpos("m2", float(msg.data))

    def _on_m3(self, msg: Float32):
        STATE.set_mpos("m3", float(msg.data))

    def _parse_joint_json(self, jname: str, s: str):
        try:
            d = json.loads(s)
        except Exception:
            return
        # 유연 파싱: speed/pos/moving 키가 일부만 와도 반영
        if "speed" in d:
            try: STATE.set_j(jname, "speed", int(d["speed"]))
            except: pass
        if "pos" in d:
            try: STATE.set_j(jname, "pos", int(d["pos"]))
            except: pass
        if "moving" in d:
            STATE.set_j(jname, "moving", bool(d["moving"]))

    def _on_j1_json(self, msg: String):
        self._parse_joint_json("j1", msg.data)

    def _on_j2_json(self, msg: String):
        self._parse_joint_json("j2", msg.data)

    # --------- Publish helpers ---------
    def send_gcode(self, script: str):
        self.pub_gcode.publish(String(data=script))

    def send_can_speed(self, joint: str, dir_: int, speed: int, acc: int):
        payload = json.dumps({"type": "speed", "dir": int(dir_), "speed": int(speed), "acc": int(acc)})
        if joint == "j1":
            self.pub_can_j1.publish(String(data=payload))
        elif joint == "j2":
            self.pub_can_j2.publish(String(data=payload))

    def send_can_stop(self, joint: str):
        payload = json.dumps({"type": "stop"})
        if joint == "j1":
            self.pub_can_j1.publish(String(data=payload))
        elif joint == "j2":
            self.pub_can_j2.publish(String(data=payload))


# -----------------------
# Spin ROS in background
# -----------------------
_rclpy_started = False
_ros_node: Optional[FastAPIBridgeNode] = None

def start_ros():
    global _rclpy_started, _ros_node
    if _rclpy_started:
        return
    rclpy.init()
    _ros_node = FastAPIBridgeNode()
    th = threading.Thread(target=lambda: rclpy.spin(_ros_node), daemon=True)
    th.start()
    _rclpy_started = True


# -----------------------
# FastAPI app
# -----------------------
app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"], allow_credentials=True, allow_methods=["*"], allow_headers=["*"],
)

@app.on_event("startup")
def _on_startup():
    start_ros()
    # 초기 상태 안정화 대기(선택)
    time.sleep(0.2)

@app.get("/state")
def get_state():
    return STATE.snapshot()

@app.post("/gcode")
def post_gcode(payload: Dict[str, Any] = Body(...)):
    script = str(payload.get("script", "")).strip()
    if not script:
        return {"ok": False, "error": "script empty"}
    _ros_node.send_gcode(script)
    return {"ok": True}

@app.post("/can/j1/speed")
def post_can_j1_speed(payload: Dict[str, Any] = Body(...)):
    dir_ = int(payload.get("dir", 0))
    speed = int(payload.get("speed", 0))
    acc = int(payload.get("acc", 0))
    _ros_node.send_can_speed("j1", dir_, speed, acc)
    return {"ok": True}

@app.post("/can/j1/stop")
def post_can_j1_stop():
    _ros_node.send_can_stop("j1")
    return {"ok": True}

@app.post("/can/j2/speed")
def post_can_j2_speed(payload: Dict[str, Any] = Body(...)):
    dir_ = int(payload.get("dir", 0))
    speed = int(payload.get("speed", 0))
    acc = int(payload.get("acc", 0))
    _ros_node.send_can_speed("j2", dir_, speed, acc)
    return {"ok": True}

@app.post("/can/j2/stop")
def post_can_j2_stop():
    _ros_node.send_can_stop("j2")
    return {"ok": True}

if __name__ == "__main__":
    # 기본 포트 8000
    uvicorn.run("app_fastapi:app", host="0.0.0.0", port=8000, reload=False)