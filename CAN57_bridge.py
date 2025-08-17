#!/usr/bin/env python3
# can57_bridge.py
import time, struct, threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Int16, Bool, String
import can  # python-can (SocketCAN)

# ====== 환경 ======
CAN_IFACE = "can0"
NODE_IDS = [0x01, 0x02]  # J1=1, J2=2  (MOTOR 메뉴에서 CanID 설정)

# ====== MKS CAN 헬퍼 ======
def checksum(can_id: int, payload: bytes) -> int:
    s = can_id & 0x7FF
    for b in payload:
        s = (s + b) & 0xFF
    return s & 0xFF

def pack_cmd(can_id: int, code: int, data: bytes) -> can.Message:
    # payload = [code] + data + [crc]
    body = bytes([code]) + data
    crc = checksum(can_id, body)
    payload = body + bytes([crc])
    return can.Message(arbitration_id=can_id, data=payload, is_extended_id=False)

def f6_speed(dir_ccw: int, speed: int, acc: int):
    # F6: [code F6] [b2(dir<<7 | speed>>8 &0x0F)] [b3(speed&0xFF)] [b4 acc]
    b2 = ((dir_ccw & 1) << 7) | ((speed >> 8) & 0x0F)
    b3 = speed & 0xFF
    b4 = acc & 0xFF
    return bytes([b2, b3, b4])

def fd_pos(dir_ccw: int, speed: int, acc: int, pulses: int):
    # FD: [b2(dir<<7 | speed>>8 &0x0F)] [b3(speed&0xFF)] [b4 acc] [b5..b7 pulses 24bit]
    b2 = ((dir_ccw & 1) << 7) | ((speed >> 8) & 0x0F)
    b3 = speed & 0xFF
    b4 = acc & 0xFF
    p = pulses & 0xFFFFFF
    return bytes([b2, b3, b4, (p >> 16) & 0xFF, (p >> 8) & 0xFF, p & 0xFF])

def i48_to_int(vbytes: bytes) -> int:
    # 6바이트 부호있는 정수
    assert len(vbytes) == 6
    b = vbytes
    sign = 1 if (b[0] & 0x80) == 0 else -1
    # 파이썬은 큰 정수 잘 처리: int.from_bytes with signed=True
    return int.from_bytes(b, byteorder='big', signed=True)

# ====== 브릿지 노드 ======
class Can57Bridge(Node):
    def __init__(self):
        super().__init__("can57_bridge")
        self.bus = can.ThreadSafeBus(interface='socketcan', channel=CAN_IFACE, bitrate=None)

        # 상태 퍼블리셔 (J1, J2)
        self.pub_axis = {
            0x01: self.create_publisher(Int64, "/can/j1/axis", 10),
            0x02: self.create_publisher(Int64, "/can/j2/axis", 10),
        }
        self.pub_speed = {
            0x01: self.create_publisher(Int16, "/can/j1/speed_rpm", 10),
            0x02: self.create_publisher(Int16, "/can/j2/speed_rpm", 10),
        }
        self.pub_enabled = {
            0x01: self.create_publisher(Bool, "/can/j1/enabled", 10),
            0x02: self.create_publisher(Bool, "/can/j2/enabled", 10),
        }

        # 제어 구독 (문자열 "dir,speed,acc")
        self.create_subscription(String, "/can/j1/run_speed", lambda m: self.on_run_speed(0x01, m), 10)
        self.create_subscription(String, "/can/j2/run_speed", lambda m: self.on_run_speed(0x02, m), 10)
        self.create_subscription(String, "/can/j1/stop",      lambda m: self.on_stop(0x01, m), 10)
        self.create_subscription(String, "/can/j2/stop",      lambda m: self.on_stop(0x02, m), 10)
        self.create_subscription(String, "/can/j1/enable",    lambda m: self.on_enable(0x01, m), 10)
        self.create_subscription(String, "/can/j2/enable",    lambda m: self.on_enable(0x02, m), 10)
        self.create_subscription(String, "/can/j1/set_zero",  lambda m: self.on_set_zero(0x01), 10)
        self.create_subscription(String, "/can/j2/set_zero",  lambda m: self.on_set_zero(0x02), 10)

        # 초기 모드/응답 ON (SR_vFOC=5 권장)
        for nid in NODE_IDS:
            self.set_mode(nid, 5)     # 0:CR_OPEN 1:CR_CLOSE 2:CR_vFOC 3:SR_OPEN 4:SR_CLOSE 5:SR_vFOC
            self.set_can_rsp(nid, 1)  # 응답 활성화

        # 수신 스레드 + 폴링 타이머
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()
        self.timer = self.create_timer(0.05, self.poll_status)  # 20Hz 폴링

    # ---------- 송신 유틸 ----------
    def send(self, msg: can.Message):
        try:
            self.bus.send(msg, timeout=0.02)
        except Exception as e:
            self.get_logger().warn(f"CAN send error: {e}")

    def set_mode(self, nid: int, mode: int):
        self.send(pack_cmd(nid, 0x82, bytes([mode & 0xFF])))

    def set_can_rsp(self, nid: int, en: int):
        self.send(pack_cmd(nid, 0x8C, bytes([1 if en else 0])))

    def cmd_enable(self, nid: int, en: int):
        self.send(pack_cmd(nid, 0xF3, bytes([1 if en else 0])))

    def cmd_speed(self, nid: int, dir_ccw: int, speed: int, acc: int):
        self.send(pack_cmd(nid, 0xF6, f6_speed(dir_ccw, speed, acc)))

    def cmd_speed_stop(self, nid: int, acc: int):
        # 감속정지: speed=0, acc>0  / 즉시정지: acc=0
        self.send(pack_cmd(nid, 0xF6, bytes([0x00, 0x00, acc & 0xFF])))

    def read_axis(self, nid: int):
        # 0x31: 누적 엔코더 값(int48)
        self.send(pack_cmd(nid, 0x31, b""))

    def read_speed(self, nid: int):
        # 0x32: int16 RPM
        self.send(pack_cmd(nid, 0x32, b""))

    def read_enabled(self, nid: int):
        # 0x3A: 1 enabled / 0 disabled
        self.send(pack_cmd(nid, 0x3A, b""))

    def set_zero_axis(self, nid: int):
        # 0x92: 현재 위치를 0으로
        self.send(pack_cmd(nid, 0x92, b""))

    # ---------- ROS 핸들러 ----------
    def on_run_speed(self, nid: int, msg: String):
        try:
            parts = [p.strip() for p in msg.data.split(",")]
            dir_ccw, speed, acc = int(parts[0]), int(parts[1]), int(parts[2])
            # 안전 한도
            speed = max(0, min(3000, speed))
            acc   = max(0, min(255, acc))
            self.cmd_enable(nid, 1)
            self.cmd_speed(nid, dir_ccw, speed, acc)
        except Exception as e:
            self.get_logger().warn(f"run_speed parse error: {e} ({msg.data})")

    def on_stop(self, nid: int, _msg: String):
        # 부드럽게 정지
        self.cmd_speed_stop(nid, acc=2)

    def on_enable(self, nid: int, msg: String):
        en = 1 if msg.data.strip() in ("1","true","True","ON","on") else 0
        self.cmd_enable(nid, en)

    def on_set_zero(self, nid: int):
        self.set_zero_axis(nid)

    # ---------- 폴링 ----------
    def poll_status(self):
        for nid in NODE_IDS:
            self.read_axis(nid)
            self.read_speed(nid)
            self.read_enabled(nid)

    # ---------- 수신 파서 ----------
    def rx_loop(self):
        while True:
            try:
                rx: can.Message = self.bus.recv(timeout=0.1)
                if not rx: 
                    continue
                can_id = rx.arbitration_id & 0x7FF
                data = bytes(rx.data)
                if len(data) < 2:
                    continue
                code = data[0]
                # CRC 체크(가끔 무시해도 되지만 깔끔하게)
                if data[-1] != checksum(can_id, data[:-1]):
                    continue

                if can_id not in NODE_IDS:
                    continue

                if code == 0x31 and len(data) == 1+6+1:
                    # axis int48
                    axis = i48_to_int(data[1:7])
                    self.pub_axis[can_id].publish(Int64(data=int(axis)))
                elif code == 0x32 and len(data) == 1+2+1:
                    rpm = struct.unpack(">h", data[1:3])[0]
                    self.pub_speed[can_id].publish(Int16(data=int(rpm)))
                elif code == 0x3A and len(data) == 1+1+1:
                    en = 1 if data[1] != 0 else 0
                    self.pub_enabled[can_id].publish(Bool(data=bool(en)))
                else:
                    # 필요시 로그
                    pass
            except Exception as e:
                # 조용히 재시도
                time.sleep(0.01)

def main():
    rclpy.init()
    node = Can57Bridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()