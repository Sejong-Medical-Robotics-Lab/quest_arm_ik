"""Isaac ROS /xr_teleop/* 토픽의 규약을 한 번 찍어 보는 도구.

    ros2 run quest_arm_ik xr_dump            # ee_poses 1개 + controller_data 1개를 받으면 종료
    ros2 run quest_arm_ik xr_dump --ros-args -p timeout:=20.0

출력: ee_poses 의 frame_id·포즈 개수·각 포즈 좌표, controller_data(msgpack) 의 키 구조(타입·값).
Isaac ROS 4.6 (isaac_ros_teleop) 기준 기대값: poses[0]=왼손, poses[1]=오른손, frame_id='world' (ROS 축, 바닥 원점),
controller_data 키 {side}_squeeze_value(그립), {side}_trigger_value, {side}_grip_position/_orientation, {side}_is_active ...
다르면 quest_bridge 의 ee_poses_order / left_engage_key / right_engage_key 를 그에 맞게 바꾼다.
"""
from __future__ import annotations

import time
from typing import Any

import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray

from .protocols import msgpack_bytes   # (pinocchio 를 끌어오지 않음 — 시스템 python 으로도 실행 가능)


def describe(obj: Any, prefix: str = "", depth: int = 0, max_list: int = 8) -> str:
    pad = "  " * depth
    if isinstance(obj, dict):
        lines = [f"{pad}{prefix}dict({len(obj)})"]
        for k, v in obj.items():
            key = k.decode() if isinstance(k, bytes) else str(k)
            lines.append(describe(v, f"{key}: ", depth + 1, max_list))
        return "\n".join(lines)
    if isinstance(obj, (list, tuple)):
        if all(isinstance(x, (int, float, bool)) for x in obj) and len(obj) <= 16:
            return f"{pad}{prefix}{type(obj).__name__}[{len(obj)}] = {[round(x, 4) if isinstance(x, float) else x for x in obj]}"
        lines = [f"{pad}{prefix}{type(obj).__name__}[{len(obj)}]"]
        for i, v in enumerate(obj[:max_list]):
            lines.append(describe(v, f"[{i}] ", depth + 1, max_list))
        if len(obj) > max_list:
            lines.append(f"{pad}  ...")
        return "\n".join(lines)
    if isinstance(obj, float):
        return f"{pad}{prefix}float = {obj:.4f}"
    if isinstance(obj, (bytes, bytearray)):
        return f"{pad}{prefix}bytes[{len(obj)}]"
    return f"{pad}{prefix}{type(obj).__name__} = {obj}"


class XrDump(Node):
    def __init__(self):
        super().__init__("xr_dump")
        self.declare_parameter("ee_poses_topic", "/xr_teleop/ee_poses")
        self.declare_parameter("controller_data_topic", "/xr_teleop/controller_data")
        self.declare_parameter("timeout", 10.0)
        self.got_poses = self.got_data = False
        self.create_subscription(PoseArray, self.get_parameter("ee_poses_topic").value, self._on_poses, 10)
        self.create_subscription(ByteMultiArray, self.get_parameter("controller_data_topic").value, self._on_data, 10)
        self.t0 = time.monotonic()

    def done(self) -> bool:
        return (self.got_poses and self.got_data) or \
            (time.monotonic() - self.t0) > float(self.get_parameter("timeout").value)

    def _on_poses(self, msg: PoseArray):
        if self.got_poses:
            return
        self.got_poses = True
        lines = [f"[ee_poses] frame_id='{msg.header.frame_id}'  stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}  "
                 f"poses={len(msg.poses)}"]
        for i, p in enumerate(msg.poses):
            lines.append(f"  poses[{i}] pos=({p.position.x:+.3f}, {p.position.y:+.3f}, {p.position.z:+.3f}) "
                         f"quat(xyzw)=({p.orientation.x:+.3f}, {p.orientation.y:+.3f}, {p.orientation.z:+.3f}, {p.orientation.w:+.3f})")
        lines.append("  → ROS 축: 정면 x, 좌측 y, 상방 z (연결 순간 머리 방향 기준, 원점은 바닥). 왼손이 y>0, 손을 들면 z 증가.")
        lines.append("    (0,0,0)+(0,0,0,1) 인 항목은 그 컨트롤러가 비활성(추적 안 됨)이라는 뜻.")
        print("\n".join(lines), flush=True)

    def _on_data(self, msg: ByteMultiArray):
        if self.got_data:
            return
        self.got_data = True
        raw = msgpack_bytes(msg)
        print(f"[controller_data] {len(raw)} bytes", flush=True)
        try:
            import msgpack
            tree = msgpack.unpackb(raw, raw=False, strict_map_key=False)
            print(describe(tree), flush=True)
            print("  → 그립 클러치 키: left_squeeze_value / right_squeeze_value (0~1). 트리거를 쓰려면 {side}_trigger_value.", flush=True)
        except Exception as e:  # noqa: BLE001
            print(f"  msgpack decode failed: {e}\n  head={raw[:32]!r}", flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = XrDump()
    while rclpy.ok() and not node.done():
        rclpy.spin_once(node, timeout_sec=0.2)
    if not node.got_poses:
        print("ee_poses 를 받지 못함 — teleop 노드가 떠 있고 헤드셋이 연결되어 있는지 확인", flush=True)
    if not node.got_data:
        print("controller_data 를 받지 못함", flush=True)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
