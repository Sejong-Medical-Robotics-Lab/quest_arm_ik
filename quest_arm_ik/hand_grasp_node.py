"""컨트롤러 트리거 → 손가락 관절 목표.  /teleop/{left,right}_hand/grasp (Float32 0~1) → /teleop/hand_joint_targets (JointState)

mujoco_sim 이 이 토픽의 관절(제어 대상 아닌 손가락 관절)을 목표로 삼고, 실기체는 Dex3 드라이버(Unitree SDK) 가 같은 토픽을 받아 쓰면 된다.
열림/쥠 각은 hand_joints_yaml ({left: {joint: [open, closed]}, right: {...}}); 비우면 Dex3-1 기본값.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

from .hand import DEX3_GRASP, SIDES, GraspMapper


class HandGraspNode(Node):
    def __init__(self):
        super().__init__("hand_grasp")
        self.declare_parameter("hand_joints_yaml", "")
        self.declare_parameter("rate", 50.0)
        self.declare_parameter("tau", 0.1)                     # s, 트리거 저역통과 시정수
        self.declare_parameter("targets_topic", "/teleop/hand_joint_targets")
        self.declare_parameter("grasp_topic_template", "/teleop/{side}_hand/grasp")
        spec_yaml = str(self.get_parameter("hand_joints_yaml").value)
        spec = DEX3_GRASP
        if spec_yaml.strip():
            import yaml
            spec = yaml.safe_load(spec_yaml)
        self.mapper = GraspMapper(spec, tau=float(self.get_parameter("tau").value))
        self.dt = 1.0 / float(self.get_parameter("rate").value)
        self.pub = self.create_publisher(JointState, str(self.get_parameter("targets_topic").value), 10)
        tmpl = str(self.get_parameter("grasp_topic_template").value)
        for side in SIDES:
            self.create_subscription(Float32, tmpl.format(side=side), lambda m, s=side: self.mapper.set(s, m.data), 10)
        self.create_timer(self.dt, self._tick)
        self._last_state = {s: None for s in SIDES}
        self.get_logger().info(f"hand_grasp: {len(self.mapper.joint_names)} finger joints, "
                               f"{tmpl.format(side='{left|right}')} → {self.get_parameter('targets_topic').value}")

    def _tick(self):
        targets = self.mapper.update(self.dt)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(targets.keys())
        msg.position = [float(v) for v in targets.values()]
        self.pub.publish(msg)
        for side in SIDES:                                  # 전환 로그: 쥠/폄
            v = self.mapper.value.get(side, 0.0)
            state = "closed" if v > 0.7 else "open" if v < 0.3 else None
            if state and state != self._last_state[side]:
                if self._last_state[side] is not None or state == "closed":
                    self.get_logger().info(f"{side} hand {state}")
                self._last_state[side] = state


def main(args=None):
    rclpy.init(args=args)
    node = HandGraspNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
