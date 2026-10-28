"""헤드셋 없이 전체 체인을 시험하는 가짜 Quest 발행기.

IK 노드가 준비될 때(wait_for_topic 에 첫 메시지)까지 기다렸다가, engage_after 초 뒤 양쪽 그립을 잡고(engaged)
컨트롤러를 앞·위로 lift_forward/lift_up 만큼 올린 뒤(팔을 내린 시작 자세에서 공간 확보) 반지름 radius 원으로 천천히 움직이고,
Joy.axes[0,1] 로 손 쥐기(0→1→0, grasp_period) 를 반복한다. 그립을 잡는 순간이 클러치 기준점이다.
(IK 가 뜨기 전에 원을 돌고 있으면 IK 가 올라온 순간의 위상이 기준이 되어, 이후 손이 그 위상만큼 치우친 원을 그린다 —
 예: 바닥점에서 잡히면 손이 13 cm 위로 올라가며 팔꿈치가 한계까지 접힘)
launch 의 input:=mock 이 quest_bridge 를 pose_joy/ros 로 맞춰 준다 (이미 ROS 축으로 발행).
    ros2 launch quest_arm_ik teleop.launch.py model_path:=... input:=mock
"""
from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy

from .mock_traj import mock_offsets, pitch_quat_xyzw


class QuestMock(Node):
    def __init__(self):
        super().__init__("quest_mock")
        self.declare_parameter("rate", 60.0)
        self.declare_parameter("radius", 0.05)        # m. 손은 radius·translation_scale 의 원 (0.08 은 SH teleop_ready 의 전방 여유 12 cm 를 넘음)
        self.declare_parameter("period", 6.0)         # s, 한 바퀴
        self.declare_parameter("engage_after", 2.0)   # s (IK 준비 확인 후부터)
        self.declare_parameter("forward_axis", [1.0, 0.0, 0.0])   # 베이스 프레임의 정면 (SH base_link 는 [0,1,0])
        self.declare_parameter("wait_for_topic", "/teleop/joint_targets")   # IK 출력(JointState). "" 이면 기다리지 않음
        self.declare_parameter("wait_timeout", 120.0)                       # s, 이 시간 안에 안 오면 경고 후 시작
        self.declare_parameter("lift_forward", 0.06)  # m, 그립 직후 손을 앞으로 올리는 양 (팔을 내린 시작 자세에서 원을 그릴 공간 확보; 컨트롤러 기준, 로봇은 ×translation_scale)
        self.declare_parameter("lift_up", 0.15)       # m, 위로 (팔이 거의 펴진 시작 자세라 위로 충분히 올려야 원의 앞쪽 끝이 닿는다 — SH 팔 0.43 m 기준 검증)
        self.declare_parameter("lift_time", 2.0)      # s, 올리는 데 걸리는 시간 (그 뒤 원 시작)
        self.declare_parameter("grasp_period", 4.0)   # s, 손 쥐기(Joy.axes[0,1] 0→1→0) 주기. 0 이면 쥐지 않음
        self.declare_parameter("lift_pitch_deg", 60.0) # 올리는 동안 손끝을 아래→앞으로 드는 각 (사람 팔 동작처럼)
        self.rate = float(self.get_parameter("rate").value)
        self.radius = float(self.get_parameter("radius").value)
        self.period = float(self.get_parameter("period").value)
        self.engage_after = float(self.get_parameter("engage_after").value)
        self.lift = (float(self.get_parameter("lift_forward").value), float(self.get_parameter("lift_up").value),
                     max(float(self.get_parameter("lift_time").value), 1e-3))
        self.grasp_period = float(self.get_parameter("grasp_period").value)
        self.lift_pitch = math.radians(float(self.get_parameter("lift_pitch_deg").value))
        f = [float(v) for v in self.get_parameter("forward_axis").value]
        n = math.sqrt(sum(v * v for v in f)) or 1.0
        self.fwd = [v / n for v in f]
        self.pubs = {s: self.create_publisher(PoseStamped, f"/quest/{s}_controller/pose", 10)
                     for s in ("left", "right")}
        self.joy_pub = self.create_publisher(Joy, "/quest/joy", 10)
        self.wait_topic = str(self.get_parameter("wait_for_topic").value)
        self.wait_timeout = float(self.get_parameter("wait_timeout").value)
        self.t_start = time.monotonic()
        self.t0 = None                                  # IK 준비 확인 시각 (여기서부터 engage_after 를 셈)
        if self.wait_topic:
            from sensor_msgs.msg import JointState
            self.create_subscription(JointState, self.wait_topic, self._on_ready, 10)
            self.get_logger().info(f"IK 준비 대기: {self.wait_topic} 첫 메시지 후 {self.engage_after:.0f} s 뒤 그립")
        else:
            self.t0 = self.t_start
        self.create_timer(1.0 / self.rate, self._tick)

    def _on_ready(self, _msg):
        if self.t0 is None:
            self.t0 = time.monotonic()
            self.get_logger().info("IK 출력 확인 — 곧 그립을 잡고 원을 시작")

    def _tick(self):
        if self.t0 is None:
            if time.monotonic() - self.t_start > self.wait_timeout:
                self.get_logger().warning(f"{self.wait_topic} 가 {self.wait_timeout:.0f} s 동안 안 옴 — 기다리지 않고 시작")
                self.t0 = time.monotonic()
            else:
                return                                   # IK 가 뜨기 전에는 아무것도 발행하지 않음
        t = time.monotonic() - self.t0
        engaged = t > self.engage_after
        te = t - self.engage_after                                              # 그립 후 경과 시간 (음수면 아직)
        lf, lu, lt = self.lift
        a, upz, g, pitch = mock_offsets(te, self.radius, self.period, lf, lu, lt, self.grasp_period, self.lift_pitch)   # 앞·위 변위, 쥐기, 피치
        qx, qy, qz, qw = pitch_quat_xyzw(pitch, self.fwd)                      # 정면 축 기준 피치 (SH 는 +y 정면 → x 축 회전)
        for side, y0 in (("left", 0.25), ("right", -0.25)):
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "quest_world"
            # 절대값은 무의미(상대 매핑) — engage 이후의 변화량만 로봇에 반영됨
            msg.pose.position.x = 0.3 * self.fwd[0] + a * self.fwd[0] + y0 * (1.0 - abs(self.fwd[0]))
            msg.pose.position.y = 0.3 * self.fwd[1] + a * self.fwd[1] + y0 * (1.0 - abs(self.fwd[1]))
            msg.pose.position.z = 1.0 + upz
            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = qx, qy, qz, qw
            self.pubs[side].publish(msg)
        joy = Joy()
        joy.header.stamp = self.get_clock().now().to_msg()
        joy.buttons = [int(engaged), int(engaged)]
        joy.axes = [g, g]                                                        # quest_bridge: left/right_grasp_axis 0/1
        self.joy_pub.publish(joy)


def main(args=None):
    rclpy.init(args=args)
    node = QuestMock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
