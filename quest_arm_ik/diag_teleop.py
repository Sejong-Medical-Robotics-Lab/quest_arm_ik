"""실행 중인 텔레옵 체인 진단 (1 Hz 표).  ros2 run quest_arm_ik diag_teleop

- 노드 목록(중복 실행 감지), 핵심 토픽의 발행자 수
- IK 명령 관절각(/teleop/joint_targets 또는 /quest_ik/joint_states) vs 측정 관절각(/joint_states): 팔꿈치 현재/최소값
- 엔게이지 토글 횟수, target_raw·joint_states 수신률
명령 팔꿈치는 IK 한계 안인데 측정값이 그 밖이면 시뮬/다른 발행자 문제, 명령 자체가 밖이면 IK(설정 미반영) 문제.
"""
from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

SIDES = ("left", "right")
TOPICS = ["/joint_states", "/teleop/joint_targets", "/quest_ik/joint_states",
          "/left_arm_forward_controller/commands", "/right_arm_forward_controller/commands",
          "/arm_controller/joint_trajectory", "/left_arm_controller/joint_trajectory",
          "/teleop/left_hand/target_raw", "/teleop/left_hand/engaged", "/quest/left_controller/pose",
          "/teleop/left_hand/grasp", "/teleop/hand_joint_targets"]


class Diag(Node):
    def __init__(self):
        super().__init__("diag_teleop")
        self.declare_parameter("elbow_joint", "left_elbow_joint")
        self.elbow = str(self.get_parameter("elbow_joint").value)
        self.meas = self.cmd = None
        self.meas_min = self.cmd_min = 9.0
        self.n_meas = self.n_cmd = self.n_raw = 0
        self.engaged = {s: None for s in SIDES}
        self.toggles = {s: 0 for s in SIDES}
        self.raw_last = None
        self.raw_gap_max = 0.0
        self.create_subscription(JointState, "/joint_states", self._on_meas, 10)
        self.create_subscription(JointState, "/teleop/joint_targets", self._on_cmd, 10)
        self.create_subscription(JointState, "/quest_ik/joint_states", self._on_cmd, 10)
        for s in SIDES:
            self.create_subscription(Bool, f"/teleop/{s}_hand/engaged", lambda m, s=s: self._on_eng(s, m), 10)
        self.create_subscription(PoseStamped, "/teleop/left_hand/target_raw", self._on_raw, 10)
        self.create_timer(1.0, self._report)
        self.t0 = time.monotonic()

    def _elbow(self, msg: JointState):
        if self.elbow in msg.name:
            return float(msg.position[list(msg.name).index(self.elbow)])
        return None

    def _on_meas(self, msg):
        v = self._elbow(msg)
        if v is not None:
            self.meas = v; self.meas_min = min(self.meas_min, v); self.n_meas += 1

    def _on_cmd(self, msg):
        v = self._elbow(msg)
        if v is not None:
            self.cmd = v; self.cmd_min = min(self.cmd_min, v); self.n_cmd += 1

    def _on_eng(self, side, msg):
        if self.engaged[side] is not None and msg.data != self.engaged[side]:
            self.toggles[side] += 1
        self.engaged[side] = msg.data

    def _on_raw(self, msg):
        now = time.monotonic()
        if self.raw_last is not None:
            self.raw_gap_max = max(self.raw_gap_max, now - self.raw_last)
        self.raw_last = now; self.n_raw += 1

    def _report(self):
        t = time.monotonic() - self.t0
        if int(t) % 10 == 1:                                  # 10 초마다 노드·발행자 점검
            names = sorted(self.get_node_names())
            dups = sorted({n for n in names if names.count(n) > 1})
            self.get_logger().info(f"nodes({len(names)}): {names}" + (f"  ★ 중복 노드: {dups} — 이전 launch 가 살아 있음!" if dups else ""))
            pubs = {tp: self.count_publishers(tp) for tp in TOPICS}
            multi = {k: v for k, v in pubs.items() if v > 1}
            self.get_logger().info("publishers: " + ", ".join(f"{k}={v}" for k, v in pubs.items() if v)
                                   + (f"  ★ 발행자 2개 이상: {multi}" if multi else ""))
        fmt = lambda v: "   -  " if v is None else f"{v:+.3f}"  # noqa: E731
        self.get_logger().info(
            f"t={t:5.1f}s  elbow cmd {fmt(self.cmd)} (min {fmt(None if self.cmd_min > 8 else self.cmd_min)})  "
            f"meas {fmt(self.meas)} (min {fmt(None if self.meas_min > 8 else self.meas_min)})  "
            f"rates: joint_states {self.n_meas}/s cmd {self.n_cmd}/s target_raw {self.n_raw}/s (max gap {self.raw_gap_max*1000:.0f} ms)  "
            f"engaged L/R {self.engaged['left']}/{self.engaged['right']} toggles {self.toggles['left']}/{self.toggles['right']}")
        self.n_meas = self.n_cmd = self.n_raw = 0


def main(args=None):
    rclpy.init(args=args)
    node = Diag()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
