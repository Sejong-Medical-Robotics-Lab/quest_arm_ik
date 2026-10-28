"""입력 어댑터: 여러 종류의 Quest 입력 → dual_arm_ik 가 듣는 공통 토픽.

발행 (공통 규약):
  /teleop/{left,right}_hand/target_raw  PoseStamped (로봇 베이스 축의 손 목표 또는 컨트롤러 자세)
  /teleop/{left,right}_hand/engaged     Bool (클러치: True 인 동안만 로봇이 따라감)

input_mode
  xr_teleop   : Isaac ROS 4.6 isaac_ros_teleop (teleop_ros2_node, IsaacTeleop 1.3.x) 가 발행하는
                /xr_teleop/ee_poses (PoseArray: [왼손, 오른손] 컨트롤러 aim 포즈, 이미 ROS 축 x 정면/y 좌측/z 상방, 바닥 원점)
                + /xr_teleop/controller_data (msgpack dict: {side}_squeeze_value 등 — 그립 클러치)
                xr_pose_source: ee_poses(기본) | controller_data({side}_grip_position/orientation, 원시 OpenXR 축 → openxr 변환)
  shpose_udp  : SH-Humanoid Unity ControllerPoseLogger.cs 의 UDP 15000
                "SHPOSE,1,{L|R},t,x,y,z,qx,qy,qz,qw" (이미 base_link 축, 그립 중에만 송신)
  shabs_udp   : SH-Humanoid Unity AbsolutePoseSender.cs 의 UDP 15001
                "SHABS,1,t,head(7),gripL,left(7),gripR,right(7)" (base_link 축, 항상 송신, 그립 플래그 포함)
  team_topics : 팀 udp_pose_bridge 가 발행하는 /teleop/left_wrist_pose, /teleop/right_wrist_pose (PoseStamped)
  pose_joy    : 자체 Quest→PC 링크(Unity ROS-TCP-Connector, rosbridge 등)의 PoseStamped 2개 + Joy(버튼)

engage_mode
  on_stream : stream_timeout 안에 해당 손의 패킷이 들어오는 동안 engaged (Unity 가 그립 중에만 보내는 경우)
  grip      : 패킷/메시지의 그립 값으로 (shabs_udp, xr_teleop, pose_joy)
  always    : 항상 engaged
카메라(MediaPipe) 파이프라인도 같은 규약으로 발행하면 IK 노드는 그대로 재사용된다.
"""
from __future__ import annotations

import socket
import threading
import time
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, ByteMultiArray, Float32

from .frames import quest_to_ros
from .protocols import (SIDES, XR_ENGAGE_KEYS, XR_GRASP_KEYS, msgpack_bytes, parse_shabs, parse_shpose, xr_controller_pose, xr_value,
                        xr_engaged, xr_pose_valid)


class QuestBridge(Node):
    def __init__(self):
        super().__init__("quest_bridge")
        defaults = {
            "input_mode": "xr_teleop",            # xr_teleop | shpose_udp | shabs_udp | team_topics | pose_joy
            "input_convention": "ros",            # unity | openxr | unity_sh | ros  (ros = 변환 없음)
            "output_frame_id": "base_link",
            "engage_mode": "on_stream",           # on_stream | grip | always
            "stream_timeout": 0.3,                # on_stream: 이 시간 안에 패킷이 오면 engaged
            "report_period": 0.0,                 # >0 이면 그 주기[s]로 입력률(L/R Hz) 로그. 0 = 입력 시작/중단 전환 때만 한 줄
            "gap_warn": 0.1,                      # 한쪽 입력이 이보다 오래 끊기면 WARN (v0.7.57 — '뚝뚝 끊김' 이 입력 쪽인지 가리기)
            "no_input_after": 5.0,                # 입력이 이 시간[s] 이상 끊기면 한 번 경고 (0 = 경고 없음)
            # --- xr_teleop ---
            "ee_poses_topic": "/xr_teleop/ee_poses",
            "ee_poses_order": ["left", "right"],  # PoseArray.poses[0], [1] 이 어느 손인지 (teleop_ros2_node: left, right)
            "controller_data_topic": "/xr_teleop/controller_data",
            "left_engage_key": XR_ENGAGE_KEYS["left"],    # grip 모드: msgpack dict 키 (그립 버튼 아날로그 0~1).
            "right_engage_key": XR_ENGAGE_KEYS["right"],  #   트리거로 바꾸려면 {side}_trigger_value. `xr_dump` 로 확인
            "engage_threshold": 0.5,              # 값이 bool 이면 그대로, 숫자면 threshold 초과 시 engaged
            "xr_pose_source": "ee_poses",         # ee_poses | controller_data (ee_poses 가 안 올 때의 대안)
            "xr_controller_pose": "grip",         # controller_data 소스일 때 grip | aim 포즈
            "left_grasp_key": XR_GRASP_KEYS["left"],     # 손 쥐기(0~1) = 검지 트리거(앞쪽 버튼). "" 이면 발행 안 함
            "right_grasp_key": XR_GRASP_KEYS["right"],
            # --- shpose_udp / shabs_udp ---
            "shpose_udp_port": 15000,
            "shabs_udp_port": 15001,
            "udp_bind": "0.0.0.0",
            # --- team_topics ---
            "left_wrist_pose_topic": "/teleop/left_wrist_pose",
            "right_wrist_pose_topic": "/teleop/right_wrist_pose",
            # --- pose_joy ---
            "left_pose_topic": "/quest/left_controller/pose",
            "right_pose_topic": "/quest/right_controller/pose",
            "joy_topic": "/quest/joy",
            "left_engage_button": 0,              # Joy.buttons 인덱스 (그립). 음수면 *_axis 사용
            "right_engage_button": 1,
            "left_engage_axis": -1,               # Joy.axes 인덱스 (아날로그 그립) + axis_threshold
            "right_engage_axis": -1,
            "axis_threshold": 0.5,
            "left_grasp_axis": -1,                # pose_joy: Joy.axes 인덱스(0~1 아날로그) 로 손 쥐기. 음수면 *_grasp_button
            "right_grasp_axis": -1,
            "left_grasp_button": -1,              # pose_joy: Joy.buttons 인덱스 (눌림 = 1.0)
            "right_grasp_button": -1,
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.mode = str(g("input_mode"))
        self.convention = str(g("input_convention"))
        self.frame_id = str(g("output_frame_id"))
        self.engage_mode = str(g("engage_mode"))
        self.stream_timeout = float(g("stream_timeout"))
        self.last_rx = {s: -1.0 for s in SIDES}
        self.grip = {s: False for s in SIDES}
        self.count = {s: 0 for s in SIDES}
        self.bad = 0
        self._warned = False
        self.report_period = float(g("report_period"))
        self.gap_warn = float(g("gap_warn"))
        self.gaps = []                                            # (side, dt) — 이번 1 s 창의 입력 끊김
        self.no_input_after = float(g("no_input_after"))
        self._streaming = False                   # 최근 report 창에 유효 입력이 있었는지 (전환 로그용)
        self._t_start = time.monotonic()
        self._since_report = 0.0
        self._sock = None
        self._running = True

        self.pose_pub = {s: self.create_publisher(PoseStamped, f"/teleop/{s}_hand/target_raw", 10) for s in SIDES}
        self.engaged_pub = {s: self.create_publisher(Bool, f"/teleop/{s}_hand/engaged", 10) for s in SIDES}
        self.grasp_pub = {s: self.create_publisher(Float32, f"/teleop/{s}_hand/grasp", 10) for s in SIDES}   # 0(폄)~1(쥠)
        self.grasp_key = {"left": str(g("left_grasp_key")), "right": str(g("right_grasp_key"))}
        self.grasp_axis = {"left": int(g("left_grasp_axis")), "right": int(g("right_grasp_axis"))}
        self.grasp_btn = {"left": int(g("left_grasp_button")), "right": int(g("right_grasp_button"))}
        self._grasp_warned = False

        if self.mode == "xr_teleop":
            self.order = [str(x) for x in g("ee_poses_order")]
            if sorted(self.order) != sorted(SIDES):
                raise ValueError("ee_poses_order 는 left/right 두 항목이어야 함")
            self.engage_key = {"left": str(g("left_engage_key")), "right": str(g("right_engage_key"))}
            self.engage_thr = float(g("engage_threshold"))
            self.xr_source = str(g("xr_pose_source"))
            self.xr_ctrl_pose = str(g("xr_controller_pose"))
            if self.xr_source not in ("ee_poses", "controller_data"):
                raise ValueError("xr_pose_source: ee_poses | controller_data")
            if self.xr_source == "ee_poses":
                self.create_subscription(PoseArray, g("ee_poses_topic"), self._on_ee_poses, 10)
            if self.engage_mode == "grip" or self.xr_source == "controller_data":
                import msgpack  # noqa: F401  (의존성 확인)
                self.create_subscription(ByteMultiArray, g("controller_data_topic"), self._on_controller_data, 10)
        elif self.mode == "shpose_udp":
            self._start_udp(str(g("udp_bind")), int(g("shpose_udp_port")), self._on_shpose_packet)
        elif self.mode == "shabs_udp":
            self._start_udp(str(g("udp_bind")), int(g("shabs_udp_port")), self._on_shabs_packet)
        elif self.mode == "team_topics":
            for side, topic in (("left", g("left_wrist_pose_topic")), ("right", g("right_wrist_pose_topic"))):
                self.create_subscription(PoseStamped, topic, lambda m, s=side: self._on_pose(s, m), 10)
        elif self.mode == "pose_joy":
            self.btn = {"left": int(g("left_engage_button")), "right": int(g("right_engage_button"))}
            self.axis = {"left": int(g("left_engage_axis")), "right": int(g("right_engage_axis"))}
            self.thr = float(g("axis_threshold"))
            for side, topic in (("left", g("left_pose_topic")), ("right", g("right_pose_topic"))):
                self.create_subscription(PoseStamped, topic, lambda m, s=side: self._on_pose(s, m), 10)
            self.create_subscription(Joy, g("joy_topic"), self._on_joy, 10)
        else:
            raise ValueError("input_mode: xr_teleop | shpose_udp | shabs_udp | team_topics | pose_joy")

        self.create_timer(0.05, self._publish_engaged)      # 20 Hz 로 engaged 플래그 발행
        self.create_timer(1.0, self._report)
        self.get_logger().info(f"quest_bridge: {self.mode} (convention={self.convention}, engage={self.engage_mode})")

    # ------------------------------------------------------------ common
    def _publish_pose(self, side: str, p, q, converted: bool = False) -> None:
        p_out, q_out = (p, q) if converted else quest_to_ros(p, q, self.convention)
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()   # 원본 스탬프 대신 수신 시각 (클록 불일치 방지)
        out.header.frame_id = self.frame_id
        out.pose.position.x, out.pose.position.y, out.pose.position.z = map(float, p_out)
        (out.pose.orientation.x, out.pose.orientation.y,
         out.pose.orientation.z, out.pose.orientation.w) = map(float, q_out)
        self.pose_pub[side].publish(out)
        now = time.monotonic()
        if self.last_rx[side] > 0 and self.gap_warn > 0 and now - self.last_rx[side] > self.gap_warn:
            self.gaps.append((side, now - self.last_rx[side]))
            self.get_logger().warning(f"Quest 입력 끊김 {side[0].upper()} {(now - self.last_rx[side]) * 1000:.0f} ms — 헤드셋이 컨트롤러를 놓침/CloudXR 스트림 지연 "
                                      "(이 순간 팔이 멈칫했다 따라가면 입력 쪽)", throttle_duration_sec=2.0)
        self.last_rx[side] = now
        self.count[side] += 1

    def _publish_engaged(self) -> None:
        now = time.monotonic()
        for side in SIDES:
            if self.engage_mode == "always":
                flag = True
            elif self.engage_mode == "on_stream":
                flag = (now - self.last_rx[side]) < self.stream_timeout
            else:  # grip
                flag = self.grip[side] and (now - self.last_rx[side]) < max(self.stream_timeout, 1.0)
            self.engaged_pub[side].publish(Bool(data=bool(flag)))

    def _report(self) -> None:
        """1 Hz. 기본은 전환 때만 로그: 유효 입력이 시작되면 info 한 줄, no_input_after 이상 끊기면 warning 한 줄.
        report_period > 0 이면 그 주기로 입력률도 찍는다 (진단용)."""
        now = time.monotonic()
        self._since_report += 1.0
        got = sum(self.count.values()) > 0
        rates = ", ".join(f"{s[0].upper()} {self.count[s]:.0f} Hz" for s in SIDES)
        extra = f", bad packets {self.bad}" if self.bad else ""
        last = max(self.last_rx.values())
        if got and not self._streaming:
            self._streaming = True
            self.get_logger().info(f"input started ({self.mode}): {rates}{extra}")
        elif not got and self._streaming and self.no_input_after > 0 and (now - last) >= self.no_input_after:
            self._streaming = False
            self.get_logger().warning(f"no input for {now - last:.0f} s ({self.mode}) — 컨트롤러 비활성/헤드셋 대기/스트림 중단")
        elif not got and not self._streaming and last < 0 and self.no_input_after > 0 \
                and self.no_input_after <= now - self._t_start < self.no_input_after + 1.0:
            self.get_logger().warning(f"no input yet ({self.mode}) — 헤드셋 연결·컨트롤러 활성 후 자동으로 시작됨 (이 경고는 한 번만)")
        if self.gaps:
            worst = max(self.gaps, key=lambda g: g[1])
            self.get_logger().warning(f"입력 끊김 {len(self.gaps)}회 (최대 {worst[0][0].upper()} {worst[1] * 1000:.0f} ms) — {rates}")
            self.gaps = []
        if self.report_period > 0 and self._since_report >= self.report_period:
            self._since_report = 0.0
            self.get_logger().info(f"input {rates}{extra}" if got else f"no input ({self.mode}){extra}")
        elif self.bad and not got:
            self.get_logger().warning(f"bad packets {self.bad} ({self.mode})", throttle_duration_sec=10.0)
        self.count = {s: 0 for s in SIDES}
        self.bad = 0

    # ------------------------------------------------------------ UDP (SH-Humanoid Unity)
    def _start_udp(self, bind: str, port: int, handler) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((bind, port))
        self._sock.settimeout(0.5)
        self._thread = threading.Thread(target=self._udp_loop, args=(handler,), daemon=True)
        self._thread.start()
        self.get_logger().info(f"UDP {bind}:{port} listening")

    def _udp_loop(self, handler) -> None:
        while self._running and rclpy.ok():
            try:
                data, _ = self._sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                handler(data.decode("ascii", "ignore"))
            except ValueError:
                self.bad += 1

    def _on_shpose_packet(self, text: str) -> None:
        pkt = parse_shpose(text)
        if pkt is None:
            self.bad += 1
            return
        side, p, q = pkt
        self._publish_pose(side, p, q)

    def _on_shabs_packet(self, text: str) -> None:
        pkt = parse_shabs(text)
        if pkt is None:
            self.bad += 1
            return
        for side, (grip, p, q) in pkt.items():
            self.grip[side] = grip
            self._publish_pose(side, p, q)

    # ------------------------------------------------------------ xr_teleop
    def _on_ee_poses(self, msg: PoseArray):
        if len(msg.poses) < 2:
            if not self._warned:
                self.get_logger().warning(f"ee_poses has {len(msg.poses)} poses (<2)")
                self._warned = True
            return
        for i, side in enumerate(self.order):
            pose = msg.poses[i]
            p = (pose.position.x, pose.position.y, pose.position.z)
            q = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
            if not xr_pose_valid(p, q):          # 비활성 컨트롤러 자리표시(0,0,0) → 무시 (engaged 도 곧 풀림)
                continue
            self._publish_pose(side, p, q)       # 이미 ROS 축 (input_convention 그대로, 보통 ros)

    def _on_controller_data(self, msg: ByteMultiArray):
        import msgpack
        try:
            tree = msgpack.unpackb(msgpack_bytes(msg), raw=False, strict_map_key=False)
        except Exception as e:  # noqa: BLE001
            if not self._warned:
                self.get_logger().warning(f"controller_data msgpack decode failed: {e}")
                self._warned = True
            return
        for side in SIDES:
            if self.engage_mode == "grip":
                flag = xr_engaged(tree, self.engage_key[side], self.engage_thr)
                if flag is None:
                    if not self._warned:
                        self.get_logger().warning(f"key '{self.engage_key[side]}' not found in controller_data; "
                                                  f"run `ros2 run quest_arm_ik xr_dump` to see the keys")
                        self._warned = True
                    flag = False
                self.grip[side] = flag
            if self.grasp_key[side]:
                gv = xr_value(tree, self.grasp_key[side])
                if gv is None:
                    if not self._grasp_warned:
                        self.get_logger().warning(f"key '{self.grasp_key[side]}' not found in controller_data — 손 쥐기 비활성")
                        self._grasp_warned = True
                else:
                    self.grasp_pub[side].publish(Float32(data=float(min(max(gv, 0.0), 1.0))))
            if self.xr_source == "controller_data":
                pq = xr_controller_pose(tree, side, self.xr_ctrl_pose)
                if pq is not None:
                    p_ros, q_ros = quest_to_ros(pq[0], pq[1], "openxr")   # 원시 OpenXR(RUB) → ROS(FLU)
                    self._publish_pose(side, p_ros, q_ros, converted=True)

    # ------------------------------------------------------------ PoseStamped 입력 (team_topics / pose_joy)
    def _on_pose(self, side: str, msg: PoseStamped):
        p, q = msg.pose.position, msg.pose.orientation
        self._publish_pose(side, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))

    def _on_joy(self, msg: Joy):
        for side in SIDES:
            b, a = self.btn[side], self.axis[side]
            if 0 <= b < len(msg.buttons):
                self.grip[side] = msg.buttons[b] > 0
            elif 0 <= a < len(msg.axes):
                self.grip[side] = msg.axes[a] > self.thr
            else:
                self.grip[side] = False
            ga, gb = self.grasp_axis[side], self.grasp_btn[side]
            if 0 <= ga < len(msg.axes):
                self.grasp_pub[side].publish(Float32(data=float(min(max(msg.axes[ga], 0.0), 1.0))))
            elif 0 <= gb < len(msg.buttons):
                self.grasp_pub[side].publish(Float32(data=1.0 if msg.buttons[gb] > 0 else 0.0))

    def destroy_node(self):
        self._running = False
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = QuestBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
