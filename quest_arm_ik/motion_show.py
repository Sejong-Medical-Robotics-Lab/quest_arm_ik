"""전시용 동작 쇼 노드 — 안무(choreography.py)를 /teleop/joint_targets(JointState, 100 Hz) 로 내보낸다. IK 노드 자리에 들어간다.

    ros2 launch quest_arm_ik teleop.launch.py robot:=sh input:=show hr:=true sim:=true viewer:=true   # 시뮬 리허설 (실기체와 같은 경로; 실기체 릴레이는 꺼 둘 것)
    ros2 launch quest_arm_ik teleop.launch.py robot:=sh input:=show hr:=true sim:=false relay:=false show_speed:=0.5   # 실기체 (릴레이는 tmux 창 2; v_max 기본 2.0)
    ros2 launch … input:=show show_acts:=heart,wave show_speed:=0.7 show_loop:=true                               # 일부만, 느리게, 반복

동작 순서(기본): heart → wave → run → kiss (각 10~13 s, 사이 자동 전환, 끝나면 팔 내린 자세로). 전체 ≈ 55 s (speed 1.0).
안전: 실기체 경로(motor_states 가 옴)에서는 upper_body_streamer 가 /upper_body/streamer_state 로 "commanding=1"(발행자 하나·드라이버 ENABLED·측정 있음)
      을 알릴 때까지 아무 목표도 내지 않고 기다린다(wait_streamer). 그 순간의 측정 관절각에서 blend_time 동안 최소 저크로 쇼의 시작 자세로 들어가고,
      틱마다 관절 속도 v_max·dt 이하·관절 한계 안으로 자른다. 그 뒤는 upper_body_streamer 가 v_max·a_max 로 한 번 더 지키고,
      명령-측정 차가 max_tracking_error 를 넘으면 명령을 측정값으로 되돌리고 선다.
      쇼 도중 스트리머가 명령을 못 내거나(disable·발행자 2개·hold·추종 오차) 목표까지 접근 중(catchup=1)이면 쇼 시계를 멈추고 마지막 목표를 유지하며,
      스트리머가 다시 TRACK 으로 목표에 닿으면 이어서 진행한다(재활성화 뒤 멀리 있는 쇼 자세로는 v_catchup 0.3 rad/s 로 천천히 간다).
      Ctrl-C 로 끝내면 마지막 자세를 유지한다(스트리머 HOLD). 시작 전 팔 반경을 비우고 비상정지 담당을 둔다.
      joint_states 만 오는 순수 시뮬(hr 아님)은 바로 시작하고, 측정이 start_timeout 안에 안 오면 REST 에서 시작한다고 가정(시뮬 전용).

2026-10-28: ① `input:=show hr:=true` 만 주면 (v0.7.53 까지) launch 의 sim 기본값이 true 라 HR 에뮬레이션이 실기체 릴레이와 함께 떠 측정값이 섞였고
   실기체가 튀었다 → v0.7.54 hr:=true 면 sim 기본 false, 스트리머는 측정 발행자가 둘이면 명령을 내지 않음.
② 쇼 시계가 enable 과 무관하게 launch 직후 돌아, enable 순간 쇼 목표가 멀리 있어 스트리머가 v_max 2.5 로 튀고 추종 오차 정지가 반복(0.71 → 2.40 rad)
   → v0.7.55 이 노드는 스트리머의 commanding=1 을 본 뒤 시작하고 스트리머 상태에 따라 일시정지, 스트리머는 재개 시 v_catchup 으로 접근.
"""
from __future__ import annotations

import os
import time
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .choreography import JOINT_NAMES, LIMITS, REST, SHOW_V_MAX, TRANSITION_V_MAX, build_show, check_trajectory, keyframes, transition_time
from .upper_body import MotorMap


class MotionShow(Node):
    def __init__(self):
        super().__init__("motion_show")
        for k, v in {"rate": 100.0, "acts": "heart,wave,run,kiss", "speed": 1.0, "loop": False, "v_max": SHOW_V_MAX, "transition_v_max": TRANSITION_V_MAX,
                     "blend_time": 3.0, "start_delay": 1.0, "start_timeout": 5.0, "config": "", "wait_enabled": True, "wait_streamer": True,
                     "joint_targets_topic": "/teleop/joint_targets", "joint_states_topic": "/joint_states",
                     "motor_state_topic": "/upper_body/motor_states", "system_state_topic": "/upper_body/system_state",
                     "streamer_state_topic": "/upper_body/streamer_state", "streamer_timeout": 1.0, "limit_margin": 0.05}.items():
            self.declare_parameter(k, v)
        p = lambda k: self.get_parameter(k).value  # noqa: E731
        self.rate = float(p("rate")); self.dt = 1.0 / self.rate
        self.acts = [a.strip() for a in str(p("acts")).split(",") if a.strip()]
        self.speed, self.loop, self.v_max = float(p("speed")), bool(p("loop")), float(p("v_max"))
        self.blend_time, self.start_delay, self.start_timeout = float(p("blend_time")), float(p("start_delay")), float(p("start_timeout"))
        self.margin = float(p("limit_margin"))
        self.map = self._load_map(str(p("config")))
        self.motor_state_topic = str(p("motor_state_topic"))
        self.transition_v_max = min(float(p("transition_v_max")), self.v_max)
        self.show = build_show(self.acts, speed=self.speed, v_max=self.transition_v_max)
        _, vmax, lim = check_trajectory(self.show, v_max=self.v_max)
        from . import __version__
        self.get_logger().info(f"motion_show v{__version__} — 쇼 {self.acts} speed {self.speed}: {self.show.duration:.1f} s, 관절 속도 최대 {vmax:.2f} rad/s (v_max {self.v_max}), 한계 여유 {lim:.2f} rad")
        if lim < self.margin:
            self.get_logger().warning(f"안무가 관절 한계 여유 {self.margin} rad 안으로 들어감 ({lim:.2f}) — 그 지점은 한계로 잘린다")
        self.pub = self.create_publisher(JointState, str(p("joint_targets_topic")), 10)
        self.q_meas: Optional[np.ndarray] = None
        self.create_subscription(JointState, str(p("joint_states_topic")), self._on_joint_states, 10)
        self.wait_enabled = bool(p("wait_enabled"))
        self.wait_streamer = bool(p("wait_streamer"))
        self.streamer_timeout = float(p("streamer_timeout"))
        self.meas_from_motor = False
        self.sys_state: Optional[int] = None
        self.t_sys: Optional[float] = None
        self.strm: dict = {}                       # 스트리머 상태 (마지막 메시지)
        self.t_strm: Optional[float] = None
        self.create_subscription(String, str(p("streamer_state_topic")), self._on_streamer_state, 10)
        self.paused_reason = ""
        self.t_pause0 = 0.0
        self.t_pause_log = 0.0
        self.cand_reason: Optional[str] = None     # 일시정지/재개 디바운스 (스트리머 상태는 10 Hz)
        self.t_cand = 0.0
        try:
            from ros2_interfaces.msg import UpperBodySystemState, UpperMotorState
            self.create_subscription(UpperMotorState, str(p("motor_state_topic")), self._on_motor_state,
                                     QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.create_subscription(UpperBodySystemState, str(p("system_state_topic")), self._on_system_state,
                                     QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        except ImportError:
            self.get_logger().warning("ros2_interfaces 없음 — motor_states 는 안 받음 (joint_states 만)")
        self.t_wall0 = time.monotonic()
        self.t_show0: Optional[float] = None       # 쇼 시계 0 (blend 끝)
        self.q_cmd: Optional[np.ndarray] = None
        self.blend = None
        self.last_name = ""
        self.timer = self.create_timer(self.dt, self._tick)
        self.get_logger().info("측정 관절각을 기다림 … (팔 반경을 비우세요)")

    def _load_map(self, path: str) -> Optional[MotorMap]:
        import yaml
        if not path:
            try:
                from ament_index_python.packages import get_package_share_directory
                path = os.path.join(get_package_share_directory("quest_arm_ik"), "config", "sh_humanoid_v2.yaml")
            except Exception:  # noqa: BLE001
                path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", "sh_humanoid_v2.yaml")
        try:
            m = yaml.safe_load(open(path))["upper_body_streamer"]["ros__parameters"]
            return MotorMap(list(m["motor_ids"]), list(m["joint_signs"]), list(m["joint_offsets"]), [int(v) for v in m.get("extra_motor_ids", []) if int(v) > 0])
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f"config 매핑 못 읽음 ({e}) — motor_states 변환 불가, joint_states 만 사용")
            return None

    # ---- 측정 (쇼 시작 자세·블렌드용; 시작(q_cmd 확정) 뒤에는 쓰지 않음 — 스트리머가 추종 오차를 지킨다)
    def _on_joint_states(self, msg: JointState):
        if self.q_cmd is not None:
            return
        d = dict(zip(msg.name, msg.position))
        if all(j in d for j in JOINT_NAMES):
            self.q_meas = np.array([d[j] for j in JOINT_NAMES], dtype=float)

    def _on_motor_state(self, msg):
        if self.q_cmd is not None or self.map is None:
            return
        q = self.map.joints_from_state(msg.motor_id, msg.position)
        if q is not None and len(q) == len(JOINT_NAMES):
            self.q_meas = np.asarray(q, dtype=float)
            self.meas_from_motor = True

    def _on_system_state(self, msg):
        self.sys_state = int(msg.state)
        self.t_sys = time.monotonic()

    def _on_streamer_state(self, msg: String):
        d = {}
        for part in msg.data.split("|"):
            if "=" in part:
                k, v = part.split("=", 1)
                d[k] = v
        self.strm = d
        self.t_strm = time.monotonic()

    def _streamer_ok(self, now: float) -> Optional[str]:
        """스트리머가 지금 쇼 목표를 그대로 따를 수 있는 상태면 None, 아니면 이유(일시정지·시작 대기 사유)."""
        if self.t_strm is None:
            return "스트리머 상태(/upper_body/streamer_state)가 아직 안 옴"
        if now - self.t_strm > self.streamer_timeout:
            return f"스트리머 상태가 {now - self.t_strm:.1f} s 동안 안 옴 (upper_body_streamer 가 살아 있는지)"
        d = self.strm
        if d.get("dup") == "1":
            return "측정 발행자가 둘 (시뮬 에뮬레이션 + 실기체 릴레이)"
        if d.get("commanding") != "1":
            return f"스트리머가 명령을 내지 않음 (driver {d.get('driver', '?')})"
        reason = d.get("reason", "")
        if reason.startswith("tracking error"):
            return f"스트리머 추종 오차 정지 ({reason})"
        if reason in ("hold topic", "external"):
            return "hold 토픽 ON"
        if d.get("catchup") == "1":
            return f"스트리머가 목표까지 접근 중 (남은 {d.get('dist', '?')} rad)"
        return None

    def _hr_path(self) -> bool:
        """스트리머·드라이버 경로인가: motor_states 를 받았거나, 스트리머 상태가 왔거나, motor_states 발행자가 있다 (joint_states 가 먼저 와도 속지 않게)."""
        if self.meas_from_motor or self.t_strm is not None:
            return True
        try:
            return self.count_publishers(self.motor_state_topic) > 0
        except Exception:  # noqa: BLE001
            return False

    def _enabled(self) -> bool:
        """쇼를 시작해도 되는가. 실기체 경로(_hr_path): 스트리머가 commanding=1 을 알리고(wait_streamer) 드라이버가 ENABLED/RUNNING
        (wait_enabled) 이어야 한다 — 둘 다 올 때까지 기다리며, 시간이 지나도 혼자 시작하지 않는다(2026-10-28 사고 ②).
        joint_states 만 오는 순수 시뮬(hr 아님)은 바로 시작."""
        now = time.monotonic()
        if not self._hr_path():
            return True
        if self.wait_streamer:
            why = self._streamer_ok(now)
            if why is not None:
                if self.t_strm is None and now - self.t_wall0 > self.start_timeout:
                    self.get_logger().warning("upper_body_streamer 의 /upper_body/streamer_state 가 안 옴 — v0.7.55 이상 스트리머가 떠 있는지 확인 (그 전엔 시작하지 않음)",
                                              throttle_duration_sec=10.0)
                else:
                    self.get_logger().info(f"시작 대기: {why}", throttle_duration_sec=5.0)
                return False
        if self.wait_enabled:
            if self.t_sys is None:
                if now - self.t_wall0 < self.start_timeout:
                    return False
                if not self.wait_streamer:
                    self.get_logger().warning("system_state 가 안 옴 — ENABLED 확인 없이 시작 (hr_udp_relay / 에뮬레이션이 system_state 를 내는지 확인)", throttle_duration_sec=30.0)
                    return True
                return False
            if self.sys_state not in (4, 5):
                self.get_logger().info("드라이버가 ENABLED 가 아님 — enable 을 기다림 (ros2 service call /upper_body/enable_motors …)", throttle_duration_sec=5.0)
                return False
        return True

    # ---- 한 틱
    def _tick(self):
        now = time.monotonic()
        if self.q_cmd is None:
            if self.q_meas is None:
                if now - self.t_wall0 < self.start_timeout:
                    return
                self.get_logger().warning("측정 관절각이 안 옴 — REST 에서 시작한다고 가정 (시뮬이 아니면 중단할 것)")
                self.q_meas = REST.copy()
            if not self._enabled():
                return
            q_start = self.show.q(0.0)
            T = max(self.blend_time, transition_time(self.q_meas, q_start, self.transition_v_max))
            self.blend = keyframes([(0.0, self.q_meas), (T, q_start)])
            self.t_blend0 = now + self.start_delay
            self.t_show0 = self.t_blend0 + T
            self.q_cmd = self.q_meas.copy()
            self.get_logger().info(f"시작: {self.start_delay:.0f} s 뒤 {T:.1f} s 동안 시작 자세로 → 쇼 {self.show.duration:.0f} s" + (" (반복)" if self.loop else ""))
        # 일시정지: 스트리머가 쇼 목표를 따를 수 없는 동안(disable·발행자 2개·hold·추종 오차·접근 중) 쇼 시계를 멈추고 마지막 목표를 유지한다.
        # 스트리머 상태는 10 Hz 라 0.15 s 디바운스 (시작 직후의 순간적 catchup=1 등).
        if self.wait_streamer and self._hr_path():
            why = self._streamer_ok(now)
            if (why or "") != (self.cand_reason or ""):
                self.cand_reason, self.t_cand = why, now
            if now - self.t_cand >= 0.15:
                if why and not self.paused_reason:
                    self.paused_reason, self.t_pause0, self.t_pause_log = why, now, now
                    self.get_logger().warning(f"쇼 일시정지: {why} — 마지막 목표 유지, 스트리머가 다시 추종하면 이어서 진행")
                elif not why and self.paused_reason:
                    dt_p = now - self.t_pause0
                    self.t_blend0 += dt_p
                    self.t_show0 += dt_p
                    self.get_logger().info(f"쇼 재개 ({dt_p:.1f} s 멈춤)")
                    self.paused_reason = ""
            if self.paused_reason:
                if now - self.t_pause_log >= 10.0:
                    self.get_logger().warning(f"쇼 일시정지 중 ({now - self.t_pause0:.0f} s): {why or self.paused_reason}")
                    self.t_pause_log = now
                self._publish(self.q_cmd)
                return
        if now < self.t_blend0:
            q_des = self.q_meas
        elif now < self.t_show0:
            q_des = self.blend(now - self.t_blend0)
        else:
            t = now - self.t_show0
            if self.loop and self.show.duration > 0:
                t = t % self.show.duration
            q_des = self.show.q(t)
            name = self.show.name_at(t)
            if name != self.last_name:
                self.get_logger().info(f"[{t:5.1f} s] {name}")
                self.last_name = name
            if not self.loop and t > self.show.duration + 1.0 and self.last_name != "done":
                self.get_logger().info("쇼 끝 — 마지막 자세 유지 중 (Ctrl-C 로 종료)")
                self.last_name = "done"
        # 안전: 관절 한계·틱당 속도
        q_des = np.clip(q_des, LIMITS[:, 0] + self.margin, LIMITS[:, 1] - self.margin)
        step = self.v_max * self.dt
        self.q_cmd = self.q_cmd + np.clip(q_des - self.q_cmd, -step, step)
        self._publish(self.q_cmd)

    def _publish(self, q: np.ndarray):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(JOINT_NAMES)
        js.position = [float(v) for v in q]
        self.pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = MotionShow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
