"""IK 관절 목표(100 Hz) → 삼현 HRController UpperBodyCommand 스트림(기본 250 Hz, 속도 피드포워드·속도 제한·워치독).

  입력  /teleop/joint_targets        sensor_msgs/JointState (dual_arm_ik: position + effort=중력토크)
        /upper_body/motor_state      ros2_interfaces/UpperMotorState (실기체 또는 mujoco_sim 의 HR 에뮬레이션)
        /upper_body/system_state     ros2_interfaces/UpperBodySystemState (ENABLED/RUNNING 아니면 명령 안 냄)
        /teleop/hold                 std_msgs/Bool (True = 그 자리 정지)
  출력  /upper_body/command          ros2_interfaces/UpperBodyCommand
        /upper_body/streamer_state   std_msgs/String 10 Hz — "state=TRACK|reason=|driver=ENABLED|commanding=1|catchup=0|err=0.012|dist=0.004"
                                     (motion_show 가 이것을 보고 시작·일시정지한다: commanding=0 / catchup=1 / tracking error 면 쇼 시계를 멈춤)
  첫 명령은 측정 모터각에서 시작하고, 매 틱 v_max·dt 이상 움직이지 않는다. IK 가 끊기거나 명령-측정 차이가 크면 정지한다.
  정지(미활성·HOLD)에서 추종을 다시 시작할 때는 목표까지 v_catchup(0.3 rad/s)으로 천천히 간다 — 2026-10-28 enable 순간 멀리 있는 쇼 목표로 v_max 2.5 로 튄 사고.
"""
from __future__ import annotations

import time
from typing import Dict, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from .upper_body import (MODE_IMPEDANCE, MODE_POSITION, MODE_POSITION_VELOCITY, STATE_NAMES, MotorMap, Streamer,
                         StreamerParams, fill_command)


class UpperBodyStreamer(Node):
    def __init__(self):
        super().__init__("upper_body_streamer")
        from ros2_interfaces.msg import UpperBodyCommand, UpperBodySystemState, UpperMotorState   # SH-Humanoid integration
        self.UpperBodyCommand = UpperBodyCommand
        defaults = {
            "controlled_joints": [""], "motor_ids": list(range(1, 15)), "joint_signs": [1.0] * 14, "joint_offsets": [0.0] * 14,
            "extra_motor_ids": [0],               # 우리가 안 쓰는 모터에 '현재 위치 유지' 명령을 같이 보낼 때만 (0 = 없음). 실기체에서 미연결 축(머리 90)에 보내면
                                                  # HRController 가 명령마다 command_commit_failed 를 내므로 보내지 않는다 (2026-10-21)
            "empty_slot_id": 255,                 # 빈 슬롯의 motor_id: 삼현 브리지는 255 를 '모터 없음' 으로 조용히 건너뜀 (0 이면 메시지마다 경고)
            "rate": 50.0,                         # 50 Hz: 2026-10-20 실측 (200 Hz 는 미니PC 브리지가 밀려 '튐'; config 가 덮어씀)
            "v_max": 0.5, "a_max": 8.0, "ik_timeout": 0.2, "max_tracking_error": 0.35,
            "v_catchup": 0.3,                     # 정지에서 추종 재개 시 목표까지 접근 속도 상한 [rad/s] (0 = v_max). 멀리 있는 목표에 튀지 않게
            "catchup_done": 0.02, "hold_recover_time": 1.0, "max_tracking_retries": 3,   # upper_body.StreamerParams 참고
            "command_mode": 4,                    # 1 POSITION | 4 POSITION_VELOCITY | 5 IMPEDANCE
            "position_only": True,                # True: kp/kd/effort 0 (드라이버 POSITION 모드 내부 게인으로 추종). velocity 피드포워드는 velocity_feedforward 로 별도
            "velocity_feedforward": True,         # 팀 trajectory_bridge 처럼 velocity 에 보간 속도를 실음 (position_only 와 무관)
            "kp": 20.0, "kd": 0.5,                # position_only=false 일 때만 (드라이버 단위)
            "extra_kp": 10.0, "extra_kd": 0.3,
            "effort_feedforward": False,          # IMPEDANCE 에서 IK 중력토크(effort)를 피드포워드로 (단위·부호 확인 전엔 끔)
            "duration": -1.0,                     # UpperBodyCommand.duration; -1 = 1/rate (형식만 팀 브리지와 동일 — 드라이버는 무시, 실측), 0 = 즉시
            "require_enabled": True,              # system_state 가 ENABLED/RUNNING 일 때만 발행
            "command_reliability": "reliable",    # 명령 발행 QoS: reliable(팀 브리지와 동일) | best_effort (유실 복구 대기 없이 최신 명령 우선 — 드라이버 구독이 BEST_EFFORT 일 때만 짝이 맞음)
            "publish_when_idle": True,            # IK 가 없을 때도 '현재 위치 유지' 명령을 계속 냄 (드라이버 워치독 유지)
            "joint_targets_topic": "/teleop/joint_targets", "motor_state_topic": "/upper_body/motor_states",
            "system_state_topic": "/upper_body/system_state", "command_topic": "/upper_body/command", "hold_topic": "/teleop/hold",
            "state_topic": "/upper_body/streamer_state",
            "gap_warn": 0.1,                      # motor_states / IK 목표 / 자기 틱이 이보다 오래 끊기면 WARN + 보고 줄에 횟수 (v0.7.57 '뚝뚝 끊김' 추적)
            "use_sim_time": False,
        }
        for k, v in defaults.items():
            if k != "use_sim_time":
                self.declare_parameter(k, v)
        p = lambda k: self.get_parameter(k).value  # noqa: E731
        self.joints = [j for j in p("controlled_joints") if j]
        if not self.joints:
            raise ValueError("controlled_joints 파라미터 필요 (config 의 dual_arm_ik.controlled_joints 와 같은 순서)")
        n = len(self.joints)
        self.map = MotorMap([int(v) for v in p("motor_ids")][:n], [float(v) for v in p("joint_signs")][:n],
                            [float(v) for v in p("joint_offsets")][:n], [int(v) for v in p("extra_motor_ids") if int(v) > 0])
        self.sp = StreamerParams(rate=float(p("rate")), v_max=float(p("v_max")), a_max=float(p("a_max")),
                                 ik_timeout=float(p("ik_timeout")), max_tracking_error=float(p("max_tracking_error")),
                                 v_catchup=float(p("v_catchup")), catchup_done=float(p("catchup_done")),
                                 hold_recover_time=float(p("hold_recover_time")), max_tracking_retries=int(p("max_tracking_retries")))
        self.streamer = Streamer(n, self.sp)
        self.mode = int(p("command_mode"))
        self.position_only = bool(p("position_only"))
        self.kp, self.kd = (0.0, 0.0) if self.position_only else (float(p("kp")), float(p("kd")))
        self.extra_kp, self.extra_kd = (0.0, 0.0) if self.position_only else (float(p("extra_kp")), float(p("extra_kd")))
        self.effort_ff = bool(p("effort_feedforward"))
        self.velocity_ff = bool(p("velocity_feedforward"))
        self.duration = float(p("duration"))
        if self.duration < 0:
            self.duration = 1.0 / float(p("rate"))
        self.require_enabled = bool(p("require_enabled"))
        self.publish_when_idle = bool(p("publish_when_idle"))
        self.effort: Optional[np.ndarray] = None
        self.extra_pos: Dict[int, float] = {}
        self.slot_order = None
        self.sys_state: Optional[int] = None
        self.motor_state_topic, self.system_state_topic = str(p("motor_state_topic")), str(p("system_state_topic"))
        self.dup_sources = False
        self.n_ik = self.n_cmd = self.n_state = 0
        self.last_state = None
        self.last_reason = None
        self.gap_warn = float(p("gap_warn"))
        self.t_last_state = self.t_last_ik = self.t_last_tick = None
        self.gaps = {"motor_states": [], "ik": [], "tick": []}
        self.commanding = False                   # 이번 틱에 명령을 낼 조건(발행자 하나·드라이버 활성·측정 있음)이었는가 → streamer_state 로 알림
        self.was_catching = False
        self.last_catch_dist = 0.0

        fast = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.empty_id = int(p("empty_slot_id"))
        rel = str(p("command_reliability")).lower()
        if rel not in ("reliable", "best_effort"):
            raise ValueError(f"command_reliability 는 reliable|best_effort: {rel!r}")
        self.pub = self.create_publisher(UpperBodyCommand, p("command_topic"),
                                         QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT if rel == "best_effort" else ReliabilityPolicy.RELIABLE))
        self.create_subscription(JointState, p("joint_targets_topic"), self._on_targets, 10)
        self.create_subscription(UpperMotorState, p("motor_state_topic"), self._on_motor_state, fast)   # best-effort 구독은 reliable 발행자와도 맞는다
        self.create_subscription(UpperBodySystemState, p("system_state_topic"), self._on_system_state, fast)   # RELIABLE 구독은 브리지 publish() 를 막는다 (2026-10-21)
        self.create_subscription(Bool, p("hold_topic"), self._on_hold, 10)
        self.state_pub = self.create_publisher(String, p("state_topic"), 10)
        self.create_timer(self.streamer.dt, self._tick)
        self.create_timer(0.1, self._publish_state)
        self.create_timer(1.0, self._report)
        self.add_on_set_parameters_callback(self._on_params)      # 실행 중 튜닝: ros2 param set /upper_body_streamer kp 30.0
        from . import __version__
        self.get_logger().info(f"upper_body_streamer v{__version__}: {n} joints → motors {list(self.map.motor_ids)} (+extra {list(self.map.extra_motor_ids)}), "
                               f"{self.sp.rate:.0f} Hz ({rel}), mode {self.mode}, {'kp/kd/effort 0' if self.position_only else f'kp/kd {self.kp}/{self.kd}'}, "
                               f"velocity FF {'on' if self.velocity_ff else 'off'}, duration {self.duration:.4f} s, v_max {self.sp.v_max} rad/s, "
                               f"v_catchup {self.sp.v_catchup} rad/s (재개 시 접근 속도)")

    # ---- callbacks
    def _on_params(self, params):
        """kp/kd/extra_kp/extra_kd/v_max/a_max/max_tracking_error/ik_timeout/command_mode 는 재시작 없이 바뀐다 (다음 틱부터)."""
        from rcl_interfaces.msg import SetParametersResult
        for prm in params:
            name, val = prm.name, prm.value
            if name in ("kp", "kd", "extra_kp", "extra_kd"):
                setattr(self, name, float(val))
            elif name in ("v_max", "a_max", "max_tracking_error", "ik_timeout", "v_catchup", "catchup_done", "hold_recover_time"):
                if name == "v_max" and not (0.0 < float(val) <= 3.0):
                    return SetParametersResult(successful=False, reason="v_max 는 0 < v ≤ 3 rad/s")
                setattr(self.sp, name, float(val))
            elif name == "max_tracking_retries":
                self.sp.max_tracking_retries = int(val)
            elif name == "command_mode":
                if int(val) not in (MODE_POSITION, MODE_POSITION_VELOCITY, MODE_IMPEDANCE):
                    return SetParametersResult(successful=False, reason="command_mode 는 1 | 4 | 5")
                self.mode = int(val)
            elif name == "effort_feedforward":
                self.effort_ff = bool(val)
            elif name == "velocity_feedforward":
                self.velocity_ff = bool(val)
            elif name == "duration":
                self.duration = float(val) if float(val) >= 0 else self.streamer.dt
            elif name == "position_only":
                self.position_only = bool(val)
                if self.position_only:
                    self.kp = self.kd = self.extra_kp = self.extra_kd = 0.0
            else:
                continue
            self.get_logger().info(f"param {name} = {val}")
        return SetParametersResult(successful=True)

    def _gap(self, key: str, t_prev, now: float, what: str):
        """스트림이 gap_warn 보다 오래 끊겼으면 기록·경고 (어느 쪽이 멈췄는지 창 4 로그에서 바로 보이게)."""
        if t_prev is not None and now - t_prev > self.gap_warn:
            dt = now - t_prev
            self.gaps[key].append(dt)
            self.get_logger().warning(f"{what} 끊김 {dt * 1000:.0f} ms (추종 오차 {self.streamer.tracking_error():.3f} rad)", throttle_duration_sec=2.0)

    def _on_targets(self, msg: JointState):
        now = time.monotonic()
        self._gap("ik", self.t_last_ik, now, "IK 목표(/teleop/joint_targets)")
        self.t_last_ik = now
        name2pos = dict(zip(msg.name, msg.position))
        if not all(j in name2pos for j in self.joints):
            return
        self.streamer.set_target([name2pos[j] for j in self.joints], time.monotonic())
        if self.effort_ff and len(msg.effort) == len(msg.name):
            name2eff = dict(zip(msg.name, msg.effort))
            self.effort = np.array([name2eff[j] for j in self.joints], dtype=float)
        self.n_ik += 1

    def _on_motor_state(self, msg):
        now = time.monotonic()
        self._gap("motor_states", self.t_last_state, now, "motor_states(미니PC 상태)")
        self.t_last_state = now
        q = self.map.joints_from_state(msg.motor_id, msg.position)
        if q is None:
            if self.n_state == 0:
                self.get_logger().warning(f"motor_state 에 motor_ids {list(self.map.motor_ids)} 가 다 없음: 수신 ID {list(msg.motor_id)}")
            self.n_state += 1
            return
        self.streamer.set_measured(q)
        ids = list(msg.motor_id)
        self.slot_order = [int(m) for m in ids]                  # 드라이버 슬롯 순서 유지 (명령도 같은 슬롯에)
        for mid in self.map.extra_motor_ids:
            if mid in ids:
                self.extra_pos[mid] = float(msg.position[ids.index(mid)])
        self.n_state += 1

    def _on_system_state(self, msg):
        self.sys_state = int(msg.state)

    def _on_hold(self, msg: Bool):
        self.streamer.set_hold(bool(msg.data), "hold topic")
        self.get_logger().info("hold 토픽 " + ("ON — 그 자리 정지" if msg.data else "OFF — 목표까지 천천히 다시 접근"))

    def _check_sources(self) -> bool:
        """측정·상태 토픽의 발행자가 둘 이상(시뮬 HR 에뮬레이션 + 실기체 릴레이가 같이 뜬 경우)이면 명령을 내지 않는다 — 2026-10-28 실기체가 튄 원인."""
        try:
            n_m = self.count_publishers(self.motor_state_topic); n_s = self.count_publishers(self.system_state_topic)
        except Exception:  # noqa: BLE001  (가짜 rclpy 등)
            return True
        dup = n_m > 1 or n_s > 1
        if dup != self.dup_sources:
            self.dup_sources = dup
            if dup:
                self.get_logger().error(f"motor_states 발행자 {n_m}개 / system_state 발행자 {n_s}개 — 시뮬(hr_emulation)과 실기체 릴레이가 함께 떠 있음. "
                                        "명령을 내지 않는다: launch 에 sim:=false 를 주거나 단독 hr_udp_relay / mujoco_sim 중 하나를 끌 것")
            else:
                self.get_logger().info("측정 발행자가 하나로 돌아옴 — 명령 재개 (측정값에서 다시 시작)")
        return not dup

    # ---- loop
    def _tick(self):
        now = time.monotonic()
        if self.t_last_tick is not None and now - self.t_last_tick > max(self.gap_warn, 3.0 * self.streamer.dt):
            self.gaps["tick"].append(now - self.t_last_tick)
            self.get_logger().warning(f"스트리머 틱 지연 {(now - self.t_last_tick) * 1000:.0f} ms (Thor CPU/스케줄링 — 그동안 명령도 안 나감)", throttle_duration_sec=2.0)
        self.t_last_tick = now
        self.commanding = False
        if not self._check_sources():
            self.streamer.reset_to_measured()
            return
        if self.require_enabled and self.sys_state not in (4, 5):
            self.streamer.reset_to_measured()                    # ENABLED/RUNNING 아님: 명령 안 냄, q_cmd 는 측정값에 고정 (enable 순간 점프 방지)
            return
        dist_before = self.streamer.distance_to_target()
        q = self.streamer.tick(time.monotonic())
        if q is None:
            return                                               # 측정값 대기
        self.commanding = True
        st = self.streamer
        if st.catching_up != self.was_catching:                  # 접근(천천히) 시작·끝 로그 — 멀리 있는 목표일 때만
            if st.catching_up and st.state == "TRACK" and dist_before > 0.05:
                self.get_logger().info(f"목표가 {dist_before:.2f} rad 떨어져 있음 — v_catchup {min(self.sp.v_max, self.sp.v_catchup):.2f} rad/s 로 접근 (약 {dist_before / max(1e-3, min(self.sp.v_max, self.sp.v_catchup)):.1f} s)")
            elif not st.catching_up and self.was_catching and self.last_catch_dist > 0.05:
                self.get_logger().info("목표에 닿음 — v_max 로 추종")
            self.was_catching = st.catching_up
            self.last_catch_dist = dist_before
        elif st.catching_up:
            self.last_catch_dist = max(self.last_catch_dist, dist_before)
        if self.streamer.state == "HOLD" and not self.publish_when_idle and self.streamer.hold_reason in ("no ik target", "ik timeout"):
            return
        cmd = self.UpperBodyCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "upper_body_link"
        cmd.command_mode = self.mode
        eff = self.effort if (self.effort_ff and self.mode == MODE_IMPEDANCE and self.effort is not None and not self.position_only) else None
        qd = self.streamer.qd_cmd if (self.velocity_ff or self.mode in (MODE_POSITION_VELOCITY, MODE_IMPEDANCE)) else np.zeros(self.streamer.n)
        arr = fill_command(self.map, q, qd, self.kp, self.kd, self.extra_pos, self.extra_kp, self.extra_kd, eff, self.slot_order, self.empty_id)
        for k, vals in arr.items():
            getattr(cmd, k)[:] = vals
        cmd.duration = float(self.duration)
        self.pub.publish(cmd)
        self.n_cmd += 1

    def _publish_state(self):
        """motion_show 등 외부가 보는 스트리머 상태 (10 Hz). commanding=1 이면 이번 틱에 명령을 낼 수 있는 조건이었다는 뜻(목표가 없으면 HOLD 로 유지 명령)."""
        st = self.streamer
        m = String()
        m.data = (f"state={st.state}|reason={st.hold_reason}|driver={STATE_NAMES.get(self.sys_state, str(self.sys_state))}|"
                  f"commanding={int(self.commanding)}|catchup={int(st.catching_up and st.state == 'TRACK')}|"
                  f"err={st.tracking_error():.3f}|dist={st.distance_to_target():.3f}|dup={int(self.dup_sources)}")
        self.state_pub.publish(m)

    def _report(self):
        st = self.streamer
        sysn = STATE_NAMES.get(self.sys_state, str(self.sys_state))
        if st.state != self.last_state or st.hold_reason != self.last_reason:
            self.get_logger().info(f"state {st.state}" + (f" ({st.hold_reason})" if st.hold_reason else "") + f", driver {sysn}")
            self.last_state, self.last_reason = st.state, st.hold_reason
        if self.n_cmd == 0 and st.q_cmd is not None and self.require_enabled and self.sys_state not in (4, 5):
            self.get_logger().warning(f"driver state {sysn} — ENABLED/RUNNING 이 아니라 명령을 내지 않음", throttle_duration_sec=5.0)
        gaps = ", ".join(f"{k} {len(v)}회(최대 {max(v) * 1000:.0f} ms)" for k, v in self.gaps.items() if v)
        self.get_logger().info(f"ik {self.n_ik} Hz, motor_state {self.n_state} Hz, cmd {self.n_cmd} Hz, tracking err {st.tracking_error():.3f} rad"
                               + (f" — 끊김: {gaps}" if gaps else ""), throttle_duration_sec=5.0)
        if gaps:
            self.gaps = {k: [] for k in self.gaps}
        if self.n_state == 0:
            self.get_logger().warning("motor_states 가 안 옴 — hr_udp_relay 가 살아 있는지 (launch 의 relay 가 'Address already in use' 로 죽었으면 "
                                      "단독 relay 를 끄거나 launch 에 relay:=false), 미니PC hr_controller/링크 확인", throttle_duration_sec=5.0)
        self.n_ik = self.n_cmd = self.n_state = 0


def main(args=None):
    rclpy.init(args=args)
    node = UpperBodyStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
