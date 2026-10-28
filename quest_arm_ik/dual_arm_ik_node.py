"""양팔 VR 원격제어 IK 노드.

입력 (quest_bridge 또는 카메라 파이프라인이 발행):
  /teleop/{left,right}_hand/target_raw : geometry_msgs/PoseStamped (로봇 베이스 축)
  /teleop/{left,right}_hand/engaged    : std_msgs/Bool (클러치)
  /joint_states (또는 /upper_body/motor_state)   : 시작 시 동기화 + 주기적 재동기화
출력:
  output_mode=forward_position  → Float64MultiArray (JointGroupPositionController)  [output_split: 팔별 7축]
  output_mode=trajectory        → JointTrajectory 1점 (JointTrajectoryController / mujoco_sim)  [output_split 가능]
  output_mode=upper_body        → ros2_interfaces/UpperBodyCommand (SH-Humanoid HRController, 모터 ID 매핑 필요)
  output_mode=joint_state       → JointState 만
  joint_state_out_topic (기본 /quest_ik/joint_states, 팀 약속은 /teleop/joint_targets):
      position=명령각, effort=중력보상 토크(publish_gravity_ff)
  /quest_ik/{side}_hand/target (디버그·RViz·Foxglove)

매핑은 mapper.ArmMapper (클러치 상대/절대 매핑 + One-Euro + max_step), IK는 ik_core.DualArmIKCore (pink + 자기충돌).
타임스탬프에 의존하지 않고 수신 시각으로만 stale 판정 → sim time 문제 없음.
"""
from __future__ import annotations

import math
import os
import time
from typing import Dict, List

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .frames import quat_xyzw_from_rotation, rot_z, se3_from_pq
from .ik_core import SIDES, DualArmIKCore, IKConfig
from .mapper import ArmMapper, MapperParams

# 기본값은 G1 29-DoF 이름 (config 로 덮어씀; SH-Humanoid 는 config/sh_humanoid_v2.yaml)
DEFAULT_JOINTS = [f"{s}_{n}_joint" for s in ("left", "right")
                  for n in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
                            "wrist_roll", "wrist_pitch", "wrist_yaw")]
DEFAULT_POSTURE = [0.3, 0.3, 0.0, 0.9, 0.0, 0.0, 0.0,
                   0.3, -0.3, 0.0, 0.9, 0.0, 0.0, 0.0]


class DualArmIKNode(Node):
    def __init__(self):
        super().__init__("dual_arm_ik")
        self._declare_params()
        p = self._p

        import yaml
        shapes = yaml.safe_load(p("collision_shapes_yaml") or "") or []
        model_path = p("model_path") or p("urdf_path")      # urdf_path 는 구버전 호환
        pw = [float(v) for v in p("posture_weights")]
        posture_weights = None if (len(pw) == 1 and pw[0] == 0.0) else pw   # [0.0] = 미지정 (ROS 파라미터는 빈 배열 불가)
        self.core = DualArmIKCore(IKConfig(
            model_path=model_path, controlled_joints=list(p("controlled_joints")),
            left_hand_frame=p("left_hand_frame"), right_hand_frame=p("right_hand_frame"),
            base_frame=str(p("base_frame")),
            package_dirs=[d for d in p("package_dirs") if d], srdf_path=p("srdf_path"), xacro_args=str(p("xacro_args")),
            nominal_posture=list(p("nominal_posture")),
            posture_weights=posture_weights, limit_margin=float(p("limit_margin")),
            joint_limits=(yaml.safe_load(p("joint_limits_yaml")) or None) if p("joint_limits_yaml") else None,
            position_cost=p("position_cost"), orientation_cost=p("orientation_cost"),
            task_gain=p("task_gain"), lm_damping=p("lm_damping"), posture_cost=p("posture_cost"),
            damping=p("damping"), max_joint_velocity=p("max_joint_velocity"),
            enable_self_collision=p("enable_self_collision"),
            collision_d_min=p("collision_d_min"), collision_gain=p("collision_gain"),
            elbow_hint_cost=float(p("elbow_hint_cost")),
            collision_activation=p("collision_activation"),
            collision_convex_hull=p("collision_convex_hull"),
            collision_shapes=shapes, collision_exclude_links=[l for l in p("collision_exclude_links") if l],
            solver=p("solver")))
        self.joint_names: List[str] = list(p("controlled_joints"))
        n = len(self.joint_names)
        self.half = n // 2
        self.dt = 1.0 / float(p("control_rate"))
        self.output_mode = str(p("output_mode"))
        self.output_split = bool(p("output_split"))
        self.base_frame = str(p("base_frame"))
        self.publish_gravity = bool(p("publish_gravity_ff"))

        # 위치 스케일: 조작자 팔 길이(어깨→손바닥, operator_arm_length)가 주어지면 로봇 팔 길이(모델에서 계산)와의 비로 자동 결정.
        # 사람·로봇의 팔 비율이 맞아야 팔을 높이 들었을 때 로봇도 같은 정도로 펴지고(과하면 팔이 먼저 다 펴져 손목이 꺾이고, 모자라면 팔꿈치가 굽은 채 남는다).
        self.robot_arm = self.core.arm_length("left")
        scale = float(p("translation_scale"))
        op_arm = float(p("operator_arm_length"))
        if op_arm > 0.0:
            if self.robot_arm:
                scale = self.robot_arm / op_arm
                self.get_logger().info(f"translation_scale = 로봇 팔 {self.robot_arm:.3f} m / 조작자 팔 {op_arm:.3f} m = {scale:.3f}")
            else:
                self.get_logger().warning("operator_arm_length 가 주어졌지만 모델에서 shoulder_pitch/elbow 관절을 못 찾아 translation_scale 을 그대로 씀")
        elif self.robot_arm:
            self.get_logger().info(f"translation_scale {scale:.2f} (로봇 팔 {self.robot_arm:.3f} m → 조작자 어깨→손바닥 {self.robot_arm / scale:.2f} m 에 해당; "
                                   f"operator_arm_length:= 로 자동 계산 가능)")
        mp = MapperParams(mode=str(p("mapping_mode")), max_step=float(p("max_step")),
                          scale=scale,
                          R_cal=rot_z(math.radians(float(p("quest_yaw_offset_deg")))),
                          input_timeout=float(p("input_timeout")), stop_on_release=bool(p("stop_on_release")),
                          filter_min_cutoff=float(p("filter_min_cutoff")), filter_beta=float(p("filter_beta")),
                          orientation_filter_alpha=float(p("orientation_filter_alpha")),
                          orientation_clutch=str(p("orientation_clutch")),
                          forearm_filter_alpha=float(p("forearm_filter_alpha")))
        self.mappers = {side: ArmMapper(side, mp) for side in SIDES}
        self.elbow_hint = float(p("elbow_hint_cost")) > 0.0
        self._hint_prev = {side: False for side in SIDES}
        if self.elbow_hint:
            self.get_logger().info(f"팔꿈치 힌트 켬 (elbow_hint_cost {float(p('elbow_hint_cost')):.2f}): 전완 길이 L {self.core.forearm_length('left'):.3f} / {self.core.forearm_length('right'):.3f} m — "
                                   "그립 순간 로봇 전완 방향을 기준으로 컨트롤러 회전 변화만큼 돌린 방향을 팔꿈치 목표로")
        # 회전 클러치 trigger: 검지 트리거(quest_bridge 가 /teleop/{side}_hand/grasp 로 0~1 발행)를 당긴 동안만 손 회전을 따라가고,
        # 그 외에는 회전 cost 0 + 손목 잠금 (손 방향은 전완을 따라감). 시작은 잠금 상태.
        self.orient_clutch = str(p("orientation_clutch"))
        self.orient_thr = float(p("orient_trigger_threshold"))
        if self.orient_clutch == "trigger":
            for side in SIDES:
                self.core.set_orientation_tracking(side, False)
                self.core.set_wrist_locked(side, True)
        self.add_on_set_parameters_callback(self._on_params)      # ros2 param set /dual_arm_ik translation_scale 0.7 | operator_arm_length 0.58
        self.target_pub = {}
        for side in SIDES:
            self.target_pub[side] = self.create_publisher(PoseStamped, f"/quest_ik/{side}_hand/target", 10)
            self.create_subscription(PoseStamped, f"/teleop/{side}_hand/target_raw",
                                     lambda m, s=side: self._on_raw(s, m), 10)
            self.create_subscription(Bool, f"/teleop/{side}_hand/engaged",
                                     lambda m, s=side: self.mappers[s].set_engaged(m.data), 10)
            if self.orient_clutch == "trigger":
                self.create_subscription(Float32, p("orient_trigger_topic").replace("{side}", side),
                                         lambda m, s=side: self.mappers[s].set_orient_engaged(m.data > self.orient_thr), 10)

        # ---- 출력 ----
        self.js_pub = self.create_publisher(JointState, p("joint_state_out_topic"), 10)
        self.traj_pub: Dict[str, object] = {}
        self.cmd_pub: Dict[str, object] = {}
        self.ub_pub = None
        if self.output_mode == "trajectory":
            if self.output_split:
                self.traj_pub = {"left": self.create_publisher(JointTrajectory, p("left_trajectory_topic"), 10),
                                 "right": self.create_publisher(JointTrajectory, p("right_trajectory_topic"), 10)}
            else:
                self.traj_pub = {"all": self.create_publisher(JointTrajectory, p("trajectory_topic"), 10)}
        elif self.output_mode == "forward_position":
            if self.output_split:
                self.cmd_pub = {"left": self.create_publisher(Float64MultiArray, p("left_forward_position_topic"), 10),
                                "right": self.create_publisher(Float64MultiArray, p("right_forward_position_topic"), 10)}
            else:
                self.cmd_pub = {"all": self.create_publisher(Float64MultiArray, p("forward_position_topic"), 10)}
        elif self.output_mode == "upper_body":
            from ros2_interfaces.msg import UpperBodyCommand  # SH-Humanoid integration/ros2_interfaces
            self.UpperBodyCommand = UpperBodyCommand
            self.ub_pub = self.create_publisher(UpperBodyCommand, p("upper_body_command_topic"), 10)
            self.motor_ids = [int(v) for v in p("motor_ids")]
            self.joint_signs = np.array([float(v) for v in p("joint_signs")], dtype=float)
            self.joint_offsets = np.array([float(v) for v in p("joint_offsets")], dtype=float)
            if len(self.motor_ids) != n or len(self.joint_signs) != n or len(self.joint_offsets) != n:
                raise ValueError("motor_ids / joint_signs / joint_offsets 길이는 controlled_joints 와 같아야 함")
            self.ub_mode = int(p("upper_body_command_mode"))
            self.ub_kp, self.ub_kd = float(p("upper_body_kp")), float(p("upper_body_kd"))
            self.get_logger().warning("output_mode=upper_body: motor_ids/joint_signs/joint_offsets 가 실제 모터 배선과 맞는지 "
                                      "반드시 dummy/저게인으로 먼저 확인하세요")
        elif self.output_mode != "joint_state":
            raise ValueError("output_mode: trajectory | forward_position | upper_body | joint_state")

        # ---- 관절 상태 소스 ----
        self.ready = not bool(p("init_from_joint_states"))
        self.q_meas = None
        self.js_frame = str(p("joint_states_frame_id"))
        self._js_ignored = 0
        self.resync_thr = float(p("resync_threshold"))
        self.resync_ticks = int(p("resync_ticks"))
        self._resync_cnt = 0
        src = str(p("joint_states_source"))
        if src == "joint_states":
            self.create_subscription(JointState, p("joint_states_topic"), self._on_joint_states, 10)
        elif src == "upper_motor_state":
            from ros2_interfaces.msg import UpperMotorState
            self.motor_ids_in = [int(v) for v in p("motor_ids")]
            self.joint_signs = np.array([float(v) for v in p("joint_signs")], dtype=float)
            self.joint_offsets = np.array([float(v) for v in p("joint_offsets")], dtype=float)
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            self.create_subscription(UpperMotorState, p("upper_motor_state_topic"), self._on_motor_state,
                                     QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))   # 드라이버가 best-effort 로 내도 받게
        else:
            raise ValueError("joint_states_source: joint_states | upper_motor_state")
        if not self.ready:
            self.get_logger().info("waiting for joint state to initialize...")

        self._last_fail = 0
        self._engaged_prev = {side: False for side in SIDES}
        self.freeze_until_engaged = bool(p("freeze_until_engaged"))
        self._ever_engaged = False
        self.resync_hold_count = int(p("resync_hold_count"))
        self.resync_hold_window = float(p("resync_hold_window"))
        self._resync_times = []
        self.hold_pub = self.create_publisher(Bool, p("hold_topic"), 10) if self.resync_hold_count > 0 else None
        self.create_timer(self.dt, self._step)
        ncp = len(self.core.collision_model.collisionPairs) if self.core.collision_model is not None else 0
        from .urdf_tools import hand_variant
        self.get_logger().info(f"dual_arm_ik ready: model={model_path} (xacro_args='{p('xacro_args')}'), "
                               f"hand={hand_variant(list(self.core.model.names) + [self.core.model.frames[i].name for i in range(self.core.model.nframes)])}, "
                               f"solver={self.core.solver}, rate={1/self.dt:.0f} Hz, output={self.output_mode}"
                               f"{'(split)' if self.output_split else ''}, mapping={mp.mode}, collision pairs={ncp}")

    # ------------------------------------------------------------ params
    def _declare_params(self):
        defaults = {
            "model_path": "", "urdf_path": "", "package_dirs": [""], "srdf_path": "", "xacro_args": "",
            "base_frame": "pelvis", "forward_axis": [1.0, 0.0, 0.0],
            "controlled_joints": DEFAULT_JOINTS,
            "left_hand_frame": "left_wrist_yaw_link", "right_hand_frame": "right_wrist_yaw_link",
            "nominal_posture": DEFAULT_POSTURE, "posture_weights": [0.0], "limit_margin": 0.0,
            "joint_limits_yaml": "",            # "{left_elbow_joint: [-1.8, 0.0], ...}" 모델보다 좁은 IK 관절 한계
            "control_rate": 100.0, "solver": "",
            "position_cost": 1.0, "orientation_cost": 0.3, "task_gain": 0.5,
            "lm_damping": 1.0e-3, "posture_cost": 0.01, "damping": 1.0e-6,
            "max_joint_velocity": 2.0,
            "mapping_mode": "relative", "max_step": 0.0,
            "translation_scale": 1.0, "operator_arm_length": 0.0, "quest_yaw_offset_deg": 0.0,
            "filter_min_cutoff": 1.5, "filter_beta": 0.5, "orientation_filter_alpha": 0.3,
            "input_timeout": 0.5, "stop_on_release": True,
            "elbow_hint_cost": 0.0,              # >0: 팔꿈치 힌트 (손 목표 − 전완 길이 × 조작자 전완 방향 을 팔꿈치 위치 과제로, 손 위치 1.0 대비 0.1~0.3). 0 = 끔
            "forearm_filter_alpha": 0.3,         # 팔꿈치 힌트 방향 저역통과 (1 = 없음)
            "orientation_clutch": "always",      # always: 그립 중 늘 컨트롤러 회전 추종 | trigger: 검지 트리거를 당긴 동안만 (그 외 손목 잠금·위치만 추종)
            "orient_trigger_topic": "/teleop/{side}_hand/grasp",   # trigger 모드의 입력 (quest_bridge 가 트리거 값 0~1 로 발행; {side} 치환)
            "orient_trigger_threshold": 0.5,
            "freeze_until_engaged": True,          # 첫 그립 전엔 IK 를 돌리지 않음 (시작 자세 그대로; false 면 바로 nominal_posture 쪽으로 영공간 정렬이 일어남)
            "enable_self_collision": False, "collision_d_min": 0.02, "collision_gain": 0.3,
            "collision_activation": 0.08, "collision_convex_hull": True,
            "collision_shapes_yaml": "", "collision_exclude_links": [""],
            "init_from_joint_states": True, "joint_states_source": "joint_states",
            "joint_states_frame_id": "",        # 비우면 모두 수신. 지정하면 header.frame_id 가 같은 메시지만 (다른 시뮬 섞임 방지)
            "joint_states_topic": "/joint_states", "upper_motor_state_topic": "/upper_body/motor_states",
            "resync_threshold": 0.3, "resync_ticks": 40,
            "resync_hold_count": 3, "resync_hold_window": 10.0,   # 이 시간 안에 재동기화가 이 횟수 이상 → /teleop/hold true 발행(스트리머 정지). 0 = 끔
            "hold_topic": "/teleop/hold",
            "output_mode": "trajectory", "output_split": False,
            "trajectory_topic": "/arm_controller/joint_trajectory",
            "left_trajectory_topic": "/left_arm_controller/joint_trajectory",
            "right_trajectory_topic": "/right_arm_controller/joint_trajectory",
            "forward_position_topic": "/forward_position_controller/commands",
            "left_forward_position_topic": "/left_arm_forward_controller/commands",
            "right_forward_position_topic": "/right_arm_forward_controller/commands",
            "upper_body_command_topic": "/upper_body/command",
            "upper_body_command_mode": 1, "upper_body_kp": 30.0, "upper_body_kd": 1.0,
            "motor_ids": list(range(1, 15)), "joint_signs": [1.0] * 14, "joint_offsets": [0.0] * 14,
            "joint_state_out_topic": "/quest_ik/joint_states", "publish_gravity_ff": False,
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        if not (self.get_parameter("model_path").value or self.get_parameter("urdf_path").value):
            raise ValueError("model_path 파라미터가 비어 있음 (.urdf / .xacro / MJCF .xml)")

    def _p(self, name):
        return self.get_parameter(name).value

    # ------------------------------------------------------------ callbacks
    def _on_raw(self, side: str, msg: PoseStamped):
        p, q = msg.pose.position, msg.pose.orientation
        self.mappers[side].set_raw(se3_from_pq((p.x, p.y, p.z), (q.x, q.y, q.z, q.w)), time.monotonic())

    def _accept_measured(self, q: np.ndarray):
        self.q_meas = q
        if not self.ready:
            self.core.reset_from_joint_positions(q)
            self.ready = True
            self.get_logger().info("initialized from measured joint positions")

    def _on_joint_states(self, msg: JointState):
        if self.js_frame and msg.header.frame_id != self.js_frame:
            self._js_ignored += 1
            if self._js_ignored in (1, 100, 1000):
                self.get_logger().warning(f"/joint_states frame_id='{msg.header.frame_id}' != '{self.js_frame}' → 무시 "
                                          f"({self._js_ignored}회). 다른 launch(시뮬)가 떠 있지 않은지 `ros2 node list` 확인")
            return
        name2pos = dict(zip(msg.name, msg.position))
        if all(j in name2pos for j in self.joint_names):
            self._accept_measured(np.array([name2pos[j] for j in self.joint_names], dtype=float))

    def _on_motor_state(self, msg):
        pos = np.array(msg.position, dtype=float)
        ids = list(msg.motor_id)
        try:
            idx = [ids.index(m) for m in self.motor_ids_in]
        except ValueError:
            self.get_logger().warning("motor_ids 중 일부가 motor_state 에 없음", throttle_duration_sec=5.0)
            return
        q = (pos[idx] - self.joint_offsets) * self.joint_signs     # 모터각 → URDF 관절각
        self._accept_measured(q)

    # ------------------------------------------------------------ params (실행 중 변경)
    def _on_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for prm in params:
            if prm.name == "translation_scale":
                v = float(prm.value)
                if not (0.1 <= v <= 3.0):
                    return SetParametersResult(successful=False, reason="translation_scale 는 0.1~3.0")
            elif prm.name == "operator_arm_length":
                v = float(prm.value)
                if v <= 0.0 or not self.robot_arm:
                    return SetParametersResult(successful=False, reason="operator_arm_length > 0 이고 모델 팔 길이를 알아야 함")
                v = self.robot_arm / v
            else:
                continue
            for m in self.mappers.values():
                m.p.scale = v
            self.get_logger().info(f"translation_scale → {v:.3f} (다음 그립부터가 아니라 즉시 반영; 쥔 채면 손 위치가 조금 옮겨짐)")
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------ control loop
    def _step(self):
        if not self.ready:
            return
        now = time.monotonic()
        for side, mapper in self.mappers.items():
            target = mapper.update(now, self.core.hand_pose(side), self.core.forearm_dir(side) if self.elbow_hint else None)
            if mapper.engaged != self._engaged_prev[side]:
                self._engaged_prev[side] = mapper.engaged
                self.get_logger().info(f"{side} hand {'engaged' if mapper.engaged else 'released'}")
            if self.orient_clutch == "trigger" and mapper.orient_changed:
                self.core.set_orientation_tracking(side, mapper.orient_engaged)
                self.core.set_wrist_locked(side, not mapper.orient_engaged)
                self.get_logger().info(f"{side} wrist {'tracking (trigger)' if mapper.orient_engaged else 'locked'}")
            if target is not None:
                self.core.set_hand_target(side, target)
                self._publish_target(side, target)
                if self.elbow_hint:
                    if mapper.forearm_dir is not None:
                        self.core.set_elbow_hint(side, target.translation, mapper.forearm_dir)
                    else:
                        self.core.clear_elbow_hint(side)
                    if self.core.elbow_hint_active[side] != self._hint_prev[side]:
                        self._hint_prev[side] = self.core.elbow_hint_active[side]
                        self.get_logger().info(f"{side} elbow hint {'on' if self._hint_prev[side] else 'off'}")
            elif mapper.released:
                self.core.hold_hand(side)
                if self.elbow_hint:
                    self.core.clear_elbow_hint(side)

        # 첫 그립 전에는 IK 를 돌리지 않고 현재(측정/시작) 관절각을 그대로 낸다 — 영공간 자세 정렬(nominal_posture 로 끌림) 때문에
        # 입력 없이 팔이 조금 움직이는 것을 막는다. 실기체에서는 enable 직후 '저절로' 움직이지 않게 하는 안전장치이기도 하다.
        if self.freeze_until_engaged and not self._ever_engaged:
            if any(m.engaged for m in self.mappers.values()):
                self._ever_engaged = True
                self.get_logger().info("첫 그립 — IK 시작 (이전까지는 시작 자세 유지)")
            else:
                if self.q_meas is not None:
                    self.core.reset_from_joint_positions(self.q_meas)     # 내부 상태를 실제 팔에 계속 맞춰 둠 (첫 그립에서 점프 없음)
                self._publish(self.core.joint_positions())
                return

        # 실제 관절각과 내부 상태가 오래 벌어지면 재동기화 (실기체에서 추종 실패·정지 시)
        if self.q_meas is not None and self.resync_thr > 0:
            if np.max(np.abs(self.core.joint_positions() - self.q_meas)) > self.resync_thr:
                self._resync_cnt += 1
                if self._resync_cnt >= self.resync_ticks:
                    self.core.reset_from_joint_positions(self.q_meas)
                    for m in self.mappers.values():
                        m.set_engaged(False)
                    self._resync_cnt = 0
                    self.get_logger().warning("내부 자세와 측정 관절각 차이 큼 → 실제 각도로 재동기화 (클러치 다시 잡으세요)")
                    self._resync_times = [t for t in self._resync_times if now - t < self.resync_hold_window] + [now]
                    if self.hold_pub is not None and len(self._resync_times) >= self.resync_hold_count:
                        self.hold_pub.publish(Bool(data=True))
                        self._resync_times = []
                        self.get_logger().error(
                            f"{self.resync_hold_window:.0f} s 안에 재동기화 {self.resync_hold_count}회 — 로봇이 IK 를 따라오지 않음 "
                            "(모터 매핑 부호/ID 오류, 모터 불능, 충돌). 스트리머 정지(/teleop/hold true). 원인 확인 후 "
                            "`ros2 topic pub -1 /teleop/hold std_msgs/msg/Bool \"{data: false}\"` 로 해제")
            else:
                self._resync_cnt = 0

        positions = self.core.step(self.dt)
        if self.core.qp_failures != self._last_fail:
            self._last_fail = self.core.qp_failures
            self.get_logger().warning(f"QP infeasible ({self.core.qp_failures}x total)", throttle_duration_sec=1.0)
        self._publish(positions)

    def _publish_target(self, side: str, T):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, T.translation)
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = quat_xyzw_from_rotation(T.rotation)
        self.target_pub[side].publish(msg)

    def _publish(self, positions: np.ndarray):
        pos = [float(v) for v in positions]
        stamp = self.get_clock().now().to_msg()
        js = JointState()
        js.header.stamp = stamp
        js.name = self.joint_names
        js.position = pos
        if self.publish_gravity:
            js.effort = [float(v) for v in self.core.gravity_torques()]
        self.js_pub.publish(js)

        parts = {"all": (self.joint_names, pos)}
        if self.output_split:
            parts = {"left": (self.joint_names[:self.half], pos[:self.half]),
                     "right": (self.joint_names[self.half:], pos[self.half:])}
        for key, pub in self.traj_pub.items():
            names, vals = parts[key]
            traj = JointTrajectory()               # header.stamp=0 → "지금부터" (sim time 무관)
            traj.joint_names = names
            pt = JointTrajectoryPoint()
            pt.positions = vals
            pt.time_from_start = Duration(seconds=2.0 * self.dt).to_msg()
            traj.points = [pt]
            pub.publish(traj)
        for key, pub in self.cmd_pub.items():
            pub.publish(Float64MultiArray(data=parts[key][1]))
        if self.ub_pub is not None:
            cmd = self.UpperBodyCommand()
            cmd.header.stamp = stamp
            cmd.command_mode = self.ub_mode
            motor = np.asarray(pos) * self.joint_signs + self.joint_offsets    # URDF 관절각 → 모터각
            n = len(self.motor_ids)
            for i in range(n):
                cmd.motor_id[i] = self.motor_ids[i]
                cmd.position[i] = float(motor[i])
                cmd.kp[i] = self.ub_kp
                cmd.kd[i] = self.ub_kd
            cmd.duration = float(2.0 * self.dt)
            self.ub_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = DualArmIKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
