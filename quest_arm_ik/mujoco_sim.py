"""가벼운 MuJoCo 상반신 시뮬레이터 (ros2_control 없이).

- 모델: MJCF(.xml). 전신 모델(G1 등)이면 베이스(free joint)를 공중에 고정하고
  controlled_joints 외 관절은 키프레임 자세로 잡아 둔다 → 상반신 텔레옵만 검증.
- 입력: controlled_joints 순서의 관절 위치 목표 (position 액추에이터 ctrl, 없으면 PD 토크)
- 출력: 관절 위치/속도, 임의 body의 베이스 기준 자세

MujocoArmSim 은 ROS 비의존(테스트·Thor 관제 SW 재사용), main() 이 ROS 2 노드로 감싼다.
"""
from __future__ import annotations

import os
import threading
import time
from typing import Dict, Optional, Sequence

import mujoco
import numpy as np

from .urdf_tools import prepare_urdf


MJ_MAX_STL_FACES = 200000      # MuJoCo stl_decoder 한계 (초과 메시는 "decoder failed")


def _stl_faces(path: str) -> int:
    """바이너리 STL 삼각형 수 (못 읽으면 -1)."""
    try:
        import struct
        with open(path, "rb") as f:
            head = f.read(84)
        if len(head) < 84 or head[:5] == b"solid" and os.path.getsize(path) != 84 + 50 * struct.unpack("<I", head[80:84])[0]:
            return -1
        return struct.unpack("<I", head[80:84])[0]
    except OSError:
        return -1


def _urdf_with_mujoco_compiler(path: str) -> str:
    """URDF 에 MuJoCo 확장 <mujoco><compiler discardvisual="false" fusestatic="false" .../></mujoco> 를 넣은 사본 경로.
    discardvisual 은 파싱 단계 옵션이라 MjSpec 로드 뒤에 바꿔도 소용없다. 이미 <mujoco> 블록이 있으면 그대로.
    MuJoCo 가 못 읽는 비주얼 메시(STL 20만 면 초과, 예: 팀 L_hand.stl 21만 면)는 그 링크의 충돌형상을 비주얼로 대신 쓴다."""
    import copy
    import xml.etree.ElementTree as ET
    text = open(path, encoding="utf-8").read()
    if "<mujoco" in text:
        return path
    root = ET.fromstring(text)
    comp = ET.SubElement(root, "mujoco")
    ET.SubElement(comp, "compiler", {"discardvisual": "false", "fusestatic": "false", "balanceinertia": "true"})
    root.remove(comp)
    root.insert(0, comp)
    for link in root.findall("link"):
        dropped = False
        for vis in list(link.findall("visual")):
            mesh = vis.find("geometry/mesh")
            fn = mesh.get("filename", "") if mesh is not None else ""
            if fn.lower().endswith(".stl") and os.path.isfile(fn) and _stl_faces(fn) > MJ_MAX_STL_FACES:
                link.remove(vis)
                dropped = True
        if dropped and not link.findall("visual"):
            for col in link.findall("collision"):
                vis = ET.SubElement(link, "visual")
                for tag in ("origin", "geometry"):
                    el = col.find(tag)
                    if el is not None:
                        vis.append(copy.deepcopy(el))
    out = os.path.splitext(path)[0] + ".mj.urdf"
    open(out, "w", encoding="utf-8").write(ET.tostring(root, encoding="unicode"))
    return out


class MujocoArmSim:
    def __init__(self, model_path: str, controlled_joints: Sequence[str],
                 keyframe: str = "", fix_base: bool = True, base_height: float = -1.0,
                 pd_kp: float = 200.0, pd_kd: float = 5.0, package_dirs: Sequence[str] = (),
                 collision_shapes: Sequence[dict] = (), srdf_path: str = "", exclude_initial_contacts: bool = True,
                 gravity_comp: bool = True, xacro_args: str = ""):
        self.model = self._load(model_path, pd_kp, pd_kd, package_dirs, collision_shapes, srdf_path,
                                keyframe, exclude_initial_contacts, xacro_args)
        self.gravity_comp = gravity_comp      # 실기체 모터 루프처럼 중력(+코리올리) 보상을 feed-forward 로 가해 처짐 제거
        self.data = mujoco.MjData(self.model)
        self.controlled_joints = list(controlled_joints)
        self.fix_base = fix_base
        self.pd_kp, self.pd_kd = pd_kp, pd_kd

        m = self.model
        # free joint (전신 모델) 탐색
        self.free_jnt = next((j for j in range(m.njnt) if int(m.jnt_type[j]) == int(mujoco.mjtJoint.mjJNT_FREE)), -1)

        # 키프레임으로 초기화
        key = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, keyframe) if keyframe else -1
        if key >= 0:
            mujoco.mj_resetDataKeyframe(m, self.data, key)
        else:
            mujoco.mj_resetData(m, self.data)
        self.q_hold = self.data.qpos.copy()
        if self.free_jnt >= 0 and fix_base:
            adr = m.jnt_qposadr[self.free_jnt]
            if base_height >= 0.0:
                self.q_hold[adr + 2] = base_height
            self.base_qpos = self.q_hold[adr:adr + 7].copy()
            self.base_qposadr, self.base_dofadr = adr, m.jnt_dofadr[self.free_jnt]

        # 관절별 qpos/dof 주소, 액추에이터 매핑
        self.jnt_id: Dict[str, int] = {}
        self.qadr: Dict[str, int] = {}
        self.dadr: Dict[str, int] = {}
        self.act_id: Dict[str, int] = {}
        for j in range(m.njnt):
            if int(m.jnt_type[j]) in (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)):
                name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)
                self.jnt_id[name], self.qadr[name], self.dadr[name] = j, m.jnt_qposadr[j], m.jnt_dofadr[j]
        for a in range(m.nu):
            if int(m.actuator_trntype[a]) == int(mujoco.mjtTrn.mjTRN_JOINT):
                jname = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, m.actuator_trnid[a][0])
                is_position = (int(m.actuator_biastype[a]) == int(mujoco.mjtBias.mjBIAS_AFFINE)
                               and m.actuator_gainprm[a][0] > 0 and m.actuator_biasprm[a][1] < 0)
                if is_position and jname not in self.act_id:
                    self.act_id[jname] = a
        missing = [j for j in self.controlled_joints if j not in self.jnt_id]
        if missing:
            raise ValueError(f"모델에 없는 관절: {missing}\n사용 가능: {list(self.jnt_id)}")
        self.hold_joints = [j for j in self.jnt_id if j not in self.controlled_joints]
        self.no_actuator = [j for j in self.controlled_joints + self.hold_joints if j not in self.act_id]

        self.q_des = np.array([self.q_hold[self.qadr[j]] for j in self.controlled_joints], dtype=float)
        self.extra_targets: Dict[str, float] = {}   # 제어 대상이 아닌 관절(손가락 등)의 목표 — set_extra_targets()
        self.dt = float(m.opt.timestep)
        self.lock = threading.Lock()       # 다중 스레드 사용 시: step()과 set_joint_targets()를 감쌀 것
        self.set_hold_targets()

    @staticmethod
    def _load(model_path: str, kp: float, kv: float, package_dirs: Sequence[str],
              collision_shapes: Sequence[dict] = (), srdf_path: str = "", keyframe: str = "",
              exclude_initial_contacts: bool = True, xacro_args: str = "") -> "mujoco.MjModel":
        """MJCF/URDF/xacro 를 MjSpec 으로 읽어 (1) URDF 의 fixed 링크 병합·비주얼 삭제를 끄고,
        (2) position 액추에이터가 없는 관절에 kp/kv 액추에이터를 달고, (3) 적분기를 implicitfast 로 올린 뒤 컴파일.
        collision_shapes 로 지정한 링크(예: 20만 면짜리 스탠드 STL)는 기본도형으로 교체되어 로드된다."""
        path = model_path
        is_urdf = os.path.splitext(path)[1].lower() in (".urdf", ".xacro")
        if is_urdf:
            path = prepare_urdf(path, package_dirs, collision_shapes, resolve_packages=True, tag="sim", xacro_args=xacro_args)
            path = _urdf_with_mujoco_compiler(path)   # <visual> 유지 (URDF 기본은 파싱 때 버림 → 뷰어에 충돌형상만 보였음)
        spec = mujoco.MjSpec.from_file(path)
        if is_urdf:
            spec.compiler.fusestatic = False     # 손·몸통 같은 fixed 링크를 body 로 유지 (body_pose_in_base 용)
            spec.compiler.discardvisual = False
            # 뷰어: 비주얼(그룹 1, 접촉 없음)만 보이게 충돌형상은 그룹 3 으로 (뷰어 기본은 그룹 0~2 표시). 물리에는 영향 없음.
            if any(g.contype == 0 and g.conaffinity == 0 for b in spec.bodies for g in b.geoms):
                for b in spec.bodies:
                    for g in b.geoms:
                        if g.contype != 0 or g.conaffinity != 0:
                            g.group = 3
        has_pos_act = set()
        for a in spec.actuators:
            if a.trntype == mujoco.mjtTrn.mjTRN_JOINT and a.biastype == mujoco.mjtBias.mjBIAS_AFFINE \
                    and a.gainprm[0] > 0 and a.biasprm[1] < 0:
                has_pos_act.add(a.target)
        for j in spec.joints:
            if j.type in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE) and j.name not in has_pos_act:
                a = spec.add_actuator()
                a.name, a.target, a.trntype = f"{j.name}_pos", j.name, mujoco.mjtTrn.mjTRN_JOINT
                a.gaintype, a.biastype = mujoco.mjtGain.mjGAIN_FIXED, mujoco.mjtBias.mjBIAS_AFFINE
                a.gainprm[0], a.biasprm[1], a.biasprm[2] = kp, -kp, -kv      # MuJoCo <position kp kv> 와 동일
        if spec.option.integrator == mujoco.mjtIntegrator.mjINT_EULER:
            spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST   # 액추에이터 감쇠를 음적분 → 고강성에서도 안정

        # 접촉 제외: (1) SRDF disable_collisions, (2) 초기 자세에서 이미 겹치는 body 쌍 (어깨 바 안의 어깨 링크 등)
        excluded = set()

        def exclude(b1: str, b2: str):
            key = tuple(sorted((b1, b2)))
            if key in excluded:
                return
            excluded.add(key)
            ex = spec.add_exclude()
            ex.name, ex.bodyname1, ex.bodyname2 = f"ex_{b1}_{b2}", b1, b2

        body_names = {b.name for b in spec.bodies}
        if srdf_path and os.path.isfile(srdf_path):
            import xml.etree.ElementTree as ET
            for el in ET.parse(srdf_path).getroot().findall("disable_collisions"):
                l1, l2 = el.get("link1"), el.get("link2")
                if l1 in body_names and l2 in body_names:
                    exclude(l1, l2)
        model = spec.compile()
        if exclude_initial_contacts:
            data = mujoco.MjData(model)
            key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, keyframe) if keyframe else -1
            if key >= 0:
                mujoco.mj_resetDataKeyframe(model, data, key)
            mujoco.mj_forward(model, data)
            pairs = set()
            for i in range(data.ncon):
                c = data.contact[i]
                b1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[c.geom1])
                b2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[c.geom2])
                if b1 != b2:
                    pairs.add(tuple(sorted((b1, b2))))
            if pairs:
                for b1, b2 in pairs:
                    exclude(b1, b2)
                model = spec.compile()
        MujocoArmSim.excluded_pairs = sorted(excluded)
        return model

    # ------------------------------------------------------------ commands
    def set_hold_targets(self) -> None:
        for j in self.hold_joints:
            if j in self.act_id:
                self.data.ctrl[self.act_id[j]] = self.q_hold[self.qadr[j]]

    def set_joint_targets(self, positions: Sequence[float]) -> None:
        self.q_des[:] = np.asarray(positions, dtype=float)

    def set_extra_targets(self, names: Sequence[str], positions: Sequence[float]) -> int:
        """제어 대상이 아닌(held) 관절의 목표를 바꾼다 (예: /teleop/hand_joint_targets 의 손가락 관절). 모델에 없는 이름은 무시. 반환: 반영 개수."""
        n = 0
        for j, q in zip(names, positions):
            if j in self.hold_joints:
                lo, hi = self.model.jnt_range[self.jnt_id[j]]
                self.extra_targets[j] = float(min(max(q, lo), hi)) if lo < hi else float(q)
                n += 1
        return n

    def _apply_control(self) -> None:
        m, d = self.model, self.data
        if self.gravity_comp:
            d.qfrc_applied[:] = d.qfrc_bias       # 직전 forward 의 bias (중력+코리올리) 를 상쇄
        q_des = self.q_des
        for j, qd in zip(self.controlled_joints, q_des):
            if j in self.act_id:
                d.ctrl[self.act_id[j]] = qd
        for j, qd in self.extra_targets.items():
            if j in self.act_id:
                d.ctrl[self.act_id[j]] = qd
        if self.no_actuator:
            targets = dict(zip(self.controlled_joints, q_des))
            targets.update(self.extra_targets)
            for j in self.no_actuator:
                qd = targets.get(j, self.q_hold[self.qadr[j]])
                dof = self.dadr[j]
                d.qfrc_applied[dof] = self.pd_kp * (qd - d.qpos[self.qadr[j]]) - self.pd_kd * d.qvel[dof]
        if self.free_jnt >= 0 and self.fix_base:
            d.qpos[self.base_qposadr:self.base_qposadr + 7] = self.base_qpos
            d.qvel[self.base_dofadr:self.base_dofadr + 6] = 0.0

    def step(self, n: int = 1) -> None:
        for _ in range(n):
            self._apply_control()
            mujoco.mj_step(self.model, self.data)

    # ------------------------------------------------------------ state
    def joint_positions(self) -> np.ndarray:
        return np.array([self.data.qpos[self.qadr[j]] for j in self.controlled_joints])

    def joint_velocities(self) -> np.ndarray:
        return np.array([self.data.qvel[self.dadr[j]] for j in self.controlled_joints])

    def fk_body_in_base(self, q: Sequence[float], body: str, base: str):
        """동역학 없이 관절값 q(controlled_joints 순서)에서의 body 자세 (검증용). 시뮬 상태는 복원."""
        m, d = self.model, self.data
        qpos0, qvel0 = d.qpos.copy(), d.qvel.copy()
        for j, v in zip(self.controlled_joints, q):
            d.qpos[self.qadr[j]] = float(v)
        mujoco.mj_forward(m, d)
        out = self.body_pose_in_base(body, base)
        d.qpos[:], d.qvel[:] = qpos0, qvel0
        mujoco.mj_forward(m, d)
        return out

    def body_pose_in_base(self, body: str, base: str):
        """body 자세를 base body 기준으로 (위치 (3,), 회전행렬 (3,3))."""
        m, d = self.model, self.data
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body)
        rid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, base)
        if bid < 0 or rid < 0:
            raise ValueError(f"body not found: {body if bid < 0 else base}")
        Rb, pb = d.xmat[rid].reshape(3, 3), d.xpos[rid]
        return Rb.T @ (d.xpos[bid] - pb), Rb.T @ d.xmat[bid].reshape(3, 3)


# ---------------------------------------------------------------- ROS 2 node
def run_realtime(sim: "MujocoArmSim", viewer=None, realtime: float = 1.0, keep_running=lambda: True, warn=None,
                 max_catchup_s: float = 0.25, sync_hz: float = 30.0) -> None:
    """벽시계를 따라가는 물리 루프. 뷰어 sync 는 물리 스텝과 분리한다 — 소프트웨어 렌더링(llvmpipe)처럼 sync 가 수십 ms
    걸려도 물리는 밀린 만큼 묶음으로 따라잡아(catch-up) 시뮬이 실시간보다 느려지지 않게 한다 (느려지면 IK 와 측정 관절각이
    벌어져 재동기화·드리프트가 생긴다). 상한(max_catchup_s)을 넘게 밀리면 그만큼의 시간은 버리고 경고한다."""
    rate = max(realtime, 1e-3)
    max_catchup = max(1, int(max_catchup_s / sim.dt))
    sync_period = 1.0 / sync_hz
    t0 = time.monotonic()
    steps = 0
    last_sync = t0
    last_warn = 0.0
    while keep_running() and (viewer is None or viewer.is_running()):
        now = time.monotonic()
        due = int((now - t0) * rate / sim.dt)          # 지금까지 돌았어야 할 스텝 수
        behind = due - steps
        if behind > max_catchup:
            steps = due - max_catchup                  # 시간을 버리고 상한만큼만 따라잡음
            behind = max_catchup
            if warn is not None and now - last_warn > 5.0:
                warn("물리 루프가 실시간보다 느림 — 뷰어 렌더링이 느리면 viewer:=false 로 띄우고 Foxglove 로 보는 것을 권장")
                last_warn = now
        if behind > 0:
            with sim.lock:
                sim.step(behind)
            steps += behind
        if viewer is not None and now - last_sync >= sync_period:
            viewer.sync()                              # 느려도 물리 시간은 위에서 따라잡음
            last_sync = time.monotonic()
        else:
            delay = t0 + (steps + 1) * sim.dt / rate - time.monotonic()
            if delay > 0:
                time.sleep(min(delay, 0.002))


def make_node_class():
    """rclpy 가 있을 때만 노드 클래스를 만든다 (MujocoArmSim 자체는 ROS 없이도 import 가능하게 유지)."""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray
    from trajectory_msgs.msg import JointTrajectory

    class MujocoSimNode(Node):
        def __init__(self):
            super().__init__("mujoco_sim")
            defaults = {
                "model_path": "", "keyframe": "stand", "fix_base": True, "base_height": -1.0,
                "controlled_joints": [""], "viewer": True, "realtime": 1.0,
                "pd_kp": 200.0, "pd_kd": 5.0, "package_dirs": [""], "collision_shapes_yaml": "",
                "srdf_path": "", "xacro_args": "", "exclude_initial_contacts": True, "gravity_comp": True,
                "hand_joint_targets_topic": "/teleop/hand_joint_targets",   # 손가락 등 비제어 관절 목표 (JointState, 이름 매칭)
                "publish_all_joints": True,       # /joint_states 에 비제어 관절(손가락)도 포함
                # 삼현 HRController 에뮬레이션: /upper_body/command 를 받고 /upper_body/motor_state·system_state 를 낸다 (실기체 경로 리허설용)
                "hr_emulation": False, "hr_motor_ids": list(range(1, 15)), "hr_joint_signs": [1.0] * 14, "hr_joint_offsets": [0.0] * 14,
                "hr_extra_motor_ids": [15], "hr_state_rate": 250.0,
                "joint_states_topic": "/joint_states", "publish_rate": 100.0,
                "trajectory_topic": "/arm_controller/joint_trajectory",
                "forward_position_topic": "/forward_position_controller/commands",
                "left_trajectory_topic": "/left_arm_controller/joint_trajectory",
                "right_trajectory_topic": "/right_arm_controller/joint_trajectory",
                "left_forward_position_topic": "/left_arm_forward_controller/commands",
                "right_forward_position_topic": "/right_arm_forward_controller/commands",
                # 뷰어 카메라: 로봇 '뒤'에서 정면 방향으로 보게 해서 화면의 앞/오른쪽 = 로봇의 앞/오른쪽 (조작자도 같은 방향을 보고 선다)
                "forward_axis": [1.0, 0.0, 0.0],  # 베이스 프레임의 정면 (sh: [0,1,0])
                "viewer_view": "behind",          # behind | front | default
                "viewer_distance": 2.6,           # m
                "viewer_elevation": -15.0,        # deg (음수 = 위에서 내려다봄)
                "viewer_lookat_height": 0.0,      # m, 관절 중심 높이에 더함
                # 시작 자세: controlled_joints 와 같은 길이로 주면 키프레임 대신 이 각도에서 시작 (준비 자세·ready.sh 역할)
                "initial_posture": [0.0],
            }
            for k, v in defaults.items():
                self.declare_parameter(k, v)
            p = lambda k: self.get_parameter(k).value  # noqa: E731
            self.viewer_cfg = {"forward_axis": [float(v) for v in p("forward_axis")], "view": str(p("viewer_view")),
                               "distance": float(p("viewer_distance")), "elevation": float(p("viewer_elevation")),
                               "lookat_height": float(p("viewer_lookat_height"))}
            joints = [j for j in p("controlled_joints") if j]
            if not p("model_path") or not joints:
                raise ValueError("model_path 와 controlled_joints 파라미터가 필요함")
            shapes = []
            if p("collision_shapes_yaml"):
                import yaml
                shapes = yaml.safe_load(p("collision_shapes_yaml")) or []
            self.sim = MujocoArmSim(p("model_path"), joints, keyframe=p("keyframe"),
                                    fix_base=bool(p("fix_base")), base_height=float(p("base_height")),
                                    pd_kp=float(p("pd_kp")), pd_kd=float(p("pd_kd")),
                                    package_dirs=[d for d in p("package_dirs") if d], collision_shapes=shapes,
                                    srdf_path=str(p("srdf_path")), exclude_initial_contacts=bool(p("exclude_initial_contacts")),
                                    gravity_comp=bool(p("gravity_comp")), xacro_args=str(p("xacro_args")))
            self.joints = joints
            init = [float(v) for v in p("initial_posture")]
            if len(init) == len(joints):
                with self.sim.lock:
                    for j, v in zip(joints, init):
                        self.sim.data.qpos[self.sim.qadr[j]] = v
                    self.sim.data.qvel[:] = 0.0
                    self.sim.set_joint_targets(np.array(init))
                    mujoco.mj_forward(self.sim.model, self.sim.data)
                self.get_logger().info("initial_posture 로 시작 (준비 자세)")
            elif len(init) > 1:
                self.get_logger().warning(f"initial_posture 길이 {len(init)} != controlled_joints {len(joints)} → 무시")
            self.viewer_enabled = bool(p("viewer"))
            self.realtime = float(p("realtime"))
            self.js_pub = self.create_publisher(JointState, p("joint_states_topic"), 10)
            # 루트 body 이름(sh: base_link, g1: pelvis)을 frame_id 로 — IK 노드가 다른 시뮬의 joint_states 를 걸러낼 수 있게
            self.root_name = mujoco.mj_id2name(self.sim.model, mujoco.mjtObj.mjOBJ_BODY, 1) or ""
            self.half = len(joints) // 2
            self.create_subscription(JointTrajectory, p("trajectory_topic"), self._on_traj, 10)
            self.create_subscription(JointTrajectory, p("left_trajectory_topic"), self._on_traj, 10)
            self.create_subscription(JointTrajectory, p("right_trajectory_topic"), self._on_traj, 10)
            self.create_subscription(Float64MultiArray, p("forward_position_topic"),
                                     lambda m: self._on_cmd(m, 0, len(joints)), 10)
            self.create_subscription(Float64MultiArray, p("left_forward_position_topic"),
                                     lambda m: self._on_cmd(m, 0, self.half), 10)
            self.create_subscription(Float64MultiArray, p("right_forward_position_topic"),
                                     lambda m: self._on_cmd(m, self.half, len(joints)), 10)
            if p("hand_joint_targets_topic"):
                self.create_subscription(JointState, p("hand_joint_targets_topic"), self._on_extra, 10)
                self._extra_warned = False
            self.pub_joints = list(joints) + (list(self.sim.hold_joints) if bool(p("publish_all_joints")) else [])
            self.create_timer(1.0 / float(p("publish_rate")), self._publish)
            self.hr = None
            if bool(p("hr_emulation")):
                self._setup_hr_emulation(p, joints)
            from .urdf_tools import hand_variant
            m = self.sim.model
            names = [m.body(i).name for i in range(m.nbody)] + [m.joint(i).name for i in range(m.njnt)]
            self.get_logger().info(
                f"mujoco_sim: {len(joints)} controlled joints, {len(self.sim.hold_joints)} held, dt={self.sim.dt}, "
                f"model={p('model_path')} (xacro_args='{p('xacro_args')}'), hand={hand_variant(names)}")

        def _setup_hr_emulation(self, p, joints):
            """가짜 HRController: UpperBodyCommand 수신 → 관절 목표, UpperMotorState/UpperBodySystemState 발행, enable/disable/set_run_mode 서비스."""
            from ros2_interfaces.msg import UpperBodyCommand, UpperBodySystemState, UpperMotorState
            from ros2_interfaces.srv import EnableMotors, SetRunMode, SimpleMotorResult
            from .upper_body import N_SLOTS, MotorMap
            n = len(joints)
            self.hr = {"map": MotorMap([int(v) for v in p("hr_motor_ids")][:n], [float(v) for v in p("hr_joint_signs")][:n],
                                       [float(v) for v in p("hr_joint_offsets")][:n], [int(v) for v in p("hr_extra_motor_ids") if int(v) > 0]),
                       "enabled": False, "run_mode": 1, "n_cmd": 0, "last_cmd": None, "extra": {}}
            self.hr["MotorState"], self.hr["SysState"] = UpperMotorState, UpperBodySystemState
            try:
                n_other = self.count_publishers("/upper_body/motor_states")
            except Exception:  # noqa: BLE001
                n_other = 0
            if n_other > 0:
                self.get_logger().error(f"/upper_body/motor_states 발행자가 이미 {n_other}개 (실기체 hr_udp_relay?) — HR 에뮬레이션을 켜면 측정값이 섞여 실기체가 튄다. "
                                        "에뮬레이션 없이 종료: 실기체면 launch 에 sim:=false, 시뮬 리허설이면 단독 릴레이를 끌 것")
                raise SystemExit(2)
            self.hr["state_pub"] = self.create_publisher(UpperMotorState, "/upper_body/motor_states", 10)   # 실제 hr_state_bridge 와 같은 이름
            self.hr["sys_pub"] = self.create_publisher(UpperBodySystemState, "/upper_body/system_state", 10)
            self.create_subscription(UpperBodyCommand, "/upper_body/command", self._on_hr_command, 10)

            def enable_cb(req, res):
                self.hr["enabled"] = True
                res.success, res.message = True, "emulation: enabled"
                res.enabled_motor_ids = list(self.hr["map"].motor_ids) + list(self.hr["map"].extra_motor_ids)
                self.get_logger().info("HR 에뮬레이션: enable_motors")
                return res

            def disable_cb(req, res):
                self.hr["enabled"] = False
                res.success, res.message = True, "emulation: disabled"
                self.get_logger().info("HR 에뮬레이션: disable_motors → 명령 무시(현재 자세 유지)")
                return res

            def mode_cb(req, res):
                self.hr["run_mode"] = int(req.mode)
                res.success, res.message = True, f"emulation: run_mode {req.mode}"
                res.applied_motor_ids = list(self.hr["map"].motor_ids)
                return res

            self.create_service(EnableMotors, "/upper_body/enable_motors", enable_cb)
            self.create_service(SimpleMotorResult, "/upper_body/disable_motors", disable_cb)
            self.create_service(SetRunMode, "/upper_body/set_run_mode", mode_cb)
            self.create_timer(1.0 / float(p("hr_state_rate")), self._publish_hr_state)
            self.create_timer(0.1, self._publish_hr_sys)
            self.get_logger().info(f"HRController 에뮬레이션 켜짐: motors {list(self.hr['map'].motor_ids)} +{list(self.hr['map'].extra_motor_ids)}, "
                                   f"motor_state {float(p('hr_state_rate')):.0f} Hz. 먼저 `ros2 service call /upper_body/enable_motors` 필요")

        def _on_hr_command(self, msg):
            hr = self.hr
            hr["n_cmd"] += 1
            if not hr["enabled"] or int(msg.command_mode) == 0:
                return
            ids = list(msg.motor_id)
            mp = hr["map"]
            for mid in mp.extra_motor_ids:                           # 우리가 안 쓰는 모터(15번): 명령 위치를 그대로 '측정값'으로 돌려줌
                if mid in ids:
                    hr["extra"][mid] = float(msg.position[ids.index(mid)])
            # 실제 HRController 처럼 모터별로 목표를 갱신한다: 메시지에 없는 모터는 마지막 목표(처음엔 측정값)를 유지 (motor_calib --only-motor 와 같은 부분 명령도 동작)
            if "m_targets" not in hr:
                with self.sim.lock:
                    q_now = np.array([self.sim.data.qpos[self.sim.qadr[j]] for j in self.joints])
                hr["m_targets"] = dict(zip(mp.motor_ids, mp.to_motor(q_now)))
            hit = 0
            for k, mid in enumerate(ids):
                if mid in hr["m_targets"]:
                    hr["m_targets"][mid] = float(msg.position[k]); hit += 1
            if hit == 0:
                if hr["n_cmd"] == 1:
                    self.get_logger().warning(f"HR 에뮬레이션: 명령의 motor_id {ids} 에 hr_motor_ids 가 하나도 없음")
                return
            q = mp.to_joint(np.array([hr["m_targets"][m] for m in mp.motor_ids]))   # 모터각 → 관절각 (매핑이 틀리면 틀린 관절이 움직임 = 실기체와 같은 증상)
            with self.sim.lock:
                self.sim.set_joint_targets(q)
            hr["last_cmd"] = time.monotonic()

        def _publish_hr_state(self):
            hr = self.hr
            msg = hr["MotorState"]()
            msg.header.stamp = self.get_clock().now().to_msg()
            mp = hr["map"]
            with self.sim.lock:
                d = self.sim.data
                q = np.array([d.qpos[self.sim.qadr[j]] for j in self.joints])
                qd = np.array([d.qvel[self.sim.dadr[j]] for j in self.joints])
                tau = np.array([d.qfrc_actuator[self.sim.dadr[j]] for j in self.joints])
            m_pos, m_vel, m_eff = mp.to_motor(q), mp.vel_to_motor(qd), mp.vel_to_motor(tau)
            k = 0
            for i, mid in enumerate(mp.motor_ids):
                msg.motor_id[k] = int(mid); msg.position[k] = float(m_pos[i]); msg.velocity[k] = float(m_vel[i]); msg.effort[k] = float(m_eff[i])
                msg.is_ready[k] = True; msg.is_enabled[k] = hr["enabled"]; msg.temperature[k] = 35.0; msg.voltage[k] = 48.0
                k += 1
            for mid in mp.extra_motor_ids:
                msg.motor_id[k] = int(mid); msg.position[k] = float(hr["extra"].get(mid, 0.0)); msg.is_ready[k] = False; msg.is_enabled[k] = False   # 실기체의 90(미연결)처럼 not-ready
                k += 1
            hr["state_pub"].publish(msg)

        def _publish_hr_sys(self):
            hr = self.hr
            msg = hr["SysState"]()
            msg.header.stamp = self.get_clock().now().to_msg()
            n_all = len(hr["map"].motor_ids) + len(hr["map"].extra_motor_ids)
            msg.state = 5 if (hr["enabled"] and hr["last_cmd"] and time.monotonic() - hr["last_cmd"] < 0.5) else (4 if hr["enabled"] else 2)
            msg.run_mode = hr["run_mode"]
            msg.ready_motor_count = n_all
            msg.enabled_motor_count = n_all if hr["enabled"] else 0
            for i in range(n_all):
                msg.is_connected[i] = True
            msg.all_motors_enabled = hr["enabled"]
            msg.command_accepted = hr["enabled"]
            hr["sys_pub"].publish(msg)

        def _on_extra(self, msg: JointState):
            with self.sim.lock:
                n = self.sim.set_extra_targets(list(msg.name), list(msg.position))
            if n == 0 and not self._extra_warned:
                self.get_logger().warning(f"hand_joint_targets: 모델에 없는 관절뿐 ({list(msg.name)[:3]} …) — 손가락 관절이 없는 모델이면 정상")
                self._extra_warned = True

        def _on_traj(self, msg: JointTrajectory):
            """전체(14) 또는 팔별(7) JointTrajectory — joint_names 로 매칭하므로 어느 쪽이든 됨."""
            if not msg.points:
                return
            name2pos = dict(zip(msg.joint_names, msg.points[-1].positions))
            with self.sim.lock:
                q = self.sim.q_des.copy()
                hit = 0
                for i, j in enumerate(self.joints):
                    if j in name2pos:
                        q[i] = name2pos[j]; hit += 1
                if hit:
                    self.sim.set_joint_targets(q)
            if hit == 0:
                self.get_logger().warning("trajectory joint_names match none of controlled_joints",
                                          throttle_duration_sec=5.0)

        def _on_cmd(self, msg: Float64MultiArray, start: int, end: int):
            if len(msg.data) != end - start:
                self.get_logger().warning(f"command length {len(msg.data)} != {end - start}", throttle_duration_sec=5.0)
                return
            with self.sim.lock:
                q = self.sim.q_des.copy()
                q[start:end] = list(msg.data)
                self.sim.set_joint_targets(q)

        def _publish(self):
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.header.frame_id = self.root_name
            js.name = self.pub_joints
            with self.sim.lock:
                d = self.sim.data
                js.position = [float(d.qpos[self.sim.qadr[j]]) for j in self.pub_joints]
                js.velocity = [float(d.qvel[self.sim.dadr[j]]) for j in self.pub_joints]
            self.js_pub.publish(js)

        def _set_camera(self, viewer):
            """기본 시점: 제어 관절들의 중심을 바라보되 forward_axis 뒤쪽(behind)에서."""
            cfg = self.viewer_cfg
            if cfg["view"] not in ("behind", "front"):
                return
            import math
            sim = self.sim
            with sim.lock:
                mujoco.mj_forward(sim.model, sim.data)
                anchors = np.array([sim.data.xanchor[sim.model.joint(j).id] for j in self.joints])
            center = anchors.mean(axis=0) + np.array([0.0, 0.0, cfg["lookat_height"]])
            fx, fy = cfg["forward_axis"][0], cfg["forward_axis"][1]
            az = math.degrees(math.atan2(fy, fx)) + (180.0 if cfg["view"] == "front" else 0.0)
            viewer.cam.lookat[:] = center
            viewer.cam.distance = cfg["distance"]
            viewer.cam.azimuth = az          # 카메라는 lookat 에서 azimuth 반대편에 놓이고 azimuth 방향(=정면)을 본다
            viewer.cam.elevation = cfg["elevation"]

        def run_physics(self):
            """메인 스레드: 벽시계에 맞춘 실시간 물리 스텝 (+뷰어). 뷰어는 플랫폼에 따라 메인 스레드가 필요."""
            sim = self.sim
            viewer = None
            if self.viewer_enabled:
                import mujoco.viewer
                viewer = mujoco.viewer.launch_passive(sim.model, sim.data)
                self._set_camera(viewer)
            try:
                run_realtime(sim, viewer, self.realtime, keep_running=rclpy.ok,
                             warn=lambda msg: self.get_logger().warning(msg))
            finally:
                if viewer is not None:
                    viewer.close()

    return MujocoSimNode


def main(args=None):
    import rclpy
    MujocoSimNode = make_node_class()
    rclpy.init(args=args)
    node = MujocoSimNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        node.run_physics()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
