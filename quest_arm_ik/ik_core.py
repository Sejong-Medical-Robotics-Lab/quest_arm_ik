"""양팔 미분 IK 코어 (pink / Pinocchio, ROS 비의존).

- 모델: URDF(.urdf) 또는 MuJoCo MJCF(.xml) — 전신 모델을 넣어도 controlled_joints 외 관절은 고정(reduced model)
- 태스크: 왼손·오른손 FrameTask + PostureTask(영공간 자세)
- 한계: 관절 위치·속도 한계(모델 기본값) + 선택적 자기충돌 배리어
- 특이점·한계 근처에서 멈추지 않고 감쇠로 느려질 뿐이다.

ROS 노드(dual_arm_ik_node)와 Jetson Thor 쪽 관제 SW 어디서든 그대로 재사용 가능.
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np
import pinocchio as pin
import pink
import qpsolvers
from pink import build_ik
from pink.exceptions import NoSolutionFound
from pink.tasks import FrameTask, PostureTask

from .urdf_tools import prepare_urdf

SIDES = ("left", "right")


@dataclass
class IKConfig:
    model_path: str                            # .urdf / .xacro / .xml(MJCF)
    controlled_joints: Sequence[str]          # 출력 순서 = 이 순서 (14개)
    left_hand_frame: str
    right_hand_frame: str
    base_frame: str = ""                       # hand_pose/set_hand_target 의 기준 프레임 (비우면 모델 universe).
                                               # MJCF 부동 베이스(G1 pelvis)는 pinocchio 3.x 가 루트 body pos 를 관절 배치에,
                                               # 4.x 는 q 에 넣어 universe 기준 손 높이가 0.79 m 어긋나므로 반드시 지정.
    package_dirs: Sequence[str] = field(default_factory=list)   # URDF package:// 메시 경로 (자기충돌용)
    srdf_path: str = ""                        # 충돌쌍 제외용 SRDF (선택, URDF만)
    xacro_args: str = ""                       # xacro 인자 "hand:=box dex3_fingers:=revolute" (model_path 가 .xacro 일 때)
    nominal_posture: Optional[Sequence[float]] = None            # controlled_joints 순서
    posture_weights: Optional[Sequence[float]] = None            # 관절별 posture 가중치 (controlled_joints 순서), None=모두 1
    limit_margin: float = 0.0                  # 관절 한계 안쪽 여유 [rad]
    joint_limits: Optional[dict] = None        # {joint_name: [lower, upper]} 모델 한계보다 좁게 쓸 때 (예: 팔꿈치 접힘 제한)
    position_cost: float = 1.0
    orientation_cost: float = 0.3
    task_gain: float = 0.5                     # 0~1, 목표를 한 스텝에 얼마나 따라갈지
    lm_damping: float = 1e-3                   # 특이점 근처 Levenberg-Marquardt 감쇠
    posture_cost: float = 0.01                 # 손 태스크 대비 작게. 크면 팔꿈치는 자연스럽지만 손 정상상태 오차↑ (0.05→약 1cm)
    damping: float = 1e-6                      # QP Tikhonov 감쇠
    max_joint_velocity: float = 2.0            # rad/s, 모델 속도 한계 상한 (모델에 한계가 없으면 이 값)
    enable_self_collision: bool = False
    collision_d_min: float = 0.02              # 유지할 최소 간격 [m]
    collision_gain: float = 0.3                # 한 스텝에 (d - d_min)의 몇 배까지 접근 허용 (0~1, 작을수록 보수적)
    collision_activation: float = 0.08         # 이 거리[m] 안의 쌍만 제약으로 추가 (active set)
    collision_convex_hull: bool = True         # 메시 충돌형상을 convex hull로 변환 (메시 거리계산은 100 Hz 불가)
    collision_shapes: Sequence[dict] = field(default_factory=list)   # URDF 링크 충돌형상을 기본도형으로 교체 [{link,type,size,xyz,rpy}]
    collision_exclude_links: Sequence[str] = field(default_factory=list)  # 충돌 검사에서 뺄 링크 (예: 다리)
    solver: str = ""                           # 비우면 quadprog 우선
    elbow_hint_cost: float = 0.0               # >0 이면 팔꿈치 위치 과제(팔꿈치 힌트) 사용: 목표 = 손 목표 − 전완 길이 × 조작자 전완 방향 (set_elbow_hint). 손 위치 cost 1.0 대비 0.1~0.3


WRIST_LOCK_VEL = 1e-6      # rad/s — 잠긴 손목 관절의 속도 한계 (0 은 QP 수치상 피함; 10 ms 에 1e-8 rad)


def _is_mjcf(path: str) -> bool:
    return os.path.splitext(path)[1].lower() == ".xml"


class WeightedPostureTask(PostureTask):
    """관절별 가중치 posture (오차·Jacobian 행에 sqrt(w) 곱). 팀 pink_ik_bridge 와 동일."""

    def __init__(self, cost: float, weights: np.ndarray):
        super().__init__(cost=cost)
        self.sqrt_w = np.sqrt(np.asarray(weights, dtype=float))

    def compute_error(self, configuration):
        return self.sqrt_w * super().compute_error(configuration)

    def compute_jacobian(self, configuration):
        return self.sqrt_w[:, None] * super().compute_jacobian(configuration)


def _geom_link_name(model: pin.Model, g) -> str:
    return model.frames[g.parentFrame].name if 0 <= g.parentFrame < len(model.frames) else ""


class DualArmIKCore:
    def __init__(self, cfg: IKConfig):
        self.cfg = cfg
        self.model: pin.Model
        self.collision_model = None
        self.qp_failures = 0
        self._load_model()
        self._build_tasks()

    # ------------------------------------------------------------ model
    def _load_full_model(self):
        """(model, collision_model|None, visual_model|None)"""
        cfg = self.cfg
        if not os.path.isfile(cfg.model_path):
            raise FileNotFoundError(f"model_path not found: {cfg.model_path}")
        if _is_mjcf(cfg.model_path):
            model = pin.buildModelFromMJCF(cfg.model_path)
            if not cfg.enable_self_collision:
                return model, None, None
            cmodel = pin.buildGeomFromMJCF(model, cfg.model_path, pin.GeometryType.COLLISION)
            vmodel = pin.buildGeomFromMJCF(model, cfg.model_path, pin.GeometryType.VISUAL)
            self._drop_excluded(model, cmodel)
            return model, cmodel, vmodel
        pkg = [d for d in cfg.package_dirs if d]
        urdf = prepare_urdf(cfg.model_path, pkg, cfg.collision_shapes if cfg.enable_self_collision else (),
                            resolve_packages=cfg.enable_self_collision, tag="ik", xacro_args=cfg.xacro_args)
        self.prepared_model_path = urdf
        if not cfg.enable_self_collision:
            return pin.buildModelFromUrdf(urdf), None, None
        robot = pin.RobotWrapper.BuildFromURDF(urdf, package_dirs=pkg or None)
        cmodel = robot.collision_model
        self._drop_excluded(robot.model, cmodel)
        return robot.model, cmodel, robot.visual_model

    def _drop_excluded(self, model, cmodel) -> None:
        """collision_exclude_links 에 적힌 링크(body)의 충돌형상 제거. 이름에 '*' 가 있으면 fnmatch 패턴 (예: '*_hand_*_link' = 손가락 전부)."""
        import fnmatch
        pats = [l for l in self.cfg.collision_exclude_links if l]
        if not pats:
            return
        drop = [g.name for g in cmodel.geometryObjects
                if any(fnmatch.fnmatch(_geom_link_name(model, g), pat) for pat in pats)]
        for name in drop:
            cmodel.removeGeometryObject(name)

    @staticmethod
    def _convexify(cmodel) -> int:
        """BVH 메시 충돌형상을 convex hull로 교체. 반환: 교체 개수.
        G1 메시(≈18k 정점) 기준 메시-메시 거리계산은 pair당 ~1 ms, hull은 ~5 µs."""
        import coal
        n = 0
        for g in cmodel.geometryObjects:
            geom = g.geometry
            if isinstance(geom, (coal.BVHModelOBBRSS, coal.BVHModelOBB)) if hasattr(coal, "BVHModelOBB") \
                    else isinstance(geom, coal.BVHModelOBBRSS):
                vec = coal.StdVec_Vec3s()
                for v in np.asarray(geom.vertices()):
                    vec.append(v)
                g.geometry = coal.Convex.convexHull(vec, True, "Qt")
                n += 1
        return n

    def _load_model(self) -> None:
        cfg = self.cfg
        self.prepared_model_path = cfg.model_path
        model, cmodel, vmodel = self._load_full_model()
        if cmodel is not None and cfg.collision_convex_hull:
            self._convexify(cmodel)

        names = [model.names[i] for i in range(1, model.njoints)]
        missing = [j for j in cfg.controlled_joints if j not in names]
        if missing:
            raise ValueError(f"모델에 없는 관절: {missing}\n사용 가능: {names}")

        # 제어 대상이 아닌 관절(전신 모델의 다리·허리·free-flyer 등)은 고정
        lock = [model.getJointId(j) for j in names if j not in cfg.controlled_joints]
        if lock:
            q_ref = pin.neutral(model)
            if cmodel is not None:
                model, geoms = pin.buildReducedModel(model, [cmodel, vmodel], lock, q_ref)
                cmodel, vmodel = geoms[0], geoms[1]
            else:
                model = pin.buildReducedModel(model, lock, q_ref)

        if model.nq != model.nv or model.nq != len(cfg.controlled_joints):
            raise ValueError("controlled_joints는 모두 1자유도 회전/직동 관절이어야 함 "
                             "(continuous 관절은 nq≠nv라 지원하지 않음)")

        self.q_index = [model.joints[model.getJointId(j)].idx_q for j in cfg.controlled_joints]
        self.v_index = [model.joints[model.getJointId(j)].idx_v for j in cfg.controlled_joints]

        if cfg.joint_limits:
            lo, hi = np.array(model.lowerPositionLimit), np.array(model.upperPositionLimit)
            for jname, (jlo, jhi) in cfg.joint_limits.items():
                if jname not in cfg.controlled_joints:
                    raise ValueError(f"joint_limits: '{jname}' 는 controlled_joints 에 없음")
                idx = model.joints[model.getJointId(jname)].idx_q
                lo[idx], hi[idx] = max(lo[idx], float(jlo)), min(hi[idx], float(jhi))   # 모델 한계보다 넓게는 못 잡음
            model.lowerPositionLimit, model.upperPositionLimit = lo, hi
        if cfg.limit_margin > 0.0:
            lo, hi = np.array(model.lowerPositionLimit), np.array(model.upperPositionLimit)
            fin = np.isfinite(lo) & np.isfinite(hi)
            lo[fin] += cfg.limit_margin
            hi[fin] -= cfg.limit_margin
            model.lowerPositionLimit, model.upperPositionLimit = lo, hi

        # 속도 한계: 모델에 없거나 0/inf면 max_joint_velocity로, 있으면 그 이하로 캡
        vl = np.array(model.velocityLimit, dtype=float)
        vl[~np.isfinite(vl) | (vl <= 0.0)] = cfg.max_joint_velocity
        model.velocityLimit = np.minimum(vl, cfg.max_joint_velocity)
        # 위치 한계가 없는(±inf) 관절은 pink의 한계 처리에서 제외되도록 그대로 둔다

        for f in (cfg.left_hand_frame, cfg.right_hand_frame):
            if not model.existFrame(f):
                raise ValueError(f"프레임 '{f}' 없음. 사용 가능: {[fr.name for fr in model.frames]}")

        self.model = model
        self.q_nominal = self._nominal_q()
        data = model.createData()

        if cmodel is not None:
            cmodel.addAllCollisionPairs()
            if cfg.srdf_path:
                pin.removeCollisionPairs(model, cmodel, cfg.srdf_path)
            self._prune_collision_pairs(model, cmodel, self.q_nominal)
            cdata = cmodel.createData()
            self.configuration = pink.Configuration(
                model, data, self.q_nominal, collision_model=cmodel, collision_data=cdata)
            self.collision_model = cmodel
        else:
            self.configuration = pink.Configuration(model, data, self.q_nominal)

        # 손 자세·목표를 base_frame 기준으로 통일 (universe→base 변환은 베이스가 고정이므로 상수)
        self.T_base = pin.SE3.Identity()
        if cfg.base_frame:
            if not model.existFrame(cfg.base_frame):
                raise ValueError(f"base_frame '{cfg.base_frame}' 없음. 사용 가능: {[fr.name for fr in model.frames]}")
            fr = model.frames[model.getFrameId(cfg.base_frame)]
            if fr.parentJoint != 0:
                raise ValueError(f"base_frame '{cfg.base_frame}' 는 고정 베이스(관절 '{model.names[fr.parentJoint]}' 아님)에 있어야 함")
            self.T_base = pin.SE3(self.configuration.get_transform_frame_to_world(cfg.base_frame))
        self.T_base_inv = self.T_base.inverse()

        # 손목 잠금(회전 클러치): pink 가 매 스텝 읽는 속도 한계 배열(model.velocity_limit.velocity_limit)의 손목 항을 ~0 으로 만들어
        # 손목 관절을 그 자리에 고정한다. 원래 한계는 _vl_orig 에 보관.
        self._vl_orig = np.array(self.configuration.model.velocity_limit.velocity_limit, dtype=float, copy=True)
        self._wrist_v_index = {side: [model.joints[model.getJointId(j)].idx_v for j in cfg.controlled_joints
                                      if j.startswith(side + "_") and "wrist" in j] for side in ("left", "right")}
        self.wrist_locked = {"left": False, "right": False}
        self.orientation_tracking = {"left": True, "right": True}

    def _prune_collision_pairs(self, model: pin.Model, cmodel, q: np.ndarray) -> None:
        """MoveIt SRDF 생성기와 비슷한 규칙으로 무의미한 충돌쌍 제거:
        (1) 트리에서 2홉 이내 관절에 붙은 형상끼리(손목 3축처럼 한 점에 모인 링크),
        (2) nominal 자세에서 이미 d_min 이내이거나 겹치는 쌍,
        (3) 무작위 자세 200개 중 95% 이상 겹치는 쌍(기하 구조상 항상 붙어 있는 쌍)."""
        def ancestors(j, hops):
            out, k = set(), j
            for _ in range(hops):
                k = model.parents[k]
                out.add(k)
                if k == 0:
                    break
            return out

        def pair(cp):
            return pin.CollisionPair(cp.first, cp.second)

        remove = []
        for cp in cmodel.collisionPairs:
            j1 = cmodel.geometryObjects[cp.first].parentJoint
            j2 = cmodel.geometryObjects[cp.second].parentJoint
            if j1 == j2 or j2 in ancestors(j1, 2) or j1 in ancestors(j2, 2):
                remove.append(pair(cp))
        for cp in remove:
            cmodel.removeCollisionPair(cp)

        data, cdata = model.createData(), cmodel.createData()
        pin.computeDistances(model, data, cmodel, cdata, q)
        margin = 1.5 * self.cfg.collision_d_min
        remove = [pair(cp) for k, cp in enumerate(cmodel.collisionPairs)
                  if cdata.distanceResults[k].min_distance < margin]
        for cp in remove:
            cmodel.removeCollisionPair(cp)

        rng = np.random.default_rng(0)
        lo, hi = model.lowerPositionLimit, model.upperPositionLimit
        finite = np.isfinite(lo) & np.isfinite(hi)
        n_samples, hits = 200, np.zeros(len(cmodel.collisionPairs), dtype=int)
        cdata = cmodel.createData()
        for _ in range(n_samples):
            qs = q.copy()
            qs[finite] = rng.uniform(lo[finite], hi[finite])
            pin.computeCollisions(model, data, cmodel, cdata, qs, False)
            hits += np.array([cdata.collisionResults[k].isCollision() for k in range(len(cmodel.collisionPairs))], dtype=int)
        remove = [pair(cp) for k, cp in enumerate(cmodel.collisionPairs) if hits[k] >= 0.95 * n_samples]
        for cp in remove:
            cmodel.removeCollisionPair(cp)

    def _nominal_q(self) -> np.ndarray:
        q = pin.neutral(self.model)
        if self.cfg.nominal_posture is not None and len(self.cfg.nominal_posture) > 0:
            if len(self.cfg.nominal_posture) != len(self.q_index):
                raise ValueError("nominal_posture 길이는 controlled_joints 개수와 같아야 함")
            for iq, val in zip(self.q_index, self.cfg.nominal_posture):
                q[iq] = float(val)
        return self._clip(q)

    def _clip(self, q: np.ndarray) -> np.ndarray:
        return np.clip(q, self.model.lowerPositionLimit, self.model.upperPositionLimit)

    # ------------------------------------------------------------ tasks
    def _build_tasks(self) -> None:
        cfg = self.cfg
        self.hand_tasks = {}
        for side, frame in (("left", cfg.left_hand_frame), ("right", cfg.right_hand_frame)):
            task = FrameTask(frame,
                             position_cost=cfg.position_cost,
                             orientation_cost=cfg.orientation_cost,
                             lm_damping=cfg.lm_damping,
                             gain=cfg.task_gain)
            task.set_target_from_configuration(self.configuration)
            self.hand_tasks[side] = task

        if cfg.posture_weights is not None and len(cfg.posture_weights) > 0:
            if len(cfg.posture_weights) != len(self.v_index):
                raise ValueError("posture_weights 길이는 controlled_joints 개수와 같아야 함")
            w = np.ones(self.model.nv)
            for iv, val in zip(self.v_index, cfg.posture_weights):
                w[iv] = float(val)
            self.posture_task = WeightedPostureTask(cfg.posture_cost, w)
        else:
            self.posture_task = PostureTask(cost=cfg.posture_cost)
        self.posture_task.set_target(self.q_nominal)
        self.tasks = [self.hand_tasks["left"], self.hand_tasks["right"], self.posture_task]

        # 팔꿈치 힌트: 팔꿈치 관절 프레임(pinocchio JOINT 프레임 = 관절 이름)에 위치만 거는 약한 과제. 목표는 set_elbow_hint 로 매 주기.
        self.elbow_tasks = {}
        self._elbow_frame = {}            # 팔꿈치 관절 프레임 (힌트가 꺼져 있어도 elbow_position/forearm_dir 진단용으로 찾아 둠)
        for side in ("left", "right"):
            jn = next((j for j in cfg.controlled_joints if j.startswith(side + "_") and "elbow" in j), None)
            if jn is not None and self.model.existFrame(jn):
                self._elbow_frame[side] = jn
        if cfg.elbow_hint_cost > 0.0:
            for side in ("left", "right"):
                if side not in self._elbow_frame:
                    raise ValueError(f"elbow_hint: '{side}' 팔꿈치 관절 프레임을 못 찾음 (controlled_joints 에 *_elbow* 필요)")
                t = FrameTask(self._elbow_frame[side], position_cost=0.0, orientation_cost=0.0, lm_damping=cfg.lm_damping, gain=cfg.task_gain)   # 힌트가 올 때까지 cost 0
                t.set_target_from_configuration(self.configuration)
                self.elbow_tasks[side] = t
                self.tasks.append(t)
        self.elbow_hint_active = {"left": False, "right": False}

        self.active_pairs = 0                 # 최근 스텝에서 제약으로 들어간 충돌쌍 수 (진단용)

        avail = list(qpsolvers.available_solvers)
        self.solver = cfg.solver or ("quadprog" if "quadprog" in avail else (avail[0] if avail else ""))
        if not self.solver:
            raise RuntimeError("QP 솔버 없음: pip install quadprog")

    # ------------------------------------------------------------ public API
    @property
    def q(self) -> np.ndarray:
        return np.array(self.configuration.q, copy=True)

    def joint_positions(self) -> np.ndarray:
        """controlled_joints 순서의 관절 위치."""
        q = self.configuration.q
        return np.array([q[i] for i in self.q_index], dtype=float)

    def hand_pose(self, side: str) -> pin.SE3:
        """현재 손 프레임 자세 (base_frame 기준; base_frame 미지정이면 모델 universe 기준)."""
        return self.T_base_inv * self.configuration.get_transform_frame_to_world(self.hand_tasks[side].frame)

    def frame_pose(self, frame: str) -> pin.SE3:
        """임의 프레임 자세 (base_frame 기준)."""
        return self.T_base_inv * self.configuration.get_transform_frame_to_world(frame)

    def arm_length(self, side: str) -> Optional[float]:
        """어깨(pitch 관절)→팔꿈치 + 팔꿈치→손 프레임 의 링크 길이 합 [m] (= 팔을 곧게 폈을 때 어깨→손 거리, 자세와 무관).
        조작자 팔 길이와의 비로 translation_scale 을 정하는 데 쓴다. 관절 이름에 shoulder_pitch / elbow 가 없으면 None."""
        names = [j for j in self.cfg.controlled_joints if j.startswith(side + "_")]
        sh = next((j for j in names if "shoulder_pitch" in j), None)
        el = next((j for j in names if "elbow" in j), None)
        if sh is None or el is None:
            return None
        model, data = self.model, self.model.createData()
        pin.forwardKinematics(model, data, pin.neutral(model))
        pin.updateFramePlacements(model, data)
        frame = self.hand_tasks[side].frame
        ps, pe = data.oMi[model.getJointId(sh)].translation, data.oMi[model.getJointId(el)].translation
        ph = data.oMf[model.getFrameId(frame)].translation
        return float(np.linalg.norm(pe - ps) + np.linalg.norm(ph - pe))

    def gravity_torques(self) -> np.ndarray:
        """현재 자세의 중력 보상 토크 (controlled_joints 순서, N·m). 임피던스 모드 feed-forward 용."""
        tau = pin.computeGeneralizedGravity(self.model, self.configuration.data, self.configuration.q)
        return np.array([tau[i] for i in self.v_index], dtype=float)

    def set_hand_target(self, side: str, target: pin.SE3) -> None:
        """손 목표 자세 (base_frame 기준, hand_pose 와 같은 프레임)."""
        self.hand_tasks[side].set_target(self.T_base * target)

    def hold_hand(self, side: str) -> None:
        """목표를 현재 손 자세로 고정 (클러치 해제 시 즉시 정지용)."""
        self.hand_tasks[side].set_target_from_configuration(self.configuration)

    def elbow_position(self, side: str) -> Optional[np.ndarray]:
        """팔꿈치 관절 위치 (base_frame 기준). 모델에 *_elbow* 관절이 없으면 None."""
        if side not in self._elbow_frame:
            return None
        return self.frame_pose(self._elbow_frame[side]).translation.copy()

    def forearm_dir(self, side: str) -> Optional[np.ndarray]:
        """현재 전완 방향 단위벡터 = (손 프레임 − 팔꿈치) / |…| (base_frame). 팔꿈치 관절이 없으면 None."""
        e = self.elbow_position(side)
        if e is None:
            return None
        v = self.hand_pose(side).translation - e
        n = float(np.linalg.norm(v))
        return v / n if n > 1e-6 else None

    def forearm_length(self, side: str) -> float:
        """|손 프레임 − 팔꿈치| [m] (자세 무관하게 거의 일정; 손목을 굽히면 조금 짧아짐)."""
        e = self.elbow_position(side)
        return 0.0 if e is None else float(np.linalg.norm(self.hand_pose(side).translation - e))

    def set_elbow_hint(self, side: str, p_hand: np.ndarray, forearm_dir: np.ndarray) -> None:
        """팔꿈치 힌트: 손 목표 p_hand(base) 에서 조작자 전완 방향 forearm_dir(base, 단위) 으로 전완 길이만큼 거슬러 간 점을 팔꿈치 목표로.
        손 위치(cost 1.0) 가 우선이고 팔꿈치는 여유 자유도(스위블·손목 여유) 안에서만 따라간다."""
        if side not in self.elbow_tasks:
            return
        d = np.asarray(forearm_dir, dtype=float)
        n = float(np.linalg.norm(d))
        if n < 1e-6:
            return
        p_elbow = np.asarray(p_hand, dtype=float) - self.forearm_length(side) * (d / n)
        self.elbow_tasks[side].set_target(self.T_base * pin.SE3(np.eye(3), p_elbow))
        if not self.elbow_hint_active[side]:
            self.elbow_tasks[side].set_position_cost(self.cfg.elbow_hint_cost)
            self.elbow_hint_active[side] = True

    def clear_elbow_hint(self, side: str) -> None:
        """팔꿈치 힌트 끔 (cost 0): 그립 해제, 트리거로 손목을 돌리는 동안."""
        if side in self.elbow_tasks and self.elbow_hint_active[side]:
            self.elbow_tasks[side].set_position_cost(0.0)
            self.elbow_tasks[side].set_target_from_configuration(self.configuration)
            self.elbow_hint_active[side] = False

    def set_orientation_tracking(self, side: str, on: bool) -> None:
        """손 회전 추종 켬/끔: 끄면 FrameTask 의 orientation cost 를 0 으로 (위치만 추종). 켜면 config 값으로."""
        self.hand_tasks[side].set_orientation_cost(self.cfg.orientation_cost if on else 0.0)
        self.orientation_tracking[side] = bool(on)

    def set_wrist_locked(self, side: str, locked: bool) -> None:
        """손목 3관절 잠금: 속도 한계를 ~0 으로 두어 그 자리에 고정 (회전 추종을 끈 채 위치만 따라갈 때 손목이 자세 과제 쪽으로
        흘러가지 않게). 풀면 원래 속도 한계로."""
        vl = self.configuration.model.velocity_limit.velocity_limit
        for iv in self._wrist_v_index[side]:
            vl[iv] = WRIST_LOCK_VEL if locked else self._vl_orig[iv]
        self.wrist_locked[side] = bool(locked)

    def reset_from_joint_positions(self, positions: Sequence[float]) -> None:
        """측정 관절각(controlled_joints 순서)으로 내부 상태 초기화. 시작 시 점프 방지."""
        q = self.q
        for iq, val in zip(self.q_index, positions):
            q[iq] = float(val)
        self.configuration.update(self._clip(q))
        for task in getattr(self, "elbow_tasks", {}).values():
            task.set_target_from_configuration(self.configuration)
        for task in self.hand_tasks.values():
            task.set_target_from_configuration(self.configuration)

    # ------------------------------------------------------------ self-collision (active set)
    def _collision_constraints(self):
        """activation 거리 안의 충돌쌍마다 선형 제약 1행:  -nᵀ(J1-J2)Δq ≤ gain·(d-d_min).
        d(q+Δq) ≈ d + nᵀ(J1-J2)Δq 이므로 다음 스텝 거리가 d_min 쪽으로 (1-gain) 비율 이상 남는다.
        pink의 SelfCollisionBarrier는 모든 쌍의 Jacobian을 매 스텝 계산해 G1(≈290쌍)에서 17 ms가 걸려
        100 Hz가 불가능하므로, 거리 계산(≈1.5 ms)만 전체 쌍에 하고 가까운 쌍만 제약으로 넣는다."""
        cfg, model, data = self.cfg, self.model, self.configuration.data
        cm, cd = self.collision_model, self.configuration.collision_data
        q = self.configuration.q
        pin.computeDistances(model, data, cm, cd, q)
        near = [(k, cd.distanceResults[k].min_distance) for k in range(len(cm.collisionPairs))
                if cd.distanceResults[k].min_distance < cfg.collision_activation]
        self.active_pairs = len(near)
        if not near:
            return None, None
        pin.computeJointJacobians(model, data, q)
        G, h = [], []
        for k, d in near:
            res, cp = cd.distanceResults[k], cm.collisionPairs[k]
            p1, p2 = np.asarray(res.getNearestPoint1()), np.asarray(res.getNearestPoint2())
            if d > 1e-6:
                n = (p1 - p2) / d                      # geom2 → geom1 방향 (coal의 normal 필드는 신뢰 불가)
            else:                                      # 이미 관통: 중심 차이로 방향 추정
                c1 = data.oMi[cm.geometryObjects[cp.first].parentJoint].translation
                c2 = data.oMi[cm.geometryObjects[cp.second].parentJoint].translation
                n = c1 - c2
                n = n / (np.linalg.norm(n) + 1e-9)
            J1 = self._point_jacobian(cm.geometryObjects[cp.first].parentJoint, p1)
            J2 = self._point_jacobian(cm.geometryObjects[cp.second].parentJoint, p2)
            G.append(-(n @ (J1 - J2)))
            h.append(cfg.collision_gain * (d - cfg.collision_d_min))
        return np.vstack(G), np.array(h)

    def _point_jacobian(self, joint_id: int, p_world: np.ndarray) -> np.ndarray:
        """관절 joint_id에 고정된 월드 점 p의 병진 Jacobian (3 × nv)."""
        if joint_id == 0:
            return np.zeros((3, self.model.nv))
        J = pin.getJointJacobian(self.model, self.configuration.data, joint_id, pin.LOCAL_WORLD_ALIGNED)
        r = p_world - self.configuration.data.oMi[joint_id].translation
        return J[:3] - np.cross(r, J[3:], axisa=0, axisb=0).T

    # ------------------------------------------------------------ step
    def _solve(self, dt: float, with_collision: bool) -> np.ndarray:
        problem = build_ik(self.configuration, self.tasks, dt, damping=self.cfg.damping)
        if with_collision and self.collision_model is not None:
            G, h = self._collision_constraints()
            if G is not None:
                problem.G = G if problem.G is None else np.vstack([problem.G, G])
                problem.h = h if problem.h is None else np.concatenate([problem.h, h])
        result = qpsolvers.solve_problem(problem, solver=self.solver)
        if not result.found or result.x is None:
            raise NoSolutionFound(problem, result)
        return result.x / dt

    def step(self, dt: float) -> np.ndarray:
        """한 제어 주기: QP 풀고 q 적분. 반환: controlled_joints 순서의 관절 위치.
        QP가 실패하면 qp_failures를 올리고, 충돌 제약 때문이면 제약 없이 절반 속도로 한 번 더 푼다."""
        try:
            v = self._solve(dt, with_collision=True)
        except NoSolutionFound:
            self.qp_failures += 1
            if self.collision_model is None:
                return self.joint_positions()
            try:
                v = 0.5 * self._solve(dt, with_collision=False)
            except NoSolutionFound:
                return self.joint_positions()
        vmax = self.cfg.max_joint_velocity
        v = np.clip(v, -vmax, vmax)
        self.configuration.integrate_inplace(v, dt)
        return self.joint_positions()
