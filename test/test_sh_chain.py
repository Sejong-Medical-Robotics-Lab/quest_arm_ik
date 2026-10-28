"""SH-Humanoid v2: Unity SHPOSE 스트림 → 파서 → ArmMapper → IK(자기충돌·스탠드 형상) → MuJoCo, ROS 없이.

    export SH_REPO=/abs/SH-Humanoid          # urdf_moveit/ 가 들어 있는 저장소 루트
    python3 test/test_sh_chain.py

Unity ControllerPoseLogger.cs 를 그대로 흉내낸다: 그립을 누른 순간을 기준으로 변화량을 base_link 축으로 바꿔
neutral(준비 자세 손 위치) 에 더한 절대 포즈를 "SHPOSE,1,L,..." 로 보낸다. 그립 중에만 송신.
검사: relative/absolute 매핑 둘 다 로봇 손이 사람 손 변화량(×scale)을 따라가는지, 기둥(스탠드) 쪽으로 밀어도
      collision_d_min 에서 멈추는지, release 후 정지·re-engage 무점프.
"""
import os
import sys
import time

import numpy as np
import pinocchio as pin
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from quest_arm_ik.ik_core import DualArmIKCore, IKConfig  # noqa: E402
from quest_arm_ik.mapper import ArmMapper, MapperParams  # noqa: E402
from quest_arm_ik.mujoco_sim import MujocoArmSim  # noqa: E402
from quest_arm_ik.protocols import parse_shpose  # noqa: E402
from quest_arm_ik.frames import se3_from_pq  # noqa: E402

REPO = os.environ.get("SH_REPO", "")
XACRO = os.path.join(REPO, "urdf_moveit", "SH_Humanoid_v2_description", "urdf", "SH_Humanoid_v2.xacro")
SRDF = os.path.join(REPO, "urdf_moveit", "SH_Humanoid_v2_moveit_config", "config", "SH_Humanoid_v2.srdf")
CONFIG = os.path.join(os.path.dirname(__file__), "..", "config", "sh_humanoid_v2.yaml")
NEUTRAL = {"left": np.array([-0.2048, 0.2531, 0.7589]), "right": np.array([0.2088, 0.2495, 0.7595])}  # 팀 quest_ik.yaml
TELEOP_READY = [0.0, 0.0, 0.0, -1.2, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0, -1.2, 0.0, 0.0, 0.3]   # SRDF teleop_ready (관절 순서 …, wrist_yaw, wrist_roll, wrist_pitch)


def build():
    cfg = yaml.safe_load(open(CONFIG))["dual_arm_ik"]["ros__parameters"]
    shapes = yaml.safe_load(cfg["collision_shapes_yaml"])
    core = DualArmIKCore(IKConfig(
        model_path=XACRO, controlled_joints=cfg["controlled_joints"], left_hand_frame=cfg["left_hand_frame"],
        right_hand_frame=cfg["right_hand_frame"], base_frame=cfg["base_frame"], package_dirs=[os.path.join(REPO, "urdf_moveit")], srdf_path=SRDF,
        nominal_posture=cfg["nominal_posture"], posture_weights=cfg["posture_weights"], limit_margin=cfg["limit_margin"],
        joint_limits=yaml.safe_load(cfg["joint_limits_yaml"]) if cfg.get("joint_limits_yaml") else None,
        posture_cost=cfg["posture_cost"], orientation_cost=cfg["orientation_cost"], max_joint_velocity=cfg["max_joint_velocity"],
        enable_self_collision=True, collision_d_min=cfg["collision_d_min"], collision_shapes=shapes))
    sim = MujocoArmSim(XACRO, cfg["controlled_joints"], keyframe="", pd_kp=300.0, pd_kd=10.0,
                       package_dirs=[os.path.join(REPO, "urdf_moveit")], collision_shapes=shapes, srdf_path=SRDF)
    # 준비 자세로 세팅 (팀 ready.sh 역할). 팀 Unity 절대 포즈(NEUTRAL)는 teleop_ready(팔꿈치 -1.2, 손목 pitch 0.3) 에서 손이 있는 곳이 기준이므로
    # config nominal_posture(v0.7.35 부터 '내린 팔')가 아니라 teleop_ready 로 시작한다.
    q_ready = np.array(TELEOP_READY)
    sim.set_joint_targets(q_ready)
    for j, v in zip(cfg["controlled_joints"], q_ready):
        sim.data.qpos[sim.qadr[j]] = v
    sim.step(500)
    core.reset_from_joint_positions(sim.joint_positions())
    return cfg, core, sim


def unity_shpose(side, t, p_unity, q_unity, p_cal, scale=1.0):
    """ControllerPoseLogger.SendControllerPose 와 동일: delta(Unity 축) → 로봇 축 (x, z, y) + neutral."""
    d = p_unity - p_cal
    robot_p = NEUTRAL[side] + np.array([d[0], d[2], d[1]])
    qx, qy, qz, qw = q_unity
    robot_q = np.array([-qx, -qz, -qy, qw])
    s = "L" if side == "left" else "R"
    return f"SHPOSE,1,{s},{t:.6f},{robot_p[0]:.6f},{robot_p[1]:.6f},{robot_p[2]:.6f},{robot_q[0]:.6f},{robot_q[1]:.6f},{robot_q[2]:.6f},{robot_q[3]:.6f}"


def run_chain(mapping_mode):
    cfg, core, sim = build()
    scale = cfg["translation_scale"]
    params = MapperParams(mode=mapping_mode, max_step=cfg["max_step"], scale=scale, input_timeout=0.5)
    mappers = {s: ArmMapper(s, params) for s in ("left", "right")}
    hand0 = {s: sim.body_pose_in_base(cfg[f"{s}_hand_frame"], "base_link")[0].copy() for s in ("left", "right")}
    # Unity 가 보내는 손 '회전': 절대 모드는 그대로 손 목표 회전이 되므로(IK 손 프레임 규약과 맞아야 함) 모델의 teleop_ready 손 프레임 회전을 보낸다
    # (팀 앱은 자기 중립 회전을 쓴다; 단위 회전을 보내면 플랜지 프레임이 68° 돌아 있는 이 모델에서는 손목 한계에 걸려 위치가 2.7 cm 밀린다).
    # robot(x,y,z,w) → unity 는 unity_shpose 의 역변환: (qx,qy,qz,qw) = (-rx, -rz, -ry, rw)
    import pinocchio as pin
    q_unity = {}
    for s in ("left", "right"):
        rq = pin.Quaternion(core.hand_pose(s).rotation).coeffs()      # x, y, z, w
        q_unity[s] = (-rq[0], -rq[2], -rq[1], rq[3])

    # Unity 쪽 컨트롤러 궤적 (Unity 축: x 우측, y 상방, z 전방). 0.5 s 에 그립, 이후 2 s 동안 앞 0.15 m + 위 0.10 m
    p_cal = {s: np.array([0.3 * (1 if s == "right" else -1), 1.0, 0.4]) for s in ("left", "right")}
    def p_unity(s, t):
        a = np.clip((t - 0.5) / 2.0, 0.0, 1.0)
        return p_cal[s] + a * np.array([0.0, 0.10, 0.15])
    dt = 0.01
    released_at = None
    p_rel = None
    for k in range(400):
        t = k * dt
        grip = 0.5 <= t < 3.0
        for s, m in mappers.items():
            if grip:                                            # Unity 는 그립 중에만 송신
                pkt = parse_shpose(unity_shpose(s, t, p_unity(s, t), q_unity[s], p_cal[s]))
                side, p, q = pkt
                m.set_raw(se3_from_pq(p, q), t)
            m.set_engaged((t - m.raw_time) < 0.3 if m.raw is not None else False)   # on_stream 엔게이지
            target = m.update(t, core.hand_pose(s))
            if target is not None:
                core.set_hand_target(s, target)
            elif m.released:
                core.hold_hand(s)
                released_at = t
                p_rel = sim.body_pose_in_base(cfg["left_hand_frame"], "base_link")[0].copy()
        q = core.step(dt)
        sim.set_joint_targets(q); sim.step(5)

    # 기대 이동: Unity delta (0, +0.10 up, +0.15 fwd) → 로봇 축 (0, +0.15 y, +0.10 z) × scale (두 모드 모두 그립 순간 기준 변화량에 scale)
    exp = np.array([0.0, 0.15, 0.10]) * scale
    for s in ("left", "right"):
        p_now = sim.body_pose_in_base(cfg[f"{s}_hand_frame"], "base_link")[0]
        if mapping_mode == "absolute":
            err = np.linalg.norm(p_now - (NEUTRAL[s] + exp))
            print(f"[{mapping_mode}] {s}: hand at {np.round(p_now,3)} vs neutral+Δ {np.round(NEUTRAL[s]+exp,3)} → err {err*1000:.1f} mm "
                  f"(팀 NEUTRAL 과 모델 teleop_ready 손 위치 차 {1000*np.linalg.norm(NEUTRAL[s]-hand0[s]):.0f} mm 는 그립 순간 그만큼 이동함)")
        else:
            err = np.linalg.norm((p_now - hand0[s]) - exp)
            print(f"[{mapping_mode}] {s}: Δ={np.round(p_now-hand0[s],3)} expected {np.round(exp,3)} → err {err*1000:.1f} mm")
        assert err < 0.02, f"{s} tracking error too large"
    assert released_at is not None and abs(released_at - 3.3) < 0.05, f"release at {released_at}"
    drift = np.linalg.norm(sim.body_pose_in_base(cfg["left_hand_frame"], "base_link")[0] - p_rel)
    print(f"[{mapping_mode}] after release moved {drift*1000:.1f} mm, qp failures {core.qp_failures}, "
          f"active pairs {core.active_pairs}, collision pairs {len(core.collision_model.collisionPairs)}")
    assert drift < 0.01 and core.qp_failures == 0
    return cfg, core, sim


def test_stand_collision():
    cfg, core, sim = build()
    # 왼손을 기둥 안쪽(x=0, y=0.0)으로 밀어 넣는 목표 → 스탠드 기둥(r=0.06)+손 상자 앞에서 멈춰야 함
    T = core.hand_pose("left"); T.translation = np.array([0.0, 0.0, 0.75]); core.set_hand_target("left", T)
    for _ in range(300):
        sim.set_joint_targets(core.step(0.01)); sim.step(5)
    import pinocchio as pin
    pin.computeDistances(core.model, core.configuration.data, core.collision_model,
                         core.configuration.collision_data, core.configuration.q)
    dmin = min(r.min_distance for r in core.configuration.collision_data.distanceResults)
    p = core.hand_pose("left").translation
    print(f"stand push: hand stopped at {np.round(p,3)}, min clearance {dmin*1000:.1f} mm (d_min {cfg['collision_d_min']*1000:.0f}), "
          f"qp failures {core.qp_failures}")
    assert dmin > cfg["collision_d_min"] - 0.005


if __name__ == "__main__":
    if not os.path.isfile(XACRO):
        print("SH_REPO 환경변수에 SH-Humanoid 저장소 경로를 지정하세요"); sys.exit(1)
    run_chain("relative")
    run_chain("absolute")
    test_stand_collision()
    print("OK")
