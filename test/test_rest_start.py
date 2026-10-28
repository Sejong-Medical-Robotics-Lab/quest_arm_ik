"""팔을 내린 시작 자세(mujoco_sim.initial_posture)에서 mock 궤적(올리기 → 원 → 쥐기)이 끝까지 추종되는지, ROS 없이.

    G1_MJCF=/abs/g1.xml python3 test/test_rest_start.py                     # G1 (손 없음: 쥐기 신호는 무시됨)
    G1H_MJCF=/abs/g1_with_hands.xml python3 test/test_rest_start.py         # G1 + Dex3 손
    SH_REPO=/abs/SH-Humanoid python3 test/test_rest_start.py                # 삼현 + Dex3 (dex3_fingers:=revolute)
검사: 올린 뒤 한 바퀴 동안 손 추종 오차 < 15 mm(이전 버그: 팔이 다 펴진 자세에서 원 아래쪽을 못 그림), QP 실패 0,
      관절이 한계에 붙지 않음, 손가락이 쥐기 신호를 따라 닫힘(손 있는 모델).
"""
import math
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from quest_arm_ik.frames import rot_z, se3_from_pq  # noqa: E402
from quest_arm_ik.hand import GraspMapper  # noqa: E402
from quest_arm_ik.ik_core import DualArmIKCore, IKConfig  # noqa: E402
from quest_arm_ik.mapper import ArmMapper, MapperParams  # noqa: E402
from quest_arm_ik.mock_traj import mock_offsets, pitch_quat_xyzw  # noqa: E402
from quest_arm_ik.mujoco_sim import MujocoArmSim  # noqa: E402

HERE = os.path.dirname(__file__)
G1 = os.environ.get("G1_MJCF", "")
G1H = os.environ.get("G1H_MJCF", "")
SH = os.environ.get("SH_REPO", "")
MOCK = dict(radius=0.05, period=6.0, lift_forward=0.06, lift_up=0.15, lift_time=2.0, grasp_period=4.0, lift_pitch=math.radians(60.0))   # = quest_mock 기본값


def build_g1(path):
    cfg = yaml.safe_load(open(os.path.join(HERE, "..", "config", "dual_arm_ik.yaml")))
    ik, sm = cfg["dual_arm_ik"]["ros__parameters"], cfg["mujoco_sim"]["ros__parameters"]
    core = DualArmIKCore(IKConfig(model_path=path, controlled_joints=ik["controlled_joints"], left_hand_frame=ik["left_hand_frame"],
                                  right_hand_frame=ik["right_hand_frame"], base_frame=ik["base_frame"], nominal_posture=ik["nominal_posture"],
                                  posture_cost=ik["posture_cost"], orientation_cost=ik["orientation_cost"], enable_self_collision=True,
                                  collision_d_min=ik["collision_d_min"], collision_exclude_links=ik.get("collision_exclude_links", [])))
    sim = MujocoArmSim(path, ik["controlled_joints"], keyframe="stand", fix_base=True)
    return ik, sm, core, sim


def build_sh():
    cfg = yaml.safe_load(open(os.path.join(HERE, "..", "config", "sh_humanoid_v2.yaml")))
    ik, sm = cfg["dual_arm_ik"]["ros__parameters"], cfg["mujoco_sim"]["ros__parameters"]
    shapes = yaml.safe_load(ik["collision_shapes_yaml"])
    xacro = os.path.join(SH, "urdf_moveit", "SH_Humanoid_v2_description", "urdf", "SH_Humanoid_v2.xacro")
    srdf = os.path.join(SH, "urdf_moveit", "SH_Humanoid_v2_moveit_config", "config", "SH_Humanoid_v2.srdf")
    pkg = [os.path.join(SH, "urdf_moveit")]
    core = DualArmIKCore(IKConfig(model_path=xacro, controlled_joints=ik["controlled_joints"], left_hand_frame=ik["left_hand_frame"],
                                  right_hand_frame=ik["right_hand_frame"], base_frame=ik["base_frame"], package_dirs=pkg, srdf_path=srdf,
                                  xacro_args="dex3_fingers:=revolute", nominal_posture=ik["nominal_posture"], posture_weights=ik["posture_weights"],
                                  limit_margin=ik["limit_margin"], joint_limits=yaml.safe_load(ik["joint_limits_yaml"]),
                                  posture_cost=ik["posture_cost"], orientation_cost=ik["orientation_cost"], max_joint_velocity=ik["max_joint_velocity"],
                                  enable_self_collision=True, collision_d_min=ik["collision_d_min"], collision_shapes=shapes))
    sim = MujocoArmSim(xacro, ik["controlled_joints"], keyframe="", pd_kp=300.0, pd_kd=10.0, package_dirs=pkg,
                       collision_shapes=yaml.safe_load(sm["collision_shapes_yaml"]), srdf_path=srdf, xacro_args="dex3_fingers:=revolute")
    return ik, sm, core, sim


def run(name, ik, sm, core, sim):
    fwd = np.array(ik["forward_axis"], float)
    yaw = math.degrees(math.atan2(fwd[1], fwd[0]))
    q0 = np.array(sm["initial_posture"], float)
    for j, v in zip(ik["controlled_joints"], q0):
        sim.data.qpos[sim.qadr[j]] = v
    sim.set_joint_targets(q0); sim.step(300)
    core.reset_from_joint_positions(sim.joint_positions())
    scale = ik["translation_scale"]
    params = MapperParams(mode="relative", max_step=ik.get("max_step", 0.03), scale=scale, R_cal=rot_z(math.radians(yaw)), input_timeout=0.5)
    mappers = {s: ArmMapper(s, params) for s in ("left", "right")}
    gm = GraspMapper(tau=0.1)
    fingers = [j for j in gm.joint_names if j in sim.hold_joints]
    base = ik["base_frame"]
    hand0 = {s: sim.body_pose_in_base(ik[f"{s}_hand_frame"], base)[0].copy() for s in ("left", "right")}
    dt, t_eng = 0.01, 0.5
    errs, closed_seen = [], 0.0
    q_min, q_max = np.full(14, np.inf), np.full(14, -np.inf)
    for k in range(int((t_eng + MOCK["lift_time"] + MOCK["period"] + 0.5) / dt)):
        t = k * dt
        a, up, g, pitch = mock_offsets(t - t_eng, **MOCK)
        quat = np.array(pitch_quat_xyzw(pitch))
        for s, y0 in (("left", 0.25), ("right", -0.25)):
            p = np.array([0.3 + a, y0, 1.0 + up])                       # mock 은 ROS 축(x 정면) 으로 발행
            m = mappers[s]
            m.set_raw(se3_from_pq(p, quat), t)
            m.set_engaged(t >= t_eng)
            target = m.update(t, core.hand_pose(s))
            if target is not None:
                core.set_hand_target(s, target)
            elif m.released:
                core.hold_hand(s)
        q = core.step(dt); sim.set_joint_targets(q)
        if fingers:
            gm.set("left", g); gm.set("right", g)
            tg = gm.update(dt); sim.set_extra_targets(list(tg), list(tg.values()))
        sim.step(5)
        q_min, q_max = np.minimum(q_min, q), np.maximum(q_max, q)
        if t > t_eng + MOCK["lift_time"] + 0.3:                                # 원 구간: 기대 변위 = scale·(앞 a, 위 up) (base 축)
            for s in ("left", "right"):
                exp = hand0[s] + scale * (a * fwd + np.array([0, 0, up]))
                errs.append(np.linalg.norm(sim.body_pose_in_base(ik[f"{s}_hand_frame"], base)[0] - exp))
        if fingers:
            closed_seen = max(closed_seen, max(abs(sim.data.qpos[sim.qadr[j]]) for j in fingers))
    lo, hi = np.array(core.model.lowerPositionLimit), np.array(core.model.upperPositionLimit)
    margin = min(np.min(q_min - lo[core.q_index]), np.min(hi[core.q_index] - q_max))
    print(f"[{name}] circle tracking err mean {np.mean(errs)*1000:.1f} / max {np.max(errs)*1000:.1f} mm, qp failures {core.qp_failures}, "
          f"min joint-limit margin {margin:.3f} rad, fingers {len(fingers)} max closed {closed_seen:.2f} rad")
    assert np.max(errs) < 0.015 and core.qp_failures == 0 and margin > 0.02
    if fingers:
        assert closed_seen > 1.0


if __name__ == "__main__":
    if not (G1 or G1H or SH):
        print("G1_MJCF / G1H_MJCF / SH_REPO 중 하나를 지정"); sys.exit(1)
    if G1:
        run("G1", *build_g1(G1))
    if G1H:
        run("G1+hands", *build_g1(G1H))
    if SH:
        run("SH+Dex3", *build_sh())
    print("OK")
