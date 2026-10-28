"""경로 B (Isaac ROS isaac_ros_teleop) 스트림 흉내 → quest_bridge 규약 → ArmMapper(요 오프셋) → IK → MuJoCo, ROS 없이.

    G1_MJCF=/abs/mujoco_menagerie/unitree_g1/g1.xml  python3 test/test_xr_chain.py      # G1  (요 0°)
    SH_REPO=/abs/SH-Humanoid                          python3 test/test_xr_chain.py      # 삼현 (요 90°)
    둘 다 지정하면 둘 다 검사.

teleop_ros2_node 가 내는 것과 같은 입력을 만든다: ee_poses = ROS 축(x 정면·y 좌측·z 상방, 바닥 원점) 의 컨트롤러 포즈,
controller_data = {side}_squeeze_value(그립). 그립을 쥔 순간이 기준, 앞으로 0.15 m + 위로 0.10 m 이동.
검사: 로봇 손이 base 축에서 정면(forward_axis)으로 0.15·scale, 위로 0.10·scale 움직이는가 (G1: +x, SH: +y),
      그립을 놓으면 멈추는가, 비활성(0,0,0) 포즈가 섞여도 점프하지 않는가.
"""
import math
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from quest_arm_ik.frames import rot_z, se3_from_pq  # noqa: E402
from quest_arm_ik.ik_core import DualArmIKCore, IKConfig  # noqa: E402
from quest_arm_ik.mapper import ArmMapper, MapperParams  # noqa: E402
from quest_arm_ik.mujoco_sim import MujocoArmSim  # noqa: E402
from quest_arm_ik.protocols import XR_ENGAGE_KEYS, xr_engaged, xr_pose_valid  # noqa: E402

HERE = os.path.dirname(__file__)
G1 = os.environ.get("G1_MJCF", "")
SH = os.environ.get("SH_REPO", "")


def build_g1():
    cfg = yaml.safe_load(open(os.path.join(HERE, "..", "config", "dual_arm_ik.yaml")))["dual_arm_ik"]["ros__parameters"]
    core = DualArmIKCore(IKConfig(model_path=G1, controlled_joints=cfg["controlled_joints"],
                                  left_hand_frame=cfg["left_hand_frame"], right_hand_frame=cfg["right_hand_frame"],
                                  base_frame=cfg["base_frame"], nominal_posture=cfg["nominal_posture"], posture_cost=cfg["posture_cost"],
                                  orientation_cost=cfg["orientation_cost"], enable_self_collision=True,
                                  collision_d_min=cfg["collision_d_min"]))
    sim = MujocoArmSim(G1, cfg["controlled_joints"], keyframe="stand", fix_base=True)
    sim.set_joint_targets(np.array(cfg["nominal_posture"])); sim.step(500)
    core.reset_from_joint_positions(sim.joint_positions())
    return cfg, core, sim, "pelvis"


def build_sh():
    cfg = yaml.safe_load(open(os.path.join(HERE, "..", "config", "sh_humanoid_v2.yaml")))["dual_arm_ik"]["ros__parameters"]
    shapes = yaml.safe_load(cfg["collision_shapes_yaml"])
    xacro = os.path.join(SH, "urdf_moveit", "SH_Humanoid_v2_description", "urdf", "SH_Humanoid_v2.xacro")
    srdf = os.path.join(SH, "urdf_moveit", "SH_Humanoid_v2_moveit_config", "config", "SH_Humanoid_v2.srdf")
    pkg = [os.path.join(SH, "urdf_moveit")]
    core = DualArmIKCore(IKConfig(model_path=xacro, controlled_joints=cfg["controlled_joints"],
                                  left_hand_frame=cfg["left_hand_frame"], right_hand_frame=cfg["right_hand_frame"],
                                  base_frame=cfg["base_frame"], package_dirs=pkg, srdf_path=srdf, nominal_posture=cfg["nominal_posture"],
                                  posture_weights=cfg["posture_weights"], limit_margin=cfg["limit_margin"],
        joint_limits=yaml.safe_load(cfg["joint_limits_yaml"]) if cfg.get("joint_limits_yaml") else None,
                                  posture_cost=cfg["posture_cost"], orientation_cost=cfg["orientation_cost"],
                                  max_joint_velocity=cfg["max_joint_velocity"], enable_self_collision=True,
                                  collision_d_min=cfg["collision_d_min"], collision_shapes=shapes))
    sim = MujocoArmSim(xacro, cfg["controlled_joints"], keyframe="", pd_kp=300.0, pd_kd=10.0,
                       package_dirs=pkg, collision_shapes=shapes, srdf_path=srdf)
    q0 = np.array(cfg["nominal_posture"])
    sim.set_joint_targets(q0)
    for j, v in zip(cfg["controlled_joints"], q0):
        sim.data.qpos[sim.qadr[j]] = v
    sim.step(500)
    core.reset_from_joint_positions(sim.joint_positions())
    return cfg, core, sim, "base_link"


def xr_stream(t, grip):
    """teleop_ros2_node 흉내: ROS 축 ee_poses(왼, 오) + controller_data. 조작자는 x 정면을 보고 섬, 원점은 바닥."""
    a = np.clip((t - 0.5) / 2.0, 0.0, 1.0)
    d = np.array([0.15, 0.0, 0.10]) * a                                  # 앞 0.15, 위 0.10
    poses = {"left": (np.array([0.4, 0.25, 1.0]) + d, np.array([0, 0, 0, 1.0])),
             "right": (np.array([0.4, -0.25, 1.0]) + d, np.array([0, 0, 0, 1.0]))}
    if t < 0.3:                                                          # 처음엔 오른손 비활성 (자리표시 포즈)
        poses["right"] = (np.zeros(3), np.array([0, 0, 0, 1.0]))
    data = {"left_squeeze_value": 1.0 if grip else 0.0, "right_squeeze_value": 1.0 if grip else 0.0,
            "left_is_active": True, "right_is_active": t >= 0.3}
    return poses, data


def run(name, build):
    cfg, core, sim, base = build()
    fwd = np.array(cfg["forward_axis"], dtype=float)
    yaw = math.degrees(math.atan2(fwd[1], fwd[0]))                      # launch 와 같은 식 (G1 0, SH 90)
    scale = cfg["translation_scale"]
    params = MapperParams(mode="relative", max_step=cfg.get("max_step", 0.03), scale=scale,
                          R_cal=rot_z(math.radians(yaw)), input_timeout=0.5)
    mappers = {s: ArmMapper(s, params) for s in ("left", "right")}
    hand0 = {s: sim.body_pose_in_base(cfg[f"{s}_hand_frame"], base)[0].copy() for s in ("left", "right")}
    dt, released_at, p_rel = 0.01, None, None
    for k in range(400):
        t = k * dt
        grip = 0.5 <= t < 3.0
        poses, data = xr_stream(t, grip)
        for s, m in mappers.items():
            p, q = poses[s]
            if xr_pose_valid(p, q):                                      # quest_bridge._on_ee_poses 와 동일
                m.set_raw(se3_from_pq(p, q), t)
            m.set_engaged(bool(xr_engaged(data, XR_ENGAGE_KEYS[s])) and m.raw is not None
                          and (t - m.raw_time) < 1.0)
            target = m.update(t, core.hand_pose(s))
            if target is not None:
                core.set_hand_target(s, target)
            elif m.released:
                core.hold_hand(s); released_at = t
                p_rel = sim.body_pose_in_base(cfg["left_hand_frame"], base)[0].copy()
        sim.set_joint_targets(core.step(dt)); sim.step(5)
    exp = (0.15 * fwd + np.array([0.0, 0.0, 0.10])) * scale
    for s in ("left", "right"):
        p_now = sim.body_pose_in_base(cfg[f"{s}_hand_frame"], base)[0]
        err = np.linalg.norm((p_now - hand0[s]) - exp)
        print(f"[{name} yaw={yaw:.0f}°] {s}: Δ={np.round(p_now - hand0[s], 3)} expected {np.round(exp, 3)} → err {err*1000:.1f} mm")
        assert err < 0.02, f"{name} {s} tracking error"
    drift = np.linalg.norm(sim.body_pose_in_base(cfg["left_hand_frame"], base)[0] - p_rel)
    print(f"[{name}] released at {released_at:.2f} s, moved after release {drift*1000:.1f} mm, qp failures {core.qp_failures}")
    assert released_at is not None and abs(released_at - 3.0) < 0.05 and drift < 0.01 and core.qp_failures == 0


if __name__ == "__main__":
    if not G1 and not SH:
        print("G1_MJCF 또는 SH_REPO 환경변수를 지정하세요"); sys.exit(1)
    if G1:
        run("G1", build_g1)
    if SH:
        run("SH", build_sh)
    print("OK")
