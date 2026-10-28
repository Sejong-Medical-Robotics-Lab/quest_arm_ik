"""손 쥐기(트리거 → 손가락 관절) 체인, ROS 없이.

    python3 test/test_grasp.py                                        # GraspMapper 만 (모델 불필요)
    SH_REPO=/abs/SH-Humanoid python3 test/test_grasp.py               # + 삼현 Dex3 (dex3_fingers:=revolute) MuJoCo 에서 쥐기/펴기
    G1H_MJCF=/abs/mujoco_menagerie/unitree_g1/g1_with_hands.xml python3 test/test_grasp.py   # + G1 손 포함 모델
검사: 매핑 범위·부드러움, 시뮬 손가락 관절이 1.5 s 안에 쥠 각의 90 % 이상 도달, 펴면 0 으로 복귀, 팔 관절은 그대로.
"""
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from quest_arm_ik.hand import DEX3_GRASP, GraspMapper, grasp_targets  # noqa: E402

HERE = os.path.dirname(__file__)
SH = os.environ.get("SH_REPO", "")
G1H = os.environ.get("G1H_MJCF", "")


def test_mapper():
    gm = GraspMapper(tau=0.1)
    assert len(gm.joint_names) == 14
    t = grasp_targets(DEX3_GRASP, {"left": 1.0, "right": 0.0})
    assert abs(t["left_hand_index_0_joint"] - (-1.3)) < 1e-9 and abs(t["right_hand_index_0_joint"] - 0.2) < 1e-9
    t = grasp_targets(DEX3_GRASP, {"left": 2.0, "right": -1.0})            # 범위 밖 입력은 0~1 로 클립
    assert abs(t["left_hand_thumb_2_joint"] - 1.2) < 1e-9 and abs(t["right_hand_thumb_2_joint"] - (-0.4)) < 1e-9
    gm.set("left", 1.0)
    vals = [gm.update(0.02)["left_hand_index_1_joint"] for _ in range(50)]      # 1 s
    assert vals[0] > -0.6 and vals[-1] < -1.49, vals[-1]                      # 저역통과: 첫 스텝은 20 %, 1 s 뒤 거의 도달
    assert all(b <= a + 1e-12 for a, b in zip(vals, vals[1:]))                # 단조
    print("mapper OK")


def run_sim(name, sim, joints_cfg):
    from quest_arm_ik.mujoco_sim import MujocoArmSim  # noqa: F401
    gm = GraspMapper(tau=0.05)
    fingers = [j for j in gm.joint_names if j in sim.hold_joints]
    assert len(fingers) == 14, f"{name}: 손가락 관절 {len(fingers)}개 (14 기대) — dex3_fingers:=revolute / g1_with_hands 인지"
    q_arm0 = sim.joint_positions().copy()
    gm.set("left", 1.0); gm.set("right", 1.0)
    for _ in range(75):                                                      # 1.5 s @ 20 ms
        t = gm.update(0.02)
        n = sim.set_extra_targets(list(t.keys()), list(t.values()))
        assert n == 14
        sim.step(10)
    closed = grasp_targets(DEX3_GRASP, {"left": 1.0, "right": 1.0})
    worst = max(abs(sim.data.qpos[sim.qadr[j]] - closed[j]) for j in fingers)
    print(f"[{name}] closed: worst joint error {worst:.3f} rad")
    assert worst < 0.1
    gm.set("left", 0.0); gm.set("right", 0.0)
    for _ in range(75):
        t = gm.update(0.02); sim.set_extra_targets(list(t.keys()), list(t.values())); sim.step(10)
    opened = grasp_targets(DEX3_GRASP, {"left": 0.0, "right": 0.0})
    worst = max(abs(sim.data.qpos[sim.qadr[j]] - opened[j]) for j in fingers)
    print(f"[{name}] open: worst joint error {worst:.3f} rad, arm drift {np.abs(sim.joint_positions() - q_arm0).max()*1000:.1f} mrad")
    assert worst < 0.1 and np.abs(sim.joint_positions() - q_arm0).max() < 0.02


if __name__ == "__main__":
    test_mapper()
    if SH:
        from quest_arm_ik.mujoco_sim import MujocoArmSim
        cfg = yaml.safe_load(open(os.path.join(HERE, "..", "config", "sh_humanoid_v2.yaml")))
        ik = cfg["dual_arm_ik"]["ros__parameters"]; sm = cfg["mujoco_sim"]["ros__parameters"]
        xacro = os.path.join(SH, "urdf_moveit", "SH_Humanoid_v2_description", "urdf", "SH_Humanoid_v2.xacro")
        sim = MujocoArmSim(xacro, ik["controlled_joints"], keyframe="", pd_kp=300.0, pd_kd=10.0, package_dirs=[os.path.join(SH, "urdf_moveit")],
                           collision_shapes=yaml.safe_load(sm["collision_shapes_yaml"]), xacro_args="dex3_fingers:=revolute")
        q0 = np.array(sm["initial_posture"])
        for j, v in zip(ik["controlled_joints"], q0):
            sim.data.qpos[sim.qadr[j]] = v
        sim.set_joint_targets(q0); sim.step(300)
        run_sim("SH+Dex3", sim, ik)
    if G1H:
        from quest_arm_ik.mujoco_sim import MujocoArmSim
        cfg = yaml.safe_load(open(os.path.join(HERE, "..", "config", "dual_arm_ik.yaml")))
        ik = cfg["dual_arm_ik"]["ros__parameters"]; sm = cfg["mujoco_sim"]["ros__parameters"]
        sim = MujocoArmSim(G1H, ik["controlled_joints"], keyframe="stand", fix_base=True)
        q0 = np.array(sm["initial_posture"])
        for j, v in zip(ik["controlled_joints"], q0):
            sim.data.qpos[sim.qadr[j]] = v
        sim.set_joint_targets(q0); sim.step(300)
        run_sim("G1 with hands", sim, ik)
    print("OK")
