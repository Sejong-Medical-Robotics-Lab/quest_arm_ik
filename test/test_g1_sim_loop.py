"""G1(MuJoCo Menagerie) 모델로 IK → MuJoCo 시뮬 폐루프를 ROS 없이 검증.

    export G1_MJCF=/path/to/mujoco_menagerie/unitree_g1/g1.xml
    python3 test/test_g1_sim_loop.py

검사 항목
 1. pink IK가 MJCF를 읽어 14축 reduced model을 만들고 손 목표에 수렴하는지
 2. 그 관절 명령을 MuJoCo(position 액추에이터)에 넣었을 때 시뮬 손목이 같은 곳에 가는지
 3. 자기충돌 배리어 옵션이 G1 메시 충돌형상으로도 돌아가는지 + 100 Hz 예산 안인지
"""
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from quest_arm_ik.ik_core import DualArmIKCore, IKConfig  # noqa: E402
from quest_arm_ik.mujoco_sim import MujocoArmSim  # noqa: E402

MJCF = os.environ.get("G1_MJCF", "")
ARM_JOINTS = [f"{s}_{n}_joint" for s in ("left", "right")
              for n in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
                        "wrist_roll", "wrist_pitch", "wrist_yaw")]
NOMINAL = [-0.2, 0.25, 0.0, -0.6, 0.0, 0.0, 0.0,
           -0.2, -0.25, 0.0, -0.6, 0.0, 0.0, 0.0]   # config/dual_arm_ik.yaml 과 동일


def make_core(**kw):
    cfg = IKConfig(model_path=MJCF, controlled_joints=ARM_JOINTS,
                   left_hand_frame="left_wrist_yaw_link", right_hand_frame="right_wrist_yaw_link",
                   base_frame="pelvis", nominal_posture=NOMINAL, **kw)
    return DualArmIKCore(cfg)


def test_ik_and_sim_agree():
    core = make_core()
    sim = MujocoArmSim(MJCF, ARM_JOINTS, keyframe="stand", fix_base=True)
    # 시뮬 초기 자세(키프레임)로 IK 상태 동기화 — 실제 노드의 init_from_joint_states와 동일
    core.reset_from_joint_positions(sim.joint_positions())
    sim.set_joint_targets(core.joint_positions())
    sim.step(500)                                     # 1 s 정착

    # IK 모델과 MuJoCo의 FK가 같은지 (모델 파서 일치 확인)
    p_pin = core.hand_pose("left").translation
    p_mj, _ = sim.body_pose_in_base("left_wrist_yaw_link", "pelvis")
    print(f"FK check  pin={np.round(p_pin,3)} mujoco={np.round(p_mj,3)} diff={np.linalg.norm(p_pin-p_mj)*1000:.1f} mm")
    assert np.linalg.norm(p_pin - p_mj) < 0.01

    # 양손 목표: 앞으로 15 cm, 위로 10 cm
    targets = {}
    for side in ("left", "right"):
        T = core.hand_pose(side)
        T.translation = T.translation + np.array([0.15, 0.0, 0.10])
        targets[side] = T
        core.set_hand_target(side, T)

    dt = 0.01
    t_solve = []
    for i in range(300):                              # 3 s @ 100 Hz
        t0 = time.perf_counter()
        q_cmd = core.step(dt)
        t_solve.append(time.perf_counter() - t0)
        sim.set_joint_targets(q_cmd)
        sim.step(5)                                   # 5 × 2 ms = 10 ms
    print(f"IK solve time: mean {np.mean(t_solve)*1000:.2f} ms, max {np.max(t_solve)*1000:.2f} ms")

    for side in ("left", "right"):
        e_ik = np.linalg.norm(core.hand_pose(side).translation - targets[side].translation)
        p_mj, _ = sim.body_pose_in_base(f"{side}_wrist_yaw_link", "pelvis")
        e_mj = np.linalg.norm(p_mj - targets[side].translation)
        print(f"{side}: IK residual {e_ik*1000:.1f} mm, MuJoCo wrist error {e_mj*1000:.1f} mm")
        assert e_ik < 0.005, f"{side} IK did not converge"
        assert e_mj < 0.02, f"{side} sim did not track (gravity sag / gains?)"
    assert np.all(np.abs(core.joint_positions() - np.array(NOMINAL)) < 2.0)


def test_self_collision_barrier():
    core = make_core(enable_self_collision=True, collision_d_min=0.03)
    n_pairs = len(core.collision_model.collisionPairs)
    print(f"collision pairs after pruning: {n_pairs}")
    # 왼손을 몸통 안쪽으로 밀어 넣는 목표 → 배리어가 d_min 에서 세워야 함
    T = core.hand_pose("left")
    T.translation = np.array([0.05, 0.0, 0.15])
    core.set_hand_target("left", T)
    t_solve = []
    for _ in range(300):
        t0 = time.perf_counter()
        core.step(0.01)
        t_solve.append(time.perf_counter() - t0)
    import pinocchio as pin
    pin.computeDistances(core.model, core.configuration.data, core.collision_model,
                         core.configuration.collision_data, core.configuration.q)
    dmin = min(r.min_distance for r in core.configuration.collision_data.distanceResults)
    print(f"min distance {dmin*1000:.1f} mm (d_min=30), qp failures {core.qp_failures}, "
          f"solve mean {np.mean(t_solve)*1000:.2f} ms max {np.max(t_solve)*1000:.2f} ms")
    assert dmin > 0.02
    assert np.mean(t_solve) < 0.01, "too slow for 100 Hz"


if __name__ == "__main__":
    if not os.path.isfile(MJCF):
        print("G1_MJCF 환경변수에 g1.xml 경로를 지정하세요"); sys.exit(1)
    test_ik_and_sim_agree()
    test_self_collision_barrier()
    print("OK")
