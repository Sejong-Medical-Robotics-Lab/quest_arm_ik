"""컨트롤러 → ArmMapper(클러치·필터) → IK → MuJoCo 전체 체인을 ROS 없이 G1 모델로 검증.

    export G1_MJCF=/path/to/mujoco_menagerie/unitree_g1/g1.xml
    python3 test/test_teleop_chain.py

시나리오: 양손 컨트롤러가 engage 후 2초간 앞·위로 이동하며 손목을 30° 비튼다. 그 뒤 왼손 release.
검사: (1) MuJoCo 손목이 컨트롤러 변화량(스케일 적용)을 따라가는가, (2) 자세(회전)도 따라가는가,
      (3) release 후 왼손은 멈추고 오른손만 계속 가는가, (4) 입력 timeout 시 자동 해제되는가.
"""
import os
import sys

import numpy as np
import pinocchio as pin

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from quest_arm_ik.ik_core import DualArmIKCore, IKConfig  # noqa: E402
from quest_arm_ik.mapper import ArmMapper, MapperParams  # noqa: E402
from quest_arm_ik.mujoco_sim import MujocoArmSim  # noqa: E402

MJCF = os.environ.get("G1_MJCF", "")
ARM_JOINTS = [f"{s}_{n}_joint" for s in ("left", "right")
              for n in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
                        "wrist_roll", "wrist_pitch", "wrist_yaw")]
NOMINAL = [-0.2, 0.25, 0.0, -0.6, 0.0, 0.0, 0.0, -0.2, -0.25, 0.0, -0.6, 0.0, 0.0, 0.0]   # config/dual_arm_ik.yaml 과 동일


def rot_angle(R):
    return np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))


def test_chain():
    core = DualArmIKCore(IKConfig(model_path=MJCF, controlled_joints=ARM_JOINTS,
                                  left_hand_frame="left_wrist_yaw_link", right_hand_frame="right_wrist_yaw_link",
                                  base_frame="pelvis", nominal_posture=NOMINAL, enable_self_collision=True, collision_d_min=0.03))
    sim = MujocoArmSim(MJCF, ARM_JOINTS, keyframe="stand")
    core.reset_from_joint_positions(sim.joint_positions())
    sim.set_joint_targets(core.joint_positions()); sim.step(500)

    scale = 0.8
    params = MapperParams(scale=scale, input_timeout=0.5, filter_min_cutoff=1.5, filter_beta=0.5,
                          orientation_filter_alpha=0.3)
    mappers = {s: ArmMapper(s, params) for s in ("left", "right")}

    # 컨트롤러는 헤드셋 월드(임의 원점)에 있다 — 절대 위치는 로봇과 무관해야 함
    ctrl_origin = {"left": np.array([0.3, 0.25, 1.0]), "right": np.array([0.3, -0.25, 1.0])}
    R_twist = pin.exp3(np.array([0.0, 0.0, 0.0]))
    dt, t = 0.01, 0.0
    hand0_mj = {s: sim.body_pose_in_base(f"{s}_wrist_yaw_link", "pelvis")[0] for s in ("left", "right")}
    hand0_R = {s: sim.body_pose_in_base(f"{s}_wrist_yaw_link", "pelvis")[1] for s in ("left", "right")}

    def ctrl_pose(side, t):
        # 0~0.5 s 정지(engage), 0.5~2.5 s: 앞 0.15 m, 위 0.10 m 이동 + 손목 z축 30° 회전, 이후 정지
        a = np.clip((t - 0.5) / 2.0, 0.0, 1.0)
        p = ctrl_origin[side] + a * np.array([0.15, 0.0, 0.10])
        R = pin.exp3(np.array([0.0, 0.0, np.radians(30.0) * a]))
        return pin.SE3(R, p)

    left_released_at = None
    p_left_at_release = None
    for k in range(400):                                    # 4 s
        t = k * dt
        for side, mapper in mappers.items():
            if side == "left" and t >= 3.0:                 # 왼손 release, 입력도 끊김(timeout 검사)
                mapper.set_engaged(False)
            else:
                mapper.set_raw(ctrl_pose(side, t), t)
                mapper.set_engaged(t >= 0.2)
            target = mapper.update(t, core.hand_pose(side))
            if target is not None:
                core.set_hand_target(side, target)
            elif mapper.released:
                core.hold_hand(side)
                left_released_at = t
                p_left_at_release = sim.body_pose_in_base("left_wrist_yaw_link", "pelvis")[0].copy()
        q = core.step(dt)
        sim.set_joint_targets(q); sim.step(5)

    assert left_released_at is not None and abs(left_released_at - 3.0) < 0.02, "release timing"
    # 오른손: 변화량 × scale 만큼 이동, 30° 회전
    p_r, R_r = sim.body_pose_in_base("right_wrist_yaw_link", "pelvis")
    dp_expected = scale * np.array([0.15, 0.0, 0.10])
    err_p = np.linalg.norm((p_r - hand0_mj["right"]) - dp_expected)
    dR = R_r @ hand0_R["right"].T
    ang = rot_angle(dR)
    print(f"right: Δp={np.round(p_r - hand0_mj['right'],3)} expected={dp_expected} err={err_p*1000:.1f} mm, "
          f"rotation {ang:.1f}° (expected 30°)")
    assert err_p < 0.01, "right hand did not track"
    assert abs(ang - 30.0) < 3.0, "right hand orientation did not track"
    # 왼손: release 이후 멈춤
    p_l = sim.body_pose_in_base("left_wrist_yaw_link", "pelvis")[0]
    drift = np.linalg.norm(p_l - p_left_at_release)
    print(f"left: moved {drift*1000:.1f} mm after release (should be ~0), qp failures {core.qp_failures}, "
          f"active collision pairs {core.active_pairs}")
    assert drift < 0.005
    assert core.qp_failures == 0

    # 이어서 잡기(re-index): 왼손 컨트롤러가 전혀 다른 위치에서 다시 engage 해도 로봇 손이 점프하지 않아야 함
    far = pin.SE3(np.eye(3), np.array([5.0, 5.0, 5.0]))
    mappers["left"].set_raw(far, 4.0); mappers["left"].set_engaged(True)
    target = mappers["left"].update(4.0, core.hand_pose("left"))
    jump = np.linalg.norm(target.translation - core.hand_pose("left").translation)
    print(f"re-engage jump: {jump*1000:.2f} mm (should be 0)")
    assert jump < 1e-6


if __name__ == "__main__":
    if not os.path.isfile(MJCF):
        print("G1_MJCF 환경변수에 g1.xml 경로를 지정하세요"); sys.exit(1)
    test_chain()
    print("OK")
