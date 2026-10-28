"""ROS 없이 IK 코어만 검증하는 스모크 테스트 겸 사용 예시.

    python3 -m pytest test/test_ik_core.py -q      # 또는
    python3 test/test_ik_core.py

간단한 몸통 + 양팔 7축(어깨 P/R/Y, 팔꿈치, 손목 R/P/Y) URDF를 생성해
목표를 5 cm 앞으로 옮겼을 때 손이 따라가는지 확인한다.
"""
import os
import sys
import tempfile

import numpy as np
import pinocchio as pin

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from quest_arm_ik.ik_core import DualArmIKCore, IKConfig  # noqa: E402

ARM_AXES = [("shoulder_pitch", "0 1 0"), ("shoulder_roll", "1 0 0"), ("shoulder_yaw", "0 0 1"),
            ("elbow", "0 1 0"), ("wrist_roll", "1 0 0"), ("wrist_pitch", "0 1 0"), ("wrist_yaw", "0 0 1")]
# 각 관절 원점의 이전 링크 길이(상완 0.25 m → 팔꿈치, 전완 0.22 m → 손목), 손목 3축은 한 점에서 교차
OFFSET_BEFORE = [0.0, 0.0, 0.0, 0.25, 0.22, 0.0, 0.0]
HAND_OFFSET = 0.05                                 # 손목 → 손 프레임(고정 관절)


def make_dual_arm_urdf(path: str) -> None:
    links = ['<link name="torso_link"/>']
    joints = []
    for side, y in (("left", 0.15), ("right", -0.15)):
        parent = "torso_link"
        origin = f"0 {y} 0.3"
        for (name, axis), offset in zip(ARM_AXES, OFFSET_BEFORE):
            child = f"{side}_{name}_link"
            if offset > 0.0:
                origin = f"0 0 {-offset}"
            links.append(f'<link name="{child}"/>')
            joints.append(
                f'<joint name="{side}_{name}_joint" type="revolute">'
                f'<parent link="{parent}"/><child link="{child}"/>'
                f'<origin xyz="{origin}" rpy="0 0 0"/><axis xyz="{axis}"/>'
                f'<limit lower="-2.5" upper="2.5" effort="30" velocity="3"/></joint>')
            parent, origin = child, "0 0 0"
        links.append(f'<link name="{side}_hand_link"/>')
        joints.append(
            f'<joint name="{side}_hand_joint" type="fixed"><parent link="{parent}"/>'
            f'<child link="{side}_hand_link"/><origin xyz="0 0 {-HAND_OFFSET}" rpy="0 0 0"/></joint>')
    with open(path, "w") as f:
        f.write('<robot name="dual_arm">' + "".join(links + joints) + "</robot>")


def controlled_joints():
    return [f"{s}_{n}_joint" for s in ("left", "right") for n, _ in ARM_AXES]


def test_tracks_target():
    with tempfile.TemporaryDirectory() as d:
        urdf = os.path.join(d, "dual_arm.urdf")
        make_dual_arm_urdf(urdf)
        cfg = IKConfig(model_path=urdf, controlled_joints=controlled_joints(),
                       left_hand_frame="left_hand_link", right_hand_frame="right_hand_link",
                       nominal_posture=[0.3, 0.2, 0.0, -0.8, 0.0, 0.0, 0.0,
                                        0.3, -0.2, 0.0, -0.8, 0.0, 0.0, 0.0])
        core = DualArmIKCore(cfg)
        dt = 0.01
        targets = {}
        for side in ("left", "right"):
            T = core.hand_pose(side)
            T.translation = T.translation + np.array([0.05, 0.0, 0.03])
            targets[side] = T
            core.set_hand_target(side, T)

        err0 = {s: np.linalg.norm(core.hand_pose(s).translation - targets[s].translation) for s in targets}
        for _ in range(300):
            q = core.step(dt)
        assert q.shape == (14,)
        for side in targets:
            err = np.linalg.norm(core.hand_pose(side).translation - targets[side].translation)
            print(f"{side}: pos err {err0[side]*1000:.1f} mm -> {err*1000:.2f} mm")
            assert err < 0.003, f"{side} hand did not converge: {err}"
        # 자세 태스크 덕에 팔꿈치가 nominal 근처에 머무는지(발산하지 않는지) 확인
        assert np.all(np.abs(q) < 2.5)


if __name__ == "__main__":
    test_tracks_target()
    print("OK")
