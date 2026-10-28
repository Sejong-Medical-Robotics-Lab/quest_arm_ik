"""회전 클러치(orientation_clutch: trigger) 오프라인 검증 — ROS 없이.

    SH_REPO=/abs/SH-Humanoid python3 test/test_orient_clutch.py

1) ArmMapper: 트리거를 당기기 전엔 컨트롤러를 돌려도 손 목표 회전이 현재 손 자세(= 변화 없음), 위치는 추종.
   당긴 순간이 회전 기준 — 그 뒤의 회전 변화만 반영. 그립을 놓으면 회전 추종도 끝.
2) DualArmIKCore(SH 모델): 손목 잠금 + 회전 cost 0 이면 손을 앞·위로 20 cm 옮겨도 손목 3관절이 움직이지 않고(≤0.01°),
   잠금을 풀고 회전 추종을 켜면 컨트롤러 회전(가로축 30°)을 손목이 따라간다.
"""
from __future__ import annotations

import os
import sys

import numpy as np
import pinocchio as pin

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from quest_arm_ik.mapper import ArmMapper, MapperParams  # noqa: E402


def se3(p, R=None):
    return pin.SE3(np.eye(3) if R is None else R, np.asarray(p, dtype=float))


def test_mapper_trigger_clutch():
    m = ArmMapper("right", MapperParams(orientation_clutch="trigger", filter_min_cutoff=1e9, orientation_filter_alpha=1.0))
    hand = se3([0.2, 0.0, 0.6])
    t = 0.0
    m.set_raw(se3([0, 0, 1.0]), t); m.set_engaged(True)
    T = m.update(t, hand)
    assert T is not None and m.engaged and not m.orient_engaged and m.orient_changed is False
    # 트리거 안 당긴 채 컨트롤러를 30° 돌리고 10 cm 앞으로 → 위치만 따라가고 회전은 현재 손 자세
    Rz30 = pin.exp3(np.array([0, 0, np.radians(30)]))
    t += 0.01; m.set_raw(se3([0.10, 0, 1.0], Rz30), t)
    T = m.update(t, hand)
    assert np.allclose(T.translation, [0.2, 0.10, 0.6], atol=1e-9) or np.allclose(T.translation, [0.30, 0.0, 0.6], atol=1e-9)
    assert np.allclose(T.rotation, hand.rotation), "트리거 전엔 회전 변화 없음"
    # 트리거 당김 → 이 순간이 기준: 바로는 회전 변화 없음
    t += 0.01; m.set_orient_engaged(True)
    T = m.update(t, hand)
    assert m.orient_engaged and m.orient_changed
    assert np.allclose(T.rotation, hand.rotation, atol=1e-9), "당긴 순간은 기준이라 회전 변화 0"
    # 당긴 채 20° 더 돌림 → 손 목표도 20° (기준 대비 변화량)
    Rz50 = pin.exp3(np.array([0, 0, np.radians(50)]))
    t += 0.01; m.set_raw(se3([0.10, 0, 1.0], Rz50), t)
    T = m.update(t, hand)
    ang = np.degrees(np.linalg.norm(pin.log3(hand.rotation.T @ T.rotation)))
    assert abs(ang - 20.0) < 1e-6, ang
    assert not m.orient_changed
    # 트리거 놓음 → 회전 추종 끝, 위치는 계속
    t += 0.01; m.set_orient_engaged(False); m.set_raw(se3([0.15, 0, 1.0], Rz50), t)
    T = m.update(t, hand)
    assert not m.orient_engaged and m.orient_changed
    assert np.allclose(T.rotation, hand.rotation)
    # 그립 놓음 → None, released
    t += 0.01; m.set_engaged(False)
    assert m.update(t, hand) is None and m.released
    # always 모드는 그립 순간이 기준 (기존 동작)
    a = ArmMapper("right", MapperParams(orientation_clutch="always", filter_min_cutoff=1e9, orientation_filter_alpha=1.0))
    a.set_raw(se3([0, 0, 1.0]), 0.0); a.set_engaged(True); a.update(0.0, hand)
    assert a.orient_engaged and a.orient_changed
    a.set_raw(se3([0, 0, 1.0], Rz30), 0.01); T = a.update(0.01, hand)
    assert abs(np.degrees(np.linalg.norm(pin.log3(hand.rotation.T @ T.rotation))) - 30.0) < 1e-6
    print("mapper trigger clutch: OK")


def test_core_wrist_lock():
    repo = os.environ.get("SH_REPO", "")
    if not repo:
        print("SH_REPO 미지정 — 코어 손목 잠금 시험 생략")
        return
    import yaml
    from quest_arm_ik.ik_core import DualArmIKCore, IKConfig
    xacro = os.path.join(repo, "urdf_moveit", "SH_Humanoid_v2_description", "urdf", "SH_Humanoid_v2.xacro")
    srdf = os.path.join(repo, "urdf_moveit", "SH_Humanoid_v2_moveit_config", "config", "SH_Humanoid_v2.srdf")
    cfg = yaml.safe_load(open(os.path.join(os.path.dirname(__file__), "..", "config", "sh_humanoid_v2.yaml")))["dual_arm_ik"]["ros__parameters"]
    core = DualArmIKCore(IKConfig(
        model_path=xacro, controlled_joints=cfg["controlled_joints"], left_hand_frame=cfg["left_hand_frame"],
        right_hand_frame=cfg["right_hand_frame"], base_frame=cfg["base_frame"], package_dirs=[os.path.join(repo, "urdf_moveit")],
        srdf_path=srdf, nominal_posture=cfg["nominal_posture"], posture_weights=cfg["posture_weights"], limit_margin=cfg["limit_margin"],
        joint_limits=yaml.safe_load(cfg["joint_limits_yaml"]), posture_cost=cfg["posture_cost"], orientation_cost=cfg["orientation_cost"],
        task_gain=cfg["task_gain"], max_joint_velocity=cfg["max_joint_velocity"], enable_self_collision=True,
        collision_d_min=cfg["collision_d_min"], collision_shapes=yaml.safe_load(cfg["collision_shapes_yaml"])))
    J = cfg["controlled_joints"]
    wr = [J.index(j) for j in J if j.startswith("right_") and "wrist" in j]
    q0 = np.zeros(14); q0[[3, 10]] = -0.25
    core.reset_from_joint_positions(q0)
    for s in ("left", "right"):
        core.set_orientation_tracking(s, False); core.set_wrist_locked(s, True)
    h0 = core.hand_pose("right").copy()
    dt = 0.01
    for k in range(300):                       # 3 s: 앞 20 cm·위 20 cm (손 회전 목표는 무시되어야 함)
        a = min(1.0, k * dt / 2.0)
        T = h0.copy(); T.translation = h0.translation + a * np.array([0.0, 0.20, 0.20])
        T.rotation = pin.exp3(np.array([np.radians(40.0) * a, 0, 0])) @ h0.rotation     # 일부러 회전도 줌
        core.set_hand_target("right", T)
        core.step(dt)
    q = core.joint_positions()
    dwr = np.degrees(np.abs(q[wr] - q0[wr]))
    err = 1000 * np.linalg.norm(core.hand_pose("right").translation - T.translation)
    print(f"손목 잠금: 손목 Δ = {dwr.round(4)}°, 손 위치오차 {err:.1f} mm, 팔꿈치 {np.degrees(q[10]):+.1f}°")
    assert dwr.max() < 0.01, "잠긴 손목이 움직임"
    assert err < 15.0, "잠근 채 위치 추종 실패"
    # 잠금 해제 + 회전 추종 → 가로축 30° 회전을 손목이 따라감
    core.set_orientation_tracking("right", True); core.set_wrist_locked("right", False)
    h1 = core.hand_pose("right").copy(); q1 = core.joint_positions()
    for k in range(300):
        a = min(1.0, k * dt / 2.0)
        T = h1.copy(); T.rotation = pin.exp3(np.array([np.radians(30.0) * a, 0, 0])) @ h1.rotation
        core.set_hand_target("right", T)
        core.step(dt)
    q2 = core.joint_positions()
    ang = np.degrees(np.linalg.norm(pin.log3(T.rotation.T @ core.hand_pose("right").rotation)))
    dwr2 = np.degrees(np.abs(q2[wr] - q1[wr]))
    print(f"잠금 해제: 손 회전 오차 {ang:.1f}°, 손목 Δ = {dwr2.round(1)}°, 위치오차 {1000*np.linalg.norm(core.hand_pose('right').translation - T.translation):.1f} mm")
    assert ang < 5.0 and dwr2.max() > 10.0
    # 양쪽 잠금 해제 → 원래 속도 한계 복원 여부
    core.set_wrist_locked("right", False); core.set_wrist_locked("left", False)
    vl = core.configuration.model.velocity_limit.velocity_limit
    assert np.allclose(vl, core._vl_orig)
    core.set_wrist_locked("left", True)
    assert vl[core._wrist_v_index["left"]].max() < 1e-5 and np.allclose(vl[core._wrist_v_index["right"]], core._vl_orig[core._wrist_v_index["right"]])
    print("core wrist lock: OK")


if __name__ == "__main__":
    test_mapper_trigger_clutch()
    test_core_wrist_lock()
    print("ALL OK")
