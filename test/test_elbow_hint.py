"""팔꿈치 힌트 검증 (ROS 없이, SH 모델): 가상의 사람 팔(상완 0.30·전완+손 0.35 m, 어깨 고정)이 손을 움직이면
컨트롤러 회전 = 전완 방향 회전으로 두고, 매퍼(relative, trigger 클러치 — 손목 잠금·위치만 추종) → IK 로 보낸다.

    SH_REPO=/abs/SH-Humanoid python3 test/test_elbow_hint.py

검사: 같은 손 궤적에서 힌트 유무로 (1) 로봇 전완 방향과 사람 전완 방향의 각도 차가 줄어드는가(팔꿈치 들기: 45° → 21°), (2) 손 위치 오차는 거의 안 커지는가(+4 mm),
      (3) 매퍼: 그립 순간 forearm_dir = 로봇 전완 방향, 컨트롤러가 가로축으로 60° 돌면 forearm_dir 도 60°, 트리거 중엔 None, 놓으면 기준 재설정.
"""
from __future__ import annotations

import os
import sys

import numpy as np
import pinocchio as pin
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from quest_arm_ik.ik_core import DualArmIKCore, IKConfig  # noqa: E402
from quest_arm_ik.mapper import ArmMapper, MapperParams  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
SH = os.environ.get("SH_REPO", "")
L_UP, L_FORE = 0.30, 0.35          # 사람 팔 (어깨→팔꿈치, 팔꿈치→손바닥)


def human_arm(p_hand_rel: np.ndarray, swivel: float = 0.0):
    """어깨 원점, 손 위치(어깨 기준) → 2링크 팔꿈치 위치. swivel = 어깨–손 축 둘레로 팔꿈치를 돌린 각 [rad] (0 = 팔꿈치가 아래로 처짐,
    오른팔은 − 가 바깥쪽/위로 들림, + 는 몸 앞으로 모임). 반환 (팔꿈치, 전완 단위벡터)."""
    d = float(np.linalg.norm(p_hand_rel))
    d = min(d, L_UP + L_FORE - 1e-3)
    u = p_hand_rel / max(1e-9, np.linalg.norm(p_hand_rel))
    cos_a = (L_UP ** 2 + d ** 2 - L_FORE ** 2) / (2 * L_UP * d)   # 어깨 각
    a = np.arccos(np.clip(cos_a, -1.0, 1.0))
    down = np.array([0.0, 0.0, -1.0])
    n = np.cross(u, down); n = n / max(1e-9, np.linalg.norm(n))
    w = np.cross(n, u)                              # u 에 수직, 아래쪽
    w = np.cos(swivel) * w + np.sin(swivel) * np.cross(u, w)   # 어깨–손 축 둘레로 swivel 만큼 (오른팔: − 가 바깥쪽)
    elbow = L_UP * (np.cos(a) * u + np.sin(a) * w)
    fore = (u * d - elbow); fore /= max(1e-9, np.linalg.norm(fore))
    return elbow, fore


def rot_from_to(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """a → b 최소 회전 행렬."""
    a = a / np.linalg.norm(a); b = b / np.linalg.norm(b)
    v = np.cross(a, b); c = float(np.dot(a, b))
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3) if c > 0 else -np.eye(3)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * (1.0 / (1.0 + c))


def build(hint: float):
    cfg = yaml.safe_load(open(os.path.join(HERE, "..", "config", "sh_humanoid_v2.yaml")))["dual_arm_ik"]["ros__parameters"]
    xacro = os.path.join(SH, "urdf_moveit", "SH_Humanoid_v2_description", "urdf", "SH_Humanoid_v2.xacro")
    srdf = os.path.join(SH, "urdf_moveit", "SH_Humanoid_v2_moveit_config", "config", "SH_Humanoid_v2.srdf")
    core = DualArmIKCore(IKConfig(
        model_path=xacro, controlled_joints=cfg["controlled_joints"], left_hand_frame=cfg["left_hand_frame"], right_hand_frame=cfg["right_hand_frame"],
        base_frame=cfg["base_frame"], package_dirs=[os.path.join(SH, "urdf_moveit")], srdf_path=srdf, nominal_posture=cfg["nominal_posture"],
        posture_weights=cfg["posture_weights"], limit_margin=cfg["limit_margin"], joint_limits=yaml.safe_load(cfg["joint_limits_yaml"]),
        posture_cost=cfg["posture_cost"], orientation_cost=cfg["orientation_cost"], task_gain=cfg["task_gain"],
        max_joint_velocity=cfg["max_joint_velocity"], enable_self_collision=True, collision_d_min=cfg["collision_d_min"],
        collision_shapes=yaml.safe_load(cfg["collision_shapes_yaml"]), elbow_hint_cost=hint))
    return cfg, core


def run_motion(hint: float, scale: float = 0.75, swivel_case: bool = False):
    """사람 손: 내린 자세(어깨 아래 0.62 m) → 앞 0.25·위 0.35 (가슴 앞) → 옆 0.2 → 앞 0.4·위 0.1 (뻗기) → 내림. 각 2 s 이동 + 1 s 유지.
    swivel_case: 가슴 앞에서 손은 그대로 두고 팔꿈치만 바깥·위로 70° 들었다(2 s) 내린다 — 손 위치만으로는 구분 안 되는 동작."""
    cfg, core = build(hint)
    q0 = np.zeros(14); q0[[3, 10]] = -0.25
    core.reset_from_joint_positions(q0)
    for s in ("left", "right"):
        core.set_orientation_tracking(s, False); core.set_wrist_locked(s, True)
    mp = MapperParams(mode="relative", max_step=cfg["max_step"], scale=scale, input_timeout=0.5, orientation_clutch="trigger",
                      filter_min_cutoff=1e9, forearm_filter_alpha=1.0)
    m = ArmMapper("right", mp)
    p_hang = np.array([0.0, 0.0, -(L_UP + L_FORE - 0.03)])         # 사람 손 (어깨 기준), 거의 쭉 편 채 내림
    chest = p_hang + [0, 0.25, 0.35]
    if swivel_case:   # (손 위치, swivel) 경유점
        way = [(p_hang, 0.0), (chest, 0.0), (chest, np.radians(-70)), (chest, 0.0)]
    else:
        way = [(p_hang, 0.0), (chest, 0.0), (p_hang + [0.2, 0.25, 0.35], 0.0), (p_hang + [0, 0.40, 0.10], 0.0), (p_hang, 0.0)]
    dt = 0.01; t = 0.0
    errs_dir, errs_pos = [], []
    _, fore0 = human_arm(p_hang)
    for k in range(len(way) - 1):
        for i in range(300):
            a = min(1.0, i * dt / 2.0); a = a * a * (3 - 2 * a)
            p = way[k][0] + a * (way[k + 1][0] - way[k][0])
            sw = way[k][1] + a * (way[k + 1][1] - way[k][1])
            _, fore = human_arm(p, sw)
            R_ctrl = rot_from_to(fore0, fore)                           # 컨트롤러 회전 = 전완 방향 회전 (가로/세로 모두)
            m.set_raw(pin.SE3(R_ctrl, p + np.array([0.0, 0.0, 1.4])), t)   # 바닥 원점 가정 (어깨 높이 1.4)
            m.set_engaged(True)
            target = m.update(t, core.hand_pose("right"), core.forearm_dir("right"))
            core.set_hand_target("right", target)
            if hint > 0 and m.forearm_dir is not None:
                core.set_elbow_hint("right", target.translation, m.forearm_dir)
            core.step(dt); t += dt
            if i >= 200:                                                 # 유지 구간에서 평가
                rd = core.forearm_dir("right")
                errs_dir.append(np.degrees(np.arccos(np.clip(np.dot(rd, fore), -1, 1))))
                errs_pos.append(1000 * np.linalg.norm(core.hand_pose("right").translation - target.translation))
    return float(np.mean(errs_dir)), float(np.max(errs_dir)), float(np.mean(errs_pos)), float(np.max(errs_pos))


def test_mapper_forearm():
    m = ArmMapper("right", MapperParams(orientation_clutch="trigger", filter_min_cutoff=1e9, forearm_filter_alpha=1.0))
    hand = pin.SE3(np.eye(3), np.array([0.2, 0.0, 0.6]))
    down = np.array([0.0, 0.0, -1.0])
    m.set_raw(pin.SE3(np.eye(3), np.array([0, 0, 1.0])), 0.0); m.set_engaged(True)
    m.update(0.0, hand, down)
    assert np.allclose(m.forearm_dir, down), m.forearm_dir
    Rx60 = pin.exp3(np.array([np.radians(60), 0, 0]))                  # 가로축 60° (손끝이 앞으로)
    m.set_raw(pin.SE3(Rx60, np.array([0, 0.1, 1.0])), 0.01)
    m.update(0.01, hand, down)
    ang = np.degrees(np.arccos(np.clip(np.dot(m.forearm_dir, down), -1, 1)))
    assert abs(ang - 60) < 1e-6 and np.allclose(m.forearm_dir, Rx60 @ down), (ang, m.forearm_dir)
    m.set_orient_engaged(True); m.update(0.02, hand, Rx60 @ down)
    assert m.forearm_dir is None, "트리거 중엔 힌트 없음"
    Rx90 = pin.exp3(np.array([np.radians(90), 0, 0]))
    m.set_raw(pin.SE3(Rx90, np.array([0, 0.1, 1.0])), 0.03); m.update(0.03, hand, Rx60 @ down)
    m.set_orient_engaged(False); m.update(0.04, hand, Rx60 @ down)      # 놓는 순간: 기준 = 지금 로봇 전완(Rx60·down) ↔ 컨트롤러 Rx90
    assert np.allclose(m.forearm_dir, Rx60 @ down), m.forearm_dir
    m.set_raw(pin.SE3(pin.exp3(np.array([np.radians(120), 0, 0])), np.array([0, 0.1, 1.0])), 0.05); m.update(0.05, hand, Rx60 @ down)
    assert np.allclose(m.forearm_dir, Rx90 @ down, atol=1e-6), m.forearm_dir   # 30° 더 돌림 → 60+30
    print("mapper forearm_dir: OK")


if __name__ == "__main__":
    test_mapper_forearm()
    if not SH:
        print("SH_REPO 미지정 — 모델 시험 생략"); sys.exit(0)
    for case, name in ((False, "앞·위·옆·뻗기"), (True, "가슴 앞에서 팔꿈치만 70° 들기")):
        off = run_motion(0.0, swivel_case=case)
        on = run_motion(0.5, swivel_case=case)
        print(f"[{name}] 힌트 없음: 전완 방향 오차 평균 {off[0]:.1f}° 최대 {off[1]:.1f}°, 손 오차 평균 {off[2]:.1f} / 최대 {off[3]:.1f} mm")
        print(f"[{name}] 힌트 0.5 : 전완 방향 오차 평균 {on[0]:.1f}° 최대 {on[1]:.1f}°, 손 오차 평균 {on[2]:.1f} / 최대 {on[3]:.1f} mm")
        assert on[0] <= off[0] + 1.0, "힌트가 전완 방향을 나쁘게 함"
        assert on[2] <= off[2] + 5.0, "힌트가 손 위치를 밀어냄"   # 가중 0.5: 팔꿈치 들기에서 +3.7 mm
        if case:
            assert on[0] < off[0] * 0.6, "팔꿈치 들기에서 힌트 효과가 충분하지 않음"
    print("elbow hint: OK")
