#!/usr/bin/env python3
"""새 로봇 모델(URDF/MJCF)과 config 가 quest_arm_ik 에서 그대로 쓰이는지 ROS 없이 검사한다.

    python3 scripts/check_model.py --model /abs/samhyun_gen2.urdf --config config/dual_arm_ik.yaml \
        [--package-dirs /abs/ros_ws/src ...]   # URDF 메시 package:// 해석용

검사: (1) 관절·프레임 이름이 모델에 있는지, (2) Pinocchio FK = MuJoCo FK, (3) IK 수렴(양손 +15 cm 앞, +10 cm 위),
      (4) MuJoCo 추종, (5) 자기충돌 제약이 돌고 100 Hz 예산 안인지, (6) 관절 한계·nominal_posture 유효성.
삼현 2세대 URDF 를 받으면 이 스크립트가 OK 를 찍을 때까지 config 만 고치면 된다.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from quest_arm_ik.ik_core import DualArmIKCore, IKConfig  # noqa: E402
from quest_arm_ik.mujoco_sim import MujocoArmSim  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--config", default=os.path.join(os.path.dirname(__file__), "..", "config", "dual_arm_ik.yaml"))
    ap.add_argument("--package-dirs", nargs="*", default=[])
    ap.add_argument("--sim-model", default="", help="시뮬에 다른 파일을 쓸 때 (기본: --model)")
    ap.add_argument("--srdf", default="", help="MoveIt SRDF (disable_collisions 반영)")
    ap.add_argument("--xacro-args", default="", help='xacro 인자, 예 "hand:=box" (model 이 .xacro 일 때)')
    args = ap.parse_args()

    cfg = yaml.safe_load(open(args.config))
    ik = cfg["dual_arm_ik"]["ros__parameters"]
    sm = cfg.get("mujoco_sim", {}).get("ros__parameters", {})
    joints = list(ik["controlled_joints"])
    shapes = yaml.safe_load(ik.get("collision_shapes_yaml", "") or "") or []
    sim_shapes = yaml.safe_load(sm.get("collision_shapes_yaml", "") or "") or []
    pkg_dirs = list(args.package_dirs) + [d for d in ik.get("package_dirs", []) if d]
    hands = {"left": ik["left_hand_frame"], "right": ik["right_hand_frame"]}
    base = ik.get("base_frame", "")
    ok = True

    def check(cond, msg):
        nonlocal ok
        print(("  OK   " if cond else "  FAIL ") + msg)
        ok = ok and bool(cond)

    print(f"[1] 모델 로드: {args.model}")
    try:
        core = DualArmIKCore(IKConfig(
            model_path=args.model, controlled_joints=joints, left_hand_frame=hands["left"],
            right_hand_frame=hands["right"], base_frame=base, package_dirs=pkg_dirs,
            srdf_path=args.srdf or ik.get("srdf_path", ""), xacro_args=args.xacro_args or ik.get("xacro_args", ""),
            nominal_posture=ik.get("nominal_posture"), posture_cost=ik.get("posture_cost", 0.01),
            posture_weights=ik.get("posture_weights"), limit_margin=ik.get("limit_margin", 0.0),
            joint_limits=yaml.safe_load(ik["joint_limits_yaml"]) if ik.get("joint_limits_yaml") else None,
            orientation_cost=ik.get("orientation_cost", 0.3), task_gain=ik.get("task_gain", 0.5),
            max_joint_velocity=ik.get("max_joint_velocity", 2.0),
            enable_self_collision=bool(ik.get("enable_self_collision", False)),
            collision_d_min=ik.get("collision_d_min", 0.02), collision_gain=ik.get("collision_gain", 0.3),
            collision_activation=ik.get("collision_activation", 0.08),
            collision_shapes=shapes, collision_exclude_links=ik.get("collision_exclude_links", [])))
    except Exception as e:  # noqa: BLE001
        print(f"  FAIL IK 모델 로드 실패: {e}")
        return 1
    check(True, f"{len(joints)}개 관절, 손 프레임 {hands['left']} / {hands['right']}")
    check(not base or core.model.existFrame(base), f"base_frame '{base}' 존재")
    lo, hi = core.model.lowerPositionLimit, core.model.upperPositionLimit
    check(np.all(np.isfinite(lo)) and np.all(np.isfinite(hi)), f"관절 한계 유한함 (없으면 URDF <limit> 추가): "
          f"[{np.round(lo, 2).tolist()}] ~ [{np.round(hi, 2).tolist()}]")
    qn = core.q_nominal
    check(np.all(qn >= lo - 1e-9) and np.all(qn <= hi + 1e-9), "nominal_posture 가 한계 안")

    print("[2] MuJoCo 시뮬 로드")
    try:
        sim = MujocoArmSim(args.sim_model or args.model, joints, keyframe=sm.get("keyframe", ""),
                           fix_base=bool(sm.get("fix_base", True)), base_height=float(sm.get("base_height", -1.0)),
                           pd_kp=float(sm.get("pd_kp", 200.0)), pd_kd=float(sm.get("pd_kd", 5.0)),
                           package_dirs=pkg_dirs, collision_shapes=sim_shapes,
                           srdf_path=args.srdf or ik.get("srdf_path", ""), xacro_args=args.xacro_args or ik.get("xacro_args", ""))
    except Exception as e:  # noqa: BLE001
        print(f"  FAIL 시뮬 모델 로드 실패: {e}")
        return 1
    check(True, f"관절 {len(sim.jnt_id)}개 (제어 {len(joints)}, 고정 {len(sim.hold_joints)}), "
                f"free joint {'있음(공중 고정)' if sim.free_jnt >= 0 else '없음'}, dt={sim.dt}")
    init = [float(v) for v in sm.get("initial_posture", [])]
    if len(init) == len(joints):                                     # launch 와 같은 시작 자세 (준비 자세)
        for j, v in zip(joints, init):
            sim.data.qpos[sim.qadr[j]] = v
        sim.data.qvel[:] = 0.0
        sim.set_joint_targets(np.array(init))
        print(f"  INFO mujoco_sim.initial_posture 로 시작: {np.round(init, 2).tolist()}")
    else:
        sim.set_joint_targets(sim.joint_positions())
    sim.step(500)                                                    # 중력 정착
    core.reset_from_joint_positions(sim.joint_positions())           # 정착된 자세로 IK 동기화 → 같은 q 로 FK 비교
    import mujoco
    if getattr(sim, "excluded_pairs", None):
        print(f"  INFO 접촉 제외 body 쌍 {len(sim.excluded_pairs)}개: {sim.excluded_pairs[:6]}{' ...' if len(sim.excluded_pairs) > 6 else ''}")
    fwd = np.array(ik.get("forward_axis", [1.0, 0.0, 0.0]), dtype=float); fwd /= np.linalg.norm(fwd)
    up = np.array([0.0, 0.0, 1.0])
    root = base if base and mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, base) >= 0 else "world"
    q_cmp = core.joint_positions()                                   # 같은 q 에서 순수 기구학 비교 (중력 처짐·한계 클립 제외)
    q_start = q_cmp.copy()                                           # [4] 작업공간 검사용 시작 자세
    for side in ("left", "right"):
        p_pin = core.hand_pose(side).translation
        try:
            p_mj, _ = sim.fk_body_in_base(q_cmp, hands[side], root)
            d = np.linalg.norm(p_pin - p_mj)
            check(d < 0.002, f"{side} FK 일치 (Pinocchio vs MuJoCo, {root} 기준, 같은 q): {d*1000:.2f} mm")
        except ValueError as e:
            check(False, f"{side} 손 프레임이 MuJoCo body 가 아님 ({e}) — 손 링크가 fixed joint 로 붙어 있으면 URDF 로 넣어야 함")

    print(f"[3] IK 수렴 + 시뮬 추종 (양손 +15 cm 전방(forward_axis={fwd.round(2).tolist()}), +10 cm 상방, 3 s)")
    targets = {}
    for side in ("left", "right"):
        T = core.hand_pose(side); T.translation = T.translation + 0.15 * fwd + 0.10 * up
        targets[side] = T; core.set_hand_target(side, T)
    dt, times = 0.01, []
    for _ in range(300):
        t0 = time.perf_counter(); q = core.step(dt); times.append(time.perf_counter() - t0)
        sim.set_joint_targets(q); sim.step(max(1, int(round(dt / sim.dt))))
    for side in ("left", "right"):
        e_ik = np.linalg.norm(core.hand_pose(side).translation - targets[side].translation)
        check(e_ik < 0.01, f"{side} IK 잔차 {e_ik*1000:.1f} mm (10 mm 이상이면 도달 불가 목표/한계/posture_cost 확인)")
        try:
            p_mj, _ = sim.body_pose_in_base(hands[side], root)
            e_mj = np.linalg.norm(p_mj - targets[side].translation)
            check(e_mj < 0.02, f"{side} MuJoCo 손 오차 {e_mj*1000:.1f} mm (크면 pd_kp↑ 또는 질량/관성 확인)")
        except ValueError:
            pass
    check(np.mean(times) < 0.005, f"IK 평균 {np.mean(times)*1000:.2f} ms / 최대 {np.max(times)*1000:.2f} ms (100 Hz 예산 10 ms)")
    check(core.qp_failures == 0, f"QP 실패 {core.qp_failures}회")
    if core.collision_model is not None:
        check(True, f"자기충돌 쌍 {len(core.collision_model.collisionPairs)}개, 마지막 스텝 활성 {core.active_pairs}개")
    else:
        print("  INFO 자기충돌 비활성 (enable_self_collision: false 또는 충돌형상 없음)")

    print("[4] 작업공간 여유 — 시작 자세에서 양손을 위 / 앞·위 대각 / 앞 / 뒤 / 좌 / 우 / 아래 로 8 cm 씩 (IK 만, 각 2 s)")
    print("    '위' 와 '앞·위 대각'(팔을 들어 올리는 첫 동작) 은 반드시 닿아야 함. 나머지는 참고 — 팔을 내린 시작 자세에서는 아래·앞(같은 높이)·몸 쪽이 막히는 게 정상")
    left = np.cross(up, fwd)
    diag = (fwd + up) / np.linalg.norm(fwd + up)
    must = ("위", "앞·위")
    res = {"left": {}, "right": {}}
    for name, d in (("위", 0.08 * up), ("앞·위", 0.08 * diag), ("앞", 0.08 * fwd), ("뒤", -0.08 * fwd),
                    ("왼쪽", 0.08 * left), ("오른쪽", -0.08 * left), ("아래", -0.08 * up)):
        core.reset_from_joint_positions(q_start)
        goal = {}
        for side in ("left", "right"):
            T = core.hand_pose(side); T.translation = T.translation + d
            goal[side] = T.translation.copy(); core.set_hand_target(side, T)
        for _ in range(200):
            core.step(dt)
        for side in ("left", "right"):
            res[side][name] = np.linalg.norm(core.hand_pose(side).translation - goal[side])
    for side in ("left", "right"):
        e, name = max((res[side][n], n) for n in must)
        check(e < 0.015, f"{side} 필수 방향 중 최악 '{name}' 잔차 {e*1000:.1f} mm "
                         f"(15 mm 이상이면 시작 자세가 작업공간 가장자리/한계 — initial_posture 를 조정)")
        info = ", ".join(f"{n} {res[side][n]*1000:.0f}" for n in ("앞", "뒤", "왼쪽", "오른쪽", "아래"))
        print(f"  INFO {side} 참고 방향 잔차 [mm]: {info}")
    core.reset_from_joint_positions(q_start)

    print("\n결과:", "OK — 이 모델로 launch 해도 됨" if ok else "FAIL — 위 항목을 고친 뒤 다시 실행")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
