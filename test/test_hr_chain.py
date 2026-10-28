"""실기체(삼현 HRController) 경로 리허설 — ROS 없이, 가짜 rclpy(test/fakeros) 로 모든 노드를 한 프로세스에서:

    SH_REPO=$SH $QPY test/test_hr_chain.py          # 약 30 s (가짜 시계로 25 s 분량)

   quest_mock → quest_bridge → dual_arm_ik(output_mode joint_state, 측정값=/upper_body/motor_state)
   → upper_body_streamer(250 Hz UpperBodyCommand) → mujoco_sim HRController 에뮬레이션 → /upper_body/motor_state → (IK, streamer)
검사: (1) 명령률 ≈250 Hz, motor_state ≈250 Hz, IK ≈100 Hz  (2) 첫 명령 = 측정값(점프 없음)  (3) 연속 명령 간 변화 ≤ v_max·dt
      (4) 추종: 시뮬 관절이 IK 목표를 따라감(정상 상태 오차 작음), streamer TRACK 유지  (5) 손(mock 원운동)이 실제로 움직임
      (6) IK 가 끊기면 0.2 s 안에 HOLD(ik timeout) 로 가고 명령이 멈춤  (7) IK 복귀 시 점프 없이 다시 추종
      (8) 15번(extra) 모터는 측정 위치 유지 명령으로 채워짐, 빈 슬롯 motor_id 0.
"""
import importlib.util
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SH = os.environ.get("SH_REPO", "")
CONFIG = os.path.join(ROOT, "config", "sh_humanoid_v2.yaml")
SH_XACRO = os.path.join(SH, "urdf_moveit", "SH_Humanoid_v2_description", "urdf", "SH_Humanoid_v2.xacro")


def _setup_fake_ros():
    """가짜 rclpy 를 실제 rclpy 보다 앞에 끼우고, 노드들이 쓰는 time.monotonic() 을 가짜 ROS 시계에 묶는다 (실시간보다 빨리 돌려도 타임아웃 논리 일관)."""
    sys.path.insert(0, os.path.join(HERE, "fakeros"))
    sys.path.insert(0, ROOT)
    import rclpy  # noqa: F401  (가짜)
    time.monotonic = rclpy.now
    time.sleep = lambda s: None
    return rclpy


def launch_params(**overrides):
    """teleop.launch.py 의 _setup 을 그대로 실행해 노드별 파라미터(config + launch 가 덧붙인 값)를 얻는다."""
    import yaml
    spec = importlib.util.spec_from_file_location("tl", os.path.join(ROOT, "launch", "teleop.launch.py"))
    m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
    ld = m.generate_launch_description()
    defaults = {e.name: e.default for e in ld.entities if hasattr(e, "name")}
    setup = [e for e in ld.entities if hasattr(e, "function")][0].function

    class Ctx:
        def __init__(self, **o): self.launch_configurations = {**defaults, **o}
    cfg = yaml.safe_load(open(CONFIG))
    out = {}
    for n in setup(Ctx(**overrides)):
        name = n.kw.get("name") or n.kw["executable"]
        params = {}
        for p in n.kw["parameters"]:
            if isinstance(p, str):
                params.update(cfg.get(name, {}).get("ros__parameters", {}))
            else:
                params.update(p)
        out[name] = params
    return out


def main():
    if not SH:
        print("SH_REPO=/abs/SH-Humanoid 가 필요 (모델 + integration/ros2_interfaces) — 건너뜀"); return
    rclpy = _setup_fake_ros()
    import numpy as np
    rclpy.reset()
    params = launch_params(robot="sh", model_path=SH_XACRO, package_dirs=os.path.dirname(os.path.dirname(os.path.dirname(SH_XACRO))), input="mock", hr="true", sim="true", viewer="false")
    params["quest_mock"].update({"engage_after": 1.0, "period": 6.0, "radius": 0.10})
    rclpy.PARAM_OVERRIDES.update(params)
    rclpy.init()

    from quest_arm_ik.quest_bridge import QuestBridge
    from quest_arm_ik.dual_arm_ik_node import DualArmIKNode
    from quest_arm_ik.upper_body_streamer_node import UpperBodyStreamer
    from quest_arm_ik.hand_grasp_node import HandGraspNode
    from quest_arm_ik.quest_mock import QuestMock
    from quest_arm_ik.mujoco_sim import make_node_class
    from ros2_interfaces.srv import EnableMotors

    sim_node = make_node_class()()
    nodes = [QuestBridge(), DualArmIKNode(), UpperBodyStreamer(), HandGraspNode(), QuestMock(), sim_node]
    bridge, ik, streamer, grasp, mock = nodes[:5]
    sim = sim_node.sim

    # 관찰용 구독 (가짜 rclpy 라 노드 없이도 등록 가능)
    cmds, states, iks = [], [], []
    rclpy._subs.setdefault("/upper_body/command", []).append(
        (lambda m: cmds.append((rclpy.now(), streamer.map.joints_from_state(m.motor_id, m.position), list(m.motor_id), m)), None))   # 관절각으로
    rclpy._subs.setdefault("/upper_body/motor_states", []).append(
        (lambda m: states.append((rclpy.now(), streamer.map.joints_from_state(m.motor_id, m.position))), None))   # 모터각 → 관절각(부호·오프셋 반영)
    rclpy._subs.setdefault("/teleop/joint_targets", []).append((lambda m: iks.append((rclpy.now(), np.array(m.position[:14]))), None))

    def run(seconds):
        n = int(round(seconds / 0.001))
        for _ in range(n):
            rclpy.advance(0.001)
            with sim.lock:
                behind = int(round((rclpy.now() - sim.data.time) / sim.dt))
                if behind > 0:
                    sim.step(behind)

    # (0) 드라이버가 ENABLED 가 아니면 명령이 나가지 않아야 한다
    run(0.5)
    assert len(cmds) == 0, f"enable 전인데 명령 {len(cmds)}개"
    assert len(states) > 100, f"motor_state 가 안 옴 ({len(states)})"
    print(f"[0] enable 전: cmd 0개, motor_state {len(states)}개/0.5 s → 정상 (require_enabled)")

    # (1) enable_motors → 명령 시작, 첫 명령 = 측정값
    res = rclpy.call_service("/upper_body/enable_motors", EnableMotors.Request())
    assert res.success
    states.clear()
    run(1.0)
    q_meas0 = states[0][1]
    t_c0, q_c0, ids0, m0 = cmds[0]
    assert np.max(np.abs(q_c0 - q_meas0)) < 1e-3, f"첫 명령이 측정값과 다름: {np.max(np.abs(q_c0 - q_meas0)):.4f}"
    import yaml
    cfg_st = yaml.safe_load(open(CONFIG))["upper_body_streamer"]["ros__parameters"]
    exp_ids = list(cfg_st["motor_ids"]) + [int(v) for v in cfg_st["extra_motor_ids"] if int(v) > 0]
    assert ids0[:len(exp_ids)] == exp_ids and ids0[len(exp_ids):] == [255] * (15 - len(exp_ids)), ids0   # 드라이버 슬롯 순서 유지, 빈 슬롯 255, 머리 90 은 명령 안 함
    exp_kp = 0.0 if cfg_st.get("position_only", False) else float(cfg_st["kp"])
    assert m0.command_mode == int(cfg_st["command_mode"]) and abs(m0.kp[0] - exp_kp) < 1e-6
    assert len(exp_ids) == 15 or m0.kp[len(exp_ids)] == 0.0
    if cfg_st.get("position_only", False):
        assert all(v == 0.0 for v in m0.effort) and all(v == 0.0 for v in m0.kd), "position_only 인데 kp/kd/effort 가 0 이 아님"
    assert abs(m0.duration - 1.0 / float(cfg_st["rate"])) < 1e-9 and m0.header.frame_id == "upper_body_link"
    rate_cmd = len(cmds) / 1.0; rate_state = len(states) / 1.0
    print(f"[1] enable 후 1 s: cmd {rate_cmd:.0f} Hz, motor_state {rate_state:.0f} Hz, 첫 명령-측정 차 {np.max(np.abs(q_c0 - q_meas0)):.2e} rad, "
          f"motor_id={ids0}, mode {m0.command_mode}, kp {m0.kp[0]}")
    assert rate_cmd > 0.75 * float(cfg_st["rate"]) and 240 <= rate_state <= 260   # system_state(10 Hz) 가 ENABLED 를 알리기까지 ≤0.1 s 는 명령 없음

    # (2) mock 이 그립을 잡고 원운동 → IK 100 Hz, 명령 연속성, 추종
    cmds.clear(); iks.clear(); states.clear()
    hand0 = ik.core.hand_pose("left").translation.copy() if hasattr(ik, "core") else None
    run(6.0)
    rate_ik = len(iks) / 6.0
    dq = np.array([np.max(np.abs(cmds[i + 1][1] - cmds[i][1])) for i in range(len(cmds) - 1)])
    v_max, dt = streamer.sp.v_max, streamer.streamer.dt
    assert dq.max() <= v_max * dt * 1.01 + 1e-9, f"명령 점프 {dq.max():.4f} > v_max·dt {v_max * dt:.4f}"
    # 추종 오차: 명령(q_cmd) 대비 측정, 그리고 IK 목표 대비 측정 (마지막 1 s 평균)
    tail_c = [c for c in cmds if c[0] > rclpy.now() - 1.0]
    tail_s = [s for s in states if s[0] > rclpy.now() - 1.0]
    err_cmd = np.mean([np.max(np.abs(c[1] - min(tail_s, key=lambda s: abs(s[0] - c[0]))[1])) for c in tail_c[::10]])
    tail_i = [k for k in iks if k[0] > rclpy.now() - 1.0]
    err_ik = np.mean([np.max(np.abs(k[1] - min(tail_s, key=lambda s: abs(s[0] - k[0]))[1])) for k in tail_i[::5]])
    moved = np.max(np.abs(states[-1][1] - states[0][1]))
    print(f"[2] 6 s 추종: ik {rate_ik:.0f} Hz, cmd {len(cmds) / 6:.0f} Hz, 틱당 최대 변화 {dq.max():.4f} (한도 {v_max * dt:.4f}) rad, "
          f"명령-측정 {err_cmd * 1e3:.1f} mrad, IK목표-측정 {err_ik * 1e3:.1f} mrad, 관절 이동 최대 {moved:.3f} rad, streamer {streamer.streamer.state}")
    R = float(cfg_st["rate"])
    assert 90 <= rate_ik <= 110 and 0.96 * R <= len(cmds) / 6 <= 1.04 * R and streamer.streamer.state == "TRACK"
    # mock 의 '손끝 들어 올리기' 피치가 SH(정면 +y)에서 롤이 되면 손목 요/피치가 ±1 rad 로 뒤틀린다 (v0.7.2 에서 수정) — 손목은 거의 0 이어야 함
    q_now = states[-1][1]
    wrists = {n: q_now[i] for i, n in enumerate(streamer.joints) if "wrist_yaw" in n or "wrist_pitch" in n}
    print(f"    손목 요/피치 (lift 후): " + ", ".join(f"{k.replace('_joint', '')}={v:+.2f}" for k, v in wrists.items()))
    assert all(abs(v) < 0.35 for v in wrists.values()), f"손목이 뒤틀림: {wrists}"
    assert moved > 0.15, "팔이 안 움직임 (mock 원운동이 전달되지 않음)"
    assert err_cmd < 0.05 and err_ik < 0.08, "추종 오차가 큼"

    # (3) IK 가 죽음 → 0.2 s 안에 HOLD, 명령은 그 자리에 멈춤 (워치독)
    for t in ik.timers:
        t.cancel()
    cmds.clear()
    run(0.5)
    hold_t = next((c[0] for c in cmds if streamer.streamer.state == "HOLD"), None)
    assert streamer.streamer.state == "HOLD" and streamer.streamer.hold_reason == "ik timeout", (streamer.streamer.state, streamer.streamer.hold_reason)
    q_hold = cmds[-1][1]
    moving_after = np.max(np.abs(cmds[-1][1] - cmds[-int(0.15 * R)][1]))
    print(f"[3] IK 정지 후 0.5 s: streamer {streamer.streamer.state} ({streamer.streamer.hold_reason}), 마지막 0.15 s 명령 변화 {moving_after:.2e} rad, "
          f"명령은 계속 {len(cmds) / 0.5:.0f} Hz (현재 위치 유지)")
    assert moving_after < 1e-6 and len(cmds) >= 0.9 * 0.5 * R

    # (4) IK 복귀 → 점프 없이 다시 추종
    for t in ik.timers:
        t.cancelled = False
        rclpy._schedule(t, rclpy.now() + t.period)
    cmds.clear()
    run(2.0)
    dq2 = np.array([np.max(np.abs(cmds[i + 1][1] - cmds[i][1])) for i in range(len(cmds) - 1)])
    first_jump = np.max(np.abs(cmds[0][1] - q_hold))
    print(f"[4] IK 복귀 2 s: 첫 명령 점프 {first_jump:.2e}, 틱당 최대 변화 {dq2.max():.4f} rad, streamer {streamer.streamer.state}")
    assert first_jump <= v_max * dt + 1e-9 and dq2.max() <= v_max * dt * 1.01 + 1e-9 and streamer.streamer.state == "TRACK"

    # (5) 외부 hold 토픽
    from std_msgs.msg import Bool
    for cb, _ in rclpy._subs["/teleop/hold"]:
        cb(Bool(True))
    cmds.clear(); run(0.3)
    assert streamer.streamer.state == "HOLD" and np.max(np.abs(cmds[-1][1] - cmds[-int(0.1 * R)][1])) < 1e-6
    for cb, _ in rclpy._subs["/teleop/hold"]:
        cb(Bool(False))
    run(0.3)
    print(f"[5] /teleop/hold true → HOLD(정지), false → {streamer.streamer.state}")
    assert streamer.streamer.state == "TRACK"

    # (5b) 실행 중 파라미터 변경 (ros2 param set): kp 반영, 잘못된 값 거부
    from rclpy.parameter import Parameter
    r = streamer.set_parameters([Parameter("position_only", value=False), Parameter("kp", value=30.0), Parameter("v_max", value=0.4)])
    assert all(x.successful for x in r) and streamer.kp == 30.0 and streamer.sp.v_max == 0.4
    cmds.clear(); run(0.1)
    assert abs(cmds[-1][3].kp[0] - 30.0) < 1e-6
    r = streamer.set_parameters([Parameter("position_only", value=True)]); cmds.clear(); run(0.1)
    assert cmds[-1][3].kp[0] == 0.0
    r = streamer.set_parameters([Parameter("command_mode", value=2)])
    assert not r[0].successful
    print(f"[5b] ros2 param set kp 30 / v_max 0.4 → 명령 kp {cmds[-1][3].kp[0]}, 한도 {streamer.sp.v_max}; command_mode 2 는 거부")

    # (6) 매핑을 일부러 틀리게(관절 1 부호 반전) → 스트리머 자신은 모름(명령·측정이 같은 맵) → 실기체(시뮬) 관절이 반대로 움직여
    #     IK 가 '내부 자세 ≠ 측정' 재동기화를 반복 → resync_hold_count 회 안에 /teleop/hold true → 스트리머 HOLD. (실기체에서 매핑 오류의 증상·안전망)
    from rclpy.node import LOG
    from quest_arm_ik.upper_body import MotorMap
    n_log = len(LOG)
    streamer.map = MotorMap(list(cfg_st["motor_ids"]), [-1.0] + [1.0] * 13, [0.0] * 14, [int(v) for v in cfg_st["extra_motor_ids"] if int(v) > 0])
    run(12.0)
    resyncs = [l for l in LOG[n_log:] if "재동기화" in l[3] and l[1] == "WARN"]
    errors = [l for l in LOG[n_log:] if l[1] == "ERROR"]
    print(f"[6] 관절 1 부호를 틀리게 12 s: IK 재동기화 {len(resyncs)}회, 자동 정지(ERROR) {len(errors)}회 → streamer {streamer.streamer.state} ({streamer.streamer.hold_reason})")
    assert len(resyncs) >= 3 and errors and streamer.streamer.state == "HOLD" and streamer.streamer.hold_reason == "hold topic"

    for n in nodes:
        n.destroy_node()
    rclpy.shutdown()
    print("\ntest_hr_chain: 전 구간 OK")


if __name__ == "__main__":
    main()
