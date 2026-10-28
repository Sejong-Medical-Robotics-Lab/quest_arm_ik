"""동작 쇼(choreography + motion_show) 검증.

    SH_REPO=/abs/SH-Humanoid python3 test/test_motion_show.py

(1) 안무: 관절 속도 ≤ v_max, 관절 한계 여유 ≥ 0.1 rad, 동작 4개 + 전환 — ROS 없이.
(2) MuJoCo 자기충돌: 전 구간 50 ms 간격으로 mj_forward 해 접촉 0 (손-손·손-몸통 포함).
(3) 가짜 rclpy 체인: motion_show → upper_body_streamer → mujoco_sim(HR 에뮬레이션): 첫 명령 = 측정값, 명령 점프 없음, 추종 오차 작음,
    쇼가 측정 자세에서 블렌드로 시작해 heart 동작까지 들어감.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
import numpy as np  # noqa: E402

from quest_arm_ik.choreography import ACTS, HEART, LIMITS, REST, SHOW_V_MAX, build_show, check_trajectory  # noqa: E402

SH = os.environ.get("SH_REPO", "")


def test_choreography():
    show = build_show()
    Q, vmax, lim = check_trajectory(show)
    names = [seg[2] for seg in show.segments]
    assert names == ["→heart", "heart", "→wave", "wave", "→run", "run", "→kiss", "kiss", "→rest"], names
    assert vmax <= SHOW_V_MAX + 1e-6, vmax
    # run(±60°, 1.64 rad/s) 만 1.0 을 넘고 나머지 동작·전환은 1.0 안; run 의 가속은 스트리머 a_max 4 근처까지
    for t0, t1, name, _ in show.segments:
        k0, k1 = int(t0 / 0.01), int(t1 / 0.01)
        V = np.abs(np.diff(Q[k0:k1 + 1], axis=0)) / 0.01
        if name == "run":
            assert 1.5 < V.max() <= SHOW_V_MAX, V.max()
            A = np.abs(np.diff(V, axis=0)) / 0.01
            assert A.max() < 4.5, A.max()
        else:
            assert V.max() <= 1.0 + 1e-6, (name, V.max())
    assert lim >= 0.1 - 1e-9, lim
    assert np.allclose(show.q(0.0), REST) and np.allclose(show.q(show.duration), REST)
    assert 40 < show.duration < 70, show.duration
    # 양팔 대칭 (run 은 반대 위상, wave 는 yaw 부호가 반대(같은 방향) 라 제외)
    for name in ("heart", "kiss"):
        a = ACTS[name]()
        for t in np.linspace(0, a.duration, 7):
            q = a.q(t)
            assert np.allclose(q[:7], q[7:], atol=0.11), (name, t, q)   # kiss 는 IK 해라 0.1 안쪽 차이
    # wave: 팔꿈치 위치 고정(어깨 pitch/roll·팔꿈치 불변), yaw 만 양팔 반대 부호; run: 팔꿈치 -1.40 고정, 어깨 pitch 반대 위상
    a = ACTS["wave"]()
    for t in np.linspace(0, a.duration, 11):
        q = a.q(t)
        assert np.allclose(q[[0, 1, 3, 7, 8, 10]], a.q(0)[[0, 1, 3, 7, 8, 10]]) and abs(q[2] + q[9]) < 1e-9, (t, q)
    a = ACTS["run"]()
    ts = np.linspace(0, a.duration, 1201); qs = np.stack([a.q(t) for t in ts])
    assert np.allclose(qs[:, [3, 10]], -np.radians(60)) and np.allclose(qs[:, 0], -qs[:, 7]), "팔꿈치 60° 고정, 어깨 pitch 반대 위상"
    assert abs(qs[:, 0].max() - np.radians(60)) < 0.02 and abs(qs[:, 0].min() + np.radians(60)) < 0.02, (qs[:, 0].max(), qs[:, 0].min())   # ±60°
    assert int(np.sum(np.diff(np.sign(qs[:, 0] - 1e-3)) != 0) // 2) == 2, "2회 왕복"
    assert np.allclose(qs[:, [4, 11]], 0.0) and np.allclose(ACTS["wave"]().q(1.0)[[4, 11]], 1.0), "wave 손목 yaw +1.0 = 손바닥 정면"
    s2 = build_show(("wave",), speed=0.5)
    _, v2, _ = check_trajectory(s2)
    assert v2 <= 0.5 + 1e-6, v2
    print(f"choreography: {show.duration:.1f} s, v_max {vmax:.2f}, 한계 여유 {lim:.2f} → OK")


def test_self_collision():
    import yaml
    from quest_arm_ik.mujoco_sim import MujocoArmSim
    import mujoco
    repo = os.path.join(SH, "urdf_moveit")
    xacro = os.path.join(repo, "SH_Humanoid_v2_description", "urdf", "SH_Humanoid_v2.xacro")
    srdf = os.path.join(repo, "SH_Humanoid_v2_moveit_config", "config", "SH_Humanoid_v2.srdf")
    cfg = yaml.safe_load(open(os.path.join(HERE, "..", "config", "sh_humanoid_v2.yaml")))
    joints = cfg["dual_arm_ik"]["ros__parameters"]["controlled_joints"]
    shapes = yaml.safe_load(cfg["mujoco_sim"]["ros__parameters"]["collision_shapes_yaml"])
    sim = MujocoArmSim(xacro, joints, keyframe="", pd_kp=300.0, pd_kd=10.0, package_dirs=[repo], collision_shapes=shapes, srdf_path=srdf, xacro_args="")
    m, d = sim.model, sim.data
    show = build_show(); Q, _, _ = check_trajectory(show)
    bad = []
    # 손가락 geom 은 contype 0(접촉 계산 제외)이라 ncon 으로는 손-손 겹침을 못 본다 → mj_geomDistance 로 손(손가락 포함)-손, 손-반대팔, 손-몸통 최소 거리를 잰다.
    # 실기체는 중력 처짐·영점 오차로 손 위치가 수 cm 틀어지므로 손-손 ≥ 60 mm(자세 설계는 ≥ 90), 손-반대팔 ≥ 40 mm, 몸통은 팔을 든 동안 ≥ 25 mm.
    def bname(i): return mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i) or ""
    def geoms(pred): return [g for g in range(m.ngeom) if pred(bname(m.geom_bodyid[g]))]
    LH, RH = geoms(lambda n: n.startswith("left_hand")), geoms(lambda n: n.startswith("right_hand"))
    LA, RA = geoms(lambda n: n.startswith("left_") and not n.startswith("left_hand")), geoms(lambda n: n.startswith("right_") and not n.startswith("right_hand"))
    TORSO = geoms(lambda n: n == "base_link")
    assert LH and RH and TORSO, (len(LH), len(RH), len(TORSO))
    ft = np.zeros(6)
    def mind(A, B):
        return min(mujoco.mj_geomDistance(m, d, a, b, 0.5, ft) for a in A for b in B)
    worst = {"hand-hand": (1e9, 0.0), "hand-arm": (1e9, 0.0), "hand-torso": (1e9, 0.0)}
    for k in range(0, len(Q), 5):
        for j, v in zip(joints, Q[k]):
            d.qpos[sim.qadr[j]] = v
        mujoco.mj_forward(m, d)
        t = k * 0.01
        if d.ncon > 0:
            c = d.contact[0]
            bad.append((round(t, 2), show.name_at(t), bname(m.geom_bodyid[c.geom1]), bname(m.geom_bodyid[c.geom2])))
        hh, ha = mind(LH, RH), min(mind(LH, RA), mind(RH, LA))
        ht = min(mind(LH, TORSO), mind(RH, TORSO)) if np.max(np.abs(Q[k] - REST)) > 0.3 else 1e9   # 팔 내린 자세는 손이 몸통 옆에 매달림(모델 6 mm) — 제외
        for key, val in (("hand-hand", hh), ("hand-arm", ha), ("hand-torso", ht)):
            if val < worst[key][0]:
                worst[key] = (val, t)
    assert not bad, bad[:5]
    assert worst["hand-hand"][0] >= 0.06, f"손-손 최소 {worst['hand-hand'][0]*1000:.0f} mm (t={worst['hand-hand'][1]:.1f}s {show.name_at(worst['hand-hand'][1])})"
    assert worst["hand-arm"][0] >= 0.04, f"손-반대팔 최소 {worst['hand-arm'][0]*1000:.0f} mm (t={worst['hand-arm'][1]:.1f}s)"
    assert worst["hand-torso"][0] >= 0.025, f"손-몸통 최소 {worst['hand-torso'][0]*1000:.0f} mm (t={worst['hand-torso'][1]:.1f}s)"
    # 하트 자세 자체: 손(손가락 포함) 사이 ≥ 90 mm
    for j, v in zip(joints, np.asarray(HEART, float)):
        d.qpos[sim.qadr[j]] = v
    mujoco.mj_forward(m, d)
    heart_gap = mind(LH, RH)
    assert heart_gap >= 0.09, heart_gap
    # 하트: 손 링크 간격 0.3~0.5 m, 손 높이 > 1.3 m (손끝이 마주 보되 겹치지 않음)
    k = int(8.0 / 0.01)
    for j, v in zip(joints, Q[k]):
        d.qpos[sim.qadr[j]] = v
    mujoco.mj_forward(m, d)
    lh = d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "left_hand_link")]; rh = d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "right_hand_link")]
    dist = float(np.linalg.norm(lh - rh))
    assert 0.3 < dist < 0.5 and lh[2] > 1.25 and rh[2] > 1.25, (dist, lh, rh)
    print(f"self-collision: {len(Q) // 5} 자세 접촉 0, 손-손 최소 {worst['hand-hand'][0]*1000:.0f} mm·손-반대팔 {worst['hand-arm'][0]*1000:.0f} mm·손-몸통 {worst['hand-torso'][0]*1000:.0f} mm, 하트 손 사이 {heart_gap*1000:.0f} mm")
    print(f"self-collision: 하트 손 링크 간격 {dist:.2f} m·높이 {lh[2]:.2f} m → OK")


def test_chain():
    import test_hr_chain as T
    rclpy = T._setup_fake_ros()
    rclpy.reset()
    params = T.launch_params(robot="sh", model_path=T.SH_XACRO, package_dirs=os.path.dirname(os.path.dirname(os.path.dirname(T.SH_XACRO))),
                             input="show", hr="true", sim="true", viewer="false")
    assert "motion_show" in params and "dual_arm_ik" not in params, list(params)
    params["motion_show"].update({"acts": "heart", "start_delay": 0.2, "blend_time": 1.0, "start_timeout": 1.0})
    rclpy.PARAM_OVERRIDES.update(params)
    rclpy.init()
    from quest_arm_ik.motion_show import MotionShow
    from quest_arm_ik.upper_body_streamer_node import UpperBodyStreamer
    from quest_arm_ik.mujoco_sim import make_node_class
    from ros2_interfaces.srv import EnableMotors
    sim_node = make_node_class()()
    show, streamer = MotionShow(), UpperBodyStreamer()
    nodes = [show, streamer, sim_node]
    sim = sim_node.sim
    cmds, states, tg = [], [], []
    rclpy._subs.setdefault("/upper_body/command", []).append((lambda m: cmds.append((rclpy.now(), streamer.map.joints_from_state(m.motor_id, m.position))), None))
    rclpy._subs.setdefault("/upper_body/motor_states", []).append((lambda m: states.append((rclpy.now(), streamer.map.joints_from_state(m.motor_id, m.position))), None))
    rclpy._subs.setdefault("/teleop/joint_targets", []).append((lambda m: tg.append((rclpy.now(), np.array(m.position[:14]))), None))

    def run(seconds):
        for _ in range(int(round(seconds / 0.001))):
            rclpy.advance(0.001)
            with sim.lock:
                behind = int(round((rclpy.now() - sim.data.time) / sim.dt))
                if behind > 0:
                    sim.step(behind)

    run(0.8)                                                  # enable 전: 측정은 오지만 쇼는 시작하지 않는다 (스트리머 commanding=0 / 드라이버 READY)
    assert len(states) > 50 and len(tg) == 0, (len(states), len(tg))
    assert show.t_strm is not None and show.strm.get("commanding") == "0", show.strm
    res = rclpy.call_service("/upper_body/enable_motors", EnableMotors.Request())
    assert res.success
    tg.clear(); states.clear()
    run(0.5)
    assert len(tg) > 20 and len(states) > 50, (len(tg), len(states))
    q_meas0 = states[0][1]
    assert np.max(np.abs(tg[0][1] - q_meas0)) < 0.02, f"쇼 첫 목표가 측정 자세에서 시작하지 않음: {np.max(np.abs(tg[0][1] - q_meas0)):.3f}"
    run(9.5)                                                  # blend 1 s + →heart + heart 중반
    rate_tg = len(tg) / 10.0
    assert 90 < rate_tg < 110, rate_tg
    dq = np.array([np.max(np.abs(tg[i + 1][1] - tg[i][1])) for i in range(len(tg) - 1)])
    assert dq.max() <= SHOW_V_MAX * 0.01 * 1.05 + 1e-9, f"목표 점프 {dq.max():.4f} rad/틱"
    assert abs(streamer.sp.v_max - 2.5) < 1e-6, streamer.sp.v_max        # launch input:=show 기본 v_max 2.0 → 스트리머 2.5
    dqc = np.array([np.max(np.abs(cmds[i + 1][1] - cmds[i][1])) for i in range(len(cmds) - 1)])
    assert dqc.max() <= streamer.sp.v_max * streamer.streamer.dt * 1.01 + 1e-9, f"명령 점프 {dqc.max():.4f}"
    # 하트 자세에 들어갔는가: 어깨 roll 이 -1.5 아래로
    assert tg[-1][1][1] < -1.5 and tg[-1][1][8] < -1.5, tg[-1][1]
    tail_c = [c for c in cmds if c[0] > rclpy.now() - 1.0]
    tail_s = [s for s in states if s[0] > rclpy.now() - 1.0]
    err = np.mean([np.max(np.abs(c[1] - min(tail_s, key=lambda s: abs(s[0] - c[0]))[1])) for c in tail_c[::10]])
    assert err < 0.08, f"추종 오차 {err:.3f} rad"
    assert streamer.streamer.state == "TRACK", streamer.streamer.state
    print(f"chain: 목표 {rate_tg:.0f} Hz, 목표 점프 최대 {dq.max() * 100:.2f} rad/s, 명령 점프 최대 {dqc.max() * 100:.2f} rad/s, 추종 오차 {err:.3f} rad, 하트 roll {tg[-1][1][1]:.2f} → OK")
    # ---- (v0.7.55, 2026-10-28 사고 ② 재현) 쇼 도중 disable → 쇼 일시정지(목표 고정) → 팔이 밀려남 → enable → 스트리머가 측정값에서 v_catchup 으로
    #      고정 목표까지 접근 (튀지 않음) → 닿으면 쇼 재개
    from ros2_interfaces.srv import SimpleMotorResult
    assert rclpy.call_service("/upper_body/disable_motors", SimpleMotorResult.Request()).success
    cmds.clear(); run(0.5)
    assert show.paused_reason, "disable 뒤 쇼가 일시정지하지 않음"
    assert len([c for c in cmds if c[0] > rclpy.now() - 0.3]) == 0, "disable 뒤에도 명령을 냄"
    q_now = states[-1][1]
    q_push = q_now.copy(); q_push[0] += 0.4; q_push[7] += 0.4                      # 사람이 밀거나 처진 것처럼 어깨 pitch 를 0.4 rad 옮김
    with sim.lock:
        sim.set_joint_targets(q_push)
    tg.clear(); run(1.5)
    T = np.array([x[1] for x in tg])
    assert np.max(np.abs(T - T[0])) < 1e-9, "일시정지 중 쇼 목표가 움직임"
    frozen = T[-1]
    assert np.max(np.abs(states[-1][1] - q_push)) < 0.05 and np.max(np.abs(frozen - states[-1][1])) > 0.3
    q_at_enable = states[-1][1]
    cmds.clear(); assert rclpy.call_service("/upper_body/enable_motors", EnableMotors.Request()).success
    run(0.3)
    assert len(cmds) > 10 and np.max(np.abs(cmds[0][1] - q_at_enable)) < 0.05, "enable 뒤 첫 명령이 측정값이 아님"
    assert show.paused_reason, "스트리머가 접근 중인데 쇼가 재개됨"
    run(2.0)
    C = np.array([c[1] for c in cmds])
    dqc2 = np.max(np.abs(np.diff(C, axis=0)), axis=1)
    v_catch = streamer.sp.v_catchup * streamer.streamer.dt
    i_done = int(np.argmax(np.max(np.abs(C - frozen), axis=1) < 0.03))                   # 고정 목표에 닿은 명령 index
    assert i_done > 50 and dqc2[:i_done].max() <= v_catch * 1.05 + 1e-9, f"접근 중 명령 속도 {dqc2[:i_done].max() / streamer.streamer.dt:.2f} rad/s > v_catchup"
    assert not show.paused_reason, "접근이 끝났는데 쇼가 재개되지 않음"
    T2 = np.array([x[1] for x in tg if x[0] > rclpy.now() - 0.3])
    assert np.max(np.abs(T2 - T2[0])) > 1e-4, "재개 뒤 쇼 목표가 움직이지 않음"
    print(f"disable→enable: 쇼 일시정지·목표 고정, 접근 명령 속도 최대 {dqc2[:i_done].max() / streamer.streamer.dt:.2f} rad/s (v_catchup {streamer.sp.v_catchup}), {i_done * streamer.streamer.dt:.1f} s 뒤 닿고 재개 OK")
    # 측정 발행자가 둘(시뮬 에뮬레이션 + 실기체 릴레이)이 되면 스트리머는 명령을 멈추고, 하나로 돌아오면 측정값에서 다시 시작한다 (2026-10-28 사고 재현)
    from ros2_interfaces.msg import UpperMotorState
    dup_pub = show.create_publisher(UpperMotorState, "/upper_body/motor_states", 10)   # 두 번째 발행자 (내용은 보내지 않아도 count 만으로 가드)
    cmds.clear(); run(1.0)
    assert len(cmds) == 0, f"발행자 2개인데 명령 {len(cmds)}개"
    assert streamer.dup_sources
    rclpy._pubs["/upper_body/motor_states"] -= 1                                          # 릴레이를 끈 상황
    cmds.clear(); run(1.0)
    assert len(cmds) > 50 and not streamer.dup_sources
    q_meas_now = states[-1][1]
    assert np.max(np.abs(cmds[0][1] - q_meas_now)) < 0.05, np.max(np.abs(cmds[0][1] - q_meas_now))   # 재개 첫 명령 = 측정값 근처 (점프 없음)
    print("duplicate-source guard: 명령 중단·재개 OK")
    for n in nodes:
        n.destroy_node()


if __name__ == "__main__":
    test_choreography()
    if not SH:
        print("SH_REPO 미지정 — 모델·체인 시험 생략"); sys.exit(0)
    test_self_collision()
    test_chain()
    print("motion_show: ALL OK")
