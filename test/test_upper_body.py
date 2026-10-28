"""upper_body.py (삼현 HRController 스트리머 로직) — ROS 없이 검증.

  $QPY test/test_upper_body.py                      # (다른 테스트처럼 스크립트로 실행; pytest 불필요)
검증: 매핑 왕복, 모터 누락 감지, 첫 명령=측정값(점프 금지), 틱당 v_max·dt 제한, a_max 제한, IK 끊김 → 정지(속도 0 으로 감속),
      추종 오차 초과 → HOLD, 외부 hold, 15 슬롯 채우기(extra 모터 유지·빈 슬롯 0), 그리고 launch 가 config 매핑을 세 노드에 같이 넘기는지.
"""
import contextlib
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from quest_arm_ik.upper_body import EMPTY_MOTOR_ID, N_SLOTS, MotorMap, Streamer, StreamerParams, fill_command  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent


def pytest_approx(x, abs=1e-9):
    return approx(x, abs)


def approx(x, abs=1e-9):
    """pytest.approx 대용 (스칼라·리스트)."""
    class _A:
        def __eq__(self, other):
            return np.allclose(np.asarray(other, dtype=float), np.asarray(x, dtype=float), atol=abs, rtol=0.0)
    return _A()


@contextlib.contextmanager
def raises(exc):
    try:
        yield
    except exc:
        return
    raise AssertionError(f"{exc.__name__} 가 나지 않음")


def _map(n=4):
    return MotorMap(motor_ids=list(range(1, n + 1)), signs=[1.0, -1.0, 1.0, -1.0][:n], offsets=[0.1, 0.2, -0.3, 0.0][:n], extra_motor_ids=[15])


def test_motor_map_roundtrip_and_state_lookup():
    m = _map()
    q = np.array([0.5, -0.25, 1.0, 0.0])
    assert np.allclose(m.to_joint(m.to_motor(q)), q)
    assert np.allclose(m.to_motor([0, 0, 0, 0]), m.offsets)
    # 메시지의 모터 순서가 섞여 있어도 ID 로 찾는다
    ids = [15, 3, 1, 4, 2] + [0] * 10
    pos = [0.0] + list(m.to_motor(q)[[2, 0, 3, 1]]) + [0.0] * 10
    assert np.allclose(m.joints_from_state(ids, pos), q)
    assert m.joints_from_state([1, 2, 3, 0], [0, 0, 0, 0]) is None        # 모터 4 누락
    with raises(ValueError):
        MotorMap([1, 1], [1, 1], [0, 0])                                   # 중복
    with raises(ValueError):
        MotorMap([1, 2], [1], [0, 0])                                      # 길이 불일치


def test_streamer_starts_at_measurement_and_rate_limits():
    p = StreamerParams(rate=250.0, v_max=0.5, a_max=0.0, ik_timeout=0.2, v_catchup=0.0)   # v_catchup 0: 첫 명령부터 v_max (접근 제한은 별도 테스트)
    s = Streamer(2, p)
    assert s.tick(0.0) is None and s.state == "IDLE"                      # 측정값 전엔 명령 없음
    s.set_measured([0.3, -0.3])
    s.set_target([1.0, -1.0], 0.0)                                         # 멀리 있는 목표
    q = s.tick(0.0)
    assert np.allclose(q, [0.3 + 0.5 / 250, -0.3 - 0.5 / 250])             # 첫 틱도 측정값에서 v_max·dt 만
    t, prev = 0.0, q.copy()
    for k in range(1, 400):
        t = k / 250.0
        s.set_target([1.0, -1.0], t)                                       # IK 는 계속 갱신
        s.set_measured(prev)                                               # 모터가 명령을 그대로 따라온다고 가정 (안 따라오면 아래 추종오차 테스트)
        q = s.tick(t)
        assert np.max(np.abs(q - prev)) <= 0.5 / 250 + 1e-12
        prev = q.copy()
    assert s.state == "TRACK" and np.allclose(q, [1.0, -1.0], atol=1e-9)   # 0.7 rad / 0.5 rad/s = 1.4 s < 1.6 s
    assert np.allclose(s.qd_cmd, 0.0, atol=1e-9)                            # 도달 후 속도 피드포워드 0


def test_streamer_acceleration_limit():
    p = StreamerParams(rate=250.0, v_max=2.0, a_max=8.0, v_catchup=0.0)                 # v_catchup 0 = 접근 제한 없음 (가속 제한만 본다)
    s = Streamer(1, p)
    s.set_measured([0.0]); s.set_target([10.0], 0.0)
    qd_prev = 0.0
    for k in range(50):
        s.tick(k / 250.0)
        assert s.qd_cmd[0] - qd_prev <= 8.0 / 250 + 1e-12 and s.qd_cmd[0] <= 2.0 + 1e-12
        qd_prev = s.qd_cmd[0]
    assert qd_prev == approx(min(2.0, 50 * 8.0 / 250), abs=1e-9)


def test_streamer_no_limit_cycle_after_abrupt_stop():
    """v0.7.47 회귀: 목표가 v_max 로 움직이다 갑자기 서면(IK 출력의 전형) 명령은 v²/2a 안에서 한 번 지나쳤다가 진동 없이 수렴해야 한다.
    (구법 데드비트 + 가속 제한은 ±v²/2a 삼각파로 영원히 진동 — 2026-10-26 실기체 2 Hz '덜덜' 의 원인)"""
    import numpy as np
    for a_max in (4.0, 8.0):
        p = StreamerParams(rate=100.0, v_max=0.875, a_max=a_max)
        s = Streamer(1, p); s.set_measured([0.0])
        goal, hist, goals = 0.0, [], []
        for k in range(300):
            t = k * 0.01
            if t < 1.0:
                goal = 0.7 * t                         # 0.7 rad/s 로 1 s 이동
            s.set_target([goal], t)                    # 1 s 뒤부터는 같은 목표(정지)
            hist.append(s.tick(t)[0]); goals.append(goal)
            s.set_measured(s.q_cmd)                    # 드라이브가 완벽히 따라온다고 가정 (tracking-error HOLD 방지)
        q = np.array(hist); e = q - np.array(goals)
        overshoot = e[100:].max()
        assert overshoot <= 0.875 ** 2 / (2 * a_max) + 0.01, (a_max, overshoot)         # v²/2a 이내
        assert abs(e[-1]) < 2e-3 and abs(s.qd_cmd[0]) < 0.05, (a_max, e[-1], s.qd_cmd)   # 2 s 안에 수렴
        sign_changes = int(np.sum(np.diff(np.sign(e[100:][np.abs(e[100:]) > 2e-3])) != 0))
        assert sign_changes <= 1, (a_max, sign_changes)                                  # 한 번 지나쳤다 돌아올 뿐, 진동 없음
        # 이동 중 추종: 피드포워드 덕에 정상상태 지연이 작아야 함 (0.7 rad/s 에서 ≤ 30 mrad)
        assert np.abs(e[50:95]).max() < 0.03, np.abs(e[50:95]).max()


def test_streamer_holds_on_ik_timeout_and_decelerates():
    p = StreamerParams(rate=250.0, v_max=1.0, a_max=8.0, ik_timeout=0.2, v_catchup=0.0)
    s = Streamer(1, p)
    s.set_measured([0.0]); s.set_target([5.0], 0.0)
    for k in range(100):
        s.tick(k / 250.0)                                                  # 0.4 s: 목표는 t=0 에 한 번만 옴
    # 0.2 s 이후부터 HOLD 로 들어가 속도가 a_max 로 0 까지 줄어든다
    assert s.state == "HOLD" and s.hold_reason == "ik timeout"
    assert s.qd_cmd[0] == approx(0.0, abs=1e-9)
    q_hold = s.q_cmd.copy()
    s.tick(1.0)
    assert np.allclose(s.q_cmd, q_hold)                                    # 정지 유지
    s.set_target([5.0], 1.0)                                               # IK 복귀 → 그 자리에서 다시 추종 (점프 없음)
    q = s.tick(1.0)
    assert q[0] - q_hold[0] <= 8.0 / 250 / 250 + 1e-12 and s.state == "TRACK"


def test_streamer_tracking_error_and_external_hold():
    """추종 오차 정지(v0.7.55): 명령을 측정값으로 되돌리고 hold_recover_time 뒤 v_catchup 으로 다시 접근; 목표에 못 닿은 채 3회면 정지 유지,
    hold 토글·reset·목표 복귀로 해제. 외부 hold 는 그 자리 정지."""
    p = StreamerParams(rate=250.0, v_max=1.0, a_max=0.0, max_tracking_error=0.35, v_catchup=0.3, hold_recover_time=1.0, max_tracking_retries=3)
    s = Streamer(2, p)
    dt = 1.0 / 250.0
    s.set_measured([0.0, 0.0]); s.set_target([0.0, 0.0], 0.0)
    assert s.tick(0.0) is not None and s.state == "TRACK"
    s.q_cmd = np.array([0.5, 0.0])                                         # 명령은 갔는데 모터가 안 따라온 상황
    s.set_target([0.5, 0.0], 0.001); q = s.tick(0.001)
    assert s.state == "HOLD" and s.hold_reason.startswith("tracking error") and np.allclose(q, [0.0, 0.0]), (s.state, s.hold_reason, q)   # 명령을 측정값으로
    s.set_target([0.5, 0.0], 0.5); q = s.tick(0.5)
    assert s.state == "HOLD" and np.allclose(q, [0.0, 0.0])                # 회복 대기 중
    # 1 s 뒤: 다시 접근하되 v_catchup 이하 (모터가 막혀 측정은 그대로 → 0.35 rad 에서 또 정지 … 3회면 정지 유지)
    t = 1.1
    fails, steps = 0, []
    for _ in range(int(12.0 / dt)):
        s.set_target([0.5, 0.0], t); q0 = s.q_cmd.copy(); s.tick(t); t += dt
        if s.state == "TRACK":
            steps.append(abs(s.q_cmd[0] - q0[0]))
        elif s.hold_reason.startswith("tracking error") and s.t_hold_until == t - dt + p.hold_recover_time:
            fails += 1
        if s.track_latched:
            break
    assert steps and max(steps) <= p.v_catchup * dt * 1.001, max(steps) / dt
    assert s.track_latched and s.n_track_fail == 3 and "정지 유지" in s.hold_reason, (s.n_track_fail, s.hold_reason)
    for _ in range(int(3.0 / dt)):                                         # 시간이 지나도 혼자 재개하지 않는다
        s.set_target([0.5, 0.0], t); s.tick(t); t += dt
    assert s.state == "HOLD" and s.track_latched
    s.set_hold(True, "hold topic"); s.set_target([0.5, 0.0], t); s.tick(t); t += dt
    assert s.state == "HOLD" and s.hold_reason == "hold topic"
    s.set_hold(False); s.set_target([0.5, 0.0], t); q0 = s.q_cmd.copy(); s.tick(t); t += dt   # 조작자 확인(hold 토글) → 천천히 재개
    assert s.state == "TRACK" and not s.track_latched and abs(s.q_cmd[0] - q0[0]) <= p.v_catchup * dt * 1.001
    # 다시 3회 막힘 → 정지 유지; 이번엔 목표가 명령 자리로 돌아오면(텔레옵 IK 재동기화) 해제
    while not s.track_latched:
        s.set_target([0.5, 0.0], t); s.tick(t); t += dt
    s.set_target(list(s.q_cmd), t); s.tick(t); t += dt
    assert s.state == "HOLD" and s.track_latched                           # 회복 대기(hold_recover_time) 안에는 아직
    t += p.hold_recover_time
    s.set_target(list(s.q_cmd), t); s.tick(t); t += dt
    assert s.state == "TRACK" and not s.track_latched
    # 외부 hold: 그 자리 정지, 해제하면 추종 (접근 속도 제한)
    s.set_measured(list(s.q_cmd))
    s.set_hold(True, "hold topic"); s.set_target([2.0, 0.0], t); q1 = s.tick(t); t += dt
    assert s.state == "HOLD" and s.hold_reason == "hold topic" and np.allclose(q1, s.q_cmd)
    s.set_hold(False); s.set_target([2.0, 0.0], t); q0 = s.q_cmd.copy(); s.tick(t)
    assert s.state == "TRACK" and abs(s.q_cmd[0] - q0[0]) <= p.v_catchup * dt * 1.001


def test_streamer_catchup_after_restart():
    """정지(미활성 → reset_to_measured)에서 추종을 다시 시작할 때 멀리 있는 목표에는 v_catchup 으로 가고, 닿은 뒤에는 v_max 로 따른다 (2026-10-28 enable 순간 튐)."""
    p = StreamerParams(rate=100.0, v_max=2.5, a_max=4.0, max_tracking_error=0.35, v_catchup=0.3, catchup_done=0.02)
    s = Streamer(1, p)
    dt = 0.01
    s.set_measured([0.0]); s.reset_to_measured()
    t, goal = 0.0, 1.0
    v = []
    while s.catching_up and t < 10.0:
        s.set_measured(list(s.q_cmd))                                      # 모터가 잘 따라온다고 가정
        s.set_target([goal], t); q0 = s.q_cmd[0]; s.tick(t); t += dt
        v.append((s.q_cmd[0] - q0) / dt)
    assert not s.catching_up and 2.5 < t < 4.5, t                          # 1 rad / 0.3 rad/s ≈ 3.3 s
    assert max(v) <= p.v_catchup * 1.001, max(v)
    # 닿은 뒤 움직이는 목표: v_max 까지 낸다
    vmax_seen = 0.0
    for _ in range(100):
        goal += 1.5 * dt
        s.set_measured(list(s.q_cmd)); s.set_target([goal], t); q0 = s.q_cmd[0]; s.tick(t); t += dt
        vmax_seen = max(vmax_seen, (s.q_cmd[0] - q0) / dt)
    assert vmax_seen > 1.0, vmax_seen
    # 다시 정지(reset) → 다시 천천히
    s.reset_to_measured()
    s.set_target([goal + 1.0], t); q0 = s.q_cmd[0]; s.tick(t)
    assert s.catching_up and (s.q_cmd[0] - q0) / dt <= p.v_catchup * 1.001


def test_fill_command_slots():
    m = _map()
    arr = fill_command(m, [0.0, 0.0, 0.0, 0.0], [1.0, 1.0, 0.0, 0.0], kp=20.0, kd=0.5, extra_positions={15: 0.7},
                       extra_kp=10.0, extra_kd=0.3, effort=[2.0, 2.0, 0.0, 0.0])
    assert all(len(v) == N_SLOTS for v in arr.values())
    assert arr["motor_id"][:5] == [1, 2, 3, 4, 15] and arr["motor_id"][5:] == [EMPTY_MOTOR_ID] * 10      # 빈 슬롯 255 (삼현 브리지 규약)
    assert arr["position"][:4] == approx(list(m.offsets)) and arr["position"][4] == 0.7
    assert arr["velocity"][:2] == [1.0, -1.0] and arr["effort"][:2] == [2.0, -2.0]      # 부호 반영
    assert arr["kp"][:5] == [20.0] * 4 + [10.0] and arr["kd"][4] == 0.3 and arr["kp"][5] == 0.0
    arr2 = fill_command(m, [0.0] * 4, [0.0] * 4, 20.0, 0.5)                                 # extra 위치 모름 → id 만 적고 kp/kd 0
    assert arr2["motor_id"][4] == 15 and arr2["kp"][4] == 0.0 and arr2["position"][4] == 0.0
    # 드라이버 슬롯 순서를 주면 같은 슬롯에 넣는다 (실기체 motor_states 순서: 예 [15, 3, 1, 4, 2, 0, …])
    order = [15, 3, 1, 4, 2] + [0] * 10
    arr3 = fill_command(m, [0.1, 0.2, 0.3, 0.4], [0.0] * 4, 20.0, 0.5, {15: 0.7}, 10.0, 0.3, slot_order=order)
    assert arr3["motor_id"][:5] == [15, 3, 1, 4, 2] and arr3["motor_id"][5:] == [EMPTY_MOTOR_ID] * 10
    assert arr3["position"][2] == pytest_approx(m.to_motor([0.1, 0.2, 0.3, 0.4])[0]) and arr3["position"][1] == pytest_approx(m.to_motor([0.1, 0.2, 0.3, 0.4])[2])
    assert arr3["position"][0] == 0.7 and arr3["kp"][0] == 10.0
    # 슬롯 순서에 없는 모터는 빈 슬롯에
    arr4 = fill_command(m, [0.0] * 4, [0.0] * 4, 20.0, 0.5, slot_order=[3, 1] + [0] * 13)
    assert arr4["motor_id"][:2] == [3, 1] and sorted(arr4["motor_id"][2:6]) == [2, 4, 15, EMPTY_MOTOR_ID] and arr4["motor_id"].count(EMPTY_MOTOR_ID) == 10
    arr5 = fill_command(m, [0.0] * 4, [0.0] * 4, 20.0, 0.5, empty_id=0)                                 # 옛 규약(0)도 선택 가능
    assert arr5["motor_id"].count(0) == 10


def test_config_mapping_is_consistent():
    """config 의 세 블록(dual_arm_ik / upper_body_streamer / mujoco_sim hr_*) 길이·관절 순서가 맞는지 (launch 가 streamer 값을 덮어쓰지만 길이는 같아야 함)."""
    import yaml
    d = yaml.safe_load(open(HERE.parent / "config" / "sh_humanoid_v2.yaml"))
    ik, st, sim = (d[k]["ros__parameters"] for k in ("dual_arm_ik", "upper_body_streamer", "mujoco_sim"))
    n = len(ik["controlled_joints"])
    assert st["controlled_joints"] == ik["controlled_joints"] == sim["controlled_joints"]
    for blk, pre in ((ik, ""), (st, ""), (sim, "hr_")):
        assert len(blk[pre + "motor_ids"]) == len(blk[pre + "joint_signs"]) == len(blk[pre + "joint_offsets"]) == n
        assert len(set(blk[pre + "motor_ids"])) == n and all(s in (1.0, -1.0) for s in blk[pre + "joint_signs"])
    assert not set(st["motor_ids"]) & set(st["extra_motor_ids"]) and st["extra_motor_ids"] == [0] and st["empty_slot_id"] == 255   # 머리 90 은 보내지 않는다 (2026-10-21)
    assert 20.0 <= st["rate"] <= 250.0 and 0 < st["v_max"] <= 1.5 and st["ik_timeout"] <= 0.5     # 50 Hz 기본 (2026-10-20 실측)


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
        print(f"  ok  {t.__name__}")
    print(f"test_upper_body: {len(tests)} tests OK")
