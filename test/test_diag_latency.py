"""diag_latency 검증 — 가짜 rclpy 로 실기체 경로(mock → bridge → IK → streamer → mujoco HR 에뮬레이션)를 돌리고 지연 표가 말이 되는지.

    SH_REPO=$SH python3 test/test_diag_latency.py

기대: 입력률 ≈ mock 발행률, 필터 지연 = One-Euro(config min_cutoff 2.0 Hz·beta 1 → 60 ms; 5/10 이면 30, 1.5 Hz 면 100 ms) 양수, 드라이브 지연 = 시뮬 PD 추종(수십 ms 이하),
      속도 포화율은 느린 mock(0.05 m/s) 에서 낮음(들어 올리는 구간만), 추종 오차 지연 보정 후 ≤ 보정 전.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import test_hr_chain as T  # noqa: E402


def main():
    if not T.SH:
        print("SH_REPO 필요 — 건너뜀"); return
    rclpy = T._setup_fake_ros()
    rclpy.reset()
    params = T.launch_params(robot="sh", model_path=T.SH_XACRO, package_dirs=os.path.dirname(os.path.dirname(os.path.dirname(T.SH_XACRO))),
                             input="mock", hr="true", sim="true", viewer="false")
    params["quest_mock"].update({"engage_after": 1.0, "period": 6.0, "radius": 0.06})   # 손 0.05 m/s → v_max 0.5 에 안 걸림
    import tempfile
    params["diag_latency"] = {"side": "left", "seconds": 1000.0, "hold_seconds": 0.4, "v_max": params["dual_arm_ik"].get("max_joint_velocity", 0.5),
                              "config": T.CONFIG, "save": os.path.join(tempfile.gettempdir(), "diag_test.npz")}
    rclpy.PARAM_OVERRIDES.update(params)
    rclpy.init()
    from quest_arm_ik.quest_bridge import QuestBridge
    from quest_arm_ik.dual_arm_ik_node import DualArmIKNode
    from quest_arm_ik.upper_body_streamer_node import UpperBodyStreamer
    from quest_arm_ik.quest_mock import QuestMock
    from quest_arm_ik.mujoco_sim import make_node_class
    from quest_arm_ik.diag_latency import DiagLatency
    from ros2_interfaces.srv import EnableMotors

    sim_node = make_node_class()()
    nodes = [QuestBridge(), DualArmIKNode(), UpperBodyStreamer(), QuestMock(), sim_node]
    sim = sim_node.sim
    res = rclpy.call_service("/upper_body/enable_motors", EnableMotors.Request())
    assert res.success

    def run(seconds):
        for _ in range(int(round(seconds / 0.001))):
            rclpy.advance(0.001)
            with sim.lock:
                behind = int(round((rclpy.now() - sim.data.time) / sim.dt))
                if behind > 0:
                    sim.step(behind)

    run(0.5)
    diag = DiagLatency()                 # 이제부터 측정: mock 그립(1.0 s) 전 0.4 s 는 정지 → 그 뒤 들어 올리고 원운동
    run(14.0)
    rep = diag.report()
    print(rep)
    import re
    rate = float(re.search(r"입력률\s+([\d.]+) Hz", rep).group(1))
    lag_f = re.search(r"필터 지연\s+(\d+) ms", rep)
    lag_d = re.search(r"드라이브 지연\s+(\d+) ms", rep)
    sat = float(re.search(r"속도 포화율\s+([\d.]+) %", rep).group(1))
    assert rate > 30.0, rate
    assert lag_f is not None and 10 <= int(lag_f.group(1)) <= 120, rep        # config 2.0/1.0 + task_gain 0.5: 시뮬 60 ms (5/10 + 0.8 이면 30, 1.5/0.5 면 ≈ 100)
    assert lag_d is not None and 0 <= int(lag_d.group(1)) <= 100, rep         # 시뮬 PD 추종 + 스트리머(a_max 4)
    assert sat < 30.0, sat                 # 들어 올리는 2 s 동안만 잠깐 포화
    assert "측정에 2~10 Hz 떨림 없음" in rep, rep   # 시뮬 체인은 떨리지 않음
    d = __import__("numpy").load(os.path.join(tempfile.gettempdir(), "diag_test.npz"))
    assert len(d["t_strm"]) > 100 and d["strm"].shape[1] == 14, d["strm"].shape   # 스트리머 출력도 저장됨
    print("diag_latency: OK")
    for n in nodes:
        n.destroy_node()


if __name__ == "__main__":
    main()
