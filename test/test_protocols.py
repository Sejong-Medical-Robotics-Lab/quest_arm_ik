"""입력 파서·좌표 변환 단위 검사 (ROS·모델 파일 불필요).

    python3 test/test_protocols.py

- Unity SHPOSE / SHABS 패킷 파서
- Isaac ROS 4.6 xr_teleop: controller_data(msgpack) 키, 비활성 포즈 판정, 원시 OpenXR grip 포즈 → ROS 축
- frames.quest_to_ros('openxr') 가 isaac_ros_teleop 의 기본 transform_rotation [0.5,-0.5,-0.5,0.5] 와 같은 변환인지
"""
import os
import sys

import numpy as np
import pinocchio as pin

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from quest_arm_ik.frames import quest_to_ros  # noqa: E402
from quest_arm_ik.protocols import (XR_ENGAGE_KEYS, msgpack_bytes, parse_shabs, parse_shpose,  # noqa: E402
                                    xr_controller_pose, xr_engaged, xr_pose_valid)


def test_sh_packets():
    side, p, q = parse_shpose("SHPOSE,1,R,12.5,0.2088,0.2495,0.7595,0,0,0,1")
    assert side == "right" and np.allclose(p, [0.2088, 0.2495, 0.7595]) and np.allclose(q, [0, 0, 0, 1])
    assert parse_shpose("SHPOSE,2,R,0,0,0,0,0,0,0,1") is None and parse_shpose("garbage") is None
    head = "0,0,1.5,0,0,0,1"
    pkt = f"SHABS,1,3.0,{head},1,-0.2,0.25,0.76,0,0,0,1,0,0.2,0.25,0.76,0,0,0,1"
    d = parse_shabs(pkt)
    assert d["left"][0] is True and d["right"][0] is False and np.allclose(d["right"][1], [0.2, 0.25, 0.76])


def test_xr_controller_data():
    tree = {"timestamp": 1, "left_squeeze_value": 0.9, "right_squeeze_value": 0.1, "left_trigger_value": 0.0,
            "left_is_active": True, "right_is_active": False,
            "left_grip_position": [0.1, 1.2, -0.4], "left_grip_orientation": [0.0, 0.0, 0.0, 1.0],
            "right_grip_position": [0.0, 0.0, 0.0], "right_grip_orientation": [0.0, 0.0, 0.0, 1.0]}
    assert xr_engaged(tree, XR_ENGAGE_KEYS["left"]) is True
    assert xr_engaged(tree, XR_ENGAGE_KEYS["right"]) is False
    assert xr_engaged(tree, "missing_key") is None
    assert xr_controller_pose(tree, "left") == ([0.1, 1.2, -0.4], [0.0, 0.0, 0.0, 1.0])
    assert xr_controller_pose(tree, "right") is None          # 비활성
    assert not xr_pose_valid([0, 0, 0], [0, 0, 0, 1]) and xr_pose_valid([0, 0, 0.5], [0, 0, 0, 1])
    # ByteMultiArray.data 가 길이 1 bytes 리스트로 올 때
    import msgpack
    raw = msgpack.packb(tree)

    class M:  # noqa: D401 - ByteMultiArray 흉내
        data = tuple(bytes([b]) for b in raw)
    assert msgpack.unpackb(msgpack_bytes(M()), raw=False) == tree


def test_openxr_matches_isaac_transform():
    # isaac_ros_teleop: p' = R p, q' = R q R⁻¹ with R = quat[0.5,-0.5,-0.5,0.5] (OpenXR RUB → ROS FLU)
    R = pin.Quaternion(0.5, 0.5, -0.5, -0.5).toRotationMatrix()          # (w, x, y, z)
    rng = np.random.default_rng(0)
    for _ in range(20):
        p = rng.normal(size=3)
        q = pin.Quaternion(pin.exp3(rng.normal(size=3)))
        p_ref = R @ p
        q_ref = pin.Quaternion(R @ q.toRotationMatrix() @ R.T)
        p_my, q_my = quest_to_ros(p, (q.x, q.y, q.z, q.w), "openxr")
        q_my = pin.Quaternion(q_my[3], q_my[0], q_my[1], q_my[2])
        assert np.allclose(p_my, p_ref, atol=1e-9)
        assert np.allclose(q_my.toRotationMatrix(), q_ref.toRotationMatrix(), atol=1e-9)
    # 눈으로: OpenXR 에서 앞(-z)으로 1 m → ROS +x, 위(+y) → +z, 오른쪽(+x) → -y
    assert np.allclose(quest_to_ros([0, 0, -1], [0, 0, 0, 1], "openxr")[0], [1, 0, 0])
    assert np.allclose(quest_to_ros([0, 1, 0], [0, 0, 0, 1], "openxr")[0], [0, 0, 1])
    assert np.allclose(quest_to_ros([1, 0, 0], [0, 0, 0, 1], "openxr")[0], [0, -1, 0])



def test_mock_pitch_axis():
    """quest_mock 의 피치 회전: 정면이 x(G1)든 y(SH)든 '아래를 보던 손끝'이 정면 쪽으로 들리고 좌우로 기울지 않아야 한다."""
    import math
    import numpy as np
    from quest_arm_ik.mock_traj import pitch_quat_xyzw

    def rot(q):
        x, y, z, w = q
        return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                         [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                         [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    down = np.array([0.0, 0.0, -1.0])
    for fwd in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0)):
        f = np.array(fwd)
        d = rot(pitch_quat_xyzw(math.radians(60), fwd)) @ down
        lateral = np.cross([0, 0, 1], f)
        assert abs(d @ lateral) < 1e-9, f"forward={fwd}: 좌우 기울기 {d @ lateral:.3f}"
        assert abs(d @ f - math.sin(math.radians(60))) < 1e-9 and abs(d[2] + math.cos(math.radians(60))) < 1e-9, f"forward={fwd}: {d}"
    assert np.allclose(pitch_quat_xyzw(0.5), pitch_quat_xyzw(0.5, (1.0, 0.0, 0.0)))     # 기본값 = ROS x 정면 (G1, 기존 동작)
    print("  ok  test_mock_pitch_axis (정면 x/y/-x/-y 모두 손끝이 정면으로 들리고 좌우 기울기 0)")

if __name__ == "__main__":
    test_sh_packets()
    test_xr_controller_data()
    test_openxr_matches_isaac_transform()
    test_mock_pitch_axis()
    print("OK")
