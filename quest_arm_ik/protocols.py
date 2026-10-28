"""입력 패킷/메시지 파서 (ROS 비의존, 테스트 가능).

SHPOSE / SHABS: SH-Humanoid 저장소의 Unity 스크립트(ControllerPoseLogger.cs / AbsolutePoseSender.cs) 형식.
"""
from __future__ import annotations

from typing import Any, Optional

SIDES = ("left", "right")


def msgpack_bytes(msg) -> bytes:
    """ByteMultiArray.data 는 rclpy에서 길이 1 bytes 의 리스트(또는 int 리스트)로 온다."""
    data = msg.data
    if not data:
        return b""
    if isinstance(data[0], (bytes, bytearray)):
        return b"".join(bytes(b) for b in data)
    return bytes(int(b) & 0xFF for b in data)


def lookup(tree: Any, path: str) -> Optional[Any]:
    """'left/grip' 또는 'controllers/0/buttons/1' 식 경로로 dict/list 를 내려간다. 없으면 None."""
    node = tree
    for key in [k for k in path.split("/") if k != ""]:
        if isinstance(node, dict):
            if key in node:
                node = node[key]
            elif key.encode() in node:            # msgpack 이 bytes 키로 줄 때
                node = node[key.encode()]
            else:
                return None
        elif isinstance(node, (list, tuple)):
            try:
                node = node[int(key)]
            except (ValueError, IndexError):
                return None
        else:
            return None
    return node


def parse_shpose(text: str):
    """SHPOSE,1,{L|R},t,x,y,z,qx,qy,qz,qw → (side, p, q) / None"""
    parts = text.strip().split(",")
    if len(parts) != 11 or parts[0] != "SHPOSE" or parts[1] != "1" or parts[2] not in ("L", "R"):
        return None
    v = [float(x) for x in parts[4:11]]
    return ("left" if parts[2] == "L" else "right"), v[0:3], v[3:7]


def parse_shabs(text: str):
    """SHABS,1,t,head(7),gripL,left(7),gripR,right(7) → {side: (grip, p, q)} / None"""
    parts = text.strip().split(",")
    if len(parts) != 26 or parts[0] != "SHABS" or parts[1] != "1":
        return None
    v = [float(x) for x in parts[2:]]
    return {"left": (v[8] > 0.5, v[9:12], v[12:16]), "right": (v[16] > 0.5, v[17:20], v[20:24])}




# ----------------------------------------------------------------- Isaac ROS xr_teleop (isaac_ros_teleop / teleop_ros2_node)
# Isaac ROS 4.6 (IsaacTeleop 1.3.x examples/teleop_ros2) 규약:
#   /xr_teleop/ee_poses        geometry_msgs/PoseArray, poses[0]=왼손, poses[1]=오른손 (컨트롤러 aim 포즈),
#                              transform_rotation 으로 이미 ROS 축(x 정면, y 좌측, z 상방)·STAGE 기준(바닥 원점). 비활성이면 (0,0,0)+단위 쿼터니언
#   /xr_teleop/controller_data std_msgs/ByteMultiArray = msgpack dict (평면 키):
#       {side}_squeeze_value, {side}_trigger_value (0~1), {side}_grip_position/_orientation, {side}_aim_position/_orientation
#       (원시 OpenXR 축: x 우측, y 상방, z 후방), {side}_primary_click, {side}_secondary_click, {side}_thumbstick, {side}_is_active ...
XR_ENGAGE_KEYS = {"left": "left_squeeze_value", "right": "right_squeeze_value"}   # 그립(측면 버튼) → 클러치
XR_GRASP_KEYS = {"left": "left_trigger_value", "right": "right_trigger_value"}     # 검지 트리거(앞쪽 버튼, 0~1) → 손 쥐기


def xr_value(tree: Any, key: str) -> Optional[float]:
    """msgpack dict 의 key 값을 float 로 (bool 은 0/1). 키가 없으면 None."""
    val = lookup(tree, key)
    if val is None:
        return None
    if isinstance(val, bool):
        return 1.0 if val else 0.0
    try:
        return float(val)
    except (TypeError, ValueError):
        return None


def xr_pose_valid(p, q, eps: float = 1e-9) -> bool:
    """teleop_ros2_node 는 비활성 컨트롤러를 (0,0,0) + 단위 쿼터니언으로 채운다 → 그 값은 무시."""
    return not (all(abs(float(v)) < eps for v in p) and
                abs(float(q[0])) < eps and abs(float(q[1])) < eps and abs(float(q[2])) < eps)


def xr_controller_pose(tree: Any, side: str, which: str = "grip"):
    """controller_data(msgpack dict) 에서 한 손의 원시 OpenXR 포즈 (p, q_xyzw) 를 꺼낸다. 없거나 비활성이면 None."""
    active = lookup(tree, f"{side}_is_active")
    if active is False:
        return None
    p = lookup(tree, f"{side}_{which}_position")
    q = lookup(tree, f"{side}_{which}_orientation")
    if p is None or q is None or len(p) != 3 or len(q) != 4:
        return None
    p = [float(v) for v in p]
    q = [float(v) for v in q]
    if not xr_pose_valid(p, q):
        return None
    return p, q


def xr_engaged(tree: Any, key: str, threshold: float = 0.5) -> Optional[bool]:
    """msgpack dict 의 key(경로) 값을 engaged 불리언으로. 키가 없으면 None."""
    val = lookup(tree, key)
    if val is None:
        return None
    if isinstance(val, bool):
        return val
    try:
        return float(val) > threshold
    except (TypeError, ValueError):
        return False
