"""quest_mock 의 컨트롤러 궤적 (ROS 비의존, 테스트 공유).

그립 후 경과 시간 te 에 대해: lift_time 동안 smoothstep 으로 앞 lift_forward·위 lift_up 만큼 올린 뒤, 반지름 radius·주기 period 의
원(정면-상방 평면, 위상 0 = 올린 위치)을 돈다. 손 쥐기 신호 g 는 올린 뒤부터 0→1→0 (grasp_period, 0 이면 항상 0).
올리는 동안 손(컨트롤러)도 아래를 보던 자세에서 lift_pitch 만큼 앞으로 들어 올린다 (사람이 팔을 들 때처럼; 손목이 억지로 아래를 유지하지 않게).
반환: (앞 방향 변위, 위 방향 변위, g, 피치 각[rad]) — 쿼터니언은 pitch_quat_xyzw(pitch, forward) 로 (정면 축 기준 손끝 아래→앞)
"""
from __future__ import annotations

import math
from typing import Tuple


def pitch_quat_xyzw(pitch: float, forward=(1.0, 0.0, 0.0)):
    """아래를 보던 손끝이 정면(forward, 베이스 축) 쪽으로 pitch 만큼 들리는 회전의 쿼터니언 (x, y, z, w).
    회전축 = 왼쪽 방향(z × forward), 각 = -pitch. forward 가 ROS x 축이면 y 축 중심 -pitch (G1), SH 처럼 정면이 +y 면 x 축 중심 +pitch.
    (v0.7.1 까지는 항상 y 축이었다 — 정면이 +y 인 SH 에서는 그것이 롤이 되어 양손이 옆으로 기울고 손목이 뒤틀렸다.)"""
    fx, fy = float(forward[0]), float(forward[1])
    n = math.hypot(fx, fy) or 1.0
    lx, ly = -fy / n, fx / n                       # z × forward (수평 성분만)
    h = -0.5 * pitch
    s = math.sin(h)
    return (lx * s, ly * s, 0.0, math.cos(h))


def mock_offsets(te: float, radius: float, period: float, lift_forward: float, lift_up: float, lift_time: float,
                 grasp_period: float = 0.0, lift_pitch: float = 1.0472) -> Tuple[float, float, float, float]:
    te = max(te, 0.0)
    lt = max(lift_time, 1e-3)
    u = min(te / lt, 1.0)
    ramp = u * u * (3.0 - 2.0 * u)
    phase = 2.0 * math.pi / period * max(te - lt, 0.0)
    forward = lift_forward * ramp + radius * (1.0 - math.cos(phase))
    up = lift_up * ramp + radius * math.sin(phase)
    g = 0.0
    if grasp_period > 0.0 and te > lt:
        g = 0.5 * (1.0 - math.cos(2.0 * math.pi * (te - lt) / grasp_period))
    return forward, up, g, lift_pitch * ramp
