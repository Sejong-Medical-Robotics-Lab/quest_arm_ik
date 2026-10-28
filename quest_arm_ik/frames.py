"""좌표계 변환·필터·SE(3) 유틸리티 (ROS 비의존).

Quest 쪽 좌표계 → ROS 좌표계(x 전방, y 좌측, z 상방, 오른손계) 변환과
One-Euro 필터(위치), slerp 저역통과(자세)를 제공한다.
"""
from __future__ import annotations

import math

import numpy as np
import pinocchio as pin


def quest_to_ros(p, q, convention: str):
    """위치 p=(x,y,z), 쿼터니언 q=(x,y,z,w)를 ROS 축으로 변환한다.

    convention:
      unity    : 왼손계, X 우측 / Y 상방 / Z 전방  (Unity, ROS-TCP-Connector) → ROS 축(x 전방, y 좌측, z 상방)
      openxr   : 오른손계, X 우측 / Y 상방 / -Z 전방 (네이티브 OpenXR, WebXR) → ROS 축
      unity_sh : Unity → SH-Humanoid base_link 축(X 우측, Y 전방, Z 상방). RobotBaseLinkCoordinates.cs 와 동일
      ros      : 이미 목표 축 → 그대로 통과
    반환: (p_ros (3,), q_ros (x,y,z,w))
    """
    x, y, z = (float(v) for v in p)
    qx, qy, qz, qw = (float(v) for v in q)
    if convention == "ros":
        return np.array([x, y, z]), np.array([qx, qy, qz, qw])
    if convention == "unity":
        # 축 매핑이 반사(handedness flip)이므로 회전각 부호가 뒤집힘 → 벡터부 부호 반전
        return np.array([z, -x, y]), np.array([-qz, qx, -qy, qw])
    if convention == "openxr":
        # 순수 회전 매핑 → 벡터부도 같은 행렬로 변환
        return np.array([-z, -x, y]), np.array([-qz, -qx, qy, qw])
    if convention == "unity_sh":
        # (x, y, z) → (x, z, y): 반사 매핑이므로 회전 벡터부 부호 반전 (Unity 스크립트와 동일식)
        return np.array([x, z, y]), np.array([-qx, -qz, -qy, qw])
    raise ValueError(f"unknown convention '{convention}' (unity | openxr | unity_sh | ros)")


def se3_from_pq(p, q_xyzw) -> pin.SE3:
    """위치 + (x,y,z,w) 쿼터니언 → pin.SE3."""
    quat = pin.Quaternion(float(q_xyzw[3]), float(q_xyzw[0]),
                          float(q_xyzw[1]), float(q_xyzw[2]))
    quat.normalize()
    return pin.SE3(quat.toRotationMatrix(), np.asarray(p, dtype=float))


def quat_xyzw_from_rotation(R: np.ndarray):
    quat = pin.Quaternion(np.asarray(R, dtype=float))
    quat.normalize()
    return float(quat.x), float(quat.y), float(quat.z), float(quat.w)


def rot_z(yaw_rad: float) -> np.ndarray:
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class OneEuroFilter:
    """n차원 신호용 One-Euro 필터 (Casiez et al., 2012).

    min_cutoff: 정지 상태 잡음 억제(작을수록 부드럽고 느림)
    beta      : 빠른 움직임에서 지연 감소(클수록 빠른 동작을 덜 지연)
    """

    def __init__(self, min_cutoff: float = 1.0, beta: float = 0.0, d_cutoff: float = 1.0):
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff)
        self.reset()

    def reset(self) -> None:
        self._t = None
        self._x = None
        self._dx = None

    @staticmethod
    def _alpha(cutoff: float, dt: float) -> float:
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def __call__(self, t: float, x) -> np.ndarray:
        x = np.asarray(x, dtype=float)
        if self._t is None:
            self._t, self._x, self._dx = t, x.copy(), np.zeros_like(x)
            return x.copy()
        dt = max(t - self._t, 1e-4)
        self._t = t
        dx = (x - self._x) / dt
        a_d = self._alpha(self.d_cutoff, dt)
        self._dx = a_d * dx + (1.0 - a_d) * self._dx
        cutoff = self.min_cutoff + self.beta * float(np.linalg.norm(self._dx))
        a = self._alpha(cutoff, dt)
        self._x = a * x + (1.0 - a) * self._x
        return self._x.copy()


class RotationLowPass:
    """회전행렬용 1차 slerp 저역통과. alpha∈(0,1], 1이면 필터 없음."""

    def __init__(self, alpha: float = 0.3):
        self.alpha = float(alpha)
        self.reset()

    def reset(self) -> None:
        self._q = None

    def __call__(self, R: np.ndarray) -> np.ndarray:
        q_new = pin.Quaternion(np.asarray(R, dtype=float))
        q_new.normalize()
        if self._q is None or self.alpha >= 1.0:
            self._q = q_new
        else:
            self._q = self._q.slerp(self.alpha, q_new)
            self._q.normalize()
        return self._q.toRotationMatrix()
