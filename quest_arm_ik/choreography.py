"""전시용 동작 쇼 — 관절 공간 안무 (ROS 비의존). motion_show 노드와 test/test_motion_show.py 가 쓴다.

관절 순서(controlled_joints): [L 어깨 pitch, 어깨 roll, 어깨 yaw, 팔꿈치, 손목 yaw, 손목 roll, 손목 pitch] + 오른팔 같은 순서.
부호(SH_Humanoid_v2, 양팔 동일): 어깨 pitch + 앞으로 들림 / 어깨 roll − 바깥(옆)으로 벌어짐 / 어깨 yaw + 바깥 회전 / 팔꿈치 − 굽힘 /
손목 yaw + 안쪽 / 손목 roll − 앞으로 휨 / 손목 pitch + 바깥(좌우).  모델 좌표: +y 정면, 어깨 z 1.03 m, 몸통 상자 윗면 z ≈ 1.10.

동작(2026-10-27 MuJoCo 렌더로 설계, 모두 자기충돌 0·관절 한계 여유 ≥ 0.10 rad, 손-손(손가락 포함) ≥ 90 mm·손-반대팔 ≥ 130 mm — test_motion_show 가 mj_geomDistance 로 검사):
  heart  머리 위 하트 — 팔을 옆으로 들어(roll −1.90) 팔꿈치를 굽혀(−1.60) 손끝이 위 중앙에서 마주 봄(손 사이 124 mm — 실기체 오차를 생각해 닿지 않게), 두 번 '두근'
  wave   양손 인사(와이퍼) — 팔을 앞·위로 들어 전완을 세우고(팔꿈치 80°) 팔꿈치 위치는 고정한 채 어깨 yaw 로 전완만 양팔 같은 방향으로
         좌우 흔들기 ±0.5 (왼팔 +, 오른팔 − 가 같은 방향), 0.3 Hz × 3회, 손목 yaw +1.0 으로 손바닥은 정면
  run    달리기 상체 — 팔꿈치 60° 로 고정, 한 팔이 앞으로 가면 반대 팔은 뒤로 (어깨 pitch ±60°, 반대 위상), 0.25 Hz × 2회 — 관절 속도 1.64 rad/s·가속 4.1 rad/s² (다른 동작은 ≤ 1.0)
  kiss   손 키스 — 양손을 '입'(몸통 위 앞 중앙) 앞으로 모았다가 앞·바깥으로 던지기 × 2

쇼 = [REST → act1 → … → actN → REST]. 동작 사이는 자동 전환(최소 저크, 관절 최대 속도 v_max 안에서 시간을 정함).
키프레임 사이는 최소 저크(양끝 속도 0), 주기 동작은 기본 자세 + 사인(1 s 램프 포장) 이라 급가속이 없다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

N = 14
SHOW_V_MAX = 2.0               # 쇼 전체의 관절 속도 상한 [rad/s] (run ±60° 가 1.64; 나머지 동작·전환은 1.0 안) — motion_show 의 틱당 제한과 launch 스트리머 v_max 의 기준
TRANSITION_V_MAX = 1.0         # 동작 사이 전환의 최대 속도
REST = np.array([0.0, 0.0, 0.0, -0.15, 0.0, 0.0, 0.0] * 2)          # 팔 내린 자세 (mujoco_sim initial_posture 와 같음)
# URDF 관절 한계 (IK 와 같이 팔꿈치는 [-1.8, 0]) — 안무는 이 안쪽 margin 이상에서만
LIMITS = np.array([[-2.618, 2.618], [-2.094, 2.094], [-1.571, 1.571], [-1.8, 0.0], [-1.484, 1.484], [-1.571, 1.571], [-1.396, 1.396]] * 2)
JOINT_NAMES = ["left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint",
               "left_wrist_yaw_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint",
               "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
               "right_wrist_yaw_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint"]


def both(sp: float, sr: float, sy: float, el: float, wy: float = 0.0, wr: float = 0.0, wp: float = 0.0) -> np.ndarray:
    """양팔 같은 값 (부호 정의가 양팔 동일하므로 거울 대칭)."""
    return np.array([sp, sr, sy, el, wy, wr, wp] * 2, dtype=float)


def minjerk(s: float) -> float:
    s = min(max(s, 0.0), 1.0)
    return s * s * s * (10.0 + s * (-15.0 + 6.0 * s))


def ramp(t: float, t_on: float, t_off: float, r: float = 1.0) -> float:
    """t_on 에서 r 초 동안 0→1, t_off 에서 r 초 동안 1→0 (사인 동작 포장)."""
    if t < t_on:
        return 0.0
    if t < t_on + r:
        return minjerk((t - t_on) / r)
    if t < t_off - r:
        return 1.0
    if t < t_off:
        return 1.0 - minjerk((t - (t_off - r)) / r)
    return 0.0


def keyframes(frames: Sequence[Tuple[float, np.ndarray]]) -> Callable[[float], np.ndarray]:
    """[(t, q)] (t 오름차순) → q(t): 구간마다 최소 저크, 마지막 뒤는 유지."""
    ts = [float(t) for t, _ in frames]
    qs = [np.asarray(q, dtype=float) for _, q in frames]

    def f(t: float) -> np.ndarray:
        if t <= ts[0]:
            return qs[0].copy()
        for k in range(1, len(ts)):
            if t <= ts[k]:
                s = minjerk((t - ts[k - 1]) / max(1e-6, ts[k] - ts[k - 1]))
                return qs[k - 1] + s * (qs[k] - qs[k - 1])
        return qs[-1].copy()
    return f


@dataclass
class Act:
    name: str
    duration: float
    fn: Callable[[float], np.ndarray]          # t ∈ [0, duration] → q(14)

    def q(self, t: float) -> np.ndarray:
        return np.asarray(self.fn(min(max(t, 0.0), self.duration)), dtype=float)


# ------------------------------------------------------------------ 동작 정의
HEART = both(-0.2, -1.90, 1.40, -1.60, 0.0, -0.30, -0.15)            # 손(손가락 포함) 사이 124 mm, 손끝 높이 1.40 m, 한계 여유 0.19(어깨 roll -2.094 기준)
#   v0.7.56: 이전 (손목 roll -0.5·pitch -0.3) 은 두 손 사이가 30 mm 라 실기체(중력 처짐·영점 오차 수 cm)에서 손끝이 서로 눌려 가장 무른 손목 pitch 가 꺾였다
#   (2026-10-28 "오른팔 마지막 관절이 혼자 꺾임"). 손목을 덜 굽혀 손 사이를 ≥ 90 mm 로 — 하트 모양은 팔이 만들고 손은 위 중앙에서 가까이 마주 보기만
HEART_MID = both(-0.2, -1.2, 0.7, -1.0)                                # 옆으로 든 중간 자세 (몸통을 스치지 않는 경로)
WAVE_BASE = both(1.4, -0.3, 0.0, -1.4, 1.0)                            # 팔 앞·위, 전완 수직(와이퍼 서 있는 자세), 팔꿈치 80°, 손목 yaw +1.0 = 손바닥이 정면(+y 성분 0.93; 0 이면 손바닥이 서로 마주봄)
RUN_BASE = both(0.0, -0.15, 0.0, -math.radians(60))                    # 팔꿈치 60°, 어깨 pitch 0 을 중심으로 ±60°
KISS_MOUTH = both(1.02, -0.15, -0.12, -1.60)                           # 입 앞: 손끝 (∓0.14, 0.36, 1.29), 두 손(엄지 포함) 사이 92 mm — v0.7.56 (이전 IK 자세는 엄지가 겹쳤음 -2 mm)
KISS_THROW = np.array([1.75, -0.42, 0.22, -0.60, 0, 0, 0, 1.75, -0.41, 0.21, -0.60, 0, 0, 0])      # 앞·바깥으로 뻗음 (팔꿈치 -0.6 로 속도 제한)


def act_heart() -> Act:
    """옆으로 들기(3 s) → 하트(3 s) → 두근 × 2(4 s) → 옆으로 내리기(3 s)."""
    base = keyframes([(0.0, HEART_MID), (3.0, HEART), (10.0, HEART), (13.0, HEART_MID)])

    def f(t):
        q = base(t)
        a = ramp(t, 4.0, 10.0, 0.8) * 0.08 * (0.5 - 0.5 * math.cos(2 * math.pi * 1.0 * (t - 4.0)))   # 팔꿈치 0~0.08 rad, 1 Hz
        q[[3, 10]] += a
        return q
    return Act("heart", 13.0, f)


def act_wave(cycles: int = 3, freq: float = 0.3, amp: float = 0.5) -> Act:
    """와이퍼: 상완(팔꿈치 위치)은 고정, 어깨 yaw 만 돌려 전완·손이 좌우로 흔들림. 왼팔 +yaw 와 오른팔 −yaw 가 같은 방향(로봇 왼쪽)이라
    부호를 반대로 주면 두 전완이 나란히 움직인다. 피크 속도 2π·f·amp = 0.94 rad/s."""
    seconds = cycles / freq
    def f(t):
        q = WAVE_BASE.copy()
        e = ramp(t, 0.0, seconds, 1.2)                  # 램프 1.2 s: 포장 × 사인의 합성 속도도 v_max 안 (0.94)
        s = math.sin(2 * math.pi * freq * t)
        q[2] += e * amp * s;  q[9] -= e * amp * s
        return q
    return Act("wave", seconds, f)


def act_run(cycles: int = 2, freq: float = 0.25, amp: float = math.radians(60)) -> Act:
    """팔꿈치 60° 고정, 어깨 pitch 를 양팔 반대 위상으로 ±amp(60°) (한 팔 앞 ↔ 반대 팔 뒤), cycles 회.
    피크 속도 2π·f·amp = 1.64 rad/s, 가속 ω²·amp = 2.6 (램프 포함 4.1) rad/s² — 쇼의 v_max 2.0, 스트리머 a_max 4 안. 0.3 Hz 면 1.97 / 4.9."""
    seconds = cycles / freq
    def f(t):
        q = RUN_BASE.copy()
        e = ramp(t, 0.0, seconds, 2.0)                  # 램프 2.0 s: 포장 × 사인의 합성 가속이 a_max 4 를 넘지 않게
        s = math.sin(2 * math.pi * freq * t)
        q[0] += e * amp * s;  q[7] -= e * amp * s
        return q
    return Act("run", seconds, f)


def act_kiss() -> Act:
    """입 앞 유지(0.8 s) → 던지기(2.6 s) → 유지(0.8) → 입 앞(2.2) → 유지(0.8) → 던지기(2.6) → 유지(1.0)."""
    return Act("kiss", 10.8, keyframes([(0.0, KISS_MOUTH), (0.8, KISS_MOUTH), (3.4, KISS_THROW), (4.2, KISS_THROW),
                                        (6.4, KISS_MOUTH), (7.2, KISS_MOUTH), (9.8, KISS_THROW), (10.8, KISS_THROW)]))


ACTS: Dict[str, Callable[[], Act]] = {"heart": act_heart, "wave": act_wave, "run": act_run, "kiss": act_kiss}


# ------------------------------------------------------------------ 쇼 (동작 + 자동 전환)
@dataclass
class Show:
    segments: List[Tuple[float, float, str, Callable[[float], np.ndarray]]] = field(default_factory=list)   # (t0, t1, name, fn(t-t0))

    @property
    def duration(self) -> float:
        return self.segments[-1][1] if self.segments else 0.0

    def q(self, t: float) -> np.ndarray:
        if not self.segments:
            return REST.copy()
        if t <= 0.0:
            return self.segments[0][3](0.0)
        for t0, t1, _, fn in self.segments:
            if t <= t1:
                return fn(t - t0)
        t0, t1, _, fn = self.segments[-1]
        return fn(t1 - t0)

    def name_at(self, t: float) -> str:
        for t0, t1, name, _ in self.segments:
            if t <= t1:
                return name
        return self.segments[-1][2] if self.segments else ""


def transition_time(q0: np.ndarray, q1: np.ndarray, v_max: float, t_min: float = 1.5) -> float:
    """최소 저크로 q0→q1 갈 때 최대 관절 속도(1.875·Δ/T)가 v_max 를 넘지 않는 시간."""
    d = float(np.max(np.abs(np.asarray(q1) - np.asarray(q0))))
    return max(t_min, 1.875 * d / max(1e-6, v_max))


def build_show(acts: Sequence[str] = ("heart", "wave", "run", "kiss"), speed: float = 1.0, v_max: float = TRANSITION_V_MAX,
               start: Optional[np.ndarray] = None, end_rest: bool = True) -> Show:
    """REST(또는 start) → 전환 → act → 전환 → … → REST. speed 는 동작·전환 시간을 1/speed 배 (0.5 = 절반 속도)."""
    show = Show()
    t = 0.0
    q_prev = REST.copy() if start is None else np.asarray(start, dtype=float).copy()
    for name in acts:
        if name not in ACTS:
            raise ValueError(f"알 수 없는 동작 '{name}' — {list(ACTS)}")
        act = ACTS[name]()
        q_in = act.q(0.0)
        T = transition_time(q_prev, q_in, v_max * speed) / speed
        show.segments.append((t, t + T, f"→{name}", keyframes([(0.0, q_prev), (T, q_in)])))
        t += T
        D = act.duration / speed
        show.segments.append((t, t + D, name, (lambda a, sp: (lambda tt: a.q(tt * sp)))(act, speed)))
        t += D
        q_prev = act.q(act.duration)
    if end_rest:
        T = transition_time(q_prev, REST, v_max * speed) / speed
        show.segments.append((t, t + T, "→rest", keyframes([(0.0, q_prev), (T, REST)])))
    return show


def check_trajectory(show: Show, dt: float = 0.01, v_max: float = 1.0, margin: float = 0.1):
    """관절 한계 여유·최대 속도를 전 구간에서 검사. 반환 (q 배열, 최대 속도, 최소 한계 여유). (run 은 1.64 rad/s 로 설계 — SHOW_V_MAX 참고)"""
    ts = np.arange(0.0, show.duration + dt, dt)
    Q = np.stack([show.q(t) for t in ts])
    V = np.abs(np.diff(Q, axis=0)) / dt
    vmax = float(V.max()) if len(V) else 0.0
    lim_margin = float(np.min(np.minimum(Q - LIMITS[:, 0], LIMITS[:, 1] - Q)))
    return Q, vmax, lim_margin
