"""컨트롤러 → 로봇 손 목표 매핑 (ROS 비의존).

클러치(engage) 방식 상대 매핑:
  engage 순간의 컨트롤러 자세 ctrl0 와 로봇 손 자세 hand0 를 저장하고 이후 변화량만 적용
    p_target = hand0.p + s · R_cal · (p_ctrl − p_ctrl0)
    R_target = R_cal · R_ctrl · R_ctrl0ᵀ · R_calᵀ · hand0.R
  절대 좌표 정렬이 필요 없고, 놓았다 다시 잡으면 이어서(re-index) 조작한다.

팔꿈치 힌트(forearm_dir): 그립 순간 로봇의 전완 방향 d0 를 기준으로, 그 뒤 컨트롤러 회전 변화만큼 돌린 방향
    d = R_cal · R_ctrl · R_ref⁻¹ · R_calᵀ · d0   (사람이 팔을 내린 채 그립을 쥐면 컨트롤러 방향 ≈ 전완 방향이므로, 회전 변화 = 전완 방향 변화)
  트리거로 손목을 돌리는 동안은 컨트롤러 회전이 전완과 무관해지므로 None 을 내고(힌트 끔), 놓는 순간 기준을 다시 잡는다.

회전 클러치(orientation_clutch):
  always  — 그립을 쥔 동안 늘 컨트롤러 회전을 따라간다 (기준 = 그립 순간).
  trigger — 검지 트리거(set_orient_engaged)를 당긴 동안만 따라가고, 기준은 **트리거를 당긴 순간**의 컨트롤러·손 자세.
            당기지 않은 동안은 호출자(IK 노드)가 회전 추종을 끄고 손목을 잠가 손 방향이 전완을 따라가게 한다
            (사람과 팔 비율이 다른 로봇이 손 방향을 맞추느라 손목을 꺾는 것을 막는다).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
import pinocchio as pin

from .frames import OneEuroFilter, RotationLowPass


@dataclass
class MapperParams:
    mode: str = "relative"              # relative: hand0 + s·R_cal·(p−p0) / absolute: R_cal·(p0 + s·(p−p0)) — 입력이 로봇 base 축의 절대 목표(중립+변화량)일 때
    max_step: float = 0.0               # >0 이면 한 주기 손 목표 이동 상한 [m] (튀는 패킷 방어)
    scale: float = 1.0
    R_cal: np.ndarray = None            # 입력 축 → 로봇 베이스 축 회전 (요 정렬), None=단위행렬
    input_timeout: float = 0.5
    stop_on_release: bool = True
    filter_min_cutoff: float = 1.5
    filter_beta: float = 0.5
    orientation_filter_alpha: float = 0.3
    orientation_clutch: str = "always"  # always | trigger (위 모듈 설명)
    forearm_filter_alpha: float = 0.3   # 팔꿈치 힌트 방향의 1차 저역통과 (1 = 없음)


class ArmMapper:
    def __init__(self, side: str, params: MapperParams):
        self.side = side
        self.p = params
        if params.orientation_clutch not in ("always", "trigger"):
            raise ValueError("orientation_clutch: always | trigger")
        self.R_cal = np.eye(3) if params.R_cal is None else np.asarray(params.R_cal, dtype=float)
        self.raw: Optional[pin.SE3] = None
        self.raw_time = -1.0
        self.engaged_flag = False
        self.was_engaged = False
        self.ctrl0: Optional[pin.SE3] = None
        self.hand0: Optional[pin.SE3] = None
        self.pos_filter = OneEuroFilter(params.filter_min_cutoff, params.filter_beta)
        self.rot_filter = RotationLowPass(params.orientation_filter_alpha)
        self.released = False
        self.last_target: Optional[pin.SE3] = None
        # 회전 클러치
        self.orient_flag = False            # 트리거 상태 (set_orient_engaged)
        self.orient_engaged = False         # 실제로 회전을 따라가는 중
        self.orient_changed = False         # 이번 update 에서 orient_engaged 가 바뀜 (호출자가 IK 설정을 바꾸는 신호)
        self.ctrl_o0: Optional[pin.SE3] = None
        self.hand_o0: Optional[pin.SE3] = None
        # 팔꿈치 힌트
        self.fore_ref: Optional[np.ndarray] = None      # 기준 시점의 로봇 전완 방향 (base)
        self.R_fore_ref: Optional[np.ndarray] = None    # 기준 시점의 컨트롤러 회전
        self.forearm_dir: Optional[np.ndarray] = None   # 이번 update 의 추정 전완 방향 (None = 힌트 끔)
        self._fore_filt: Optional[np.ndarray] = None

    def set_raw(self, T: pin.SE3, t: float) -> None:
        self.raw, self.raw_time = T, t

    def set_engaged(self, flag: bool) -> None:
        self.engaged_flag = bool(flag)

    def set_orient_engaged(self, flag: bool) -> None:
        """회전 클러치 입력 (trigger 모드에서만 의미; 검지 트리거 당김)."""
        self.orient_flag = bool(flag)

    @property
    def engaged(self) -> bool:
        return self.was_engaged

    def update(self, now: float, hand_now: pin.SE3, forearm_now: Optional[np.ndarray] = None) -> Optional[pin.SE3]:
        """매 제어 주기 호출. 반환: 새 손 목표(SE3) / None (추종 안 함).
        hand_now: 현재 로봇 손 자세 (engage 순간 hand0 로 저장). forearm_now: 현재 로봇 전완 방향(base, 단위) — 주면 팔꿈치 힌트
        forearm_dir 를 계산한다. 해제 순간에는 released 플래그를 세워 호출자가 hold 하게 한다."""
        self.released = False
        self.orient_changed = False
        self.forearm_dir = None
        stale = self.raw is None or (now - self.raw_time) > self.p.input_timeout
        engaged = self.engaged_flag and not stale
        if engaged and not self.was_engaged:            # 클러치 잡는 순간
            self.ctrl0 = self.raw.copy()
            self.hand0 = hand_now.copy()
            self.pos_filter.reset()
            self.rot_filter.reset()
            self._fore_capture(forearm_now)
        if not engaged and self.was_engaged:            # 놓는 순간
            self.released = self.p.stop_on_release
        self.was_engaged = engaged
        if not engaged:
            if self.orient_engaged:                     # 그립을 놓으면 회전 추종도 끝
                self.orient_engaged, self.orient_changed = False, True
            return None

        # 회전 클러치: always 는 그립과 함께, trigger 는 검지 트리거를 당긴 동안만. 기준은 켜지는 순간의 컨트롤러·손 자세.
        want = True if self.p.orientation_clutch == "always" else self.orient_flag
        if want and not self.orient_engaged:
            self.ctrl_o0, self.hand_o0 = self.raw.copy(), hand_now.copy()
            self.rot_filter.reset()
            self.orient_engaged, self.orient_changed = True, True
        elif not want and self.orient_engaged:
            self.orient_engaged, self.orient_changed = False, True
            if self.p.orientation_clutch == "trigger":
                self._fore_capture(forearm_now)         # 손목을 돌린 뒤이므로 전완 기준을 다시 잡는다

        if self.p.mode == "absolute":       # 그립 순간 입력(=Unity 가 보내는 중립 위치)을 기준으로 변화량만 scale
            p_raw = self.R_cal @ (self.ctrl0.translation + self.p.scale * (self.raw.translation - self.ctrl0.translation))
        else:
            p_raw = self.hand0.translation + self.R_cal @ (self.raw.translation - self.ctrl0.translation) * self.p.scale
        p_t = self.pos_filter(now, p_raw)
        if self.orient_engaged:
            if self.p.mode == "absolute":
                R_raw = self.R_cal @ self.raw.rotation
            else:                            # 회전 기준 = 회전 추종이 켜진 순간 (always 면 그립 순간과 같음)
                R_raw = (self.R_cal @ self.raw.rotation @ self.ctrl_o0.rotation.T @ self.R_cal.T) @ self.hand_o0.rotation
            R_t = self.rot_filter(R_raw)
        else:
            # 회전 추종이 꺼진 동안은 현재 손 자세를 그대로 넘긴다 (IK 쪽은 회전 cost 0 + 손목 잠금이라 쓰지 않음; 혹시 쓰더라도 '지금 자세 유지')
            R_t = hand_now.rotation.copy()
        if self.p.max_step > 0.0:
            d = p_t - hand_now.translation
            n = float(np.linalg.norm(d))
            if n > self.p.max_step:
                p_t = hand_now.translation + d * (self.p.max_step / n)
        # 팔꿈치 힌트: 기준이 있고, (trigger 모드에서) 손목을 돌리는 중이 아니면 전완 방향 추정. always 모드는 늘 추정(손목 굽힘만큼 오차는 감수)
        if self.fore_ref is not None and (not self.orient_engaged or self.p.orientation_clutch == "always"):
            dvec = (self.R_cal @ self.raw.rotation @ self.R_fore_ref.T @ self.R_cal.T) @ self.fore_ref
            a = self.p.forearm_filter_alpha
            self._fore_filt = dvec if (self._fore_filt is None or a >= 1.0) else (a * dvec + (1.0 - a) * self._fore_filt)
            nrm = float(np.linalg.norm(self._fore_filt))
            self.forearm_dir = self._fore_filt / nrm if nrm > 1e-6 else None
        self.last_target = pin.SE3(R_t, p_t)
        return self.last_target

    def _fore_capture(self, forearm_now: Optional[np.ndarray]) -> None:
        """팔꿈치 힌트 기준 (현재 로봇 전완 방향 ↔ 현재 컨트롤러 회전). forearm_now 가 없으면 힌트 없음."""
        if forearm_now is None or self.raw is None:
            self.fore_ref = self.R_fore_ref = None
        else:
            self.fore_ref = np.asarray(forearm_now, dtype=float).copy()
            self.R_fore_ref = self.raw.rotation.copy()
        self._fore_filt = None
