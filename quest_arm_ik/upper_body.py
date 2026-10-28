"""삼현 HRController(UpperBodyCommand/UpperMotorState) 연동용 ROS 비의존 로직.

- MotorMap   : URDF 관절각 ↔ 모터각 (motor_ids 순서, 부호, 오프셋). 15 슬롯 메시지의 남는 자리는 extra_motor_ids(예: 15번) 로 채운다.
- Streamer   : IK 가 100 Hz 로 주는 관절 목표를 매끈한 명령열로 바꾼다 (속도 v_max·가속 a_max 안에서).
               v0.7.47: 속도 지령 = 목표 속도(연속 목표의 차분, 저역) + 제동 거리 법칙 sign(e)·min(√(2·a_max·|e|), |e|/dt) — a_max 로 감속해 목표에
               딱 멈추는 시간최적 접근. 이전의 '(목표−명령)/dt 를 v_max 로 자른' 데드비트는 가속 제한과 만나면 멈춘 목표 둘레를 ±v²/2a 로 영원히
               진동했다(2026-10-26 실기체 2 Hz '덜덜' 의 원인 — diag npz 로 확인). IK 가 끊기면(ik_timeout) 그 자리에서 정지, 측정값과 너무
               벌어지면(max_tracking_error) 정지 + 경고. 첫 명령은 항상 측정 모터각에서 시작한다(점프 금지).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

import numpy as np

N_SLOTS = 15
EMPTY_MOTOR_ID = 255           # 삼현 ros2_state_bridge.py: motor_id 255 = 빈 슬롯(조용히 건너뜀). 0 은 unknown motor_id 경고                                   # UpperBodyCommand / UpperMotorState 배열 길이

# UpperBodyCommand.command_mode
MODE_STOP, MODE_POSITION, MODE_VELOCITY, MODE_TORQUE, MODE_POSITION_VELOCITY, MODE_IMPEDANCE = 0, 1, 2, 3, 4, 5
# UpperBodySystemState.state
STATE_NAMES = {0: "POWER_OFF", 1: "DRIVER_INIT", 2: "READY", 3: "ENABLING", 4: "ENABLED", 5: "RUNNING", 6: "FAULT"}


@dataclass
class MotorMap:
    motor_ids: Sequence[int]                   # controlled_joints 순서의 모터 ID (14개)
    signs: Sequence[float]                     # 모터각 = sign * 관절각 + offset
    offsets: Sequence[float]
    extra_motor_ids: Sequence[int] = field(default_factory=list)   # 우리가 제어하지 않는 모터 (현재 위치 유지로 채움)

    def __post_init__(self):
        n = len(self.motor_ids)
        if len(self.signs) != n or len(self.offsets) != n:
            raise ValueError("motor_ids / joint_signs / joint_offsets 길이가 다름")
        if len(set(self.motor_ids) | set(self.extra_motor_ids)) != n + len(self.extra_motor_ids):
            raise ValueError("motor_ids 에 중복이 있음")
        if n + len(self.extra_motor_ids) > N_SLOTS:
            raise ValueError(f"모터 수 {n + len(self.extra_motor_ids)} > {N_SLOTS}")
        self.signs = np.asarray(self.signs, dtype=float)
        self.offsets = np.asarray(self.offsets, dtype=float)

    def to_motor(self, q: Sequence[float]) -> np.ndarray:
        return np.asarray(q, dtype=float) * self.signs + self.offsets

    def to_joint(self, m: Sequence[float]) -> np.ndarray:
        return (np.asarray(m, dtype=float) - self.offsets) * self.signs      # sign ∈ {±1} 이라 나누기 대신 곱

    def vel_to_motor(self, qd: Sequence[float]) -> np.ndarray:
        return np.asarray(qd, dtype=float) * self.signs

    def joints_from_state(self, state_ids: Sequence[int], state_pos: Sequence[float]) -> Optional[np.ndarray]:
        """UpperMotorState(motor_id, position) → controlled_joints 순서 관절각. 모터가 빠져 있으면 None."""
        ids = list(state_ids)
        try:
            idx = [ids.index(m) for m in self.motor_ids]
        except ValueError:
            return None
        return self.to_joint(np.asarray(state_pos, dtype=float)[idx])


@dataclass
class StreamerParams:
    rate: float = 250.0                        # 명령 발행 주기 [Hz]
    v_max: float = 0.8                         # 관절 속도 상한 [rad/s] — 첫날은 0.3~0.5
    a_max: float = 8.0                         # 가속도 상한 [rad/s²] (0 이면 끔)
    ik_timeout: float = 0.2                    # IK 목표가 이 시간 이상 안 오면 정지
    max_tracking_error: float = 0.35           # 명령-측정 차이가 이보다 크면 정지 (충돌·모터 불능 감지)
    settle_tolerance: float = 0.02             # 시작 시 측정값에 이만큼 가까워질 때까지는 '접근 중'
    ff_alpha: float = 0.5                      # 목표 속도 피드포워드의 1차 저역 계수 (1 = 필터 없음, 0 = 피드포워드 끔)
    v_catchup: float = 0.3                     # 정지(미활성·HOLD)에서 추종을 (다시) 시작할 때 목표까지 접근하는 속도 상한 [rad/s] (0 = v_max 그대로).
                                               # 2026-10-28: enable 순간 쇼 목표가 멀리 있어 v_max 2.5 로 튐 → 멀리 있는 목표에는 천천히 간다
    catchup_done: float = 0.02                 # 목표와 이만큼 가까워지면 접근 끝 → v_max
    hold_recover_time: float = 1.0             # 추종 오차 정지 뒤 다시 접근을 시작하기까지 [s]
    max_tracking_retries: int = 3              # 목표에 한 번도 닿지 못한 채 추종 오차 정지가 이 횟수면 정지 유지 (hold 토글·재활성화·목표 복귀로 해제; 0 = 무제한)


class Streamer:
    """상태 기계: IDLE(측정값 대기) → TRACK(목표 추종) → HOLD(정지; IK 끊김/오차 초과/외부 hold). 모든 명령은 q_cmd 연속.
    정지에서 추종으로 (다시) 들어갈 때는 목표까지 v_catchup 으로 천천히 간다(catching_up). 추종 오차 정지는 명령을 측정값으로 되돌린다."""

    def __init__(self, n: int, params: StreamerParams):
        self.n, self.p = n, params
        self.dt = 1.0 / params.rate
        self.q_cmd: Optional[np.ndarray] = None
        self.qd_cmd = np.zeros(n)
        self.target: Optional[np.ndarray] = None
        self.t_target = -1.0
        self.v_target = np.zeros(n)                              # 목표 속도 추정 (피드포워드)
        self.measured: Optional[np.ndarray] = None
        self.hold_reason = ""
        self.external_hold = False
        self.state = "IDLE"
        self.catching_up = True                                  # 정지에서 추종으로 들어가 목표에 아직 못 닿음 → 속도 상한 v_catchup
        self.t_hold_until = -1.0                                 # 추종 오차 정지 뒤 회복 대기 끝 시각
        self.n_track_fail = 0                                    # 목표에 닿지 못한 채 반복된 추종 오차 정지 횟수
        self.track_latched = False                               # max_tracking_retries 초과 → 정지 유지

    # ---- 입력
    def set_measured(self, q_meas: Sequence[float]) -> None:
        self.measured = np.asarray(q_meas, dtype=float)
        if self.q_cmd is None:                                   # 첫 명령 = 현재 측정값 (점프 금지)
            self.q_cmd = self.measured.copy()

    def reset_to_measured(self) -> None:
        """명령을 내지 않는 동안(드라이버 미활성 등) 호출: q_cmd 를 측정값에 고정해 두어, 발행을 시작하는 첫 명령이 점프하지 않게 한다.
        다시 발행을 시작하면 목표까지 v_catchup 으로 천천히 간다."""
        if self.measured is not None:
            self.q_cmd = self.measured.copy()
        self.qd_cmd = np.zeros(self.n)
        self.state, self.hold_reason = "IDLE", ""
        self.catching_up = True
        self.t_hold_until, self.n_track_fail, self.track_latched = -1.0, 0, False

    def set_target(self, q: Sequence[float], t: float) -> None:
        q = np.asarray(q, dtype=float)
        if self.target is not None and (t - self.t_target) <= 1e-4:
            self.target = q                                      # 같은 시각의 중복 호출: 속도 추정은 그대로
            return
        if self.target is not None and (t - self.t_target) < 0.1 and self.p.ff_alpha > 0.0:
            v_new = (q - self.target) / (t - self.t_target)
            self.v_target = self.p.ff_alpha * v_new + (1.0 - self.p.ff_alpha) * self.v_target
        else:
            self.v_target = np.zeros(self.n)                     # 첫 목표 / 끊겼다 다시 옴
        self.target = q
        self.t_target = t

    def set_hold(self, on: bool, reason: str = "external") -> None:
        self.external_hold = on
        if on:
            self.hold_reason = reason
        else:                                                    # hold 해제 = 조작자의 확인: 추종 오차 정지 유지도 푼다 (천천히 다시 접근)
            self.t_hold_until, self.n_track_fail, self.track_latched = -1.0, 0, False

    def distance_to_target(self) -> float:
        if self.target is None or self.q_cmd is None:
            return 0.0
        return float(np.max(np.abs(self.target - self.q_cmd)))

    # ---- 한 틱
    def tick(self, t: float) -> Optional[np.ndarray]:
        """q_cmd 를 갱신해 돌려준다 (None = 아직 측정값이 없어 명령 보류)."""
        if self.q_cmd is None:
            self.state = "IDLE"
            return None
        goal = None
        err = self.tracking_error()
        if self.external_hold:
            self.state, self.hold_reason = "HOLD", self.hold_reason or "external"
        elif self.target is None or (t - self.t_target) > self.p.ik_timeout:
            self.state, self.hold_reason = "HOLD", "ik timeout" if self.target is not None else "no ik target"
        elif self.track_latched and self.distance_to_target() > self.p.catchup_done:
            self.state = "HOLD"                                  # 추종 오차 정지 유지 (목표가 명령 자리로 돌아오면 해제 — 텔레옵 IK 재동기화)
        elif t < self.t_hold_until:
            self.state = "HOLD"                                  # 추종 오차 정지 뒤 회복 대기 (hold_reason 유지)
        elif self.measured is not None and err > self.p.max_tracking_error:
            # 추종 오차 정지: 명령을 측정값(로봇이 실제 있는 곳)으로 되돌려 드라이브가 더 당기지 않게 하고(2026-10-28: 명령을 0.7 rad 앞에 세워 두면
            # 드라이브가 거기까지 전속으로 간다), hold_recover_time 뒤 v_catchup 으로 다시 접근한다. 목표에 닿지 못한 채 반복되면 정지 유지.
            self.n_track_fail += 1
            self.q_cmd = self.measured.copy()
            self.qd_cmd = np.zeros(self.n)
            self.t_hold_until = t + self.p.hold_recover_time
            self.hold_reason = f"tracking error {err:.2f} rad"
            if self.p.max_tracking_retries > 0 and self.n_track_fail >= self.p.max_tracking_retries:
                self.track_latched = True
                self.hold_reason += f" x{self.n_track_fail} — 정지 유지 (hold 토글·재활성화·목표 복귀로 해제)"
            self.state = "HOLD"
            return self.q_cmd
        else:
            if self.state != "TRACK":
                self.catching_up = True                          # 정지 → 추종: 목표까지 천천히
                self.track_latched = False
            self.state, self.hold_reason = "TRACK", ""
            goal = self.target
        if goal is None:                                          # 정지: 속도를 0 으로 (가속도 제한 안에서)
            qd_new = np.zeros(self.n)
        else:
            e = goal - self.q_cmd
            catch = self.catching_up and self.p.v_catchup > 0.0
            if self.catching_up:
                # 접근 끝 판정: 남은 거리가 catchup_done 이하이면서 제동 거리 법칙이 v_catchup 보다 빠른 속도를 주지 않는 거리(v²/2a) 안
                done_tol = min(self.p.catchup_done, self.p.v_catchup ** 2 / (2.0 * self.p.a_max)) if (catch and self.p.a_max > 0) else self.p.catchup_done
                if np.max(np.abs(e)) <= done_tol:
                    self.catching_up = False                     # 목표에 닿음 → 다음 틱부터 v_max, 추종 실패 횟수 초기화
                    self.n_track_fail = 0
            v_cap = min(self.p.v_max, self.p.v_catchup) if catch else self.p.v_max
            if self.p.a_max > 0:
                # 제동 거리 법칙: 지금 속도로 a_max 감속해 목표에 멈출 수 있는 최대 속도 √(2·a·|e|) 까지만 — 멈춘 목표를 지나치지 않는다.
                # 목표가 움직이면 그 속도를 더한다(피드포워드). 목표가 갑자기 서면(IK 출력은 속도 한계에서 즉시 멈춤) v²/2a 만큼은 지나쳤다 돌아오지만 진동하지 않는다.
                v_brake = np.sqrt(2.0 * self.p.a_max * np.abs(e))
                v_ff = self.v_target if ((t - self.t_target) < 0.05 and not catch) else np.zeros(self.n)   # 새 목표가 안 오면(멈춤) 피드포워드도 끔
                qd_new = v_ff + np.sign(e) * np.minimum(v_brake, np.abs(e) / self.dt)
            else:
                qd_new = e / self.dt
            qd_new = np.clip(qd_new, -v_cap, v_cap)
        if self.p.a_max > 0:
            dv = self.p.a_max * self.dt
            qd_new = self.qd_cmd + np.clip(qd_new - self.qd_cmd, -dv, dv)
        self.q_cmd = self.q_cmd + qd_new * self.dt
        self.qd_cmd = qd_new
        return self.q_cmd

    def tracking_error(self) -> float:
        if self.measured is None or self.q_cmd is None:
            return 0.0
        return float(np.max(np.abs(self.q_cmd - self.measured)))


def fill_command(mapping: MotorMap, q_cmd: Sequence[float], qd_cmd: Sequence[float], kp: float, kd: float,
                 extra_positions: Optional[Dict[int, float]] = None, extra_kp: float = 0.0, extra_kd: float = 0.0,
                 effort: Optional[Sequence[float]] = None, slot_order: Optional[Sequence[int]] = None,
                 empty_id: int = EMPTY_MOTOR_ID) -> Dict[str, List[float]]:
    """UpperBodyCommand 의 15 슬롯 배열 내용 (dict) — ROS 메시지에 그대로 복사한다. 빈 슬롯은 motor_id = empty_id(기본 255 = 삼현 브리지의
    '모터 없음' 값; 0 을 주면 브리지가 메시지마다 unknown motor_id 경고를 낸다), kp/kd 0.
    slot_order: 드라이버 motor_states 의 motor_id 배열(슬롯 순서). 주면 각 모터를 **같은 슬롯 번호**에 넣는다
    (드라이버가 motor_id 가 아니라 슬롯 번호(axis)로 명령을 찾아도 맞도록). 없으면 controlled 순서대로 앞에서부터."""
    ids = [int(empty_id)] * N_SLOTS
    used = [False] * N_SLOTS
    pos = [0.0] * N_SLOTS
    vel = [0.0] * N_SLOTS
    eff = [0.0] * N_SLOTS
    kps = [0.0] * N_SLOTS
    kds = [0.0] * N_SLOTS
    m_pos = mapping.to_motor(q_cmd)
    m_vel = mapping.vel_to_motor(qd_cmd)
    m_eff = mapping.vel_to_motor(effort) if effort is not None else None
    order = [int(m) for m in slot_order] if slot_order is not None else []

    def slot_for(mid: int, fallback: int) -> int:
        if mid in order:
            return order.index(mid)
        k = fallback
        while k < N_SLOTS and used[k]:
            k += 1
        return k

    for i, mid in enumerate(mapping.motor_ids):
        k = slot_for(int(mid), i)
        if k >= N_SLOTS:
            raise ValueError("빈 슬롯이 없음")
        ids[k], pos[k], vel[k], kps[k], kds[k] = int(mid), float(m_pos[i]), float(m_vel[i]), float(kp), float(kd)
        used[k] = True
        if m_eff is not None:
            eff[k] = float(m_eff[i])
    for mid in mapping.extra_motor_ids:
        k = slot_for(int(mid), len(mapping.motor_ids))
        if k >= N_SLOTS:
            raise ValueError("빈 슬롯이 없음")
        ids[k] = int(mid); used[k] = True
        if extra_positions and mid in extra_positions:
            pos[k], kps[k], kds[k] = float(extra_positions[mid]), float(extra_kp), float(extra_kd)
    return {"motor_id": ids, "position": pos, "velocity": vel, "effort": eff, "kp": kps, "kd": kds}
