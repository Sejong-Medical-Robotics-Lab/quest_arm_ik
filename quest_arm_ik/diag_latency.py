"""텔레옵 체인 지연·떨림 측정 (실기체·시뮬 공용, 한 번 돌리고 표로 끝).

    ros2 run quest_arm_ik diag_latency                                   # 오른팔, 정지 3 s + 동작 20 s
    ros2 run quest_arm_ik diag_latency --ros-args -p side:=left -p seconds:=30 -p v_max:=1.0

절차: 텔레옵이 돌고 있는 상태(그립 쥔 채)에서 실행 → 처음 hold_seconds 동안은 손을 **가만히** → 그 뒤 seconds 동안 평소처럼
팔을 앞뒤·위아래로 천천히 반복(0.5~1 Hz). 끝나면 아래 표를 찍고 종료한다.

  입력률       /teleop/{side}_hand/target_raw (quest_bridge 가 낸 컨트롤러 포즈) Hz
  정지 떨림    처음 hold_seconds 동안 target_raw 위치의 표준편차·최대 편차 [mm]  (헤드셋 추적 품질)
  입력 추적    연속 샘플 간 5 cm 넘는 점프 횟수·150 ms 넘는 끊김 횟수 (0 이어야 정상; 있으면 헤드셋이 컨트롤러를 놓치는 것)
  필터 지연    target_raw → /quest_ik/{side}_hand/target (One-Euro 필터 + 매핑) [ms]  — 속도 신호 교차상관
  드라이브 지연 /teleop/joint_targets (IK 출력) → /upper_body/motor_states (측정 관절) [ms]  — 그 팔 7관절 속도 교차상관
  속도 포화율  IK 출력 관절 속도가 v_max 의 95 % 이상인 틱 비율 [%]  (크면 v_max 가 동작을 자르고 있음)
  추종 오차    IK 출력 vs 측정, 지연 보정 전/후 RMS·최대 [rad]
  떨림 분석    (v0.7.46) 관절마다 IK 출력 / 스트리머 출력(/upper_body/command) / 측정의 **2~10 Hz 대역 속도 RMS** 와 피크 주파수, 지연 보정 추종 오차.
               떨림이 측정에만 있으면 드라이브(가속 한계·게인), 스트리머 출력부터 있으면 a_max/v_max, IK 출력부터 있으면 입력·필터·IK.
  저장         모든 시계열을 npz 로 (-p save:=<경로>, 기본 ./diag_<side>_<HHMMSS>.npz — 워크스페이스 루트에서 실행하면 호스트 ~/workspaces/isaac_ros-dev/ 에 생김)
목표(시연): 필터 지연 ≤ 50 ms, 드라이브 지연 ≤ 60 ms, 정지 떨림 σ ≤ 2 mm, 속도 포화율 ≤ 5 %.

모터→관절 매핑(motor_ids/joint_signs/joint_offsets)은 config 의 upper_body_streamer 블록에서 읽는다 (-p config:=<yaml>, 기본 sh_humanoid_v2.yaml).
"""
from __future__ import annotations

import os
import time
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from .upper_body import MotorMap

GRID_HZ = 100.0
MAX_LAG_S = 0.6


def _stamp(msg, fallback: float) -> float:
    """header.stamp (발행 노드 시각, 수신 지터 없음) 이 있으면 그것, 없거나 0 이면 수신 시각."""
    try:
        st = msg.header.stamp
        t = float(st.sec) + float(st.nanosec) * 1e-9
        return t if t > 0.0 else fallback
    except AttributeError:
        return fallback


def _resample(ts: np.ndarray, xs: np.ndarray, grid: np.ndarray) -> np.ndarray:
    """(N,) 시각 / (N,D) 값 → grid 위 선형보간 (grid 밖은 끝값 유지)."""
    xs = np.atleast_2d(xs.T).T if xs.ndim == 1 else xs
    return np.stack([np.interp(grid, ts, xs[:, d]) for d in range(xs.shape[1])], axis=1)


def _lag_ms(x: np.ndarray, y: np.ndarray, dt: float, max_lag_s: float = MAX_LAG_S) -> Tuple[Optional[float], float]:
    """x 가 y 보다 얼마나 앞서는지 [ms]: 속도(차분) 신호의 정규화 교차상관이 최대가 되는 양의 시프트. (lag, 상관계수)."""
    return _lag_ms_v(np.diff(x, axis=0) / dt, np.diff(y, axis=0) / dt, dt, max_lag_s)


def _align_axes(vr: np.ndarray, vt: np.ndarray) -> np.ndarray:
    """목표 속도 vt 의 축 j 마다 |상관| 이 가장 큰 입력 축 i 를 골라 부호를 맞춘 입력 속도 (N,3) — 요 0/±90/180° 매핑을 흡수."""
    out = np.zeros_like(vt)
    a = vr - vr.mean(axis=0); b = vt - vt.mean(axis=0)
    for j in range(vt.shape[1]):
        best_i, best_c = j, 0.0
        for i in range(vr.shape[1]):
            den = np.linalg.norm(a[:, i]) * np.linalg.norm(b[:, j])
            c = float(np.dot(a[:, i], b[:, j]) / den) if den > 1e-12 else 0.0
            if abs(c) > abs(best_c):
                best_i, best_c = i, c
        out[:, j] = vr[:, best_i] * (1.0 if best_c >= 0 else -1.0)
    return out


def _lag_ms_v(vx: np.ndarray, vy: np.ndarray, dt: float, max_lag_s: float = MAX_LAG_S) -> Tuple[Optional[float], float]:
    vx = vx - vx.mean(axis=0); vy = vy - vy.mean(axis=0)
    nx, ny = np.linalg.norm(vx), np.linalg.norm(vy)
    if nx < 1e-9 or ny < 1e-9:
        return None, 0.0
    best_k, best_c = 0, -2.0
    for k in range(0, int(max_lag_s / dt) + 1):
        a, b = vx[: len(vx) - k], vy[k:]
        if len(a) < 10:
            break
        c = float(np.sum(a * b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-12))
        if c > best_c:
            best_k, best_c = k, c
    return best_k * dt * 1000.0, best_c


def _band(v: np.ndarray, dt: float, f_lo: float = 2.0, f_hi: float = 10.0) -> Tuple[float, float]:
    """(N,) 속도 신호의 f_lo~f_hi Hz 대역 RMS [같은 단위] 와 그 대역 피크 주파수 [Hz] (rfft, Parseval)."""
    n = len(v)
    if n < 50:
        return 0.0, 0.0
    V = np.fft.rfft(v - v.mean()); fr = np.fft.rfftfreq(n, dt)
    band = (fr >= f_lo) & (fr <= f_hi)
    if not np.any(band):
        return 0.0, 0.0
    rms = float(np.sqrt(2.0 * np.sum(np.abs(V[band]) ** 2)) / n)
    return rms, float(fr[band][np.argmax(np.abs(V[band]))])


class DiagLatency(Node):
    def __init__(self):
        super().__init__("diag_latency")
        for k, v in {"side": "right", "seconds": 20.0, "hold_seconds": 3.0, "v_max": 0.5, "config": "", "save": "",
                     "joint_targets_topic": "/teleop/joint_targets", "motor_state_topic": "/upper_body/motor_states",
                     "command_topic": "/upper_body/command"}.items():
            self.declare_parameter(k, v)
        p = lambda k: self.get_parameter(k).value  # noqa: E731
        self.side = str(p("side"))
        self.seconds, self.hold = float(p("seconds")), float(p("hold_seconds"))
        self.v_max = float(p("v_max"))
        self.map, self.n_joints, self.joint_names = self._load_map(str(p("config")))
        self.save_path = str(p("save")) or os.path.join(os.getcwd(), f"diag_{self.side}_{time.strftime('%H%M%S')}.npz")
        self.j0 = 0 if self.side == "left" else self.n_joints // 2       # 그 팔의 관절 인덱스 (controlled_joints: 왼팔 7, 오른팔 7)
        self.j1 = self.j0 + self.n_joints // 2
        self.raw: List[Tuple[float, np.ndarray]] = []
        self.tgt: List[Tuple[float, np.ndarray]] = []
        self.cmd: List[Tuple[float, np.ndarray]] = []
        self.meas: List[Tuple[float, np.ndarray]] = []
        self.strm: List[Tuple[float, np.ndarray]] = []            # 스트리머 출력 (/upper_body/command, 모터 단위 → 관절)
        self.t0 = time.monotonic()
        self.create_subscription(PoseStamped, f"/teleop/{self.side}_hand/target_raw",
                                 lambda m: self.raw.append((_stamp(m, time.monotonic()), np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z]))), 50)
        self.create_subscription(PoseStamped, f"/quest_ik/{self.side}_hand/target",
                                 lambda m: self.tgt.append((_stamp(m, time.monotonic()), np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z]))), 50)
        self._t_wall0 = time.monotonic()
        self._t_stamp0 = None                         # 첫 수신 메시지의 stamp (정지 구간 판정용 기준)
        self.create_subscription(JointState, p("joint_targets_topic"), self._on_cmd, 50)
        try:
            from ros2_interfaces.msg import UpperBodyCommand, UpperMotorState
            self.create_subscription(UpperMotorState, p("motor_state_topic"), self._on_meas,
                                     QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.create_subscription(UpperBodyCommand, p("command_topic"), self._on_strm, 50)
        except ImportError:
            self.get_logger().warning("ros2_interfaces 없음 — 드라이브 지연은 측정 안 함 (시뮬 joint_states 는 미지원)")
        self.done = False
        self.get_logger().info(f"{self.side} 팔: 처음 {self.hold:.0f} s 는 손을 가만히 → 그 뒤 {self.seconds:.0f} s 동안 평소처럼 움직이세요 (v_max {self.v_max} rad/s 기준)")
        self.create_timer(self.hold + self.seconds, self._finish)

    def _load_map(self, path: str):
        import yaml
        if not path:
            try:
                from ament_index_python.packages import get_package_share_directory
                path = os.path.join(get_package_share_directory("quest_arm_ik"), "config", "sh_humanoid_v2.yaml")
            except Exception:  # noqa: BLE001
                path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", "sh_humanoid_v2.yaml")
        m = yaml.safe_load(open(path))["upper_body_streamer"]["ros__parameters"]
        n = len(m["motor_ids"])
        names = [str(j) for j in m.get("controlled_joints", [])] or [f"j{i}" for i in range(n)]
        return MotorMap(list(m["motor_ids"]), list(m["joint_signs"]), list(m["joint_offsets"]), [int(v) for v in m.get("extra_motor_ids", []) if int(v) > 0]), n, names

    def _on_cmd(self, msg: JointState):
        if len(msg.position) >= self.n_joints:
            self.cmd.append((_stamp(msg, time.monotonic()), np.array(msg.position[: self.n_joints], dtype=float)))

    def _on_meas(self, msg):
        q = self.map.joints_from_state(msg.motor_id, msg.position)
        if q is not None:
            self.meas.append((_stamp(msg, time.monotonic()), q))

    def _on_strm(self, msg):
        q = self.map.joints_from_state(msg.motor_id, msg.position)
        if q is not None:
            self.strm.append((_stamp(msg, time.monotonic()), q))

    def _save(self) -> str:
        def arr(buf):
            return (np.array([b[0] for b in buf]), np.stack([b[1] for b in buf])) if buf else (np.zeros(0), np.zeros((0, 1)))
        out = {}
        for name, buf in (("raw", self.raw), ("tgt", self.tgt), ("cmd", self.cmd), ("strm", self.strm), ("meas", self.meas)):
            out[f"t_{name}"], out[name] = arr(buf)
        out["joint_names"] = np.array(self.joint_names); out["side"] = np.array(self.side)
        out["v_max"] = np.array(self.v_max); out["hold_seconds"] = np.array(self.hold); out["j0"] = np.array(self.j0)
        try:
            np.savez_compressed(self.save_path, **out)
            return self.save_path
        except OSError as e:
            return f"(저장 실패 {e})"

    # ------------------------------------------------------------ 분석
    def _finish(self):
        if self.done:
            return
        self.done = True
        print(self.report(), flush=True)
        raise SystemExit(0)

    def report(self) -> str:
        t_end = time.monotonic()
        dt = 1.0 / GRID_HZ
        lines = [f"==== diag_latency ({self.side}) — 정지 {self.hold:.0f} s + 동작 {self.seconds:.0f} s ===="]

        def series(buf):
            if len(buf) < 5:
                return None, None
            ts = np.array([b[0] for b in buf]); xs = np.stack([b[1] for b in buf])
            return ts, xs

        # 입력률·정지 떨림
        ts_r, xs_r = series(self.raw)
        t_start = ts_r[0] if ts_r is not None else time.monotonic()      # 정지 구간 = 첫 입력 샘플부터 hold_seconds (stamp 기준)
        if ts_r is None:
            lines.append("입력(target_raw) 없음 — 그립을 쥐고 있었는지, quest_bridge 가 떠 있는지")
        else:
            rate = len(ts_r) / max(1e-6, ts_r[-1] - ts_r[0])
            hold = xs_r[(ts_r - t_start) < self.hold]
            if len(hold) >= 10:
                dev = hold - hold.mean(axis=0)
                lines.append(f"입력률        {rate:5.1f} Hz   정지 떨림 σ = {1000 * np.sqrt((dev ** 2).sum(axis=1).mean()):.2f} mm, 최대 {1000 * np.linalg.norm(dev, axis=1).max():.2f} mm  (처음 {self.hold:.0f} s)")
                th = ts_r[(ts_r - t_start) < self.hold] - t_start
                pts = []
                for tq in np.arange(0.0, self.hold + 1e-6, max(0.5, self.hold / 6)):
                    k = int(np.argmin(np.abs(th - tq)))
                    pts.append(f"{th[k]:.1f}s ({hold[k][0]:+.3f},{hold[k][1]:+.3f},{hold[k][2]:+.3f})")
                rng = 1000 * (hold.max(axis=0) - hold.min(axis=0))
                lines.append(f"  처음 {self.hold:.0f} s 궤적 [m]: " + " → ".join(pts) + f"   축별 범위 x {rng[0]:.0f} / y {rng[1]:.0f} / z {rng[2]:.0f} mm")
                # 사람이 언제 멈췄든 상관없도록: 전체에서 가장 조용한 2 s 창의 σ (진짜 추적 잡음)
                win = max(10, int(round(2.0 * rate)))
                if len(xs_r) > win:
                    best = None
                    for k in range(0, len(xs_r) - win, max(1, win // 10)):
                        seg = xs_r[k:k + win]; sd = float(np.sqrt(((seg - seg.mean(axis=0)) ** 2).sum(axis=1).mean()))
                        if best is None or sd < best[0]:
                            best = (sd, ts_r[k] - t_start, 1000 * np.linalg.norm(seg - seg.mean(axis=0), axis=1).max())
                    lines.append(f"  가장 조용한 2 s (t={best[1]:.1f}~{best[1] + 2:.1f} s): σ = {1000 * best[0]:.2f} mm, 최대 {best[2]:.2f} mm  ← 추적 잡음은 이 값으로 판단 (목표 ≤ 3 mm)")
            else:
                lines.append(f"입력률        {rate:5.1f} Hz   (정지 구간 샘플 부족)")
            # 추적 건강: 연속 샘플 간 점프(> 5 cm 는 손 동작이 아니라 추적 튐)와 끊김(> 150 ms 동안 샘플 없음 = 추적 상실/비활성)
            step = np.linalg.norm(np.diff(xs_r, axis=0), axis=1)
            gaps = np.diff(ts_r)
            n_jump, n_gap = int(np.sum(step > 0.05)), int(np.sum(gaps > 0.15))
            lines.append(f"입력 추적     점프(>5 cm) {n_jump}회 (최대 {1000 * step.max():.0f} mm/샘플), 끊김(>150 ms) {n_gap}회 (최장 {1000 * gaps.max():.0f} ms)"
                         + ("  ← 헤드셋이 컨트롤러를 놓치고 있음: 거리·각도·조명부터" if (n_jump > 0 or n_gap > 0) else "  (양호)"))
            if n_jump > 0:
                kj = np.nonzero(step > 0.05)[0][:6]
                lines.append("  점프 시각: " + ", ".join(f"t={ts_r[k + 1] - t_start:.1f}s {1000 * step[k]:.0f} mm" + (" (정지 구간)" if ts_r[k + 1] - t_start < self.hold else "") for k in kj)
                             + "  ← 정지 구간 안이면 자세 잡는 손 동작일 수 있음; 동작 중이면 추적 튐")
        # 필터 지연 — 그립을 쥔(IK 목표가 나오는) 구간만 (놓은 동안은 둘 다 0 으로 두어 상관에서 제외)
        ts_t, xs_t = series(self.tgt)
        if ts_r is not None and ts_t is not None:
            lo, hi = max(ts_r[0], ts_t[0], t_start + self.hold), min(ts_r[-1], ts_t[-1])
            if hi - lo > 2.0:
                grid = np.arange(lo, hi, dt)
                idx = np.searchsorted(ts_t, grid)
                near = np.minimum(np.abs(ts_t[np.clip(idx, 0, len(ts_t) - 1)] - grid), np.abs(ts_t[np.clip(idx - 1, 0, len(ts_t) - 1)] - grid))
                engaged = near < 0.05
                xr, xt = _resample(ts_r, xs_r, grid), _resample(ts_t, xs_t, grid)
                xr = np.where(engaged[:, None], xr, np.nan); xt = np.where(engaged[:, None], xt, np.nan)
                # NaN 구간의 속도는 0 으로 (양쪽 동일하게 제외). 입력(ROS 축)과 손 목표(base 축)는 요 90° 단위 회전(축 바꿈·부호)이 다르므로
                # 목표 축마다 가장 잘 맞는 입력 축(부호 포함)을 골라 맞춘 뒤 상관을 본다 (일반 선형 사상을 맞추면 원운동의 지연이 회전으로 흡수되어 지연이 사라짐)
                vr = np.nan_to_num(np.diff(xr, axis=0) / dt, nan=0.0)
                vt = np.nan_to_num(np.diff(xt, axis=0) / dt, nan=0.0)
                vr = _align_axes(vr, vt)
                lag, c = _lag_ms_v(vr, vt, dt)
                eng_pct = 100.0 * float(np.mean(engaged))
                lines.append(f"필터 지연     {'-' if lag is None else f'{lag:4.0f} ms'}  (target_raw → IK 손 목표, 상관 {c:.2f}, 그립 중 {eng_pct:.0f} %; One-Euro min_cutoff·beta, max_step)")
            else:
                lines.append("필터 지연     측정 불가 (동작 구간이 짧음)")
        # 드라이브 지연·추종 오차·속도 포화율
        ts_c, xs_c = series(self.cmd)
        ts_m, xs_m = series(self.meas)
        if ts_c is not None:
            dts = np.diff(ts_c)
            ok = dts > 1e-4
            v = np.abs(np.diff(xs_c[:, self.j0:self.j1], axis=0))[ok] / dts[ok, None]      # 연속 메시지 간 (stamp 기준)
            vmax_t = v.max(axis=1) if len(v) else np.zeros(0)
            moving = vmax_t > 0.02
            sat = float(np.mean(vmax_t >= 0.95 * self.v_max)) * 100.0 if len(v) else 0.0
            n_jump = int(np.sum(vmax_t > 1.5 * self.v_max))
            lines.append(f"속도 포화율   {sat:4.1f} %  (IK 출력 관절속도 ≥ 0.95·v_max {self.v_max} rad/s 인 틱; 동작 중 틱 {int(moving.sum())}개, 최고 {vmax_t.max() if len(v) else 0:.2f} rad/s"
                         + (f"; v_max 의 1.5배 넘는 틱 {n_jump}개 = 재동기화 점프" if n_jump else "") + ")")
        if ts_c is not None and ts_m is not None:
            lo, hi = max(ts_c[0], ts_m[0], t_start + self.hold), min(ts_c[-1], ts_m[-1])
            if hi - lo > 2.0:
                grid = np.arange(lo, hi, dt)
                qc = _resample(ts_c, xs_c[:, self.j0:self.j1], grid)
                qm = _resample(ts_m, xs_m[:, self.j0:self.j1], grid)
                lag, c = _lag_ms(qc, qm, dt)
                e = qc - qm
                rms, mx = float(np.sqrt((e ** 2).mean())), float(np.abs(e).max())
                if lag is not None:
                    k = int(round(lag / 1000.0 / dt))
                    e2 = qc[: len(qc) - k] - qm[k:] if k > 0 else e
                    rms2, mx2 = float(np.sqrt((e2 ** 2).mean())), float(np.abs(e2).max())
                    lines.append(f"드라이브 지연 {lag:4.0f} ms  (IK 출력 → 측정 관절, 상관 {c:.2f}; loc_kp·acc_set_pp)")
                    lines.append(f"추종 오차     RMS {rms:.3f} / 최대 {mx:.3f} rad  → 지연 보정 후 RMS {rms2:.3f} / 최대 {mx2:.3f} rad (남는 것은 게인·포화·한계)")
                else:
                    lines.append(f"드라이브 지연 측정 불가(움직임 없음)  추종 오차 RMS {rms:.3f} / 최대 {mx:.3f} rad")
                # 관절별 떨림 분석: IK 출력 / 스트리머 출력 / 측정의 2~10 Hz 대역 속도 RMS (어디서부터 떨리는지)
                ts_s, xs_s = series(self.strm)
                qs = _resample(ts_s, xs_s[:, self.j0:self.j1], grid) if ts_s is not None and ts_s[0] <= hi and ts_s[-1] >= lo else None
                k = int(round(lag / 1000.0 / dt)) if lag is not None else 0
                lines.append("떨림 분석     관절            | 2~10 Hz 속도 RMS [rad/s]: IK출력  스트리머  측정(피크 Hz) | 지연보정 오차 RMS/최대 [rad]")
                vc_all, vs_all, vm_all = np.diff(qc, axis=0) / dt, (np.diff(qs, axis=0) / dt if qs is not None else None), np.diff(qm, axis=0) / dt
                worst = []
                for j in range(self.j1 - self.j0):
                    bc, _ = _band(vc_all[:, j], dt)
                    bs, _ = (_band(vs_all[:, j], dt) if vs_all is not None else (float("nan"), 0.0))
                    bm, fm = _band(vm_all[:, j], dt)
                    ej = (qc[: len(qc) - k, j] - qm[k:, j]) if k > 0 else (qc[:, j] - qm[:, j])
                    worst.append((bm, j))
                    name = self.joint_names[self.j0 + j] if self.j0 + j < len(self.joint_names) else f"j{self.j0 + j}"
                    lines.append(f"              {name[:24]:<24}|            {bc:6.3f}   {bs:6.3f}   {bm:6.3f} ({fm:4.1f})    | {float(np.sqrt((ej ** 2).mean())):.3f} / {float(np.abs(ej).max()):.3f}")
                bm_max, jw = max(worst)
                bc_w = _band(vc_all[:, jw], dt)[0]
                bs_w = _band(vs_all[:, jw], dt)[0] if vs_all is not None else float("nan")
                if bm_max < 0.05:
                    verdict = "측정에 2~10 Hz 떨림 없음 (≤ 0.05 rad/s)"
                elif bc_w >= 0.6 * bm_max:
                    verdict = "IK 출력부터 떨림 → 입력(추적 점프)·필터·IK 쪽"
                elif not np.isnan(bs_w) and bs_w >= 0.6 * bm_max:
                    verdict = "스트리머 출력부터 떨림 → a_max·v_max (ros2 param set /upper_body_streamer a_max 4.0 으로 즉시 비교)"
                else:
                    verdict = "측정에서만 떨림 → 드라이브/기구 자체 진동 (명령과 비간섭) — 런북 6-1f H: verify 의 '이동 중 리플' 로 acc_set_pp·loc_kp 스윕"
                lines.append(f"              판정: {verdict}  (가장 심한 관절: {self.joint_names[self.j0 + jw] if self.j0 + jw < len(self.joint_names) else jw}, 측정 {bm_max:.3f} rad/s)")
            else:
                lines.append("드라이브 지연 측정 불가 (동작 구간이 짧음)")
        elif ts_m is None:
            lines.append("측정(motor_states) 없음 — hr_udp_relay / 실기체 경로가 아니면 드라이브 지연은 측정 안 됨")
        # 끊김(v0.7.57): 어느 스트림이 100 ms 넘게 멈췄는지 — '뚝뚝 끊김' 이 입력(Quest)·IK·스트리머·미니PC 상태 중 어디서 오는지 바로 가른다
        gap_lines = []
        for label, buf, thr in (("Quest 입력", self.raw, 0.1), ("IK 출력(joint_targets)", self.cmd, 0.1), ("스트리머 출력(command)", self.strm, 0.1), ("motor_states(미니PC)", self.meas, 0.1)):
            if len(buf) > 10:
                tb = np.array([b[0] for b in buf]); d = np.diff(tb)
                idx = np.where(d > thr)[0]
                if len(idx):
                    worst = idx[np.argmax(d[idx])]
                    times = ", ".join(f"t={tb[i] - t_start:.1f}s {d[i] * 1000:.0f} ms" for i in idx[:6]) + (" …" if len(idx) > 6 else "")
                    gap_lines.append(f"  {label:<22} {len(idx)}회 (최대 {d[worst] * 1000:.0f} ms at t={tb[worst] - t_start:.1f}s): {times}")
        if gap_lines:
            lines.append("끊김(>100 ms)  — 같은 시각에 여러 스트림이 같이 끊기면 가장 앞단(입력) 쪽이 원인, motor_states 만 끊기면 미니PC/링크 (창 2 릴레이 로그의 '상태 패킷 끊김' 분류 참고)")
            lines.extend(gap_lines)
        else:
            lines.append("끊김(>100 ms)  없음 (입력·IK·스트리머·motor_states 모두 연속)")
        span_r = (ts_r[-1] - ts_r[0]) if ts_r is not None else 0.0
        lines.append(f"(수신: raw {len(self.raw)} ({span_r:.1f} s 분량), ik target {len(self.tgt)}, joint_targets {len(self.cmd)}, motor_states {len(self.meas)}; 측정 {t_end - self._t_wall0:.1f} s)")
        if ts_r is not None and span_r < 0.8 * (t_end - self._t_wall0):
            lines.append(f"  ! 입력이 측정 시간의 {100 * span_r / max(1e-6, t_end - self._t_wall0):.0f} % 동안만 왔음 — 컨트롤러가 비활성(추적 상실·절전)이 된 구간이 있음")
        saved = self._save()
        host = saved.replace("/workspaces/", "~/workspaces/", 1) if saved.startswith("/workspaces/") else saved
        lines.append(f"저장: {saved}  (맥북에서 `scp thor:{host} ~/Downloads/` 로 가져와 분석에 첨부)")
        return "\n".join(lines)


def main(args=None):
    rclpy.init(args=args)
    node = DiagLatency()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if not node.done:
            print(node.report(), flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
