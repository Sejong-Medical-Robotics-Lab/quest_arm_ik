"""삼현 HRController 모터 매핑 실측 도구.  (시스템 python 으로 실행 가능: ros2 run quest_arm_ik motor_calib -- …)

  ros2 run quest_arm_ik motor_calib -- watch                  # 15 모터의 위치/속도/토크/온도/오류를 5 Hz 로 표 출력. 손으로 관절을 움직여 단위(rad?)·부호 확인
  ros2 run quest_arm_ik motor_calib -- axes                   # /upper_body/axis_map(JSON) 을 표로: axis_id ↔ channel ↔ motor_id(UID)
  ros2 run quest_arm_ik motor_calib -- verify [--joint wrist] [--delta 0.3]   # --config 생략 시 설치된 config/sh_humanoid_v2.yaml (v0.7.43); 안쪽 구간 상한 --cap wrist_pitch=0.3 기본 (v0.7.44)
                                                              # config 매핑 검증: 관절마다 URDF + 방향으로 0.15 rad 움직이며 '이렇게 움직여야 함' 을 보여 줌 (enable 후)
  ros2 run quest_arm_ik motor_calib -- watch --motor-state-topic /upper_body/motor_states --system-state-topic /upper_body/system_state --command-topic /upper_body/command
                                                              # 드라이버 토픽 이름이 다를 때 (ros2 topic list -t 로 확인)
  ros2 run quest_arm_ik motor_calib -- zero                   # 팔을 URDF 0 자세(양팔 아래로 곧게)에 두고 실행 → joint_offsets 후보 출력
  ros2 run quest_arm_ik motor_calib -- jog 3 0.1              # 모터 3 을 현재 위치에서 +0.1 (저게인 POSITION) 으로 2 s → 어느 관절이 어느 방향으로 가는지
  ros2 run quest_arm_ik motor_calib -- jog 34 0.1 --mode 1               # 기본은 위치만(kp/kd 0, velocity/effort 0). 게인 시험: --kp 20 --kd 0.5 --mode 5
  ros2 run quest_arm_ik motor_calib -- hold                   # 모든 모터를 현재 위치로 유지 명령 (드라이버 워치독·명령 수신 확인)
  ros2 run quest_arm_ik motor_calib -- jog 3 0.2 --once --duration 2.0   # 명령 1개만 보내고 3 s 관찰 → duration 이 보간 시간인지(천천히 감) / 무시(즉시 점프)
  ros2 run quest_arm_ik motor_calib -- watchdog --seconds 3 --watch 10    # 3 s 유지 명령 후 송신 중단 → 드라이버가 언제·어떻게 반응하는지(state 변화·위치 변화) 기록
  ros2 run quest_arm_ik motor_calib -- stiffness 3 --kp 20 --kd 0.5 --seconds 15   # 모터 3 유지 명령 중 손으로 밀어 effort/Δ위치 = 실효 강성 [effort단위/rad] 측정
jog/hold/verify 는 enable_motors 가 된 상태여야 하고, 비상정지 담당이 옆에 있을 때만.
명령 형식은 팀 trajectory_bridge_node 와 동일: command_mode POSITION, 15 슬롯 전부(축 순서, 90 포함), position + velocity 피드포워드,
effort/kp/kd 0, duration = 1/rate, frame_id upper_body_link. 보간은 이쪽에서 --rate(기본 50 Hz)로 한다(smoothstep).
  ★ 송신 주기 50 Hz 가 기본인 이유(2026-10-20 실측): 200 Hz 로 보내면 미니PC 브리지가 0.5~2.4 s 씩 처리를 멈췄다가 한 번에 점프(튐)했고,
    50 Hz 에서는 한 번은 매끄럽게 추종했지만(측정 속도 최대 0.27 vs 명령 0.24 rad/s, 지연 ≈0.13 s) 다음 번엔 다시 1.5~2.5 s 멈춤 → 브리지의 간헐 정지가 근본 원인.
    v0.7.15 부터 verify/jog 요약이 '정지 구간' 과 그동안 motor_states 가 계속 왔는지(브리지 전체 블록 vs 명령 경로만)를 찍는다. duration 은 드라이버가 무시한다.
"""
from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from .upper_body import EMPTY_MOTOR_ID, N_SLOTS, STATE_NAMES


def summarize_motion(rows, uid, rate):
    """rows: [(t, cmd, meas, meas_vel[, state_seq])] — 명령 vs 측정으로 '튐' 을 수치화: 틱당 측정 변화 최대, 명령-측정 지연, 과도 진폭,
    그리고 **정지 구간**(명령은 움직이는데 측정이 멈춘 시간)과 그동안 motor_states 갱신이 계속 왔는지(브리지 전체 블록 vs 명령 경로만). CSV 도 남긴다."""
    import csv
    path = f"/tmp/motor_calib_{uid}.csv"
    with open(path, "w", newline="") as f:
        w = csv.writer(f); w.writerow(["t", "cmd", "meas", "meas_vel", "state_seq"][:len(rows[0]) if rows else 5]); w.writerows(rows)
    if len(rows) < 10:
        print(f"  (로그 {len(rows)} 행 — 너무 짧음) {path}"); return
    ts = [r[0] for r in rows]; cmd = [r[1] for r in rows]; meas = [r[2] for r in rows]
    seq = [r[4] for r in rows] if len(rows[0]) > 4 else None
    dstep_cmd = max(abs(cmd[i + 1] - cmd[i]) for i in range(len(cmd) - 1))
    dstep_meas = max(abs(meas[i + 1] - meas[i]) for i in range(len(meas) - 1))
    err = [m - c for m, c in zip(meas, cmd)]
    print(f"  로그 {len(rows)} 행 → {path}")
    dts = [ts[i + 1] - ts[i] for i in range(len(ts) - 1)]
    if dts:
        i_max = max(range(len(dts)), key=lambda i: dts[i])
        print(f"  송신 간격: 평균 {sum(dts) / len(dts) * 1000:.1f} ms, 최대 {dts[i_max] * 1000:.1f} ms (t={ts[i_max] - ts[0]:.2f}s)"
              + ("  ← 주기의 3배 이상 벌어진 틱이 있음: 그 직후 정지가 시작되면 드라이버 수신 워치독 의심" if dts[i_max] > 3.0 / rate else ""))
    print(f"  틱(1/{rate:.0f} s)당 변화 최대: 명령 {dstep_cmd * 1000:.2f} mrad, 측정 {dstep_meas * 1000:.2f} mrad  "
          f"(측정이 명령의 3배 이상이면 드라이버가 자체 프로파일로 '뛰는' 것)")
    print(f"  명령-측정 차: 최대 {max(abs(e) for e in err) * 1000:.1f} mrad, 끝에서 {err[-1] * 1000:.1f} mrad;  측정 속도 최대 {max(abs(r[3]) for r in rows):.3f} rad/s "
          f"(명령 최대 속도 ≈ {dstep_cmd * rate:.3f} rad/s)")
    # 리플(v0.7.48): 이동 중(명령 속도 ≥ 0.1 rad/s) 측정 속도 − 명령 속도의 2~10 Hz 성분 — 드라이브가 매끈한 램프를 따라가며 자체로 떠는 양.
    # 2026-10-26 텔레옵 npz 에서 어깨·팔꿈치가 3~5 Hz·σ 0.13~0.3 rad/s 로 떨었다(명령과 비간섭 → 드라이브/기구 자체). acc_set_pp·loc_kp 를 바꿔 가며 이 값을 비교한다.
    try:
        import numpy as _np
        t_a = _np.array(ts); c_a = _np.array(cmd); m_a = _np.array(meas)
        grid = _np.arange(t_a[0], t_a[-1], 0.01)
        vc = _np.diff(_np.interp(grid, t_a, c_a)) / 0.01; vm = _np.diff(_np.interp(grid, t_a, m_a)) / 0.01
        if len(vc) > 80:
            # 지연 보정: 측정을 명령에 가장 잘 맞는 만큼(0~150 ms) 당긴 뒤 차를 본다 (지연 자체가 램프 모서리에서 2~10 Hz 성분을 만들지 않게)
            ks = range(0, 16)
            k = max(ks, key=lambda kk: float(_np.corrcoef(vc[: len(vc) - kk], vm[kk:])[0, 1]) if kk < len(vc) - 10 else -1.0)
            vca, vma = vc[: len(vc) - k], vm[k:]
            mov = _np.abs(vca) >= 0.1
            if mov.sum() >= 50:
                r = (vma - vca)[mov]; R = _np.abs(_np.fft.rfft(r - r.mean())); fr = _np.fft.rfftfreq(len(r), 0.01); b = (fr >= 2.0) & (fr <= 10.0)
                rip = float(_np.sqrt(2.0 * _np.sum(R[b] ** 2)) / len(r)); pk = float(fr[b][_np.argmax(R[b])]) if b.any() else 0.0
                print(f"  이동 중 리플: 측정−명령 속도의 2~10 Hz σ {rip:.3f} rad/s (피크 {pk:.1f} Hz, 지연 {10 * k} ms 보정 후; 명령 속도 ≥ 0.1 인 {mov.sum() / 100:.1f} s)  "
                      + ("← 0.1 이상이면 드라이브가 자체로 떪: acc_set_pp·loc_kp 조정 대상" if rip >= 0.1 else "(양호)"))
    except Exception as e:  # noqa: BLE001
        print(f"  (리플 계산 생략: {e})")
    # 0.5 s 구간별 측정 변화 (대략의 궤적 모양) — 각 구간의 끝 값
    t0 = ts[0]; bins = {}
    for r in rows:
        bins.setdefault(int((r[0] - t0) / 0.5), []).append((r[1], r[2]))
    print("  0.5 s 구간 끝 [명령 → 측정] (rad):", "  ".join(f"~{(k + 1) * 0.5:.1f}s:[{v[-1][0]:+.3f}→{v[-1][1]:+.3f}]" for k, v in sorted(bins.items())))
    # motor_states 갱신: 횟수·평균 주기·0.3 s 이상 공백
    gaps = []
    if seq is not None:
        n_upd = seq[-1] - seq[0]; span = ts[-1] - ts[0]
        last_t = ts[0]
        for i in range(1, len(rows)):
            if seq[i] != seq[i - 1]:
                if ts[i] - last_t > 0.3:
                    gaps.append((last_t - t0, ts[i] - last_t))
                last_t = ts[i]
        if ts[-1] - last_t > 0.3:
            gaps.append((last_t - t0, ts[-1] - last_t))
        print(f"  motor_states 갱신: {n_upd}회 / {span:.1f} s ≈ {n_upd / max(span, 1e-6):.0f} Hz" + (f", 0.3 s 이상 공백 {len(gaps)}회: " + ", ".join(f"{a:.1f}s 부터 {d:.2f}s" for a, d in gaps) if gaps else ", 공백 없음"))
    # 정지 구간: 명령은 측정에서 0.05 rad 이상 떨어져 있는데 측정이 2 mrad 도 안 변한 채 0.3 s 이상
    stalls = []; i = 0
    while i < len(rows):
        j = i
        while j + 1 < len(rows) and abs(meas[j + 1] - meas[i]) < 0.002 and abs(cmd[j + 1] - meas[j + 1]) > 0.05:
            j += 1
        if ts[j] - ts[i] >= 0.3:
            upd = (seq[j] - seq[i]) if seq is not None else None
            stalls.append((ts[i] - t0, ts[j] - ts[i], abs(cmd[j] - meas[j]), upd))
            i = j + 1
        else:
            i += 1
    if stalls:
        print("  ★ 정지 구간(명령은 가는데 측정 안 움직임): " + "; ".join(
            f"{a:.1f}s 부터 {d:.2f}s (명령-측정 {e * 1000:.0f} mrad" + (f", 그동안 상태 갱신 {u}회)" if u is not None else ")") for a, d, e, u in stalls))
        if seq is not None:
            if all(u == 0 for _, _, _, u in stalls):
                print("    → 정지 중 motor_states 도 안 옴: 미니PC 브리지(상태+명령 루프) 전체가 멈춘 것")
            elif all(u > 0 for _, _, _, u in stalls):
                print("    → 정지 중에도 motor_states 는 계속 옴: 명령 경로(수신 콜백/명령 전달)만 멈춘 것")
    else:
        print("  정지 구간 없음 (명령을 연속으로 따라감)")


def command_ids(st, include_not_ready=False):
    """명령을 보낼 모터 ID (드라이버 슬롯 순서). 기본은 is_ready 인 모터만 — 미연결 축(머리 90)에 보내면 HRController 가 매번 거부(command_commit_failed)."""
    return [int(m) for i, m in enumerate(st.motor_id) if m and int(m) != EMPTY_MOTOR_ID and (include_not_ready or bool(st.is_ready[i]))]


class Pacer:
    """고정 주기 송신: 다음 송신 시각을 절대 시각으로 잡아 틱마다 한 일(수신 처리·출력)과 무관하게 간격을 유지한다 (sleep(1/rate) 누적 방식은 밀리고 흔들림)."""

    def __init__(self, rate):
        self.period = 1.0 / rate
        self.next_t = time.monotonic() + self.period

    def wait(self):
        dt = self.next_t - time.monotonic()
        if dt > 0:
            time.sleep(dt)
        elif dt < -0.5:                      # 반 초 이상 밀렸으면 (정지·중단 뒤) 따라잡으려 몰아 보내지 않고 다시 맞춘다
            self.next_t = time.monotonic()
        self.next_t += self.period


def spin_pending(node, n=4):
    """대기 중인 콜백을 비운다. rclpy.spin_once 는 호출당 콜백 **하나**만 처리하므로, 100 Hz 토픽을 두세 개 구독하는 상태에서
    틱마다 한 번만 부르면 motor_states 가 2~3틱에 한 번씩만 갱신돼 '측정이 계단으로 뛰는' 것처럼 보인다 (2026-10-22 실기체에서 확인:
    상태는 100 Hz 로 오는데 요약이 33 Hz 로 찍힘). timeout 0 이라 할 일이 없으면 바로 돌아온다."""
    for _ in range(n):
        rclpy.spin_once(node, timeout_sec=0.0)


class Calib(Node):
    def __init__(self, motor_state_topic="/upper_body/motor_states", system_state_topic="/upper_body/system_state",
                 command_topic="/upper_body/command", best_effort=False):
        super().__init__("motor_calib")
        from ros2_interfaces.msg import UpperBodyCommand, UpperBodySystemState, UpperMotorState
        self.Cmd = UpperBodyCommand
        self.state = None
        self.n_state = 0
        self.sys = None
        self.topics = (motor_state_topic, system_state_topic, command_topic)
        self.axes = None                                     # /upper_body/axis_map (JSON) → {motor_id: {axis_id, channel_id, ...}}
        # 상태 토픽 구독은 전부 BEST_EFFORT (2026-10-21 실측): 삼현 브리지는 RELIABLE 로 발행하는데, 우리 쪽에 RELIABLE 구독자가 있으면
        # 허브에서 패킷(특히 4 KB axis_map 의 IP 조각)이 하나 빠질 때마다 브리지의 publish() 가 ACK 를 기다리며 ~2 s 막혀 브리지 전체가 얼었다.
        # BEST_EFFORT 구독자는 ACK 가 없어 발행자를 막지 않는다. (RELIABLE 발행자 ↔ BEST_EFFORT 구독자는 짝이 맞는다)
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(UpperMotorState, motor_state_topic, self._on_state, qos)
        self.create_subscription(UpperBodySystemState, system_state_topic, self._on_sys, qos)
        from std_msgs.msg import String
        self.create_subscription(String, "/upper_body/axis_map", self._on_axis_map, qos)
        # 명령 발행 QoS. 기본 RELIABLE(팀 브리지와 동일). --best-effort 면 BEST_EFFORT: 패킷이 유실돼도 재전송을 기다리지 않고 다음 명령으로 간다
        # (RELIABLE 은 유실된 한 개가 복구될 때까지 뒤의 명령을 모두 붙들었다가 한꺼번에 넘기므로 '멈췄다 점프' 가 된다). 드라이버 구독이 RELIABLE 이면
        # BEST_EFFORT 발행자와는 짝이 안 맞아 아무것도 안 움직인다 → `ros2 topic info -v /upper_body/command` 의 Subscription QoS 로 확인.
        self.pub = self.create_publisher(UpperBodyCommand, command_topic,
                                         QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT if best_effort else ReliabilityPolicy.RELIABLE))

    def _on_axis_map(self, msg):
        import json
        try:
            d = json.loads(msg.data)
            self.axes = {int(a["motor_id"]): a for a in d.get("connected_axes", [])}
        except Exception as e:  # noqa: BLE001
            self.axes = {"error": str(e), "raw": msg.data[:200]}

    def _on_state(self, msg):
        self.state = msg
        self.n_state += 1                                # 수신 횟수 (요약의 '상태 갱신' 계산용)

    def _on_sys(self, msg):
        self.sys = msg

    def wait_state(self, timeout=5.0):
        t0 = time.monotonic()
        while self.state is None and time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.state is None:
            print(f"motor_state 가 안 옴 ({self.topics[0]}) — HRController(또는 mujoco_sim hr_emulation) 가 떠 있고 같은 ROS_DOMAIN_ID 인지, "
                  f"토픽 이름이 다르면 `ros2 topic list -t` 로 확인해 --motor-state-topic / --system-state-topic / --command-topic 지정")
            sys.exit(1)
        return self.state

    def table(self):
        st = self.state
        sysn = STATE_NAMES.get(int(self.sys.state), "?") if self.sys is not None else "?"
        lines = [f"driver {sysn}  run_mode {self.sys.run_mode if self.sys is not None else '?'}   "
                 f"(slot axis ch  id: pos[rad]  vel  effort  temp  en/rdy/fault  err)"]
        for i in range(N_SLOTS):
            mid = int(st.motor_id[i])
            if mid == 0 and not st.is_ready[i]:
                continue
            a = self.axes.get(mid) if isinstance(self.axes, dict) and mid in self.axes else None
            ax = f"a{a['axis_id']:<2d} c{a.get('channel_id', '?')}" if a else "        "
            lines.append(f"  {i + 1:2d} {ax} {mid:3d}: {st.position[i]:+8.4f} {st.velocity[i]:+7.3f} {st.effort[i]:+7.2f} {st.temperature[i]:5.1f}  "
                         f"{int(st.is_enabled[i])}/{int(st.is_ready[i])}/{int(st.has_fault[i])}  {st.error_code[i]}")
        return "\n".join(lines)

    def sys_name(self):
        return STATE_NAMES.get(int(self.sys.state), "?") if self.sys is not None else "?"

    def send(self, mode, ids, positions, kp, kd, duration=0.0, velocities=None):
        """팀 trajectory_bridge_node 와 같은 형식: POSITION, 15 슬롯 전부(축 순서, 미연결 90 포함), position + velocity(피드포워드),
        effort/kp/kd 0 (기본), duration = 송신 주기, frame_id upper_body_link."""
        cmd = self.Cmd()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "upper_body_link"
        cmd.command_mode = int(mode)
        for k in range(len(ids), N_SLOTS):
            cmd.motor_id[k] = EMPTY_MOTOR_ID                      # 빈 슬롯 = 255 (삼현 브리지가 조용히 건너뜀; 0 이면 경고)
        for k, (mid, pos) in enumerate(zip(ids, positions)):
            cmd.motor_id[k] = int(mid); cmd.position[k] = float(pos); cmd.kp[k] = float(kp); cmd.kd[k] = float(kd)
            if velocities is not None:
                cmd.velocity[k] = float(velocities[k])
        cmd.duration = float(duration)
        self.pub.publish(cmd)


def main(args=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["watch", "zero", "jog", "hold", "watchdog", "stiffness", "axes", "verify"])
    ap.add_argument("motor", nargs="?", type=int, help="jog: 모터 ID")
    ap.add_argument("delta", nargs="?", type=float, default=0.1, help="jog: 변위 [모터 단위, 보통 rad]")
    ap.add_argument("--kp", type=float, default=0.0, help="기본 0 = 위치만 보냄(드라이버 POSITION 모드 내부 제어). MIT/임피던스 시험 때만 지정")
    ap.add_argument("--kd", type=float, default=0.0)
    ap.add_argument("--mode", type=int, default=1, help="UpperBodyCommand.command_mode (1 POSITION; 삼현 브리지는 1·6 만 받고 4/5 는 거부)")
    ap.add_argument("--rate", type=float, default=50.0, help="명령 송신 주기 [Hz]. 기본 50; 제어기 내장 UDP 링크(런북 6-1e)에서는 100 권장 (지령 슬롯 200 Hz 라 200 도 가능)")
    ap.add_argument("--no-velocity", action="store_true", help="velocity 피드포워드를 보내지 않음 (기본은 팀 브리지처럼 보냄)")
    ap.add_argument("--only-motor", action="store_true", help="verify: 시험 중인 관절의 모터 하나에만 명령을 보냄 (메시지당 IPC 1건 — 미니PC 처리량 한계인지 가르는 시험)")
    ap.add_argument("--include-not-ready", action="store_true", help="is_ready 가 아닌 모터(머리 90)에도 명령을 보냄. 기본은 제외 — 보내면 HRController 가 명령마다 command_commit_failed 를 냄")
    ap.add_argument("--best-effort", action="store_true", help="명령 발행 QoS 를 BEST_EFFORT 로 (기본 RELIABLE). 삼현 브리지 구독이 RELIABLE 이라 실기체에는 짝이 안 맞음 — 에뮬레이션 시험용")
    ap.add_argument("--seconds", type=float, default=2.0)
    ap.add_argument("--once", action="store_true", help="jog: 명령을 1개만 보내고 관찰 (duration 의미 확인)")
    ap.add_argument("--duration", type=float, default=-1.0, help="UpperBodyCommand.duration [s]; 기본(-1) = 1/rate (팀 브리지와 동일), 0 = 즉시")
    ap.add_argument("--watch", type=float, default=10.0, help="watchdog: 송신 중단 후 관찰 시간 [s]")
    ap.add_argument("--config", default="", help="verify: config yaml (upper_body_streamer 블록의 motor_ids/joint_signs/joint_offsets 사용). 생략 시 설치된 share/quest_arm_ik/config/sh_humanoid_v2.yaml")
    ap.add_argument("--delta", type=float, default=0.15, dest="vdelta", help="verify: 관절당 움직일 각 [rad] (URDF 기준, 기대 방향으로 한 번)")
    ap.add_argument("--delta2", type=float, default=None, dest="vdelta2",
                    help="verify: 양방향 시험 — 어깨 roll 을 뺀 모든 관절을 현재 위치에서 +X → -X → 0 (URDF 기준) 으로 움직임. 어깨 roll 은 --delta 만큼 바깥쪽으로만. "
                         "관절 한계(URDF, 여유 0.05 rad)를 넘는 쪽은 자동으로 잘라냄 (팔꿈치는 곧게 편 자세에서 + 쪽이 없음)")
    ap.add_argument("--velocity", type=float, default=None, dest="vel",
                    help="verify: 이동 최고 속도 [rad/s] (smoothstep 의 피크). 주면 --ramp 대신 구간마다 시간 = 1.5·|이동각|/속도 (최소 0.2 s). 예 0.3 / 0.5 / 1.0")
    ap.add_argument("--ramp", type=float, default=2.0, help="verify: 한 구간의 이동 시간 [s] (--velocity 가 없을 때; smoothstep 보간, --rate 로 송신)")
    ap.add_argument("--joint", default="", help="verify: 이 관절(부분 문자열)만")
    ap.add_argument("--cap", action="append", default=[], help="verify: 기대 방향의 반대(안쪽) 구간 진폭 상한 [rad], 관절키=값 (기본 wrist_pitch=0.3 — 팔 내린 자세에서 손이 몸통에 닿음)")
    ap.add_argument("--motor-state-topic", default="/upper_body/motor_states", help="드라이버의 UpperMotorState 토픽 (ros2 topic list -t 로 확인)")
    ap.add_argument("--system-state-topic", default="/upper_body/system_state")
    ap.add_argument("--command-topic", default="/upper_body/command")
    ns = ap.parse_args([a for a in (args if args is not None else sys.argv[1:]) if not a.startswith("--ros-args")])
    if ns.duration < 0:
        ns.duration = 1.0 / ns.rate
    rclpy.init()
    node = Calib(ns.motor_state_topic, ns.system_state_topic, ns.command_topic, ns.best_effort)
    if ns.best_effort:
        print("[qos] 명령 발행 BEST_EFFORT — 드라이버 구독이 RELIABLE 이면 짝이 안 맞아 안 움직인다 (ros2 topic info -v /upper_body/command 로 확인)")
    st = node.wait_state()
    try:
        if ns.cmd == "watch":
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.2)
                print("\033[2J\033[H" + node.table(), flush=True)
        elif ns.cmd == "verify":
            # config 의 매핑(모터 ID·부호·오프셋)이 맞는지: 관절마다 URDF + 방향으로 delta 만큼 움직여 '이렇게 움직여야 함' 을 보여 주고 사람이 확인
            import os
            import yaml
            if not ns.config:   # 기본: 설치된 share/quest_arm_ik/config/sh_humanoid_v2.yaml (launch 가 쓰는 것과 같은 파일; 소스 트리는 그 다음)
                try:
                    from ament_index_python.packages import get_package_share_directory
                    ns.config = os.path.join(get_package_share_directory("quest_arm_ik"), "config", "sh_humanoid_v2.yaml")
                except Exception:  # noqa: BLE001
                    ns.config = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", "sh_humanoid_v2.yaml")
                if not os.path.exists(ns.config):
                    print(f"--config <sh_humanoid_v2.yaml> 필요 (기본 경로 없음: {ns.config})"); sys.exit(1)
            print(f"[verify] config: {ns.config}")
            cfg = yaml.safe_load(open(ns.config))["upper_body_streamer"]["ros__parameters"]
            joints, mids = list(cfg["controlled_joints"]), [int(v) for v in cfg["motor_ids"]]
            signs, offs = [float(v) for v in cfg["joint_signs"]], [float(v) for v in cfg["joint_offsets"]]
            expect = {  # URDF + 방향의 물리적 의미 (양팔 동일하게 정의됨: SH_Humanoid_v2 축 분석 2026-10-20)
                "shoulder_pitch": ("팔 전체가 **앞으로** 들림", +1),
                "shoulder_roll": ("팔이 **바깥쪽(옆)** 으로 벌어짐 — URDF 는 +가 안쪽(몸 쪽)이라 -방향으로 시험 (안쪽은 몸통에 닿을 수 있음)", -1),
                "shoulder_yaw": ("상완이 **바깥으로** 회전 (손바닥이 앞을 향하는 쪽)", +1),
                "elbow": ("전완이 **앞으로 들림(굽힘)** — URDF 는 -가 굽힘이라 -방향으로 시험", -1),
                "wrist_yaw": ("전완이 **바깥쪽으로** 회전 (손바닥이 앞으로 도는 쪽) — URDF 는 +가 안쪽이라 -방향으로 먼저 (2026-10-23 요청)", -1),
                "wrist_roll": ("손이 **앞쪽으로** 휨 (앞뒤 흔들림) — URDF 는 +가 뒤쪽이라 -방향으로 먼저 (2026-10-23 요청)", -1),
                "wrist_pitch": ("손이 **바깥쪽으로** 휨 (좌우 흔들림)", +1)}
            limits = {  # URDF 관절 한계 [rad] (SH_Humanoid_v2.xacro, 양팔 동일) — --delta2 의 양방향 이동을 이 안으로 자른다
                "shoulder_pitch": (-2.618, 2.618), "shoulder_roll": (-2.094, 2.094), "shoulder_yaw": (-1.571, 1.571),
                "elbow": (-2.618, 0.0), "wrist_yaw": (-1.484, 1.484), "wrist_pitch": (-1.396, 1.396), "wrist_roll": (-1.571, 1.571)}
            LIMIT_MARGIN = 0.05
            inward_cap = {"wrist_pitch": 0.3}   # 팔 내린 자세에서 손이 몸통에 닿는 방향(기대 방향의 반대 쪽)의 최대 진폭 [rad] — 2026-10-26 실측: 안쪽 0.44~0.49 rad 에서 접촉
            for item in ns.cap:
                k, v = item.split("="); inward_cap[k] = float(v)
            if ns.vel is not None and ns.vel > 2.0:
                print(f"[verify] --velocity {ns.vel} rad/s 는 큼 — 팔 주변을 비우고 받칠 사람을 두세요")
            ids_all = command_ids(st, ns.include_not_ready)
            ids = ids_all
            if ns.only_motor:
                print("[verify] --only-motor: 시험 관절의 모터 하나에만 명령 (나머지 축은 드라이버가 마지막 목표를 유지). 메시지당 IPC 1건 → 멈춤이 사라지면 미니PC 처리량(명령 건수) 한계")
            results = []
            # 세션 내내 --rate 송신을 끊지 않는다 (명령 공백 뒤 재개 시 드라이버가 '움찔' 하는 것을 피함): Enter 를 기다리는 동안은
            # 백그라운드 스레드가 마지막 목표(현재 유지)를 계속 보낸다. 실기체 스트리머(hr:=true)와 같은 조건.
            import threading
            hold_state = {"targets": None, "run": True}
            lock = threading.Lock()

            def keep_streaming():
                pace = Pacer(ns.rate)
                while hold_state["run"] and rclpy.ok():
                    with lock:
                        tg = hold_state["targets"]                          # (ids, targets) 쌍 — 함께 바꿔야 길이가 어긋나지 않는다
                    if tg is not None:
                        node.send(ns.mode, tg[0], tg[1], ns.kp, ns.kd, ns.duration, [0.0] * len(tg[0]))
                    pace.wait()

            bg = threading.Thread(target=keep_streaming, daemon=True)
            bg.start()
            pos_now = {int(m): float(st.position[i]) for i, m in enumerate(st.motor_id) if m}
            with lock:
                hold_state["targets"] = None if ns.only_motor else (list(ids), [pos_now[m] for m in ids])   # 첫 명령 = 현재 측정 위치 유지 (--only-motor 는 관절을 고른 뒤부터)
            print(f"[verify] 현재 위치 유지 명령 송신 시작 ({ns.rate:.0f} Hz, duration {ns.duration:.3f}, {'BEST_EFFORT' if ns.best_effort else 'RELIABLE'}, 세션 내내 유지). 이 순간 팔이 움찔하면 '공백 뒤 첫 명령' 문제 — 이후 관절별 동작에서는 없어야 함")
            t0 = time.monotonic(); dev = {m: 0.0 for m in ids}; n_state0 = node.n_state
            while time.monotonic() - t0 < 1.0 and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.005)
                st4 = node.state
                for i, m in enumerate(st4.motor_id):
                    if m and int(m) in dev:                               # 명령 대상(is_ready) 모터만 — 머리 90 등은 제외 (v0.7.20 KeyError 수정)
                        dev[int(m)] = max(dev[int(m)], abs(float(st4.position[i]) - pos_now[int(m)]))
            worst = sorted(dev.items(), key=lambda kv: -kv[1])[:3]
            print("[verify] 첫 명령 후 1 s 동안 측정 위치 변화 최대 (상위 3 모터): " + ", ".join(f"UID {m}: {d * 1000:.1f} mrad" for m, d in worst)
                  + "  ← 수 mrad 면 정상(제어 맞물림), 수십 mrad 이상이면 드라이버 내부 목표와 측정의 불일치")
            print(f"[verify] motor_states 수신 {node.n_state - n_state0} Hz (이 값이 10~15 면 미니PC 브리지 상태 송신이 70 ms 주기인 것)")
            for j, mid, sg, off in zip(joints, mids, signs, offs):
                if ns.joint and ns.joint not in j:
                    continue
                key = next((k for k in expect if k in j), None)
                desc, direction = expect.get(key, ("(설명 없음)", +1))
                lo, hi = limits.get(key, (-3.14, 3.14))
                two_sided = ns.vdelta2 is not None and key != "shoulder_roll"
                amp = abs(ns.vdelta2) if two_sided else abs(ns.vdelta)
                # 현재 위치(URDF) — 한계 안으로 자르기 위해 미리 읽는다 (모터각 = sign·q + offset → q = sign·(모터각 − offset))
                spin_pending(node)
                stp = node.state
                if mid not in list(stp.motor_id):
                    print(f"\n[{j}] 모터 {mid} 가 motor_state 에 없음 — 건너뜀"); results.append((j, mid, "없음")); continue
                q0 = sg * (float(stp.position[list(stp.motor_id).index(mid)]) - off)

                def clamp_leg(d):                                           # URDF 기준 이동량 d 를 관절 한계 안으로
                    return min(max(q0 + d, lo + LIMIT_MARGIN), hi - LIMIT_MARGIN) - q0
                amp_in = min(amp, inward_cap.get(key, amp))                 # 반대(안쪽) 구간은 몸통 접촉을 피해 작게
                legs = [direction * amp, -direction * amp_in, 0.0] if two_sided else [direction * amp, 0.0]
                legs_c = [clamp_leg(d) if d != 0.0 else 0.0 for d in legs]
                cut = [f"{d:+.2f}→{c:+.2f}" for d, c in zip(legs, legs_c) if abs(d - c) > 1e-6]
                if two_sided and amp_in < amp:
                    cut.append(f"안쪽 {amp_in:.2f} 로 제한(몸통 접촉 방지, --cap {key}=… 로 변경)")
                plan = []                                                     # (from, to) URDF 이동량, 각 구간 뒤 1 s 유지
                cur = 0.0
                for d in legs_c:
                    if abs(d - cur) < 0.01:                                   # 잘려서 사라진 구간은 건너뜀 (예: 곧게 편 팔꿈치의 + 쪽)
                        continue
                    plan.append((cur, d)); cur = d
                if abs(cur) > 1e-6:
                    plan.append((cur, 0.0))
                T_hold = 1.0
                segs = []; tt = 0.0                                           # (t0, t1, from, to, hold?)
                for a, b in plan:
                    T = ns.ramp if ns.vel is None else max(0.2, 1.5 * abs(b - a) / ns.vel)   # smoothstep 피크 속도 = 1.5·Δ/T
                    segs.append((tt, tt + T, a, b, False)); tt += T
                    segs.append((tt, tt + T_hold, b, b, True)); tt += T_hold
                total = tt
                v_peak = max((1.5 * abs(b - a) / (t1 - t0s) for t0s, t1, a, b, h in segs if not h), default=0.0)
                route = " → ".join(f"{b:+.2f}" for a, b in plan) if plan else "(이동 없음)"
                print(f"\n[{j}] 모터 {mid} (sign {sg:+.0f}, offset {off:+.3f}) 현재 URDF {q0:+.2f} rad, 이동 {route} (URDF 상대, 모터는 ×{sg:+.0f}); "
                      f"{len(plan)}구간 {total:.1f} s, 피크 {v_peak:.2f} rad/s" + (f"; 한계로 잘림 {cut}" if cut else "") + f"\n   기대 동작(첫 이동): {desc}")
                if not plan:
                    print("   관절 한계 안에서 움직일 여지가 없음 — 건너뜀"); results.append((j, mid, "한계")); continue
                try:
                    ans0 = input("   Enter 를 누르면 움직입니다 (건너뛰려면 s + Enter): ").strip().lower()
                except EOFError:
                    ans0 = ""
                if ans0 == "s":
                    results.append((j, mid, "건너뜀")); continue
                rclpy.spin_once(node, timeout_sec=0.1)
                st2 = node.state
                if mid not in list(st2.motor_id):
                    print(f"   모터 {mid} 가 motor_state 에 없음 — 건너뜀"); results.append((j, mid, "없음")); continue
                if ns.only_motor:
                    ids = [mid]
                with lock:
                    tg = hold_state["targets"]
                    hold_state["targets"] = None                            # 램프 동안은 이 루프가 보낸다
                base = list(tg[1]) if tg is not None and list(tg[0]) == list(ids) else [float(st2.position[list(st2.motor_id).index(m)]) for m in ids]
                pos0 = {m: base[i] for i, m in enumerate(ids)}
                t0 = time.monotonic()
                prev = base; rows = []; pace = Pacer(ns.rate)
                while time.monotonic() - t0 < total and rclpy.ok():
                    t = time.monotonic() - t0
                    dq = plan[-1][1]
                    for t0s, t1, a, b, hold in segs:
                        if t < t1:
                            if hold:
                                dq = b
                            else:
                                u = (t - t0s) / (t1 - t0s)
                                u = u * u * (3 - 2 * u)                                        # smoothstep: 시작·끝 속도 0
                                dq = a + u * (b - a)
                            break
                    targets = [pos0[m] + (sg * dq if m == mid else 0.0) for m in ids]
                    vels = None if ns.no_velocity else [(a - b) * ns.rate for a, b in zip(targets, prev)]
                    node.send(ns.mode, ids, targets, ns.kp, ns.kd, ns.duration, vels)
                    prev = targets
                    spin_pending(node); pace.wait()
                    st3 = node.state; k = list(st3.motor_id).index(mid)
                    rows.append((t, targets[ids.index(mid)], float(st3.position[k]), float(st3.velocity[k]), node.n_state))
                with lock:
                    hold_state["targets"] = (list(ids), list(prev))         # 끝 위치(=시작 위치) 유지로 복귀
                summarize_motion(rows, mid, ns.rate)
                try:
                    ans = input("   첫 이동이 기대 동작과 같았으면 Enter, 반대 방향이면 r, 다른 관절이 움직였으면 o: ").strip().lower()
                except EOFError:
                    ans = ""
                results.append((j, mid, {"": "OK", "r": "반대(부호 뒤집기)", "o": "다른 관절(ID 확인)"}.get(ans, ans)))
            hold_state["run"] = False
            print("\n==== verify 결과")
            for j, mid, r in results:
                print(f"  {j:28s} 모터 {mid:3d}: {r}")
            flips = [j for j, _, r in results if r.startswith("반대")]
            if flips:
                print("→ config joint_signs 에서 부호를 뒤집을 관절:", flips)
        elif ns.cmd == "axes":
            t0 = time.monotonic()
            while node.axes is None and time.monotonic() - t0 < 5.0:
                rclpy.spin_once(node, timeout_sec=0.1)
            if not isinstance(node.axes, dict) or "error" in node.axes:
                print(f"/upper_body/axis_map 을 못 받았거나 JSON 해석 실패: {node.axes}"); sys.exit(1)
            print("axis_id  axis_index  channel_id  motor_id  connected   (config motor_ids 는 이 axis 순서로 관절을 대응시킨 것)")
            for mid, a in sorted(node.axes.items(), key=lambda kv: kv[1].get("axis_id", 0)):
                print(f"  {a.get('axis_id', '?'):>5}  {a.get('axis_index', '?'):>10}  {a.get('channel_id', '?'):>10}  {mid:>8}  {a.get('connected', '?')}")
            print("motor_ids (axis 순서):", [mid for mid, a in sorted(node.axes.items(), key=lambda kv: kv[1].get("axis_id", 0)) if a.get("connected", True)])
        elif ns.cmd == "zero":
            print(node.table())
            ids = command_ids(st, ns.include_not_ready)
            print("\njoint_offsets 후보 (motor_ids 순서대로 골라 쓰기):")
            print("  motor_ids:     [" + ", ".join(f"{m}" for m in ids) + "]")
            print("  position:      [" + ", ".join(f"{st.position[i]:.4f}" for i, m in enumerate(st.motor_id) if m) + "]")
            print("팔이 URDF 0 자세(양팔 아래로 곧게, 손바닥 안쪽)였다면 위 position 이 그대로 joint_offsets. 부호(joint_signs)는 jog 로.")
        elif ns.cmd in ("jog", "hold"):
            ids = command_ids(st, ns.include_not_ready)
            pos0 = {int(m): float(st.position[i]) for i, m in enumerate(st.motor_id) if m}
            if ns.cmd == "jog":
                if ns.motor not in pos0:
                    print(f"모터 {ns.motor} 가 motor_state 에 없음: {ids}"); sys.exit(1)
                print(f"모터 {ns.motor}: {pos0[ns.motor]:+.4f} → {pos0[ns.motor] + ns.delta:+.4f} (kp {ns.kp}, kd {ns.kd}, mode {ns.mode}) {ns.seconds:.1f} s 동안 — 어느 관절이 어느 쪽으로 가는지 보세요")
            else:
                print(f"{len(ids)} 모터 현재 위치 유지 명령 {ns.seconds:.1f} s (kp {ns.kp}, kd {ns.kd})")
            t0 = time.monotonic()
            n = 0
            if ns.cmd == "jog" and ns.once:
                targets = [pos0[m] + (ns.delta if m == ns.motor else 0.0) for m in ids]
                node.send(ns.mode, ids, targets, ns.kp, ns.kd, ns.duration); n = 1
                print(f"명령 1개 송신 (duration {ns.duration}). {ns.seconds:.1f} s 동안 0.25 s 마다 위치 출력:")
                rows = []
                while time.monotonic() - t0 < ns.seconds and rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=0.005); spin_pending(node)
                    i = list(node.state.motor_id).index(ns.motor)
                    rows.append((time.monotonic() - t0, targets[ids.index(ns.motor)], float(node.state.position[i]), float(node.state.velocity[i]), node.n_state))
                    if int((time.monotonic() - t0) * 4) != int((time.monotonic() - t0 - 0.005) * 4):
                        print(f"  t={time.monotonic() - t0:5.2f}  모터 {ns.motor} 위치 {float(node.state.position[i]):+.4f} (Δ {float(node.state.position[i]) - pos0[ns.motor]:+.4f})")
                summarize_motion(rows, ns.motor, 1.0 / 0.005)
            prev = None; rows = []; pace = Pacer(ns.rate)
            while not (ns.cmd == "jog" and ns.once) and time.monotonic() - t0 < ns.seconds and rclpy.ok():
                u = min((time.monotonic() - t0) / max(0.5 * ns.seconds, 1e-3), 1.0)        # 반 동안 부드럽게 올림
                u = u * u * (3 - 2 * u)
                targets = [pos0[m] + (ns.delta * u if (ns.cmd == "jog" and m == ns.motor) else 0.0) for m in ids]
                vels = None if (ns.no_velocity or prev is None) else [(a - b) * ns.rate for a, b in zip(targets, prev)]
                node.send(ns.mode, ids, targets, ns.kp, ns.kd, ns.duration, vels)
                prev = targets
                n += 1
                spin_pending(node)
                pace.wait()
                if ns.cmd == "jog":
                    st3 = node.state; k = list(st3.motor_id).index(ns.motor)
                    rows.append((time.monotonic() - t0, targets[ids.index(ns.motor)], float(st3.position[k]), float(st3.velocity[k]), node.n_state))
            if ns.cmd == "jog" and rows:
                summarize_motion(rows, ns.motor, ns.rate)
            rclpy.spin_once(node, timeout_sec=0.2)
            st2 = node.state
            print(f"\n보낸 명령 {n}개. 이동량 (모터별 Δ):")
            for i, m in enumerate(st2.motor_id):
                if m:
                    d = float(st2.position[i]) - pos0[int(m)]
                    flag = "  ◀ 명령한 모터" if (ns.cmd == "jog" and int(m) == ns.motor) else ("  ◀ 움직임!" if abs(d) > 0.02 else "")
                    print(f"  {int(m):2d}: {d:+.4f}{flag}")
            if ns.cmd == "jog":
                print("→ 로봇에서 실제 움직인 관절(URDF 이름)과 방향(URDF + 방향이면 sign +1, 반대면 -1)을 motor_ids / joint_signs 에 적는다.")
        elif ns.cmd == "watchdog":
            ids = command_ids(st, ns.include_not_ready)
            pos0 = {int(m): float(st.position[i]) for i, m in enumerate(st.motor_id) if m}
            print(f"[watchdog] {len(ids)} 모터 유지 명령 {ns.seconds:.1f} s (kp {ns.kp}, kd {ns.kd}, mode {ns.mode}) → 송신 중단 → {ns.watch:.0f} s 관찰. 팔을 받칠 준비!")
            t0 = time.monotonic(); pace = Pacer(ns.rate)
            while time.monotonic() - t0 < ns.seconds and rclpy.ok():
                node.send(ns.mode, ids, [pos0[m] for m in ids], ns.kp, ns.kd, ns.duration)
                spin_pending(node); pace.wait()
            t_stop = time.monotonic()
            print(f"[watchdog] t=0.000 송신 중단. driver {node.sys_name()}, accepted {getattr(node.sys, 'command_accepted', '?')}")
            last_state, last_acc, last_report = node.sys_name(), getattr(node.sys, "command_accepted", None), 0.0
            moved = set()
            while time.monotonic() - t_stop < ns.watch and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.02)
                t = time.monotonic() - t_stop
                sn, acc = node.sys_name(), getattr(node.sys, "command_accepted", None)
                if sn != last_state or acc != last_acc:
                    print(f"[watchdog] t={t:6.3f} driver {last_state}→{sn}, command_accepted {last_acc}→{acc}")
                    last_state, last_acc = sn, acc
                st2 = node.state
                for i, m in enumerate(st2.motor_id):
                    if m and int(m) not in moved and abs(float(st2.position[i]) - pos0[int(m)]) > 0.03:
                        moved.add(int(m)); print(f"[watchdog] t={t:6.3f} 모터 {int(m)} 위치 변화 {float(st2.position[i]) - pos0[int(m)]:+.3f} (유지 못 함 → 탈력/낙하?)")
                    if m and int(m) not in moved and st2.has_fault[i]:
                        moved.add(int(m)); print(f"[watchdog] t={t:6.3f} 모터 {int(m)} fault, error_code {st2.error_code[i]}")
                if t - last_report >= 2.0:
                    print(f"[watchdog] t={t:6.3f} driver {sn}, 위치 유지 중 {len(ids) - len(moved)}/{len(ids)}"); last_report = t
            print("[watchdog] 끝 — 상태가 안 바뀌고 위치도 유지됐으면 '명령 끊김 = 마지막 명령 유지'. 드라이버 설정의 워치독 시간과 비교.")
        elif ns.cmd == "stiffness":
            ids = command_ids(st, ns.include_not_ready)
            if ns.motor not in ids:
                print(f"모터 {ns.motor} 가 motor_state 에 없음: {ids}"); sys.exit(1)
            pos0 = {int(m): float(st.position[i]) for i, m in enumerate(st.motor_id) if m}
            print(f"[stiffness] 모터 {ns.motor} 를 현재 위치에 유지(kp {ns.kp}, kd {ns.kd}, mode {ns.mode}) {ns.seconds:.0f} s — 그 관절을 손으로 천천히 밀었다 놓으세요 (0.1~0.2 rad). 0.5 s 마다 Δ위치·effort·실효 강성 출력")
            t0, last = time.monotonic(), 0.0
            samples = []; pace = Pacer(ns.rate)
            while time.monotonic() - t0 < ns.seconds and rclpy.ok():
                node.send(ns.mode, ids, [pos0[m] for m in ids], ns.kp, ns.kd, ns.duration)
                spin_pending(node); pace.wait()
                t = time.monotonic() - t0
                if t - last >= 0.5:
                    last = t; st2 = node.state; i = list(st2.motor_id).index(ns.motor)
                    dq, eff = float(st2.position[i]) - pos0[ns.motor], float(st2.effort[i])
                    k = eff / dq if abs(dq) > 0.02 else float("nan")
                    samples.append((dq, eff))
                    print(f"[stiffness] t={t:5.1f} Δ={dq:+.3f}  effort={eff:+.3f}  effort/Δ={k:+.1f}   vel={float(st2.velocity[i]):+.3f}  temp={float(st2.temperature[i]):.1f}")
            good = [(d, e) for d, e in samples if abs(d) > 0.05]
            if good:
                import statistics
                ks = [e / d for d, e in good]
                print(f"[stiffness] 실효 강성 중앙값 {statistics.median(ks):+.1f} (effort 단위/rad). kp {ns.kp} 와 비교 → effort 단위가 N·m 면 kp 가 N·m/rad 인지(MIT 식) 알 수 있음. 부호가 음수면 effort 부호 규약이 반대")
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
