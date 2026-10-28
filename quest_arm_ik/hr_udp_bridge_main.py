"""미니PC 쪽 UDP 브리지 본체 — 삼현 ros2_state_bridge.py 를 대신한다 (ROS/DDS 없음, 표준 라이브러리만).

  Thor 의 hr_udp_relay 가 보내는 CMD/SVC/HELLO 를 받아 HRController 의 IPC 소켓(/tmp/hr_controller_cmd.sock)으로 넘기고,
  HRController 가 100 ms 마다 쓰는 /tmp/hr_controller_state.json 을 읽어 STATE 패킷으로 Thor 에 보낸다.
  단일 스레드 select 루프, 모든 소켓은 논블로킹 — 어디서도 기다리지 않는다 (DDS publish 처럼 멈추는 자리가 없음).

  python3 hr_udp_bridge.py [--listen 0.0.0.0:7001] [--peer 192.168.10.1:7002] [--state-file …] [--ipc …] [--state-rate 20]
  --peer 를 주지 않으면 HELLO/CMD 를 보낸 상대(Thor)의 주소로 상태를 보낸다.
"""
from __future__ import annotations

import argparse
import json
import os
import select
import socket
import struct
import sys
import time

try:  # 패키지 안에서 실행될 때
    from .hr_udp_proto import (FSM_FROM_NAME, FSM_POWER_OFF, N_AXES, SVC_DISABLE_ALL, SVC_EMERGENCY_STOP, SVC_ENABLE_ALL,
                               SVC_RESCAN_UID, SVC_RESET_FAULT, SVC_SET_RUN_MODE, Cmd, Hello, Svc, pack_ack, pack_state,
                               parse_hostport, unpack)
except ImportError:  # 단독 스크립트(tools/hr_udp_bridge.py)에서는 같은 파일 안에 proto 가 들어 있다
    pass

IPC_MAGIC = 0x4852434D          # 'HRCM' — HRController main.cpp 의 k_ipc_magic
IPC_VERSION = 1
IPC_MOVE_TO_POS = 8
_IPC = struct.Struct("<IHHB3xfffff")   # magic, version, type, axis_id, position, velocity, kp, kd, torque_ff (= IpcCommandPacket)
AXIS_RUN = 2                     # AxisState::Run — enable 된 축


def ipc_packet(cmd_type: int, axis_id: int = 0, position: float = 0.0, velocity: float = 0.0, kp: float = 0.0, kd: float = 0.0,
               torque_ff: float = 0.0) -> bytes:
    return _IPC.pack(IPC_MAGIC, IPC_VERSION, int(cmd_type) & 0xFFFF, int(axis_id) & 0xFF, float(position), float(velocity),
                     float(kp), float(kd), float(torque_ff))


class Counters:
    def __init__(self):
        self.cmd = self.cmd_drop = self.ipc = self.ipc_eagain = self.ipc_retry = self.ipc_err = self.state = self.unknown_motor = self.skipped_disabled = 0
        self.bad_packet = 0


class HrUdpBridge:
    def __init__(self, listen=("0.0.0.0", 7001), peer=None, state_file="/tmp/hr_controller_state.json",
                 ipc_path="/tmp/hr_controller_cmd.sock", state_rate=20.0, log_every=5.0, out=sys.stdout):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(listen)
        self.sock.setblocking(False)
        self.ipc = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.ipc.setblocking(False)
        self.ipc_path = ipc_path
        self.peer_fixed = peer
        self.peer_learned = None
        self.state_file = state_file
        self.state_period = 1.0 / max(float(state_rate), 1.0)
        self.log_every = float(log_every)
        self.out = out
        self.state = None               # 마지막으로 읽은 JSON (dict)
        self.state_mtime_ns = -1
        self.state_read_fail_logged = 0.0
        self.axis_by_motor = {}         # motor_id -> axis_id (1..15)
        self.axis_state = [255] * N_AXES
        self.state_seq = 0
        self.c = Counters()
        self._warn_ts = {}

    # ---- helpers
    def log(self, msg):
        print(f"[hr_udp_bridge] {time.strftime('%H:%M:%S')} {msg}", file=self.out, flush=True)

    def warn_throttled(self, key, msg, period=5.0):
        now = time.monotonic()
        if now - self._warn_ts.get(key, -1e9) >= period:
            self._warn_ts[key] = now
            self.log("WARN " + msg)

    @property
    def peer(self):
        return self.peer_fixed or self.peer_learned

    def send_ipc(self, packet: bytes) -> bool:
        """논블로킹 전송. 제어기 수신 큐(net.unix.max_dgram_qlen, 기본 10개)가 차 있으면 2 ms 씩 최대 5번 기다렸다 재시도
        (제어기 메인 루프가 5 ms 마다 큐를 비우므로 한 번의 재시도로 보통 풀린다). 그래도 안 되면 버리고 센다."""
        for attempt in range(6):
            try:
                self.ipc.sendto(packet, self.ipc_path)
                self.c.ipc += 1
                if attempt:
                    self.c.ipc_retry += 1
                return True
            except BlockingIOError:
                if attempt < 5:
                    time.sleep(0.002)
                    continue
                self.c.ipc_eagain += 1
                self.warn_throttled("ipc_eagain", "IPC 소켓이 가득 참 (hr_controller 메인 루프가 비우지 않음) — 명령을 버림. "
                                    "sudo sysctl -w net.unix.max_dgram_qlen=512 권장")
                return False
            except OSError as exc:
                self.c.ipc_err += 1
                self.warn_throttled("ipc_err", f"IPC 전송 실패 {self.ipc_path}: {exc} (hr_controller 가 떠 있는지)")
                return False
        return False

    # ---- state file → packet
    def read_state(self) -> bool:
        """파일이 바뀌었을 때만 읽는다. 읽었으면 True."""
        try:
            st = os.stat(self.state_file)
        except OSError:
            self.warn_throttled("state_missing", f"상태 파일 없음 {self.state_file} (hr_controller 가 떠 있는지)")
            return False
        if st.st_mtime_ns == self.state_mtime_ns:
            return False
        try:
            with open(self.state_file, "r", encoding="utf-8") as f:
                data = json.load(f)
        except (OSError, ValueError):
            return False               # 쓰는 중(tmp→rename 직전) 이면 다음 틱에
        self.state_mtime_ns = st.st_mtime_ns
        self.state = data
        ids = _ints(data.get("motor_id", []), 255)
        self.axis_by_motor = {mid: i + 1 for i, mid in enumerate(ids) if mid != 255}
        self.axis_state = _ints(data.get("axis_state", []), 255)
        return True

    def state_packet(self) -> bytes:
        d = self.state
        ids = _ints(d.get("motor_id", []), 255)
        conn = _bools(d.get("is_connected", []))
        ast = _ints(d.get("axis_state", []), 255)
        flt = _bools(d.get("has_fault", []))
        ch = _ints(d.get("channel_id", []), 0)
        pos = _floats(d.get("position", []))
        vel = _floats(d.get("velocity", []))
        eff = _floats(d.get("effort", []))
        axes = [(ids[i], conn[i], ast[i], flt[i], ch[i], pos[i], vel[i], eff[i]) for i in range(N_AXES)]
        fsm = FSM_FROM_NAME.get(str(d.get("fsm_state", "")).upper(), FSM_POWER_OFF)
        self.state_seq += 1
        return pack_state(self.state_seq, int(d.get("timestamp_us", 0)), fsm, int(d.get("run_mode", 0)),
                          int(d.get("axes_connected", 0)), int(d.get("axes_enabled", 0)), int(d.get("axes_faulted", 0)),
                          bool(d.get("can_ok", False)), axes)

    # ---- incoming
    def handle_cmd(self, cmd: Cmd):
        self.c.cmd += 1
        if cmd.mode not in (1, 6):     # POSITION / POSITION_SERVO 만 (제어기도 위치 전용)
            self.warn_throttled("mode", f"command_mode {cmd.mode} 는 지원 안 함 (1 POSITION 만) — 버림")
            self.c.cmd_drop += 1
            return
        if self.state is None:
            self.c.cmd_drop += 1
            self.warn_throttled("nostate", "상태 파일을 아직 못 읽어 motor_id→axis 를 모름 — 명령을 버림")
            return
        for mid, pos in cmd.entries:
            axis = self.axis_by_motor.get(mid)
            if axis is None:
                self.c.unknown_motor += 1
                self.warn_throttled(f"unk{mid}", f"motor_id {mid} 는 제어기 축 목록에 없음 — 건너뜀")
                continue
            if self.axis_state[axis - 1] != AXIS_RUN:
                self.c.skipped_disabled += 1   # enable 안 된 축엔 보내지 않음 (제어기가 거부하며 command_commit_failed 를 찍음)
                continue
            self.send_ipc(ipc_packet(IPC_MOVE_TO_POS, axis, pos))

    def handle_svc(self, svc: Svc):
        if svc.svc in (SVC_ENABLE_ALL, SVC_DISABLE_ALL, SVC_EMERGENCY_STOP, SVC_RESET_FAULT):
            ok = self.send_ipc(ipc_packet(svc.svc, 0))
            return ok, ("ok" if ok else "ipc send failed")
        axis = self.axis_by_motor.get(svc.motor_id) if self.state is not None else None
        if svc.svc == SVC_RESCAN_UID and svc.motor_id == 0:
            axis = 0
        if axis is None:
            return False, f"unknown motor_id {svc.motor_id}"
        value = float(svc.value) if svc.svc == SVC_SET_RUN_MODE else 0.0
        ok = self.send_ipc(ipc_packet(svc.svc, axis, value))
        return ok, ("ok" if ok else "ipc send failed")

    def drain(self):
        while True:
            try:
                data, addr = self.sock.recvfrom(4096)
            except BlockingIOError:
                return
            except OSError:
                return
            pkt = unpack(data)
            if pkt is None:
                self.c.bad_packet += 1
                continue
            if isinstance(pkt, Hello):
                if self.peer_learned != addr:
                    self.log(f"peer {addr[0]}:{addr[1]} 등록 (HELLO)")
                self.peer_learned = addr
            elif isinstance(pkt, Cmd):
                if self.peer_learned is None:
                    self.peer_learned = addr
                self.handle_cmd(pkt)
            elif isinstance(pkt, Svc):
                ok, msg = self.handle_svc(pkt)
                self.log(f"svc {pkt.svc} motor {pkt.motor_id} value {pkt.value:g} → {'ok' if ok else 'FAIL'} {msg}")
                try:
                    self.sock.sendto(pack_ack(pkt.seq, ok, msg), addr)
                except OSError:
                    pass

    def tick_state(self):
        self.read_state()
        peer = self.peer
        if self.state is None or peer is None:
            return
        try:
            self.sock.sendto(self.state_packet(), peer)
            self.c.state += 1
        except OSError as exc:
            self.warn_throttled("state_send", f"상태 전송 실패 → {peer}: {exc}")

    def status_line(self, dt):
        c = self.c
        fsm = str(self.state.get("fsm_state", "?")) if self.state else "(상태 없음)"
        n_conn = sum(_bools(self.state.get("is_connected", []))) if self.state else 0
        n_en = sum(1 for s in self.axis_state if s == AXIS_RUN) if self.state else 0
        peer = f"{self.peer[0]}:{self.peer[1]}" if self.peer else "(없음 — Thor 의 HELLO 대기)"
        self.log(f"peer={peer} cmd {c.cmd / dt:.1f}/s (drop {c.cmd_drop}) ipc {c.ipc / dt:.0f}/s (retry {c.ipc_retry} drop {c.ipc_eagain} err {c.ipc_err}) "
                 f"state {c.state / dt:.1f}/s fsm={fsm} connected {n_conn}/{N_AXES} enabled {n_en} unknown_motor {c.unknown_motor} "
                 f"skipped_disabled {c.skipped_disabled} bad {c.bad_packet}")
        self.c = Counters()

    def run(self, duration=None):
        self.log(f"listen {self.sock.getsockname()} → ipc {self.ipc_path}, state {self.state_file} @ {1 / self.state_period:.0f} Hz, "
                 f"peer {'고정 ' + str(self.peer_fixed) if self.peer_fixed else '(HELLO 로 학습)'}")
        try:
            qlen = int(open("/proc/sys/net/unix/max_dgram_qlen").read())
            if qlen < 64:
                self.log(f"참고: net.unix.max_dgram_qlen = {qlen} (명령 한 번에 IPC 14개가 몰리면 큐가 참 → 재시도로 버팀). "
                         "영구 설정: echo 'net.unix.max_dgram_qlen = 512' | sudo tee /etc/sysctl.d/99-hr.conf && sudo sysctl --system")
        except (OSError, ValueError):
            pass
        t0 = time.monotonic()
        next_state = t0
        next_log = t0 + self.log_every
        while duration is None or time.monotonic() - t0 < duration:
            timeout = max(0.0, next_state - time.monotonic())
            try:
                r, _, _ = select.select([self.sock], [], [], min(timeout, 0.1))
            except InterruptedError:
                continue
            if r:
                self.drain()
            now = time.monotonic()
            if now >= next_state:
                self.tick_state()
                next_state += self.state_period
                if next_state < now - 1.0:          # 오래 밀렸으면 몰아 보내지 않고 다시 맞춤
                    next_state = now + self.state_period
            if now >= next_log:
                self.status_line(now - (next_log - self.log_every))
                next_log = now + self.log_every


def _ints(values, default):
    out = [default] * N_AXES
    for i in range(min(N_AXES, len(values))):
        try:
            out[i] = int(values[i])
        except (TypeError, ValueError):
            pass
    return out


def _bools(values):
    out = [False] * N_AXES
    for i in range(min(N_AXES, len(values))):
        out[i] = bool(values[i])
    return out


def _floats(values):
    out = [0.0] * N_AXES
    for i in range(min(N_AXES, len(values))):
        try:
            out[i] = float(values[i])
        except (TypeError, ValueError):
            pass
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description="HRController ↔ Thor UDP 브리지 (미니PC, ROS 불필요)")
    ap.add_argument("--listen", default="0.0.0.0:7001", help="명령을 받을 주소:포트 (기본 0.0.0.0:7001)")
    ap.add_argument("--peer", default="", help="상태를 보낼 Thor 주소:포트 (기본: HELLO/CMD 를 보낸 상대에게)")
    ap.add_argument("--state-file", default="/tmp/hr_controller_state.json")
    ap.add_argument("--ipc", default="/tmp/hr_controller_cmd.sock")
    ap.add_argument("--state-rate", type=float, default=20.0, help="상태 송신 주기 [Hz] (파일은 10 Hz 로 바뀜)")
    ap.add_argument("--log-every", type=float, default=5.0)
    ap.add_argument("--duration", type=float, default=None, help="시험용: 이 시간 뒤 종료")
    ns = ap.parse_args(argv)
    listen = parse_hostport(ns.listen, 7001)
    peer = parse_hostport(ns.peer, 7002) if ns.peer else None
    bridge = HrUdpBridge(listen, peer, ns.state_file, ns.ipc, ns.state_rate, ns.log_every)
    try:
        bridge.run(ns.duration)
    except KeyboardInterrupt:
        bridge.log("종료")
    return 0


if __name__ == "__main__":
    sys.exit(main())
