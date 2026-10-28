"""Thor ↔ 미니PC 사이의 가벼운 UDP 프로토콜 (DDS 를 링크 위에 태우지 않는다 — 2026-10-21 Fast DDS publish 블로킹 원인 미해결 대응).

모든 패킷은 리틀엔디언. 한 패킷이 이더넷 프레임 하나(≤ 1400 B)에 들어가도록 설계 (IP 조각 없음 → 유실에 강함).
  HELLO  Thor → 미니PC  'HRUH'  ver u16, seq u32                                   — 상태를 보낼 상대 주소 등록 (1 Hz)
  CMD    Thor → 미니PC  'HRUC'  ver u16, seq u32, mode u8, n u8, n×(motor_id u8, position f32)   — 위치 명령 (50 Hz)
  SVC    Thor → 미니PC  'HRUS'  ver u16, seq u32, svc u16, motor_id u8, pad u8, value f32          — 서비스 (enable/disable/…)
  ACK    미니PC → Thor  'HRUA'  ver u16, seq u32, ok u8, message(utf-8)                            — SVC 응답
  STATE  미니PC → Thor  'HRUT'  ver u16, seq u32, timestamp_us u64, fsm u8, run_mode u8, axes_connected u8,
                               axes_enabled u8, axes_faulted u8, can_ok u8, 15×(motor_id u8, connected u8, axis_state u8,
                               has_fault u8, channel u8, pad 3, position f32, velocity f32, effort f32)   — 상태 (20 Hz)
svc 번호는 HRController IPC 의 IpcCommandType 과 같다: 1 ENABLE_ALL, 2 DISABLE_ALL, 3 EMERGENCY_STOP, 4 RESET_FAULT,
6 ENABLE_AXIS, 7 DISABLE_AXIS, 9 SET_RUN_MODE(value=mode), 12 HOME_AXIS, 13 RESCAN_UID. motor_id 0 = 전체/해당 없음.
fsm 는 ros2_interfaces/UpperBodySystemState 의 값: 0 POWER_OFF 1 DRIVER_INIT 2 READY 3 ENABLING 4 ENABLED 5 RUNNING 6 FAULT.
"""
from __future__ import annotations

import struct
from typing import List, Optional, Sequence, Tuple

VERSION = 1
N_AXES = 15
MAGIC_HELLO, MAGIC_CMD, MAGIC_SVC, MAGIC_ACK, MAGIC_STATE = b"HRUH", b"HRUC", b"HRUS", b"HRUA", b"HRUT"

SVC_ENABLE_ALL, SVC_DISABLE_ALL, SVC_EMERGENCY_STOP, SVC_RESET_FAULT = 1, 2, 3, 4
SVC_ENABLE_AXIS, SVC_DISABLE_AXIS, SVC_SET_RUN_MODE, SVC_HOME_AXIS, SVC_RESCAN_UID = 6, 7, 9, 12, 13

FSM_POWER_OFF, FSM_DRIVER_INIT, FSM_READY, FSM_ENABLING, FSM_ENABLED, FSM_RUNNING, FSM_FAULT = 0, 1, 2, 3, 4, 5, 6
FSM_FROM_NAME = {"BOOT": FSM_POWER_OFF, "INIT": FSM_DRIVER_INIT, "READY": FSM_READY, "ENABLING": FSM_ENABLING,
                 "IMPEDANCE_CONTROL": FSM_ENABLED, "OD_CONTROL": FSM_ENABLED, "MOTION_RUNNING": FSM_RUNNING, "PAUSE": FSM_ENABLED,
                 "FAULT": FSM_FAULT, "EMERGENCY_STOP": FSM_FAULT}

_HELLO = struct.Struct("<4sHI")
_CMD_HDR = struct.Struct("<4sHIBB")
_CMD_ENTRY = struct.Struct("<Bf")
_SVC = struct.Struct("<4sHIHBBf")
_ACK_HDR = struct.Struct("<4sHIB")
_STATE_HDR = struct.Struct("<4sHIQBBBBBB")
_STATE_AXIS = struct.Struct("<BBBBB3xfff")
STATE_SIZE = _STATE_HDR.size + N_AXES * _STATE_AXIS.size          # 24 + 15*20 = 324 B


def pack_hello(seq: int) -> bytes:
    return _HELLO.pack(MAGIC_HELLO, VERSION, seq & 0xFFFFFFFF)


def pack_cmd(seq: int, mode: int, entries: Sequence[Tuple[int, float]]) -> bytes:
    """entries: [(motor_id, position_rad), …] — 0/255 는 호출자가 걸러서 넘긴다."""
    if len(entries) > 255:
        raise ValueError("too many entries")
    out = bytearray(_CMD_HDR.pack(MAGIC_CMD, VERSION, seq & 0xFFFFFFFF, int(mode) & 0xFF, len(entries)))
    for mid, pos in entries:
        out += _CMD_ENTRY.pack(int(mid) & 0xFF, float(pos))
    return bytes(out)


def pack_svc(seq: int, svc: int, motor_id: int = 0, value: float = 0.0) -> bytes:
    return _SVC.pack(MAGIC_SVC, VERSION, seq & 0xFFFFFFFF, int(svc) & 0xFFFF, int(motor_id) & 0xFF, 0, float(value))


def pack_ack(seq: int, ok: bool, message: str = "") -> bytes:
    return _ACK_HDR.pack(MAGIC_ACK, VERSION, seq & 0xFFFFFFFF, 1 if ok else 0) + message.encode("utf-8")[:900]


def pack_state(seq: int, timestamp_us: int, fsm: int, run_mode: int, axes_connected: int, axes_enabled: int, axes_faulted: int,
               can_ok: bool, axes: Sequence[Tuple[int, bool, int, bool, int, float, float, float]]) -> bytes:
    """axes: 15 × (motor_id, connected, axis_state, has_fault, channel, position, velocity, effort). 모자라면 빈 축(255)으로 채움."""
    out = bytearray(_STATE_HDR.pack(MAGIC_STATE, VERSION, seq & 0xFFFFFFFF, int(timestamp_us) & 0xFFFFFFFFFFFFFFFF, int(fsm) & 0xFF,
                                    int(run_mode) & 0xFF, int(axes_connected) & 0xFF, int(axes_enabled) & 0xFF, int(axes_faulted) & 0xFF,
                                    1 if can_ok else 0))
    for i in range(N_AXES):
        if i < len(axes):
            mid, conn, st, flt, ch, p, v, e = axes[i]
            out += _STATE_AXIS.pack(int(mid) & 0xFF, 1 if conn else 0, int(st) & 0xFF, 1 if flt else 0, int(ch) & 0xFF, float(p), float(v), float(e))
        else:
            out += _STATE_AXIS.pack(255, 0, 255, 0, 0, 0.0, 0.0, 0.0)
    return bytes(out)


class Hello:
    __slots__ = ("seq",)

    def __init__(self, seq):
        self.seq = seq


class Cmd:
    __slots__ = ("seq", "mode", "entries")

    def __init__(self, seq, mode, entries):
        self.seq, self.mode, self.entries = seq, mode, entries


class Svc:
    __slots__ = ("seq", "svc", "motor_id", "value")

    def __init__(self, seq, svc, motor_id, value):
        self.seq, self.svc, self.motor_id, self.value = seq, svc, motor_id, value


class Ack:
    __slots__ = ("seq", "ok", "message")

    def __init__(self, seq, ok, message):
        self.seq, self.ok, self.message = seq, ok, message


class State:
    __slots__ = ("seq", "timestamp_us", "fsm", "run_mode", "axes_connected", "axes_enabled", "axes_faulted", "can_ok", "axes")

    def __init__(self, seq, timestamp_us, fsm, run_mode, axes_connected, axes_enabled, axes_faulted, can_ok, axes):
        self.seq, self.timestamp_us, self.fsm, self.run_mode = seq, timestamp_us, fsm, run_mode
        self.axes_connected, self.axes_enabled, self.axes_faulted, self.can_ok, self.axes = axes_connected, axes_enabled, axes_faulted, can_ok, axes


def unpack(data: bytes):
    """패킷 → Hello | Cmd | Svc | Ack | State, 모르는/깨진 패킷이면 None."""
    if len(data) < 6 or data[4:6] != VERSION.to_bytes(2, "little"):
        return None
    magic = data[:4]
    try:
        if magic == MAGIC_HELLO and len(data) >= _HELLO.size:
            return Hello(_HELLO.unpack_from(data)[2])
        if magic == MAGIC_CMD and len(data) >= _CMD_HDR.size:
            _, _, seq, mode, n = _CMD_HDR.unpack_from(data)
            if len(data) < _CMD_HDR.size + n * _CMD_ENTRY.size:
                return None
            entries = [_CMD_ENTRY.unpack_from(data, _CMD_HDR.size + k * _CMD_ENTRY.size) for k in range(n)]
            return Cmd(seq, mode, [(int(m), float(p)) for m, p in entries])
        if magic == MAGIC_SVC and len(data) >= _SVC.size:
            _, _, seq, svc, mid, _, value = _SVC.unpack_from(data)
            return Svc(seq, svc, mid, value)
        if magic == MAGIC_ACK and len(data) >= _ACK_HDR.size:
            _, _, seq, ok = _ACK_HDR.unpack_from(data)
            return Ack(seq, bool(ok), data[_ACK_HDR.size:].decode("utf-8", errors="replace"))
        if magic == MAGIC_STATE and len(data) >= STATE_SIZE:
            _, _, seq, ts, fsm, run_mode, nc, ne, nf, can_ok = _STATE_HDR.unpack_from(data)
            axes = []
            for i in range(N_AXES):
                mid, conn, st, flt, ch, p, v, e = _STATE_AXIS.unpack_from(data, _STATE_HDR.size + i * _STATE_AXIS.size)
                axes.append((mid, bool(conn), st, bool(flt), ch, p, v, e))
            return State(seq, ts, fsm, run_mode, nc, ne, nf, bool(can_ok), axes)
    except struct.error:
        return None
    return None


def parse_hostport(text: str, default_port: int) -> Tuple[str, int]:
    host, _, port = text.rpartition(":")
    if not host:
        return text, default_port
    return host, int(port)
