"""Thor 쪽 UDP 릴레이 — 미니PC 의 hr_udp_bridge.py 와 짝. 삼현 브리지가 내던 토픽·서비스 이름을 그대로 Thor 안에서 제공한다
(/upper_body/command 구독 → UDP CMD, UDP STATE → /upper_body/motor_states·system_state·joint_states·axis_map, 서비스 6종 → UDP SVC/ACK).
DDS 는 Thor 안에서만 쓰이고 링크 위로는 UDP 패킷(≤ 324 B, 조각 없음)만 오간다.

  ros2 run quest_arm_ik hr_udp_relay --ros-args -p peer_host:=192.168.10.2
  (launch hr:=true 는 hr_link:=udp(기본)일 때 이 노드를 같이 띄운다)
"""
from __future__ import annotations

import json
import select
import errno
import socket
import time
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .hr_udp_proto import (FSM_ENABLED, FSM_RUNNING, N_AXES, SVC_DISABLE_ALL, SVC_DISABLE_AXIS, SVC_EMERGENCY_STOP, SVC_ENABLE_ALL,
                           SVC_ENABLE_AXIS, SVC_HOME_AXIS, SVC_RESET_FAULT, SVC_SET_RUN_MODE, Ack, State, pack_cmd, pack_hello,
                           pack_svc, unpack)

EMPTY = 255
AXIS_RUN = 2


class HrUdpRelay(Node):
    def __init__(self):
        super().__init__("hr_udp_relay")
        from ros2_interfaces.msg import UpperBodyCommand, UpperBodySystemState, UpperMotorState
        from ros2_interfaces.srv import EnableMotors, SetRunMode, SimpleMotorResult
        self.MotorState, self.SysState = UpperMotorState, UpperBodySystemState
        defaults = {
            "peer_host": "192.168.10.2", "peer_port": 7001, "local_port": 7002, "bind": "0.0.0.0", "gap_warn": 0.1,
            "command_topic": "/upper_body/command", "motor_state_topic": "/upper_body/motor_states",
            "system_state_topic": "/upper_body/system_state", "joint_state_topic": "/upper_body/joint_states",
            "axis_map_topic": "/upper_body/axis_map",
            "poll_rate": 200.0,            # UDP 수신 폴링 [Hz]
            "hello_period": 1.0,           # 미니PC 에 상대 주소를 알리는 HELLO 주기 [s]
            "axis_map_period": 1.0,
            "state_timeout": 1.0,          # 이 시간 동안 상태가 없으면 경고 [s]
            "svc_timeout": 0.5,            # 서비스 ACK 대기 [s] (재시도 2회)
        }
        for k, v in defaults.items():
            if not self.has_parameter(k):
                self.declare_parameter(k, v)
        p = lambda k: self.get_parameter(k).value  # noqa: E731
        self.peer = (str(p("peer_host")), int(p("peer_port")))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # SO_REUSEADDR 를 쓰지 않는다: 릴레이가 둘이면 미니PC 의 STATE 가 한쪽에만 가서 다른 쪽(스트리머가 보는 쪽)이 굶는다.
        # 이전 릴레이가 끝나는 중일 수 있으니 5 s 동안 재시도하고, 그래도 안 되면 누가 잡고 있는지 알려 주고 끝낸다.
        bind_addr = (str(p("bind")), int(p("local_port")))
        deadline = time.monotonic() + 5.0
        while True:
            try:
                self.sock.bind(bind_addr)
                break
            except OSError as exc:
                if exc.errno != errno.EADDRINUSE or time.monotonic() > deadline:
                    self.get_logger().error(
                        f"UDP {bind_addr[0]}:{bind_addr[1]} 를 열 수 없음 ({exc}). 다른 hr_udp_relay 가 떠 있는 것 — "
                        f"컨테이너에서 `pgrep -af hr_udp_relay` / `ss -lunp | grep {bind_addr[1]}` 로 찾아 끄거나(pkill -f hr_udp_relay), "
                        f"그 릴레이를 그대로 쓰려면 launch 에 relay:=false")
                    raise SystemExit(1)
                time.sleep(0.25)
        self.sock.setblocking(False)
        self.seq = 0
        self.last_state: Optional[State] = None
        self.last_state_t = None
        self.n_cmd = self.n_state = 0
        self.svc_timeout = float(p("svc_timeout"))
        self.state_timeout = float(p("state_timeout"))
        self._no_state_warned = False
        # 끊김 감지 (v0.7.57, 2026-10-28 텔레옵 '뚝뚝 끊김'): STATE 가 gap_warn 보다 오래 안 오면 seq·제어기 timestamp 로 원인을 가른다
        self.gap_warn = float(p("gap_warn"))
        self.prev_state = None                                   # (seq, timestamp_us, 도착 monotonic)
        self.gaps: list = []                                     # 이번 보고 창의 끊김 [s]
        self.gap_kinds: dict = {}
        self.poll_prev = None
        self.poll_late = 0
        self.poll_late_max = 0.0

        rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)       # Thor 안에서만 쓰이는 토픽 — ros2 topic echo 도 바로 됨
        fast = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(UpperBodyCommand, p("command_topic"), self._on_command, fast)
        self.pub_motor = self.create_publisher(UpperMotorState, p("motor_state_topic"), rel)
        self.pub_sys = self.create_publisher(UpperBodySystemState, p("system_state_topic"), rel)
        self.pub_joint = self.create_publisher(JointState, p("joint_state_topic"), rel)
        self.pub_axis_map = self.create_publisher(String, p("axis_map_topic"), rel)
        self.create_service(EnableMotors, "/upper_body/enable_motors", self._srv_enable)
        self.create_service(SimpleMotorResult, "/upper_body/disable_motors", self._srv_disable)
        self.create_service(SimpleMotorResult, "/upper_body/clear_fault", lambda rq, rs: self._simple(rs, SVC_RESET_FAULT))
        self.create_service(SimpleMotorResult, "/upper_body/emergency_stop", lambda rq, rs: self._simple(rs, SVC_EMERGENCY_STOP))
        self.create_service(SimpleMotorResult, "/upper_body/home_axis", self._srv_home)
        self.create_service(SetRunMode, "/upper_body/set_run_mode", self._srv_run_mode)
        self.poll_period = 1.0 / float(p("poll_rate"))
        self.create_timer(self.poll_period, self._poll)
        self.create_timer(float(p("hello_period")), self._hello)
        self.create_timer(float(p("axis_map_period")), self._axis_map)
        self.create_timer(5.0, self._report)
        self._hello()
        self.get_logger().info(f"hr_udp_relay: 미니PC {self.peer[0]}:{self.peer[1]} ↔ 로컬 :{int(p('local_port'))}, "
                               f"{p('command_topic')} → UDP, UDP → {p('motor_state_topic')}/{p('system_state_topic')}")

    # ---- outgoing
    def _next_seq(self):
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        return self.seq

    def _send(self, data: bytes) -> bool:
        try:
            self.sock.sendto(data, self.peer)
            return True
        except OSError as exc:
            self.get_logger().warning(f"UDP 전송 실패 → {self.peer}: {exc}", throttle_duration_sec=5.0)
            return False

    def _hello(self):
        self._send(pack_hello(self._next_seq()))

    def _on_command(self, msg):
        entries = [(int(mid), float(msg.position[k])) for k, mid in enumerate(msg.motor_id) if mid not in (0, EMPTY)]
        if not entries:
            return
        if self._send(pack_cmd(self._next_seq(), int(msg.command_mode), entries)):
            self.n_cmd += 1

    # ---- incoming
    def _poll(self):
        now = time.monotonic()
        if self.poll_prev is not None and now - self.poll_prev > max(0.05, 5.0 * self.poll_period):
            self.poll_late += 1
            self.poll_late_max = max(self.poll_late_max, now - self.poll_prev)
        self.poll_prev = now
        self._pump(deadline=None, want_seq=None)
        if self.last_state_t is not None and time.monotonic() - self.last_state_t > self.state_timeout and not self._no_state_warned:
            self.get_logger().warning(f"미니PC 상태가 {self.state_timeout:.1f} s 이상 안 옴 (hr_udp_bridge.py 가 떠 있는지, 링크)")
            self._no_state_warned = True

    def _pump(self, deadline, want_seq) -> Optional[Ack]:
        """소켓을 비우며 STATE 는 발행, ACK 는 want_seq 와 맞으면 반환. deadline(monotonic)까지 기다릴 수 있음."""
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
            except BlockingIOError:
                if deadline is None:
                    return None
                wait = deadline - time.monotonic()
                if wait <= 0:
                    return None
                r, _, _ = select.select([self.sock], [], [], min(wait, 0.05))
                if not r:
                    if time.monotonic() >= deadline:
                        return None
                continue
            except OSError:
                return None
            pkt = unpack(data)
            if isinstance(pkt, State):
                self._publish_state(pkt)
            elif isinstance(pkt, Ack) and want_seq is not None and pkt.seq == want_seq:
                return pkt

    def _publish_state(self, st: State):
        now = time.monotonic()
        if self.prev_state is not None:
            pseq, pts, pt = self.prev_state
            dt = now - pt
            if dt > self.gap_warn:
                dseq = (int(st.seq) - int(pseq)) & 0xFFFFFFFF
                dts = ((int(st.timestamp_us) - int(pts)) & 0xFFFFFFFFFFFFFFFF) / 1e6
                if dseq > 1:
                    key, kind = "패킷 유실", f"패킷 {dseq - 1}개 유실 — 네트워크(이더넷/스위치) 쪽"
                elif dts > 0.7 * dt:
                    key, kind = "미니PC 정지", "미니PC 제어기가 그동안 상태를 안 냄 — 미니PC 정지(제어기 루프/CAN/부하)"
                else:
                    key, kind = "늦게 도착", "미니PC 는 제때 냈는데 늦게 도착 — 링크 지연 또는 Thor 릴레이가 그동안 못 돈 것(보고 줄의 '폴 지연' 과 대조)"
                self.gaps.append(dt)
                self.gap_kinds[key] = self.gap_kinds.get(key, 0) + 1
                self.get_logger().warning(f"상태 패킷 끊김 {dt * 1000:.0f} ms: {kind} (seq +{dseq}, 제어기 시각 +{dts * 1000:.0f} ms)", throttle_duration_sec=2.0)
        self.prev_state = (st.seq, st.timestamp_us, now)
        self.last_state, self.last_state_t, self.n_state = st, now, self.n_state + 1
        self._no_state_warned = False
        stamp = self.get_clock().now().to_msg()
        m = self.MotorState()
        m.header.stamp = stamp
        j = JointState()
        j.header.stamp = stamp
        j.name = [f"axis{i + 1}" for i in range(N_AXES)]
        n_en = 0
        for i, (mid, conn, ast, flt, _ch, pos, vel, eff) in enumerate(st.axes):
            en = bool(conn and ast == AXIS_RUN)
            n_en += en
            m.motor_id[i] = int(mid)
            m.is_ready[i] = bool(conn)
            m.is_enabled[i] = en
            m.has_fault[i] = bool(flt)
            m.position[i], m.velocity[i], m.effort[i] = float(pos), float(vel), float(eff)
            j.position.append(float(pos)); j.velocity.append(float(vel)); j.effort.append(float(eff))
        s = self.SysState()
        s.header.stamp = stamp
        s.state = int(st.fsm)
        s.run_mode = int(st.run_mode)
        s.ready_motor_count = min(N_AXES, int(st.axes_connected))
        s.enabled_motor_count = min(N_AXES, max(int(st.axes_enabled), n_en))
        s.fault_motor_count = min(N_AXES, int(st.axes_faulted))
        for i, ax in enumerate(st.axes):
            s.is_connected[i] = bool(ax[1])
        s.all_motors_enabled = bool(st.axes_connected > 0 and n_en >= st.axes_connected)
        s.command_accepted = st.fsm in (FSM_ENABLED, FSM_RUNNING)
        self.pub_sys.publish(s)
        self.pub_motor.publish(m)
        self.pub_joint.publish(j)

    def _axis_map(self):
        st = self.last_state
        if st is None:
            return
        rows = [{"axis_id": i + 1, "axis_index": i, "channel_id": int(ax[4]), "motor_id": int(ax[0]), "connected": bool(ax[1])}
                for i, ax in enumerate(st.axes)]
        d = {"connected_axes": [r for r in rows if r["motor_id"] != EMPTY], "live_connected_axes": [r for r in rows if r["connected"]], "all_axes": rows}
        self.pub_axis_map.publish(String(data=json.dumps(d, ensure_ascii=True)))

    def _report(self):
        age = (time.monotonic() - self.last_state_t) if self.last_state_t is not None else None
        fsm = self.last_state.fsm if self.last_state else None
        extra = ""
        if self.gaps:
            extra += f", 끊김 {len(self.gaps)}회 (최대 {max(self.gaps) * 1000:.0f} ms; " + ", ".join(f"{k} {n}" for k, n in self.gap_kinds.items()) + ")"
        if self.poll_late:
            extra += f", 릴레이 폴 지연 {self.poll_late}회 (최대 {self.poll_late_max * 1000:.0f} ms — Thor 쪽 CPU/스케줄링)"
        self.get_logger().info((f"cmd {self.n_cmd / 5:.0f} Hz → 미니PC, state {self.n_state / 5:.0f} Hz ← 미니PC "
                                f"(마지막 {age:.2f} s 전, fsm {fsm})" if age is not None else "미니PC 상태 아직 없음 — hr_udp_bridge.py 와 링크 확인") + extra)
        self.n_cmd = self.n_state = 0
        self.gaps, self.gap_kinds, self.poll_late, self.poll_late_max = [], {}, 0, 0.0

    # ---- services (UDP SVC → ACK)
    def _call(self, svc, motor_id=0, value=0.0):
        for _attempt in range(3):
            seq = self._next_seq()
            if not self._send(pack_svc(seq, svc, motor_id, value)):
                continue
            ack = self._pump(deadline=time.monotonic() + self.svc_timeout, want_seq=seq)
            if ack is not None:
                return ack.ok, ack.message
        return False, f"미니PC 응답 없음 ({self.peer[0]}:{self.peer[1]}, svc {svc})"

    def _connected_motor_ids(self):
        st = self.last_state
        return [int(ax[0]) for ax in st.axes if ax[0] != EMPTY and ax[1]] if st else []

    def _simple(self, resp, svc):
        resp.success, resp.message = self._call(svc)
        return resp

    def _srv_enable(self, req, resp):
        if bool(req.enable_all):
            ok, msg = self._call(SVC_ENABLE_ALL)
            ids = self._connected_motor_ids()
            resp.success, resp.message = ok, msg
            resp.enabled_motor_ids, resp.failed_motor_ids = (ids if ok else []), ([] if ok else ids)
            return resp
        done, failed = [], []
        for mid in req.motor_id_list:
            ok, _ = self._call(SVC_ENABLE_AXIS, int(mid))
            (done if ok else failed).append(int(mid))
        resp.success, resp.message = (not failed), ("ok" if not failed else f"failed {failed}")
        resp.enabled_motor_ids, resp.failed_motor_ids = done, failed
        return resp

    def _srv_disable(self, req, resp):
        if len(req.motor_id_list) == 0:
            resp.success, resp.message = self._call(SVC_DISABLE_ALL)
            return resp
        failed = [int(mid) for mid in req.motor_id_list if not self._call(SVC_DISABLE_AXIS, int(mid))[0]]
        resp.success, resp.message = (not failed), ("ok" if not failed else f"disable failed motor_id={failed}")
        return resp

    def _srv_home(self, req, resp):
        ids = [int(m) for m in req.motor_id_list if int(m) != 0] or self._connected_motor_ids()
        if not ids:
            resp.success, resp.message = False, "no mapped motors available"
            return resp
        failed = [mid for mid in ids if not self._call(SVC_HOME_AXIS, mid)[0]]
        resp.success, resp.message = (not failed), ("ok" if not failed else f"home_axis failed motor_id={failed}")
        return resp

    def _srv_run_mode(self, req, resp):
        mode = int(req.mode)
        if mode not in (1, 5):
            resp.success, resp.message = False, f"unsupported mode={mode} (position-only controller: 1 position, 5 position_alt)"
            return resp
        ids = [int(m) for m in req.motor_id_list] or self._connected_motor_ids()
        applied, failed = [], []
        for mid in ids:
            (applied if self._call(SVC_SET_RUN_MODE, mid, float(mode))[0] else failed).append(mid)
        resp.success, resp.message = (not failed), ("ok" if not failed else f"set_run_mode partial/failed motor_id={failed}")
        resp.applied_motor_ids, resp.failed_motor_ids = applied, failed
        return resp


def main(args=None):
    rclpy.init(args=args)
    try:
        node = HrUdpRelay()
    except SystemExit:
        rclpy.try_shutdown(); return 1
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    main()
