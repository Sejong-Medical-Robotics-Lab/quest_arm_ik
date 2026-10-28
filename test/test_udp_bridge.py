"""UDP 링크를 ROS 없이 끝까지 돌려 본다 — 미니PC 쪽 두 구현 모두.

    SH_REPO=$SH $QPY test/test_udp_bridge.py          # (A) tools/hr_udp_bridge.py  (Python, 삼현 제어기 무수정 + IPC/JSON)
    SH_REPO=$SH $QPY test/test_udp_bridge.py --cpp    # (B) hr_controller 내장 링크 (tools/hr_controller_v2/hr_udp_link.cpp 를 g++ 로 빌드한 가짜 제어기)
    SH_REPO=$SH $QPY test/test_udp_bridge.py --all    # 둘 다

  - (A) 가짜 HRController(이 프로세스): IPC 데이터그램을 받아 상태 JSON 을 50 ms 마다 tmp→rename 으로 씀 (실제와 같은 형식);
        tools/hr_udp_bridge.py 를 실제 서브프로세스로 띄움 (단독 스크립트가 최신인지도 확인)
  - (B) hr::udp::Link 위에 만든 가짜 제어기(C++) 를 서브프로세스로 띄움 — main.cpp 가 쓰는 것과 같은 모듈·같은 결정 규칙
  - 공통: hr_udp_relay 노드를 가짜 rclpy 위에서 — 토픽·서비스 이름이 삼현 브리지와 같은지, 상태가 motor_states/system_state 로 나오는지,
    /upper_body/command 가 제어기까지 가는지, enable/disable/clear_fault/set_run_mode 서비스가 ACK 로 끝나는지
"""
import json
import os
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(HERE, "fakeros"))
sys.path.insert(0, ROOT)
if not os.environ.get("SH_REPO"):
    print("SH_REPO 필요 (ros2_interfaces .msg 위치)"); sys.exit(1)

import rclpy  # noqa: E402  (가짜)
from quest_arm_ik import hr_udp_proto as P  # noqa: E402

IPC = struct.Struct("<IHHB3xfffff")
MOTORS = [34, 113, 125, 52, 105, 115, 57, 51, 32, 33, 31, 35, 36, 37, 255]
PORT_BRIDGE, PORT_RELAY = 17001, 17002


class FakeController:
    """HRController 흉내 (A): IPC 소켓에서 명령을 받고 상태 JSON 을 쓴다."""

    def __init__(self, sock_path, state_path):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.sock.bind(sock_path)
        self.sock.setblocking(False)
        self.state_path = state_path
        self.fsm = "READY"
        self.axis_state = [0] * 14 + [255]
        self.pos = [0.0] * 15
        self.target = [0.0] * 15
        self.received = []          # (type, axis, value)
        self.last_write = 0.0
        self.write()
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)   # 실제 제어기처럼 5 ms 마다 큐를 비운다
        self.thread.start()

    def _loop(self):
        while self.running:
            self.step()
            time.sleep(0.005)

    def step(self):
        while True:
            try:
                data = self.sock.recv(256)
            except BlockingIOError:
                break
            if len(data) != IPC.size:
                continue
            magic, ver, typ, axis, position, *_ = IPC.unpack(data)
            assert magic == 0x4852434D and ver == 1
            self.received.append((typ, axis, position))
            if typ == 1:      # ENABLE_ALL
                self.axis_state = [2] * 14 + [255]; self.fsm = "OD_CONTROL"
            elif typ == 2:    # DISABLE_ALL
                self.axis_state = [0] * 14 + [255]; self.fsm = "READY"
            elif typ == 3:
                self.fsm = "EMERGENCY_STOP"
            elif typ == 4:
                self.fsm = "READY"
            elif typ == 8 and 1 <= axis <= 14 and self.axis_state[axis - 1] == 2:
                self.target[axis - 1] = position
        for i in range(14):
            self.pos[i] += 0.5 * (self.target[i] - self.pos[i])    # 1차 지연 추종
        if time.monotonic() - self.last_write >= 0.05:
            self.write()

    def write(self):
        d = {"timestamp_us": int(time.monotonic() * 1e6), "fsm_state": self.fsm, "axes_connected": 14,
             "axes_enabled": sum(1 for s in self.axis_state if s == 2), "axes_faulted": 0, "can_ok": True,
             "is_connected": [True] * 14 + [False], "motor_id": MOTORS, "channel_id": [1] * 7 + [2] * 8,
             "feedback_counter": [100] * 15, "position": self.pos, "raw_position": [0] * 15, "velocity": [0.0] * 15,
             "effort": [0.0] * 15, "has_fault": [False] * 15, "axis_state": self.axis_state, "run_mode": 1}
        tmp = self.state_path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(d, f)
        os.replace(tmp, self.state_path)
        self.last_write = time.monotonic()

    def moves(self):
        return [(a, p) for t, a, p in self.received if t == 8]

    def saw_enable_all(self):
        return any(t == 1 for t, _, _ in self.received)

    def saw_run_mode(self, value):
        return any(t == 9 and abs(v - value) < 1e-6 for t, _, v in self.received)


def file_from_patch(patch_text, path):
    """패치 안의 '새 파일' diff 에서 내용을 꺼낸다 (tools 의 복사본이 패치와 같은지 확인용)."""
    marker = f"diff --git a/{path} b/{path}\n"
    i = patch_text.index(marker)
    body = patch_text[i:]
    j = body.find("\ndiff --git ", 1)
    body = body if j < 0 else body[:j + 1]
    lines = body.split("\n")
    k = next(n for n, l in enumerate(lines) if l.startswith("@@"))
    out = [l[1:] for l in lines[k + 1:] if l.startswith("+")]
    return "\n".join(out) + "\n"


def build_cpp_fake(tmp):
    """tools/hr_controller_v2 의 링크 모듈 + 가짜 제어기를 g++ 로 빌드 (복사본이 패치의 새 파일과 같은지도 확인)."""
    src = os.path.join(ROOT, "tools", "hr_controller_v2")
    patch = open(os.path.join(src, "hr_controller_demo.patch"), encoding="utf-8").read()
    for rel, local in (("src/udp/hr_udp_link.cpp", "hr_udp_link.cpp"), ("include/hr/udp/hr_udp_link.hpp", os.path.join("hr", "udp", "hr_udp_link.hpp"))):
        assert file_from_patch(patch, rel) == open(os.path.join(src, local), encoding="utf-8").read(), f"{local} 가 hr_controller_demo.patch 와 다름"
    gxx = shutil.which("g++") or shutil.which("c++")
    assert gxx, "g++ 가 필요 (apt install g++)"
    out = os.path.join(tmp, "fake_ctrl")
    cmd = [gxx, "-std=c++20", "-O2", "-Wall", "-Wextra", "-Wpedantic", "-I", src,
           os.path.join(src, "hr_udp_link_fake_controller.cpp"), os.path.join(src, "hr_udp_link.cpp"), "-o", out]
    r = subprocess.run(cmd, capture_output=True, text=True)
    assert r.returncode == 0, f"C++ 빌드 실패:\n{r.stderr}"
    assert "warning" not in r.stderr.lower(), f"C++ 경고:\n{r.stderr}"
    return out


def run_scenario(label, ctrl_proc, py_ctrl=None, expected_state_hz=50):
    """릴레이 쪽 시나리오 — ctrl_proc 는 미니PC 역할 서브프로세스, py_ctrl 은 (A) 의 가짜 제어기(없으면 로그로 검사)."""
    from quest_arm_ik.hr_udp_relay_node import HrUdpRelay
    from ros2_interfaces.msg import UpperBodyCommand
    from ros2_interfaces.srv import EnableMotors, SetRunMode, SimpleMotorResult
    from rclpy.node import Node

    rclpy.reset()
    rclpy.PARAM_OVERRIDES["hr_udp_relay"] = {"peer_host": "127.0.0.1", "peer_port": PORT_BRIDGE, "local_port": PORT_RELAY, "svc_timeout": 0.5}
    rclpy.init()
    relay = HrUdpRelay()
    probe = Node("probe")
    states, syss = [], []
    probe.create_subscription(object, "/upper_body/motor_states", lambda m: states.append(m), 10)
    probe.create_subscription(object, "/upper_body/system_state", lambda m: syss.append(m), 10)
    cmd_pub = probe.create_publisher(UpperBodyCommand, "/upper_body/command", 10)

    def run(seconds):
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            rclpy.advance(0.005)
            time.sleep(0.004)

    run(1.0)
    assert syss and states, f"[{label}] 상태가 안 옴 (제어기 → relay)"
    assert syss[-1].state == 2 and syss[-1].command_accepted is False, f"READY 여야 함: {syss[-1].state}"
    assert list(states[-1].motor_id) == MOTORS and states[-1].is_ready[0] and not states[-1].is_enabled[0], "motor_states 내용"
    rate = len(states) / 1.0
    print(f"[{label} 1] 상태 수신 {rate:.0f} Hz (제어기 {expected_state_hz} Hz 송신), fsm READY, motor_id 15슬롯 = 제어기 순서 OK")
    assert rate >= expected_state_hz * 0.6

    req = EnableMotors.Request(); req.enable_all = True
    res = rclpy.call_service("/upper_body/enable_motors", req)
    assert res.success and 34 in list(res.enabled_motor_ids), (res.success, res.message)
    run(0.3)
    assert syss[-1].state == 4 and syss[-1].command_accepted and states[-1].is_enabled[3], f"enable 후 state {syss[-1].state}"
    if py_ctrl is not None:
        assert py_ctrl.saw_enable_all(), "ENABLE_ALL IPC 가 제어기에 도착해야 함"
    print(f"[{label} 2] enable_motors → ACK {res.message}, fsm ENABLED(4), is_enabled OK")

    if py_ctrl is not None:
        py_ctrl.received.clear()
    n_sent = 0
    t_end = time.monotonic() + 0.6
    while time.monotonic() < t_end:
        m = UpperBodyCommand(); m.command_mode = 1
        for k in range(15):
            m.motor_id[k] = MOTORS[k]; m.position[k] = 0.3 if MOTORS[k] == 52 else 0.0
        cmd_pub.publish(m); n_sent += 1
        rclpy.advance(0.02); time.sleep(0.018)
    run(0.3)
    i52 = list(states[-1].motor_id).index(52)
    assert abs(states[-1].position[i52] - 0.3) < 0.02, states[-1].position[i52]
    if py_ctrl is not None:
        moves = py_ctrl.moves()
        assert moves, "MoveToPos IPC 가 안 옴"
        assert all(a in range(1, 15) for a, _ in moves) and not any(a == 15 for a, _ in moves), "미연결 15번(255) 에는 보내지 않아야"
        elbow = [p for a, p in moves if a == 4]
        assert elbow and abs(elbow[-1] - 0.3) < 1e-6, elbow[-3:]
        print(f"[{label} 3] 명령 {n_sent}개 → IPC MoveToPos {len(moves)}건 (축당 {len(moves) // 14}), 팔꿈치 측정 {states[-1].position[i52]:+.3f} (목표 +0.300)")
    else:
        print(f"[{label} 3] 명령 {n_sent}개 → 팔꿈치 측정 {states[-1].position[i52]:+.3f} (목표 +0.300)")

    r_req = SetRunMode.Request(); r_req.mode = 5
    r = rclpy.call_service("/upper_body/set_run_mode", r_req)
    assert r.success and len(r.applied_motor_ids) == 14, (r.success, r.message)
    bad = SetRunMode.Request(); bad.mode = 2
    assert not rclpy.call_service("/upper_body/set_run_mode", bad).success
    assert rclpy.call_service("/upper_body/clear_fault", SimpleMotorResult.Request()).success
    d = rclpy.call_service("/upper_body/disable_motors", SimpleMotorResult.Request())
    assert d.success
    run(0.3)
    assert syss[-1].state == 2 and not states[-1].is_enabled[0], "disable 후 READY"
    if py_ctrl is not None:
        assert py_ctrl.saw_run_mode(5.0), "SET_RUN_MODE value=5 가 IPC 로"
    print(f"[{label} 4] set_run_mode(5) 14축 OK / mode 2 거부 / clear_fault / disable_motors → READY")
    return n_sent


def scenario_python():
    assert subprocess.call([sys.executable, os.path.join(ROOT, "tools", "build_hr_udp_bridge.py"), "--check"]) == 0, "단독 스크립트가 오래됨"
    tmp = tempfile.mkdtemp(prefix="hrudp_")
    sock_path, state_path = os.path.join(tmp, "cmd.sock"), os.path.join(tmp, "state.json")
    ctrl = FakeController(sock_path, state_path)
    bridge = subprocess.Popen([sys.executable, "-I", os.path.join(ROOT, "tools", "hr_udp_bridge.py"), "--listen", f"127.0.0.1:{PORT_BRIDGE}",
                               "--state-file", state_path, "--ipc", sock_path, "--state-rate", "50", "--log-every", "2"],
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        time.sleep(0.3)
        run_scenario("A", bridge, py_ctrl=ctrl, expected_state_hz=50)
        ctrl.running = False
        bridge.terminate(); out = bridge.communicate(timeout=5)[0]
        assert "WARN" not in out, out
        print("[A 5] hr_udp_bridge.py 로그: " + out.strip().splitlines()[-1])
    finally:
        if bridge.poll() is None:
            bridge.kill()


def scenario_cpp():
    tmp = tempfile.mkdtemp(prefix="hrudp_cpp_")
    exe = build_cpp_fake(tmp)
    print("[B 0] tools/hr_controller_v2/hr_udp_link.cpp + 가짜 제어기 g++ 빌드 OK (경고 없음)")
    ctrl = subprocess.Popen([exe, "--listen-port", str(PORT_BRIDGE), "--state-rate", "100", "--duration", "20"],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        time.sleep(0.2)
        n_sent = run_scenario("B", ctrl, py_ctrl=None, expected_state_hz=100)
        ctrl.terminate()
        try:
            out = ctrl.communicate(timeout=5)[0]
        except subprocess.TimeoutExpired:
            ctrl.kill(); out = ctrl.communicate()[0]
        reports = [l for l in out.splitlines() if l.startswith("[fake] peer=")]
        exit_line = [l for l in out.splitlines() if l.startswith("[fake] exit")]
        assert reports and exit_line, out
        tot_cmd = sum(int(l.split("cmd=")[1].split()[0]) for l in reports)
        tot_entries = sum(int(l.split("entries=")[1].split()[0]) for l in reports)
        tot_bad = sum(int(l.split("bad=")[1].split()[0]) for l in reports)
        tot_fail = sum(int(l.split("fail=")[1].split()[0]) for l in reports)
        assert tot_cmd >= n_sent, (tot_cmd, n_sent)
        assert tot_entries == tot_cmd * 14, (tot_entries, tot_cmd)       # 릴레이는 255 슬롯을 빼고 14개만 보낸다
        assert tot_bad == 0 and tot_fail == 0, out
        moves = int(exit_line[0].split("moves=")[1].split()[0])
        assert moves == tot_entries, (moves, tot_entries)                # enable 된 14축 전부 목표 갱신
        assert "last_mode=5" in exit_line[0], exit_line[0]
        print(f"[B 5] 제어기 쪽 집계: CMD {tot_cmd}개(항목 {tot_entries} = 14/개 = 목표 갱신 {moves}), 깨진 패킷 0, 전송 실패 0, 마지막 run_mode 5")
    finally:
        if ctrl.poll() is None:
            ctrl.kill()


def test_relay_gap_classification():
    """(v0.7.57) 릴레이의 STATE 끊김 분류: seq 비면 '유실', 제어기 시각도 같이 비면 '미니PC 정지', 제어기 시각은 연속인데 늦게 오면 '지연'.
    가짜 시계로 _publish_state 를 직접 호출한다 (소켓은 바인드만)."""
    from quest_arm_ik.hr_udp_relay_node import HrUdpRelay
    from quest_arm_ik.hr_udp_proto import State
    rclpy.reset()
    rclpy.PARAM_OVERRIDES["hr_udp_relay"] = {"peer_host": "127.0.0.1", "peer_port": PORT_BRIDGE, "local_port": PORT_RELAY + 7, "svc_timeout": 0.5}
    rclpy.init()
    real_mono = time.monotonic
    time.monotonic = rclpy.now
    try:
        relay = HrUdpRelay()
        axes = [(i + 1, 1, 2, 0, i, 0.0, 0.0, 0.0) for i in range(15)]
        def st(seq, ts_us):
            return State(seq, ts_us, 4, 1, 15, 15, 0, 1, axes)
        seq, ts = 0, 0
        def send(dt_arrival, dseq=1, dts=None):
            nonlocal seq, ts
            rclpy.advance(dt_arrival)
            seq += dseq; ts += int(1e6 * (dts if dts is not None else dt_arrival))
            relay._publish_state(st(seq, ts))
        for _ in range(20):
            send(0.01)
        assert not relay.gaps
        send(0.3, dseq=25)                       # 24개 유실 (네트워크)
        send(0.01)
        send(0.3, dseq=1, dts=0.3)               # 미니PC 가 그동안 안 냄
        send(0.01)
        send(0.3, dseq=1, dts=0.01)              # 미니PC 는 냈는데 늦게 도착
        kinds = list(relay.gap_kinds)
        assert len(relay.gaps) == 3 and all(abs(g - 0.3) < 1e-6 for g in relay.gaps), relay.gaps
        assert sorted(kinds) == sorted(["패킷 유실", "미니PC 정지", "늦게 도착"]), kinds
        relay._report()
        assert not relay.gaps and not relay.gap_kinds
        relay.destroy_node()
    finally:
        time.monotonic = real_mono
    print("relay gap classification: 유실 / 미니PC 정지 / 지연 분류 OK")


def main():
    which = sys.argv[1:] or ["--py"]
    test_relay_gap_classification()
    if "--all" in which:
        which = ["--py", "--cpp"]
    if "--py" in which:
        scenario_python()
    if "--cpp" in which:
        scenario_cpp()
    print("\ntest_udp_bridge: 전 구간 OK (" + ", ".join(w.strip("-") for w in which) + ")")


if __name__ == "__main__":
    main()
