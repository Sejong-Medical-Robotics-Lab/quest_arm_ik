#!/usr/bin/env python3
"""삼현 ros2_state_bridge.py 를 그대로 실행하면서 '어디서 얼어붙는지' 를 찍는 진단 래퍼 (브리지 원본은 수정하지 않음).

  미니PC: 이 파일을 ~/hr_controller/tools/ 에 두고
    sed 's/ros2_state_bridge.py/hr_bridge_timed.py/' ~/hr_controller/tools/run_ros2_state_bridge.sh > ~/hr_controller/tools/run_bridge_timed.sh
    chmod +x ~/hr_controller/tools/run_bridge_timed.sh
    ~/hr_controller/tools/run_bridge_timed.sh 2>&1 | tee ~/bridge_timed.log
  Thor 에서 verify 를 돌린 뒤 로그의 [timed] 줄을 본다:
    [timed] _read_state 850 ms        → 상태 JSON 파일 읽기(디스크 /tmp)에서 막힘
    [timed] _send_ipc 1200 ms         → 제어기 쪽 IPC 소켓 큐가 차서 sendto 가 블로킹 (제어기 메인 루프 정지)
    [timed] _on_upper_body_command …  → 명령 콜백 전체 시간 (위 둘의 합이 아니면 그 밖의 파이썬 코드)
    [timed] TIMER GAP 1.9 s           → 브리지 타이머(상태 발행)가 그만큼 안 돌았음 (= Thor 에서 본 motor_states 공백)
    [timed] PROCESS STALL 900 ms      → 50 ms 마다 깨는 감시 스레드까지 못 깼음 = 프로세스 전체가 CPU 를 못 받음(실시간 스레드에 밀림)
                                        또는 실행기 스레드가 GIL 을 쥔 채 C 호출(publish 등) 안에서 막힘 — 이땐 같은 시각에 아래 줄이 같이 뜸
    [timed] publish[motor_pub] 2080 ms → 그 토픽의 DDS 발행이 막힘 (같은 PC 의 구독자(rosbridge 등)가 SHM 을 안 비우거나 RELIABLE 이력이 참)
    [timed] logger.warn 2100 ms       → rclpy 로거(rosout 발행)가 막힘
  TIMER GAP 만 있고 느린 단계도 PROCESS STALL 도 없으면: 실행기가 rclpy/DDS 안에서 막힌 것.
  publish 가 막히면 실험: 브리지를 `FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ~/hr_controller/tools/run_bridge_timed.sh` 로 띄워(공유메모리 전송 끔) 다시 본다.
  logger 가 막히면 실험: `HR_BRIDGE_ROS_ARGS="--log-level error" ~/hr_controller/tools/run_bridge_timed.sh` (경고 로그 자체를 끔) 또는
                         `HR_BRIDGE_ROS_ARGS="--disable-rosout-logs --disable-external-lib-logs" …` (rosout 발행·파일 기록만 끔).
"""
import importlib.util
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "ros2_state_bridge.py")
SLOW_S = 0.02            # 이보다 오래 걸린 단계를 찍는다
GAP_S = 0.3              # 타이머 공백 기준
STALL_S = 0.2            # 감시 스레드 지연 기준 (50 ms 주기)


def log(msg):
    print(f"[timed] {time.strftime('%H:%M:%S')} {msg}", file=sys.stderr, flush=True)


spec = importlib.util.spec_from_file_location("ros2_state_bridge", SRC)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
Bridge = mod.HrStateBridge


def timed(name, fn):
    def wrapped(self, *args, **kwargs):
        t0 = time.monotonic()
        try:
            return fn(self, *args, **kwargs)
        finally:
            dt = time.monotonic() - t0
            if dt > SLOW_S:
                log(f"{name} {dt * 1000:.0f} ms")
    return wrapped


for _name in ("_read_state", "_send_ipc", "_on_upper_body_command", "_tick"):
    setattr(Bridge, _name, timed(_name, getattr(Bridge, _name)))

# v2: _tick 안의 어느 호출인지까지 — 발행(publish)별, 로거별 시간. (2026-10-21 1차 결과: _tick 2.1 s, _read_state 는 정상 → publish 또는 logger)
_orig_init = Bridge.__init__


def _init_wrapped(self, *args, **kwargs):
    _orig_init(self, *args, **kwargs)
    for attr in ("system_pub", "motor_pub", "joint_pub", "axis_map_pub"):
        pub = getattr(self, attr, None)
        if pub is None:
            continue
        orig_pub = pub.publish

        def make(name, fn):
            def wrapped(msg):
                t0 = time.monotonic()
                try:
                    return fn(msg)
                finally:
                    dt = time.monotonic() - t0
                    if dt > SLOW_S:
                        log(f"publish[{name}] {dt * 1000:.0f} ms")
            return wrapped
        pub.publish = make(attr, orig_pub)
    try:
        import rclpy
        log(f"rmw = {rclpy.utilities.get_rmw_implementation_identifier()}, FASTDDS_BUILTIN_TRANSPORTS={os.environ.get('FASTDDS_BUILTIN_TRANSPORTS', '(unset)')}")
    except Exception as exc:  # noqa: BLE001
        log(f"rmw ? ({exc})")


Bridge.__init__ = _init_wrapped

try:
    import rclpy.impl.rcutils_logger as _rl
    for _n in ("warn", "warning", "info", "error", "debug"):
        if hasattr(_rl.RcutilsLogger, _n):
            setattr(_rl.RcutilsLogger, _n, timed(f"logger.{_n}", getattr(_rl.RcutilsLogger, _n)))
except Exception as exc:  # noqa: BLE001
    log(f"logger wrap skipped ({exc})")

_tick_timed = Bridge._tick


def _tick_with_gap(self):
    now = time.monotonic()
    last = getattr(self, "_timed_last_tick", None)
    if last is not None and now - last > GAP_S:
        log(f"TIMER GAP {now - last:.2f} s (상태 발행 타이머가 돌지 못한 시간)")
    self._timed_last_tick = now
    return _tick_timed(self)


Bridge._tick = _tick_with_gap


def _watchdog():
    """50 ms 마다 깨어나 자기 지연을 잰다: 프로세스 전체가 CPU 를 못 받으면 이 스레드도 늦게 깬다 (실행기 스레드가 시스템 콜에 막힌 것과 구분)."""
    period = 0.05
    nxt = time.monotonic() + period
    while True:
        time.sleep(max(0.0, nxt - time.monotonic()))
        now = time.monotonic()
        late = now - nxt
        if late > STALL_S:
            log(f"PROCESS STALL {late * 1000:.0f} ms (감시 스레드도 못 깸 = CPU 기아 또는 GIL 점유)")
        nxt = max(nxt + period, now)


# v3: 환경변수 HR_BRIDGE_ROS_ARGS 로 ros-args 를 넣는다 (브리지의 argparse 는 --ros-args 를 모르므로 rclpy.init 에만 전달).
#   예) HR_BRIDGE_ROS_ARGS="--log-level error"                              → WARN 이하 로그(타이머 jitter 경고 포함) 끔
#       HR_BRIDGE_ROS_ARGS="--disable-rosout-logs --disable-external-lib-logs" → rosout 발행·spdlog 파일 기록 끔
_ros_args = os.environ.get("HR_BRIDGE_ROS_ARGS", "").split()
if _ros_args:
    _real_init = mod.rclpy.init

    def _init_with_args(*args, **kwargs):
        kwargs.pop("args", None)
        return _real_init(args=["hr_state_bridge", "--ros-args", *_ros_args], **kwargs)

    mod.rclpy.init = _init_with_args
    log(f"rclpy.init ros-args = {_ros_args}")

threading.Thread(target=_watchdog, daemon=True).start()
log(f"wrapping {SRC} (slow>{SLOW_S * 1000:.0f} ms, gap>{GAP_S} s)")
sys.exit(mod.main())
