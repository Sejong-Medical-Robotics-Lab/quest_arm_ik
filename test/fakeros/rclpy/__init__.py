"""단일 프로세스·단일 스레드 가짜 rclpy: 토픽 라우팅, 타이머, 서비스, 파라미터 오버라이드, 가짜 단조시계.
harness 가 clock.advance(dt) 로 시간을 밀면 기한이 된 타이머가 실행되고, publish 된 메시지는 큐를 통해 구독자에 전달된다."""
import heapq
import itertools
import time as _time

PARAM_OVERRIDES = {}            # node_name -> {param: value}
_subs = {}                      # topic -> [(callback, node)]
_pubs = {}                      # topic -> 발행자 수 (count_publishers 흉내)
_services = {}                  # name -> (srv_type, callback)
_timers = []                    # heap of (due, seq, timer)
_queue = []                     # pending (callback, msg)
_seq = itertools.count()
_ok = False
_now = 0.0                      # 가짜 단조시계 [s]


def now():
    return _now


def init(args=None):
    global _ok
    _ok = True


def ok():
    return _ok


def shutdown():
    global _ok
    _ok = False


def try_shutdown():
    shutdown()


class _Timer:
    def __init__(self, period, cb):
        self.period, self.cb, self.cancelled = period, cb, False

    def cancel(self):
        self.cancelled = True


def _schedule(timer, due):
    heapq.heappush(_timers, (due, next(_seq), timer))


def deliver():
    """큐에 쌓인 메시지를 모두 전달 (콜백이 또 publish 하면 그것도 이어서)."""
    n = 0
    while _queue:
        cb, msg = _queue.pop(0)
        cb(msg)
        n += 1
    return n


def advance(dt):
    """시간을 dt 만큼 밀며 기한이 된 타이머를 순서대로 실행하고 메시지를 전달."""
    global _now
    target = _now + dt
    while _timers and _timers[0][0] <= target + 1e-12:
        due, _, timer = heapq.heappop(_timers)
        if timer.cancelled:
            continue
        _now = max(_now, due)
        timer.cb()
        deliver()
        _schedule(timer, due + timer.period)
    _now = target
    deliver()


def spin_once(node=None, timeout_sec=0.0):
    advance(max(timeout_sec, 0.0))


def spin(node=None):
    while _ok:
        advance(0.001)


def call_service(name, request):
    srv, cb = _services[name]
    return cb(request, srv.Response())


def reset():
    global _now
    PARAM_OVERRIDES.clear(); _subs.clear(); _pubs.clear(); _services.clear(); _timers.clear(); _queue.clear(); _now = 0.0
