import rclpy
from builtin_interfaces.msg import Time as _TimeMsg

LOG = []


class _Param:
    def __init__(self, value):
        self.value = value


class _Logger:
    def __init__(self, name):
        self.name = name
        self._last = {}

    def _log(self, level, msg, throttle_duration_sec=None, **kw):
        if throttle_duration_sec:
            t = rclpy.now(); last = self._last.get(msg)
            if last is not None and t - last < throttle_duration_sec:
                return
            self._last[msg] = t
        LOG.append((rclpy.now(), level, self.name, str(msg)))
        print(f"[{rclpy.now():7.3f}] [{level}] [{self.name}] {msg}")

    def info(self, msg, **kw): self._log("INFO", msg, **kw)
    def warning(self, msg, **kw): self._log("WARN", msg, **kw)
    warn = warning
    def error(self, msg, **kw): self._log("ERROR", msg, **kw)
    def debug(self, msg, **kw): pass


class _Time:
    def __init__(self, t):
        self.t = t
        self.nanoseconds = int(t * 1e9)

    def to_msg(self):
        m = _TimeMsg(); m.sec = int(self.t); m.nanosec = int((self.t - int(self.t)) * 1e9)
        return m

    def __sub__(self, other):
        from rclpy.duration import Duration
        return Duration(nanoseconds=self.nanoseconds - other.nanoseconds)


class _Clock:
    def now(self):
        return _Time(rclpy.now())


class _Publisher:
    def __init__(self, topic, msg_type):
        self.topic, self.msg_type, self.count = topic, msg_type, 0

    def publish(self, msg):
        self.count += 1
        for cb, _node in rclpy._subs.get(self.topic, []):
            rclpy._queue.append((cb, msg))

    def get_subscription_count(self):
        return len(rclpy._subs.get(self.topic, []))


class Node:
    def __init__(self, name, **kw):
        self._name = name
        self._params = {}
        self._overrides = dict(rclpy.PARAM_OVERRIDES.get(name, {}))
        self._logger = _Logger(name)
        self._clock = _Clock()
        self.publishers = {}
        self.timers = []

    def get_name(self): return self._name
    def get_logger(self): return self._logger
    def get_clock(self): return self._clock

    def declare_parameter(self, name, value=None, descriptor=None):
        v = self._overrides.get(name, value)
        if isinstance(value, list) and isinstance(v, list) and value and v:
            if isinstance(value[0], float): v = [float(x) for x in v]
            elif isinstance(value[0], int) and not isinstance(value[0], bool): v = [int(x) for x in v]
        elif isinstance(value, float) and isinstance(v, int) and not isinstance(v, bool): v = float(v)
        self._params[name] = v
        return _Param(v)

    def declare_parameters(self, ns, params):
        return [self.declare_parameter(n if not ns else f"{ns}.{n}", d) for n, d, *rest in [tuple(x) for x in params]]

    def has_parameter(self, name): return name in self._params

    def get_parameter(self, name):
        if name not in self._params:
            if name == "use_sim_time":
                return _Param(bool(self._overrides.get(name, False)))
            raise KeyError(f"{self._name}: parameter {name} not declared")
        return _Param(self._params[name])

    def create_publisher(self, msg_type, topic, qos=10):
        pub = _Publisher(topic, msg_type); self.publishers[topic] = pub
        rclpy._pubs[topic] = rclpy._pubs.get(topic, 0) + 1
        return pub

    def count_publishers(self, topic):
        return rclpy._pubs.get(topic, 0)

    def create_subscription(self, msg_type, topic, cb, qos=10):
        rclpy._subs.setdefault(topic, []).append((cb, self))
        return (topic, cb)

    def create_timer(self, period, cb, clock=None):
        t = rclpy._Timer(period, cb); self.timers.append(t)
        rclpy._schedule(t, rclpy.now() + period)
        return t

    def create_service(self, srv_type, name, cb):
        rclpy._services[name] = (srv_type, cb)
        return (name, cb)

    def add_on_set_parameters_callback(self, cb):
        self._param_cbs = getattr(self, "_param_cbs", []) + [cb]

    def set_parameters(self, params):
        """테스트용: 콜백을 거쳐 값 반영 (실 rclpy 의 set_parameters 와 같은 이름)."""
        results = []
        for cb in getattr(self, "_param_cbs", []):
            r = cb(params)
            results.append(r)
            if not r.successful:
                return results
        for prm in params:
            self._params[prm.name] = prm.value
        return results

    def destroy_node(self):
        for t in self.timers: t.cancel()
        for topic in list(rclpy._subs):
            rclpy._subs[topic] = [(cb, n) for cb, n in rclpy._subs[topic] if n is not self]
