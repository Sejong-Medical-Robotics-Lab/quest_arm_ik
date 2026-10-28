from builtin_interfaces.msg import Duration as _DurMsg


class Duration:
    def __init__(self, seconds=0.0, nanoseconds=0):
        self.nanoseconds = int(seconds * 1e9) + int(nanoseconds)

    def to_msg(self):
        m = _DurMsg(); m.sec = self.nanoseconds // 1_000_000_000; m.nanosec = self.nanoseconds % 1_000_000_000
        return m
