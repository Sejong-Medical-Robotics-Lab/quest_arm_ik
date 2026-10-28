class ReliabilityPolicy:
    RELIABLE, BEST_EFFORT = 1, 2


class DurabilityPolicy:
    TRANSIENT_LOCAL, VOLATILE = 1, 2


class HistoryPolicy:
    KEEP_LAST, KEEP_ALL = 1, 2


class QoSProfile:
    def __init__(self, depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST):
        self.depth, self.reliability, self.durability, self.history = depth, reliability, durability, history
