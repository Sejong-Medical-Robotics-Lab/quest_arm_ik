from builtin_interfaces.msg import Time
class Header:
    def __init__(self): self.stamp, self.frame_id = Time(), ""
class Bool:
    def __init__(self, data=False): self.data = data
class Float32:
    def __init__(self, data=0.0): self.data = data
class Float64:
    def __init__(self, data=0.0): self.data = data
class String:
    def __init__(self, data=""): self.data = data
class MultiArrayLayout:
    def __init__(self): self.dim, self.data_offset = [], 0
class Float64MultiArray:
    def __init__(self, data=None): self.layout, self.data = MultiArrayLayout(), list(data) if data is not None else []
class ByteMultiArray:
    def __init__(self, data=None): self.layout, self.data = MultiArrayLayout(), list(data) if data is not None else []
