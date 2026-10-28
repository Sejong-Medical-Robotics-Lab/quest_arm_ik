from std_msgs.msg import Header
class Point:
    def __init__(self, x=0.0, y=0.0, z=0.0): self.x, self.y, self.z = x, y, z
class Vector3(Point): pass
class Quaternion:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0): self.x, self.y, self.z, self.w = x, y, z, w
class Pose:
    def __init__(self): self.position, self.orientation = Point(), Quaternion()
class PoseStamped:
    def __init__(self): self.header, self.pose = Header(), Pose()
class PoseArray:
    def __init__(self): self.header, self.poses = Header(), []
class Transform:
    def __init__(self): self.translation, self.rotation = Vector3(), Quaternion()
class TransformStamped:
    def __init__(self): self.header, self.child_frame_id, self.transform = Header(), "", Transform()
