from std_msgs.msg import Header
from builtin_interfaces.msg import Duration
class JointTrajectoryPoint:
    def __init__(self): self.positions, self.velocities, self.accelerations, self.effort, self.time_from_start = [], [], [], [], Duration()
class JointTrajectory:
    def __init__(self): self.header, self.joint_names, self.points = Header(), [], []
