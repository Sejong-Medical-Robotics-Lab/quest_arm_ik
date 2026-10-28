from std_msgs.msg import Header
class JointState:
    def __init__(self): self.header, self.name, self.position, self.velocity, self.effort = Header(), [], [], [], []
class Joy:
    def __init__(self): self.header, self.axes, self.buttons = Header(), [], []
