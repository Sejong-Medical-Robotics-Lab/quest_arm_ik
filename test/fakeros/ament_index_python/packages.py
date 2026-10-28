import os
def get_package_share_directory(name):
    if name == "quest_arm_ik":
        return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
    raise RuntimeError(name)
