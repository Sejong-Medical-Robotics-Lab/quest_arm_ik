import os
import re
from glob import glob

from setuptools import find_packages, setup

package_name = "quest_arm_ik"

setup(
    name=package_name,
    version=re.search(r'__version__ = "([^"]+)"', open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "quest_arm_ik", "__init__.py")).read()).group(1),   # quest_arm_ik/__init__.py 와 package.xml 을 같이 올릴 것
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "scripts"), glob("scripts/*.py") + glob("scripts/*.sh")),
        (os.path.join("share", package_name, "tools"), glob("tools/*.py") + glob("tools/*.sh") + glob("tools/*.xml")),
        (os.path.join("share", package_name, "tools", "hr_controller_v2"), glob("tools/hr_controller_v2/*.cpp") + glob("tools/hr_controller_v2/*.patch") + glob("tools/hr_controller_v2/*.py")),
        (os.path.join("share", package_name, "tools", "hr_controller_v2", "hr", "udp"), glob("tools/hr_controller_v2/hr/udp/*.hpp")),
    ],
    install_requires=["setuptools", "numpy", "pin", "pin-pink", "qpsolvers", "quadprog", "mujoco", "msgpack", "pyyaml", "xacro"],
    zip_safe=True,
    maintainer="Medical Robotics Lab, Sejong Univ.",
    maintainer_email="todo@sejong.ac.kr",
    description="Meta Quest 3 → pink/Pinocchio differential IK → dual-arm (14-DoF) humanoid upper-body teleoperation",
    license="MIT",
    entry_points={
        "console_scripts": [
            "quest_bridge = quest_arm_ik.quest_bridge:main",
            "dual_arm_ik = quest_arm_ik.dual_arm_ik_node:main",
            "quest_mock = quest_arm_ik.quest_mock:main",
            "mujoco_sim = quest_arm_ik.mujoco_sim:main",
            "xr_dump = quest_arm_ik.xr_dump:main",
            "diag_teleop = quest_arm_ik.diag_teleop:main",
            "hand_grasp = quest_arm_ik.hand_grasp_node:main",
            "upper_body_streamer = quest_arm_ik.upper_body_streamer_node:main",
            "motor_calib = quest_arm_ik.motor_calib:main",
            "hr_udp_relay = quest_arm_ik.hr_udp_relay_node:main",
            "diag_latency = quest_arm_ik.diag_latency:main",
            "motion_show = quest_arm_ik.motion_show:main",
        ],
    },
)
