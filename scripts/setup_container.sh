#!/usr/bin/env bash
# (대안) 컨테이너 안에서 실행: 이미지 레이어를 못 쓰는 경우, 컨테이너가 새로 만들어질 때마다 한 번 돌린다.
#   bash $ISAAC_ROS_WS/src/quest_arm_ik/scripts/setup_container.sh
# docker/Dockerfile.zz_quest_teleop 과 같은 내용: isaac_ros_teleop(apt) + 우리 의존성은 전용 venv /opt/quest_py.
# 시스템 사이트(NVIDIA 의 pin 2.7.0 / numpy 1.26)는 건드리지 않고, pip --user 도 쓰지 않는다 (pinocchio 두 벌 충돌 방지).
set -euo pipefail
sudo apt-get update
sudo apt-get install -y --no-install-recommends python3-pip python3-venv python3-dev iputils-ping iproute2 "ros-${ROS_DISTRO:-jazzy}-rosidl-default-generators" "ros-${ROS_DISTRO:-jazzy}-isaac-ros-teleop" \
     libgl1 libglx0 libglvnd0 libglx-mesa0 libgl1-mesa-dri libegl1

QPY_DIR=/opt/quest_py
[ -x "$QPY_DIR/bin/python" ] || sudo python3 -m venv "$QPY_DIR"
# cmeel-urdfdom<5 / cmeel-tinyxml2<11: pin 3.x 바이너리의 soname 유지 (urdfdom 6 + tinyxml2 혼합 → libtinyxml2.so.11 ImportError 방지)
sudo "$QPY_DIR/bin/python" -m pip install --no-cache-dir "setuptools" \
     "pin>=3.3" "pin-pink>=3.1" "cmeel-urdfdom<5" "cmeel-tinyxml2<11" \
     "qpsolvers>=4.3" "quadprog>=0.1.12" "mujoco>=3.2" "msgpack>=1.0" "pyyaml>=6.0" "xacro>=2.0" "numpy>=1.24"
"$QPY_DIR/bin/python" -c "import pinocchio, coal, pink, qpsolvers, quadprog, mujoco, msgpack, yaml, xacro, numpy; print('quest_arm_ik deps ok: pin', pinocchio.__version__, 'pink', pink.__version__, 'mujoco', mujoco.__version__, 'numpy', numpy.__version__)"
PYTHONNOUSERSITE=1 python3 -c "import isaacteleop.cloudxr, dex_retargeting.retargeting_config, pinocchio; print('isaac_ros_teleop deps ok (system pin', pinocchio.__version__ + ')')"
echo "완료. 이제  source \$ISAAC_ROS_WS/src/quest_arm_ik/scripts/env.sh"
