# 컨테이너 안 새 터미널마다:  source /workspaces/isaac_ros-dev/src/quest_arm_ik/scripts/env.sh
# (호스트 ~/.bashrc 가 컨테이너에 읽기 전용으로 마운트되어 호스트용 ISAAC_ROS_WS=/home/<user>/... 가 컨테이너 값(/workspaces/isaac_ros-dev)을
#  덮어쓸 수 있다. 그래서 $ISAAC_ROS_WS 대신 절대 경로로 source 하고, 여기서 컨테이너 안 실제 경로를 다시 잡아 준다.)
if [ -d /workspaces/isaac_ros-dev/src/quest_arm_ik ]; then
  WS=/workspaces/isaac_ros-dev                      # 컨테이너 안
else
  WS="${ISAAC_ROS_WS:-$HOME/workspaces/isaac_ros-dev}"   # 호스트에서 소싱한 경우
fi
export ISAAC_ROS_WS="$WS"
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
export G1="$WS/src/mujoco_menagerie/unitree_g1/g1.xml"
export G1H="$WS/src/mujoco_menagerie/unitree_g1/g1_with_hands.xml"   # Dex3-1 손 포함 G1 (model_path:=$G1H 면 트리거로 손 쥐기까지 시뮬)
export SH="$WS/src/SH-Humanoid"
# quest_arm_ik 전용 파이썬(venv: pin 3.x·pink·mujoco·numpy 2). 시스템 python 은 NVIDIA isaac_ros_teleop 용(pin 2.7.0·numpy 1.26)이라 분리.
# teleop.launch.py 는 QPY 가 있으면 우리 노드를 이 인터프리터로 실행한다. 오프라인 테스트·check_model 도 $QPY 로 실행.
if [ -x /opt/quest_py/bin/python3 ]; then export QPY=/opt/quest_py/bin/python3; else export QPY=python3; fi
# NVIDIA 노드(시스템 python)가 과거 ~/.local 설치본을 보지 못하게 차단. pip --user 금지.
export PYTHONNOUSERSITE=1
# 실기체(삼현 미니PC)와 DDS 통신: 양쪽 같은 값이어야 함. 미니PC 설정에 맞춰 바꾼다.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
# 실기체 링크 방식 (2026-10-21): udp = hr_udp_relay(Thor) ↔ hr_udp_bridge.py(미니PC). DDS 는 Thor 안에서만 쓰도록 발견 범위를 LOCALHOST 로 —
# 링크 위의 Fast DDS 발행 블로킹(2 s 멈춤)을 피하고, 미니PC 에 삼현 DDS 브리지가 떠 있어도 명령이 두 갈래로 가지 않게. dds 로 되돌리려면 HR_LINK=dds.
export HR_LINK="${HR_LINK:-udp}"
if [ "$HR_LINK" = "udp" ]; then export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"; fi
echo "quest_arm_ik env: WS=$WS QPY=$QPY ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW=$RMW_IMPLEMENTATION HR_LINK=$HR_LINK DISCOVERY=${ROS_AUTOMATIC_DISCOVERY_RANGE:-default}"
