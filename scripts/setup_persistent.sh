#!/usr/bin/env bash
# Thor 호스트(컨테이너 밖)에서 1회 실행: Isaac ROS 개발 이미지에 quest_arm_ik + isaac_ros_teleop 의존성 레이어를 등록한다.
#   bash $ISAAC_ROS_WS/src/quest_arm_ik/scripts/setup_persistent.sh
#   isaac-ros activate --build-local        # 레이어 빌드 (최초 1회) 후 컨테이너 진입. 이후엔 isaac-ros activate 만.
set -euo pipefail
WS="${ISAAC_ROS_WS:-$HOME/workspaces/isaac_ros-dev}"
PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KEY=zz_quest_teleop

mkdir -p "$WS/docker" "$WS/scripts" "$WS/.isaac-ros-cli"
cp "$PKG/docker/Dockerfile.$KEY" "$WS/docker/Dockerfile.$KEY"

# 1) Dockerfile 검색 경로: 우리 디렉터리 + CLI 기본 디렉터리 (기본 레이어들을 계속 찾을 수 있게)
CFG="$WS/scripts/.isaac_ros_common-config"
if ! grep -qs "CONFIG_DOCKER_SEARCH_DIRS" "$CFG"; then
  echo 'CONFIG_DOCKER_SEARCH_DIRS=(${ISAAC_ROS_WS}/docker /etc/isaac-ros-cli/docker)' >> "$CFG"
fi

# 2) 이미지 키 등록 (워크스페이스 설정; 기존 파일이 있으면 키만 추가)
YAML="$WS/.isaac-ros-cli/config.yaml"
if [ ! -s "$YAML" ]; then
  printf 'docker:\n  image:\n    additional_image_keys:\n      - %s\n' "$KEY" > "$YAML"
elif ! grep -q "$KEY" "$YAML"; then
  python3 - "$YAML" "$KEY" <<'PY'
import sys, yaml
path, key = sys.argv[1], sys.argv[2]
cfg = yaml.safe_load(open(path)) or {}
keys = cfg.setdefault("docker", {}).setdefault("image", {}).setdefault("additional_image_keys", []) or []
if key not in keys:
    keys.append(key)
cfg["docker"]["image"]["additional_image_keys"] = keys
yaml.safe_dump(cfg, open(path, "w"), sort_keys=False)
PY
fi

echo "등록 완료:"
echo "  $WS/docker/Dockerfile.$KEY"
echo "  $CFG"
echo "  $YAML"
echo
echo "다음: 실행 중인 컨테이너가 있으면 모두 exit 한 뒤, 호스트에서"
echo "  isaac-ros activate --build-local      # 레이어 빌드 (apt+pip, 수 분~수십 분). 끝나면 컨테이너로 들어간다"
echo "이후 날마다는  isaac-ros activate  만 실행 (빌드된 이미지 재사용)."
