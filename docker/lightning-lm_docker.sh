#!/bin/bash

set -euo pipefail

CONTAINER_NAME=${CONTAINER_NAME:-lightning-lm-dev}
IMAGE_NAME=${IMAGE_NAME:-docker.cnb.cool/gpf2025/slam:demo}

# 用户未指定命令时，默认进入交互式 /bin/bash
if [ $# -gt 0 ]; then
  CMD=("$@")
else
  CMD=(/bin/bash)
fi

if docker ps --filter "name=^/${CONTAINER_NAME}$" --format '{{.ID}}' | grep -q .; then
  echo "容器 ${CONTAINER_NAME} 已在运行，直接 exec..."
  docker exec -it "${CONTAINER_NAME}" "${CMD[@]}"
  exit 0
fi

docker run -it --rm \
  --name "${CONTAINER_NAME}" \
  --gpus all \
  --net=host \
  -e DISPLAY \
  --workdir="/home/$USER" \
  --volume="/home/$USER:/home/$USER" \
  -e XAUTHORITY=/tmp/.Xauthority \
  -v "${XAUTHORITY}:/tmp/.Xauthority" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  "${IMAGE_NAME}" \
  "${CMD[@]}"
