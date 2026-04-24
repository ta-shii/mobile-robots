#!/bin/bash
set -e

IMAGE_NAME=pioneer-t9
CONTAINER_NAME=pioneer_container

docker rm -f $CONTAINER_NAME 2>/dev/null || true

docker run -it \
  --name $CONTAINER_NAME \
  --privileged \
  --net=host \
  --ipc=host \
  -v /dev:/dev \
  -v /run/udev:/run/udev \
  -v "$(pwd)/ros2_ws:/workspace/ros2_ws" \
  -v "$(pwd)/outputs:/workspace/outputs" \
  $IMAGE_NAME