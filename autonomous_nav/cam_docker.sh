#!/bin/bash
set -e

IMAGE_NAME=g9_part3:latest
CONTAINER_NAME=g9_part3_container

docker rm -f $CONTAINER_NAME 2>/dev/null || true

docker run -it \
  --name $CONTAINER_NAME \
  --privileged \
  --net=host \
  --ipc=host \
  -v /dev:/dev \
  -v /run/udev:/run/udev \
  -v "$(pwd)/ros2_ws:/workspace/autonomous_nav/ros2_ws" \
  -v "$(pwd)/outputs:/workspace/autonomous_nav/outputs" \
  $IMAGE_NAME