#!/bin/bash
IMAGE_NAME=g9_part3:latest
CONTAINER_NAME=g9_part3_container

# Remove old container if it exists
docker rm -f $CONTAINER_NAME 2>/dev/null || true

docker run -it \
  --name $CONTAINER_NAME \
  --privileged \
  --net=host \
  --ipc=host \
  -v /dev:/dev \
  -v /run/udev:/run/udev \
  -v ~/Desktop/group9/mobile-robots/autonomous_nav:/workspace/autonomous_nav \
  $IMAGE_NAME bash
