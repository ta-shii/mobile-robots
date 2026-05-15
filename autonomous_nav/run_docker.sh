docker run -it --rm \
  --network host \
  --privileged \
  -v ~/Desktop/group9/mobile-robots/autonomous_nav:/workspace/autonomous_nav \
    -v /dev/input:/dev/input \
  g9_part3:latest bash