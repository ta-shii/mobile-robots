docker run -it --rm \
  --network host \
  --privileged \
  -v ~/Desktop/group9/autonomous_nav:/workspace/autonomous_nav \
  g9_part3:latest bash