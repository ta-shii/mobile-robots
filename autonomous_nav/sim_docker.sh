cd /home/tashi/Desktop/proj/mobile-robots/autonomous_nav

xhost +local:docker

docker run -it \
  --name part3_sim \
  --rm \
  --net=host \
  --ipc=host \
  -e DISPLAY=$DISPLAY \
  -e ROS_DOMAIN_ID=9 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd)/ros2_ws:/workspace/autonomous_nav/ros2_ws \
  g9_part3:latest \
  bash