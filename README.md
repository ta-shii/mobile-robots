# Mobile Robot (Pioneer 3-AT) – Autonomous Navigation

## Part 1

### Overview

In Part 1 of this project, we develop and simulate a mobile robot system using ROS 2 + Gazebo . The goal is to build a complete pipeline where a robot can:
- Spawn in a custom simulation world
- Perceive its environment using sensors (LiDAR, camera, IMU)
- Build a map using SLAM
- Navigate through predefined waypoints while avoiding obstacles

### What is implemented

**1. Simulation Environment**
- Custom Gazebo world (James Oval) with:
- Ground plane
- Track area
- Static obstacles

**2. Robot Model**
- Pioneer mobile robot (URDF)
- Differential drive system
	- Sensors:
	- LiDAR (/scan)
	- Camera (/camera/image)
	- IMU (/imu)

**3. ROS–Gazebo Integration**
- ros_gz_bridge used to connect simulation and ROS topics:
	- /cmd_vel
	- /odom
	- /scan
	- /imu
	- /camera

**4. TF & State Publishing**
- robot_state_publisher
- joint_state_publisher
- Custom odom → base_link broadcaster

**5. SLAM Mapping**
- Implemented using:
	- slam_toolbox
	- Builds a live map using LiDAR data

**6. Autonomous Navigation**
- Custom node:
	- obstacle_aware_waypoint_controller
- Robot:
	- Follows predefined waypoints
	- Avoids obstacles using LiDAR

**7. Visualization**
- RViz used for:
	- Map visualization
	- Robot pose
	- Sensor data


### How to Run

***1. Build the workspace***

``` bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

***2. Launch the full simulation***
``` bash
ros2 launch p3at pioneer_sim.launch.py
```


***3. Optional launch arguments***

You can enable/disable components:
``` bash
ros2 launch p3at pioneer_sim.launch.py \
    use_slam:=true \
    use_controller:=true \
    use_recorder:=false \
    use_rviz:=true \
    use_steering:=false
```
### What you should see

- Gazebo opens with the custom world
- Robot spawns in the environment
- SLAM starts building a map
- Robot autonomously follows waypoints
- RViz shows live map and robot movement

	