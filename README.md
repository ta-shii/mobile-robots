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

## Part 2 

### 1. Build Docker Image
From the robot PC host:

```bash
cd ~/Desktop/p2/autonomous_nav
docker build -t pioneer-t9:latest .
```

### 2. Start Docker Container

Run:
``` bash
./run_docker.sh
```

### 3. Build ROS Packages Inside Container

Inside the container:

``` bash
cd /workspace/autonomous_nav
source /opt/ros/jazzy/setup.bash
colcon build --packages-select part2_nav ariaNode
source install/setup.bash
```

### 4. Run Full System Without Robot Motion

Use this first for safe testing:
``` bash
ros2 launch part2_nav full_system.launch.py enable_robot:=false
```

This starts:
- OAK-D camera
- GPS driver
- joystick driver
- gamepad controller
- waypoint navigator
- cone detector
- cone weaver
- object detector
- journey summary

The real robot driver is not started.

### 5. Run Full System With Robot Motion

Run this when ready for outdoor robot testing:
``` bash
ros2 launch part2_nav full_system.launch.py enable_robot:=true
```
This also starts: ariaNode which connects /cmd_vel to the real Pioneer robot.

### 6. Check Important Topics

Open another terminal and go inside the same contianer by:
``` bash
docker ps
docker exec -it {contianer_ID} bash
```

Then inside:
``` bash
cd /workspace/autonomous_nav
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

#### 6.1. Camera
``` bash
ros2 topic hz /oak/rgb/image_raw
```
Expected: average rate: about 30 Hz

#### 6.2. GPS
``` bash
ros2 topic echo /fix
```

Expected outdoors:
- latitude: real value
- longitude: real value

#### 6.3. Gamepad Mode
``` bash
ros2 topic echo /auto_mode
```

Controls:

- O = manual mode
- X = autonomous mode

#### 6.4. Velocity Command
``` bash
ros2 topic echo /cmd_vel
```

In manual mode:
- hold back trigger
- move joystick

Expected:
- linear.x changes
- angular.z changes

#### 6.5. Autonomous Command
```bash
ros2 topic echo /auto_cmd_vel
```

Expected when GPS/navigation is active:

- linear.x and/or angular.z values

#### 6.6. Cone Detection
``` bash
ros2 topic echo /cone_detected
ros2 topic echo /cone_position
```

Expected when cone is visible:
- cone_detected: true
- cone_position x/y/z values

#### 6.7. Output Files

Photos and summary files are saved to:

Inside container:
```
/workspace/autonomous_nav/outputs
```
On robot PC host:
```
~/Desktop/p2/autonomous_nav/outputs
```

#### 6.8. Manual Driving Test

Launch with robot enabled:
``` bash
ros2 launch part2_nav full_system.launch.py enable_robot:=true
```
Then:

1. Press O for manual mode
2. Hold back trigger as dead-man switch
3. Move joystick gently
4. Release trigger to stop

Expected:
- robot moves only while trigger is held
- robot stops immediately when trigger is released

#### 6.9. Autonomous Driving Test

1. Make sure GPS has valid coordinates
2. Press X for autonomous mode
3. Hold back trigger
4. Robot follows waypoint navigation commands

Expected:
- /auto_mode becomes true
- /auto_cmd_vel publishes navigation commands
- /cmd_vel follows auto command only while dead-man trigger is held

Paste this after your Part 2 section.

## Part 3
### Overview
Part 3 extends the robot system to support autonomous mapping, marker detection, obstacle detection, rapid waypoint navigation, live Foxglove monitoring, and rosbag recording.
The system supports two mapping modes:
- Manual mapping using the DS4 controller
- Autonomous mapping using m-explore frontier exploration

After mapping, the robot uses the detected Greek marker positions as waypoints and navigates to them in the shortest order before returning home.

---
### 1. Start Docker Container
On the robot PC host:
```bash
cd ~/Desktop/group9/mobile-robots/autonomous_nav
./run_docker.sh
```

2. Build Part 3 Package Inside Container

Inside the container:

```bash
cd /workspace/autonomous_nav
source /opt/ros/jazzy/setup.bash
colcon build --packages-select part3_explore --build-base ros2_ws/build --install-base install
source install/setup.bash
```

3. Clear Old Waypoints

Before a new run, clear previous waypoint data:
```bash
echo '[]' > /workspace/autonomous_nav/outputs/markers/waypoints.json
```

4. Launch Full Part 3 System
```bash 
ros2 launch part3_explore robot.launch.py
```

5. Connect Foxglove

On laptop:

Chrome → https://app.foxglove.dev

Connect to: `ws://LocalHost:8765`

Add an Image panel and select: `/display/image/compressed`

Foxglove should show:

* mission state
* SLAM map
* robot position
* detected markers
* waypoint progress
* Nav2 planned path
* e-stop status


6. Phase 1 — Mapping

Option A: Manual Mapping

Press: Triangle (△)

Expected state: MANUAL_MAPPING

Drive using:

L1 deadman + left stick = forward/back
L1 deadman + right stick = turn

Use this mode to manually cover the area while SLAM builds the map. To read the robot map position:
```bash
ros2 run tf2_ros tf2_echo map base_link
```
Read x and y from the Translation line.


Option B: Autonomous Mapping

Press: Cross (X)

Expected state: MAPPING

The robot explores using m-explore frontier exploration. It builds the map, searches for unexplored frontiers, sends goals to Nav2, and avoids obstacles using lidar costmaps.

Expected completion message: No frontiers found, stopping.


7. Manually Write Waypoints to drive to 3 waypoints
```bash
cd /workspace/autonomous_nav
./write_waypoints.sh
```

Follow the prompts:
* select Greek letter
* enter x coordinate
* enter y coordinate

Verify saved waypoints:
```bash
python3 -c "import json; print(json.load(open('/workspace/autonomous_nav/outputs/markers/waypoints.json')))"
```

8. Phase 2 — Rapid Waypoint Navigation

Press: Square (□)

Expected state: RAPID_NAV

The robot will:
* load marker waypoints from waypoints.json
* compute the shortest visit order
* send each waypoint to Nav2
* avoid obstacles using Nav2
* return to the starting/home position

Monitor in Foxglove:
* planned path
* robot pose
* waypoint progress
* return-to-home status

9. Emergency Stop and Recovery

If e-stop is triggered: Wait 3 seconds for auto-clear or press: Circle (O) to return to: IDLE

If the robot gets stuck: Press Circle (O) → return to IDLE
Then restart mapping or waypoint navigation

10. Check Output Files After Run

Waypoints:
```bash
cat /workspace/autonomous_nav/outputs/markers/waypoints.json
```
Obstacles:
```bash
cat /workspace/autonomous_nav/outputs/markers/obstacles.json
```
Detected photos:
```bash
ls /workspace/autonomous_nav/outputs/markers/photos/
```
Rosbag recordings:
```bash
ls /workspace/autonomous_nav/outputs/bags/
```
E-stop clips:
```bash
ls /workspace/autonomous_nav/outputs/bags/estop_clips/
```

11. Useful Debug Commands

Mission state:
```bash
ros2 topic echo /mission/state
```
E-stop status:
```bash
ros2 topic echo /estop/triggered
```
Velocity output to robot:
```bash
ros2 topic echo /cmd_vel
```
Nav2 velocity before safety filter:
```bash
ros2 topic echo /cmd_vel_nav
```
Detected markers:
```bash
ros2 topic echo /markers/detected
```
Waypoint index:
```bash
ros2 topic echo /waypoint_driver/index
```
Rosbag info:
```bash
ros2 bag info /workspace/autonomous_nav/outputs/bags/<bag_folder_name>
```