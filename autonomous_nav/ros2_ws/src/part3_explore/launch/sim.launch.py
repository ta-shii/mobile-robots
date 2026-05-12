"""
sim.launch.py  –  Incremental simulation launch

Built up stage by stage to match the Week 1 test plan.
Each stage is tested and committed before the next is added.

Current stage: 1 — World + Robot + Bridges
  Starts Gazebo, spawns the Pioneer robot, and bridges all
  Gazebo topics into ROS2.  No navigation or custom nodes yet.

  What runs:
    1. Gazebo          – physics sim with part3_world.sdf
    2. robot_state_pub – TF tree from URDF (base_link → laser_frame etc.)
    3. joint_state_pub – wheel joint states for URDF
    4. robot_spawn     – drops Pioneer into Gazebo at (0, 0, 0.1)
    5. lidar_bridge    – gz /scan      → ROS /scan      (LaserScan)
    6. odom_bridge     – gz /odom      → ROS /odom      (Odometry)
    7. cmd_vel_bridge  – ROS /cmd_vel  → gz /cmd_vel    (Twist)
    8. camera_bridge   – gz /camera/image → ROS /image_raw (Image)

  What to test after launching:
    ros2 topic list                      # should show /scan /odom /image_raw
    ros2 topic echo /scan --once         # ranges array should have data
    ros2 topic echo /odom --once         # position should be ~(0,0,0)
    ros2 run tf2_tools view_frames       # odom→base_link→laser_frame chain

Stages to add next:
  Stage 2  + slam_toolbox
  Stage 3  + mission_manager + safety_monitor
  Stage 4  + Nav2
  Stage 5  + velocity_safety_filter
  Stage 6  + explorer + waypoint_driver
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_p3  = get_package_share_directory('part3_explore')
    pkg_p1  = get_package_share_directory('p3at')
    pkg_gz  = get_package_share_directory('ros_gz_sim')

    world_sdf   = os.path.join(pkg_p3, 'worlds', 'part3_world.sdf')
    robot_urdf  = os.path.join(pkg_p1, 'urdf',   'pioneer.urdf')

    with open(robot_urdf, 'r') as f:
        robot_description = f.read()

    # Let Gazebo find the p3at meshes (URDF references them by package path)
    pkg_share_parent = os.path.dirname(pkg_p1)
    gz_resource  = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',     pkg_share_parent)
    ign_resource = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH',  pkg_share_parent)

    # 1. Gazebo simulator
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': world_sdf}.items(),
    )

    # 2. Robot state publisher – broadcasts TF tree from URDF
    #    use_sim_time=True so timestamps align with Gazebo clock
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen',
    )

    # 3. Joint state publisher – needed for wheel joints in URDF
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # 4. Spawn Pioneer into Gazebo at arena centre
    robot_spawn = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-topic', 'robot_description',
            '-name',  'pioneer',
            '-x', '0.0', '-y', '0.0', '-z', '0.0',
        ],
        output='screen',
    )

    # 5. Bridges: Gazebo topics ↔ ROS2 topics
    #
    #    Format: ros_topic@ros_msg_type@gz_msg_type
    #    Single @ = bidirectional

    # Lidar: Gazebo publishes on /scan → ROS /scan
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        output='screen',
    )

    # Odometry: Gazebo DiffDrive publishes on /odom → ROS /odom
    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='odom_bridge',
        arguments=['/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
        output='screen',
    )

    # cmd_vel: ROS /cmd_vel → Gazebo (one direction only, ros→gz)
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        output='screen',
    )

    # Camera: Gazebo /camera/image → ROS /image_raw
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        arguments=[
            '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        ],
        remappings=[('/camera/image', '/image_raw')],
        output='screen',
    )

    return LaunchDescription([
        gz_resource,
        ign_resource,
        gazebo,
        rsp,
        jsp,
        robot_spawn,
        lidar_bridge,
        odom_bridge,
        cmd_vel_bridge,
        camera_bridge,
    ])
