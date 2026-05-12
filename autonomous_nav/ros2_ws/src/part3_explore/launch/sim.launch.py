"""
sim.launch.py  –  Incremental simulation launch

Built up stage by stage to match the Week 1 test plan.
Each stage is tested and committed before the next is added.

Current stage: 3 — + mission_manager + safety_monitor
  Everything from Stage 2 plus:
   12.  mission_manager  – state machine IDLE→MAPPING→RAPID_NAV
                           publishes /mission/state at 5 Hz
                           robot stays still until MAPPING is triggered
   13.  safety_monitor   – stops robot if obstacle within 0.5 m
                           publishes /estop/triggered + zeroes /cmd_vel

  Stage 1 recap (infrastructure):
    1. Gazebo          – physics sim with part3_world.sdf
    2. robot_state_pub – TF tree from URDF
    3. joint_state_pub – wheel joint states
    4. robot_spawn     – Pioneer at (0, 0, 0)
    5. lidar_bridge    – gz /scan          → ROS /scan
    6. odom_bridge     – gz /odom          → ROS /odom
    7. cmd_vel_bridge  – ROS /cmd_vel      → gz /cmd_vel
    8. camera_bridge   – gz /camera/image  → ROS /image_raw
       model_tf_bridge – gz /model/pioneer/tf → ROS /tf
       base_link_tf    – static: pioneer/base_link → base_link
       laser_tf        – static: pioneer/base_link/laser (offset 0.2/0/0.104)

  Stage 2 recap (SLAM + coverage):
    9.  slam_toolbox   – builds /map from /scan + /odom
   10.  wander         – lawnmower coverage (gated by /mission/state)
   11.  rviz2          – Map + LaserScan + RobotModel

  What to test after launching:
    ros2 topic echo /mission/state --once   # should print IDLE, robot stationary
    ros2 service call /start_mapping_phase std_srvs/srv/Trigger {}
    ros2 topic echo /mission/state --once   # should print MAPPING, robot starts
    ros2 topic echo /estop/incident_log     # watch for safety events near walls

Stages to add next:
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
    pkg_p3   = get_package_share_directory('part3_explore')
    pkg_p1   = get_package_share_directory('p3at')
    pkg_gz   = get_package_share_directory('ros_gz_sim')
    pkg_slam = get_package_share_directory('slam_toolbox')

    world_sdf   = os.path.join(pkg_p3, 'worlds', 'part3_world.sdf')
    robot_urdf  = os.path.join(pkg_p1, 'urdf',   'pioneer.urdf')
    slam_params = os.path.join(pkg_p3, 'config', 'slam_params_sim.yaml')
    p3_params   = os.path.join(pkg_p3, 'config', 'part3_params.yaml')

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
    #    z=0.1 so robot sits on ground instead of clipping through it
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

    # Bridge Gazebo's model TF into ROS
    # The DiffDrive plugin publishes odom → pioneer/base_link on the
    # model-specific topic /model/pioneer/tf, NOT on the global /tf.
    # [ means gz→ros only (one direction).
    # Remapped to /tf so it enters the ROS TF tree normally.
    model_tf_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='model_tf_bridge',
        arguments=['/model/pioneer/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
        remappings=[('/model/pioneer/tf', '/tf')],
        output='screen',
    )

    # Gazebo names frames as  model_name/link_name  →  pioneer/base_link
    # Our URDF and all ROS nodes expect just  base_link.
    # This static identity transform bridges the two names so the TF chain
    # odom → pioneer/base_link → base_link → chassis → laser_frame works.
    base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_tf',
        arguments=['0', '0', '0', '0', '0', '0',
                   'pioneer/base_link', 'base_link'],
        output='screen',
    )

    # /scan arrives with frame_id = pioneer/base_link/laser (Gazebo naming).
    # SLAM looks up base_link → pioneer/base_link/laser in TF to place each
    # scan hit in the correct spot on the map.
    # We publish pioneer/base_link/laser as a child of pioneer/base_link at
    # the same offset the URDF puts laser_frame under chassis (0.2m fwd, 0.104m up).
    # This puts pioneer/base_link/laser in the connected tree:
    #   odom → pioneer/base_link → pioneer/base_link/laser   ✓
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_tf',
        arguments=['0.2', '0', '0.104', '0', '0', '0',
                   'pioneer/base_link', 'pioneer/base_link/laser'],
        output='screen',
    )

    # Stage 2: SLAM + manual teleop

    # 9. slam_toolbox – builds /map from /scan + /odom
    #    online_async = processes scans as they arrive (no batch post-processing)
    #    slam_params_sim.yaml has use_sim_time=true and base_frame=base_link
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time':     'true',
            'slam_params_file': slam_params,
        }.items(),
    )

    # 10. Wander – reactive obstacle-avoidance driver for map building
    #     Drives straight until something is within SLOW_DIST, then steers away.
    #     Replaces gamepad teleop in sim (no PS4 controller available).
    #     Will be replaced by Nav2 + explorer in Stage 4/6.
    wander_node = Node(
        package='part3_explore',
        executable='wander',
        name='wander',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # 11. RViz2 – visualiser with pre-configured layout
    #     Shows: map (occupancy grid), laser scan, robot model
    #     Fixed frame = map so everything is in the SLAM map frame
    rviz_config = os.path.join(pkg_p3, 'config', 'sim_rviz.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ── Stage 3: mission_manager + safety_monitor ──────────────────────────

    # 12. mission_manager — state machine, starts in IDLE.
    #     Wander will NOT drive until operator triggers MAPPING via service call:
    #       ros2 service call /start_mapping_phase std_srvs/srv/Trigger {}
    #     Publishes /mission/state at 5 Hz → consumed by wander + safety_monitor.
    mission_manager = Node(
        package='part3_explore',
        executable='mission_manager',
        name='mission_manager',
        parameters=[p3_params, {'use_sim_time': False}],
        output='screen',
    )

    # 13. safety_monitor — watches /scan every callback.
    #     Active only when mission != IDLE.
    #     When any beam < DANGER_RADIUS (0.5 m):
    #       • publishes True on /estop/triggered → mission_manager logs it
    #       • publishes Twist(0) on /cmd_vel at 20 Hz → overrides wander motion
    #     NOTE: cmd_vel arbitration (safety vs wander) is properly resolved in
    #     Stage 5 by velocity_safety_filter. For now safety_monitor wins by rate.
    safety_monitor = Node(
        package='part3_explore',
        executable='safety_monitor',
        name='safety_monitor',
        parameters=[p3_params, {'use_sim_time': False}],
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
        model_tf_bridge,
        base_link_tf,
        laser_tf,
        # Stage 2
        slam_launch,
        wander_node,
        rviz_node,
        # Stage 3
        mission_manager,
        safety_monitor,
    ])
