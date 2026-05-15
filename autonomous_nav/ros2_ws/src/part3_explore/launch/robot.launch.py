"""
robot.launch.py  –  Full stack for Part 3 (Task 1 mapping + Task 2 nav)

Nodes started
  1.  ariaNode              – Pioneer motor driver (/cmd_vel → wheels, /odom)
  2.  joy_node              – reads PS4 gamepad, publishes /joy
  3.  robot_state_publisher – TF tree from URDF (base_link → laser_frame)
  4.  lakibeam driver       – publishes /scan from Lakibeam lidar
  5.  slam_toolbox          – builds /map from /scan + /odom
  6.  Nav2 stack            – path planning + obstacle avoidance
                              (controller publishes to /cmd_vel_nav)
  7.  explore_lite          – frontier exploration (MAPPING state only)
  8.  mission_manager       – state machine: IDLE/MAPPING/MANUAL_MAPPING/RAPID_NAV
  9.  safety_monitor        – e-stop if moving object within 1 m
  10. velocity_safety_filter – sole /cmd_vel publisher (estop > manual L1 > Nav2)
  11. color_detector        – detects Greek-letter markers via camera

Operator workflow
  △ (Triangle) – IDLE → MANUAL_MAPPING  (drive with L1 + sticks, SLAM maps)
  X (Cross)    – IDLE → MAPPING         (m-explore drives autonomously)
  □ (Square)   – MAPPING|MANUAL → RAPID_NAV  (Phase 2 waypoint navigation)
  O (Circle)   – any  → IDLE            (abort / e-stop clear)
  L1 + sticks  – manual override at any time (in any non-IDLE state)

Usage
  ros2 launch part3_explore robot.launch.py
  ros2 launch part3_explore robot.launch.py lakibeam_ip:=192.168.198.2
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_p3   = get_package_share_directory('part3_explore')
    pkg_p1   = get_package_share_directory('p3at')
    pkg_slam = get_package_share_directory('slam_toolbox')

    slam_params    = os.path.join(pkg_p3, 'config', 'slam_params.yaml')
    nav2_params    = os.path.join(pkg_p3, 'config', 'nav2_params.yaml')
    explore_params = os.path.join(pkg_p3, 'config', 'explore_params.yaml')
    p3_params      = os.path.join(pkg_p3, 'config', 'part3_params.yaml')
    urdf_path      = os.path.join(pkg_p1, 'urdf', 'pioneer.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # ── Launch arguments ────────────────────────────────────────────────────
    declared_args = [
        DeclareLaunchArgument(
            'lakibeam_ip',
            default_value='192.168.198.2',
            description='IP address of the Lakibeam lidar sensor',
        ),
        DeclareLaunchArgument(
            'robot_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for Pioneer ariaNode',
        ),
    ]

    lakibeam_ip = LaunchConfiguration('lakibeam_ip')
    robot_port  = LaunchConfiguration('robot_port')

    # ── 1. Pioneer motor driver ─────────────────────────────────────────────
    aria_node = Node(
        package='ariaNode',
        executable='ariaNode',
        name='aria_node',
        output='screen',
        arguments=['--rp', robot_port],
    )

    # ── 2. PS4 gamepad ──────────────────────────────────────────────────────
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'autorepeat_rate': 20.0}],
    )

    # ── 3. TF tree from URDF ────────────────────────────────────────────────
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
    )

    # ── 4. Lakibeam lidar driver ────────────────────────────────────────────
    lakibeam_node = Node(
        package='lakibeam1',
        executable='lakibeam1_scan_node',
        name='lakibeam_node',
        output='screen',
        parameters=[{
            'sensorip':     lakibeam_ip,
            'hostip':       '192.168.198.50',
            'port':         '2368',
            'frame_id':     'laser_frame',
            'output_topic': 'scan',
        }],
        arguments=['--ros-args', '--log-level', 'warn'],
    )

    # ── 5. SLAM Toolbox ─────────────────────────────────────────────────────
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time':     'false',
            'slam_params_file': slam_params,
        }.items(),
    )

    # ── 6. Nav2 stack (individual nodes only – avoids Jazzy extras) ────────────
    # We start only the nodes we need: planner, controller, bt_navigator,
    # behavior_server.  collision_monitor, route_server, docking_server etc.
    # are intentionally excluded — they are not needed for exploration/nav.
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params],
        arguments=['--ros-args', '--log-level', 'warn'],
    )
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params],
        arguments=['--ros-args', '--log-level', 'warn'],
    )
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params],
        arguments=['--ros-args', '--log-level', 'warn'],
    )
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params],
        arguments=['--ros-args', '--log-level', 'warn'],
    )
    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart':    True,
            'node_names':   [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
            ],
        }],
    )

    # ── 7. m-explore (explore_lite) ─────────────────────────────────────────
    # Picks frontier goals and sends NavigateToPose to Nav2.
    # Controlled by mission_manager: cancels Nav2 goals on state change.
    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore',
        output='screen',
        parameters=[explore_params],
    )

    # ── 8, 9, 10, 11. Our Part 3 nodes ──────────────────────────────────────
    def p3_node(executable):
        return Node(
            package='part3_explore',
            executable=executable,
            name=executable,
            parameters=[p3_params],
            output='screen',
        )

    mission_manager        = p3_node('mission_manager')
    safety_monitor         = p3_node('safety_monitor')
    velocity_safety_filter = p3_node('velocity_safety_filter')

    return LaunchDescription(
        declared_args + [
            aria_node,
            joy_node,
            rsp_node,
            lakibeam_node,
            slam_launch,
            controller_server,
            planner_server,
            behavior_server,
            bt_navigator,
            nav2_lifecycle_manager,
            explore_node,
            mission_manager,
            safety_monitor,
            velocity_safety_filter,
        ]
    )
