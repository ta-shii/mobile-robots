"""
test_marker_detector.launch.py  –  Minimal launch for testing marker_detector alone.

Starts only:
  1. robot_state_publisher  – TF tree (base_link → laser_frame)
  2. Static TF publishers   – map→odom→base_link (so position lookups return 0,0
                               instead of failing; good enough for bench testing)
  3. Lakibeam lidar         – publishes /scan  (needed for obstacle ranging)
  4. OAK-D camera           – publishes /oak/rgb/image_raw
  5. /mission/state latched – publishes "MAPPING" so the detector runs immediately
  6. marker_detector        – the node under test

Usage:
  ros2 launch part3_explore test_marker_detector.launch.py
  ros2 launch part3_explore test_marker_detector.launch.py lakibeam_ip:=192.168.198.2

Watch output:
  ros2 topic echo /markers/detected
  ros2 topic echo /markers/all
  ls /workspace/autonomous_nav/outputs/markers/photos/
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

_DEPTHAI_SHARE = '/opt/ros/jazzy/share/depthai_ros_driver_v3'


def generate_launch_description():

    pkg_p3  = get_package_share_directory('part3_explore')
    pkg_p1  = get_package_share_directory('p3at')
    p3_params = os.path.join(pkg_p3, 'config', 'part3_params.yaml')
    urdf_path = os.path.join(pkg_p1, 'urdf', 'pioneer.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    declared_args = [
        DeclareLaunchArgument(
            'lakibeam_ip',
            default_value='192.168.198.2',
            description='IP address of the Lakibeam lidar sensor',
        ),
    ]

    lakibeam_ip = LaunchConfiguration('lakibeam_ip')

    # ── 1. TF tree from URDF ────────────────────────────────────────────────
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

    # ── 2. Static TF: map → odom → base_link ───────────────────────────────
    # Lets _map_position() and _obstacle_map_position() resolve without error.
    # Robot is fixed at the origin — position in saved JSON will be (0, 0).
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_odom_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
    )

    # ── 3. Lakibeam lidar ───────────────────────────────────────────────────
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

    # ── 4. OAK-D camera ─────────────────────────────────────────────────────
    camera_node = Node(
        package='depthai_ros_driver_v3',
        executable='driver_node',
        name='oak',
        output='screen',
        parameters=[os.path.join(_DEPTHAI_SHARE, 'config', 'driver.yaml')],
        arguments=['--ros-args', '--log-level', 'fatal'],
    )

    # ── 5. Publish MAPPING state so marker_detector runs immediately ────────
    state_pub = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--rate', '1',
            '/mission/state', 'std_msgs/msg/String',
            '{data: MAPPING}',
        ],
        output='screen',
    )

    # ── 6. marker_detector ──────────────────────────────────────────────────
    marker_detector = Node(
        package='part3_explore',
        executable='marker_detector',
        name='marker_detector',
        parameters=[
            p3_params,
            {'output_dir': '/workspace/autonomous_nav/outputs'},
        ],
        output='screen',
    )

    return LaunchDescription(
        declared_args + [
            rsp_node,
            map_to_odom,
            odom_to_base,
            lakibeam_node,
            camera_node,
            state_pub,
            marker_detector,
        ]
    )
