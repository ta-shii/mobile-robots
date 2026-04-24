"""
Part 2 — Laptop / simulation launch file (no robot hardware).

Identical to part2.launch.py EXCEPT:
  - No nmea_serial_driver  (GPS comes from mock_gps.py)
  - No lidar_safety        (no SICK LiDAR on laptop — mock separately if needed)
  - Everything else runs identically

Usage:
  Terminal 1:  ros2 launch part2_nav part2_laptop.launch.py
  Terminal 2:  python3 part2_nav/scripts/mock_gps.py
  Terminal 3:  ros2 topic pub /auto_mode std_msgs/msg/Bool "data: true" -r 1
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    args = [
        DeclareLaunchArgument('waypoints_file',
            default_value=os.path.join(
                os.path.dirname(__file__), '..', 'config', 'waypoints.yaml')),
        DeclareLaunchArgument('save_dir',
            default_value='/tmp/cone_photos'),
    ]

    wf = LaunchConfiguration('waypoints_file')
    sd = LaunchConfiguration('save_dir')

    nodes = [

        # ── Gamepad (optional — skip if no controller) ─────────────────────
        Node(package='joy', executable='joy_node', name='joy_node',
             parameters=[{'autorepeat_rate': 20.0}]),

        Node(package='part2_nav', executable='gamepad_controller',
             name='gamepad_controller',
             parameters=[{'max_linear': 0.3, 'max_angular': 0.5}]),

        # ── Task 1: Waypoint Navigator (GPS from mock_gps.py) ──────────────
        Node(package='part2_nav', executable='waypoint_navigator',
             name='waypoint_navigator',
             parameters=[{
                 'waypoints_file': wf,
                 'arrival_radius': 5.0,   # rubric: visit within 5 m
                 'max_linear':     0.3,
                 'max_angular':    0.5,
                 'heading_gain':   1.2,
             }]),

        # ── Task 2: Orange Cone Detection ──────────────────────────────────
        Node(package='part2_nav', executable='cone_detector',
             name='cone_detector',
             parameters=[{'save_dir': sd, 'min_area': 800, 'right_bias': 0.05}]),

        # ── Task 3: Slalom Weaving ─────────────────────────────────────────
        Node(package='part2_nav', executable='cone_weaver',
             name='cone_weaver',
             parameters=[{
                 'steer_gain':    0.8,
                 'max_angular':   0.5,
                 'min_cone_area': 500.0,
             }]),

        # ── Task 4: Coloured Object Detection ─────────────────────────────
        Node(package='part2_nav', executable='object_detector',
             name='object_detector',
             parameters=[{'save_dir': sd}]),

        # ── Task 5: Journey Summary ────────────────────────────────────────
        Node(package='part2_nav', executable='journey_summary',
             name='journey_summary',
             parameters=[{'save_dir': sd}]),

        # Note: lidar_safety not started on laptop (no SICK hardware)
        # To test it: ros2 topic pub /scan sensor_msgs/msg/LaserScan ...
    ]

    return LaunchDescription(args + nodes)
