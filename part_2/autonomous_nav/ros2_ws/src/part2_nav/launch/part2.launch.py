"""
Part 2 — Robot launch file (all tasks, real hardware).

Nodes:
  joy_node           gamepad input
  gamepad_controller safety mux  (X=auto+deadman, O=manual)
  nmea_serial_driver real GPS
  waypoint_navigator Task 1 — GPS nav + weave_mode signal
  cone_detector      Task 2 — orange cone detection + photos
  cone_weaver        Task 3 — slalom WP1→WP2
  object_detector    Task 4 — coloured shape + OAK-D distance
  journey_summary    Task 5 — collects events, prints summary at end
  lidar_safety       Task 6 — SICK TiM781 collision avoidance

SICK LiDAR is launched separately (sick_scan_xd package):
  ros2 launch sick_scan_xd sick_tim_7xx.launch.py hostname:=192.168.0.1

Usage:
  ros2 launch part2_nav part2.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── arguments ──────────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('waypoints_file',
            default_value=os.path.join(
                os.path.dirname(__file__), '..', 'config', 'waypoints.yaml'),
            description='Path to waypoints YAML'),
        DeclareLaunchArgument('save_dir',
            default_value='/tmp/cone_photos',
            description='Directory for photos + summary'),
        DeclareLaunchArgument('gps_port',
            default_value='/dev/ttyACM0',
            description='GPS serial port'),
        DeclareLaunchArgument('scan_topic',
            default_value='/scan',
            description='LiDAR scan topic from sick_scan_xd'),
    ]

    wf   = LaunchConfiguration('waypoints_file')
    sd   = LaunchConfiguration('save_dir')
    gps  = LaunchConfiguration('gps_port')
    scan = LaunchConfiguration('scan_topic')

    nodes = [

        # ── Gamepad ────────────────────────────────────────────────────────
        Node(package='joy', executable='joy_node', name='joy_node',
             parameters=[{'autorepeat_rate': 20.0}]),

        Node(package='part2_nav', executable='gamepad_controller',
             name='gamepad_controller',
             parameters=[{'max_linear': 0.3, 'max_angular': 0.5}]),

        # ── GPS ────────────────────────────────────────────────────────────
        Node(package='nmea_navsat_driver', executable='nmea_serial_driver',
             name='gps_driver',
             parameters=[{'port': gps, 'baud': 9600, 'frame_id': 'gps_link'}]),

        # ── Task 1: GPS Waypoint Navigation ────────────────────────────────
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

        # ── Task 3: Slalom Weaving (WP1 → WP2 leg) ────────────────────────
        Node(package='part2_nav', executable='cone_weaver',
             name='cone_weaver',
             parameters=[{
                 'steer_gain':    0.8,
                 'max_angular':   0.5,
                 'min_cone_area': 500.0,
             }]),

        # ── Task 4: Coloured Shape Detection + Distance ────────────────────
        Node(package='part2_nav', executable='object_detector',
             name='object_detector',
             parameters=[{'save_dir': sd}]),

        # ── Task 5: Journey Summary ────────────────────────────────────────
        Node(package='part2_nav', executable='journey_summary',
             name='journey_summary',
             parameters=[{'save_dir': sd}]),

        # ── Task 6: LiDAR Collision Avoidance ─────────────────────────────
        Node(package='part2_nav', executable='lidar_safety',
             name='lidar_safety',
             parameters=[{
                 'scan_topic':      scan,
                 'forward_arc_deg': 60.0,
                 'danger_m':        0.40,
                 'warn_m':          0.80,
                 'caution_m':       1.50,
             }]),
    ]

    return LaunchDescription(args + nodes)
