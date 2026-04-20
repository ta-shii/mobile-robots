"""
Laptop / simulation launch file — no GPS hardware, no Pioneer connected.

Differences from part2.launch.py:
  - No nmea_serial_driver (GPS comes from mock_gps.py instead)
  - joy_node still launched (use a USB/Bluetooth gamepad if you have one)
  - waypoint_navigator and cone_detector run identically to the robot version

Usage:
  ros2 launch part2_nav part2_laptop.launch.py

Then in a separate terminal (or the mock_gps Docker service):
  python3 part2_nav/scripts/mock_gps.py

If you have no gamepad, enable auto mode manually:
  ros2 topic pub /auto_mode std_msgs/msg/Bool "data: true" --once
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    waypoints_file_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value=os.path.join(
            os.path.dirname(__file__), '..', 'config', 'waypoints.yaml'
        ),
    )
    save_dir_arg = DeclareLaunchArgument(
        'save_dir',
        default_value='/tmp/cone_photos',
    )

    return LaunchDescription([
        waypoints_file_arg,
        save_dir_arg,

        # ── Gamepad (optional — skip if no controller connected) ──────────
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'autorepeat_rate': 20.0}],
        ),
        Node(
            package='part2_nav',
            executable='gamepad_controller',
            name='gamepad_controller',
            parameters=[{'max_linear': 0.3, 'max_angular': 0.5}],
        ),

        # ── Navigation (Task 1) — GPS comes from mock_gps.py ─────────────
        Node(
            package='part2_nav',
            executable='waypoint_navigator',
            name='waypoint_navigator',
            parameters=[{
                'waypoints_file': LaunchConfiguration('waypoints_file'),
                'arrival_radius': 1.5,
                'max_linear': 0.3,
                'max_angular': 0.5,
                'heading_gain': 1.2,
            }],
        ),

        # ── Cone detection (Task 2) ───────────────────────────────────────
        # On laptop: point your webcam at an orange object to test detection.
        # Camera topic remapped from /dev/video0 → /oak/rgb/image_raw via
        # the webcam bridge described in the README testing section.
        Node(
            package='part2_nav',
            executable='cone_detector',
            name='cone_detector',
            parameters=[{
                'save_dir': LaunchConfiguration('save_dir'),
                'min_area': 800,
                'right_bias': 0.05,
            }],
        ),
    ])
