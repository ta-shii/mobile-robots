"""
Part 2 launch file — starts everything needed for Tasks 1 & 2.

Nodes launched:
  joy_node           — reads DualShock IV bluetooth gamepad
  gamepad_controller — mode mux (X=auto+deadman, O=manual) → cmd_vel
  waypoint_navigator — GPS nav → auto_cmd_vel
  cone_detector      — OAK-D orange cone detection + photography
  nmea_driver        — GPS serial driver (adjust port as needed)
  ariaNode           — Pioneer motor control (separate profile in docker-compose)

Usage:
  ros2 launch part2_nav part2.launch.py waypoints_file:=<abs_path>
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    waypoints_file_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value=os.path.join(
            os.path.dirname(__file__), '..', 'config', 'waypoints.yaml'
        ),
        description='Absolute path to waypoints YAML file',
    )
    save_dir_arg = DeclareLaunchArgument(
        'save_dir',
        default_value='/tmp/cone_photos',
        description='Directory to save cone photographs',
    )
    gps_port_arg = DeclareLaunchArgument(
        'gps_port',
        default_value='/dev/ttyACM0',
        description='Serial port for NMEA GPS',
    )
    joy_dev_arg = DeclareLaunchArgument(
        'joy_device',
        default_value='/dev/input/js0',
        description='Joystick device path',
    )

    waypoints_file = LaunchConfiguration('waypoints_file')
    save_dir = LaunchConfiguration('save_dir')
    gps_port = LaunchConfiguration('gps_port')
    joy_device = LaunchConfiguration('joy_device')

    return LaunchDescription([
        waypoints_file_arg,
        save_dir_arg,
        gps_port_arg,
        joy_dev_arg,

        # ── Gamepad ──────────────────────────────────────────────────────
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device_name': joy_device,
                'autorepeat_rate': 20.0,
            }],
        ),

        Node(
            package='part2_nav',
            executable='gamepad_controller',
            name='gamepad_controller',
            parameters=[{
                'max_linear': 0.3,
                'max_angular': 0.5,
            }],
        ),

        # ── GPS driver ───────────────────────────────────────────────────
        Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            name='gps_driver',
            parameters=[{
                'port': gps_port,
                'baud': 9600,
                'frame_id': 'gps_link',
            }],
        ),

        # ── Navigation (Task 1) ──────────────────────────────────────────
        Node(
            package='part2_nav',
            executable='waypoint_navigator',
            name='waypoint_navigator',
            parameters=[{
                'waypoints_file': waypoints_file,
                'arrival_radius': 1.5,
                'max_linear': 0.3,
                'max_angular': 0.5,
                'heading_gain': 1.2,
            }],
        ),

        # ── Cone detection (Task 2) ───────────────────────────────────────
        Node(
            package='part2_nav',
            executable='cone_detector',
            name='cone_detector',
            parameters=[{
                'save_dir': save_dir,
                'min_area': 800,
                'right_bias': 0.05,
            }],
        ),
    ])
