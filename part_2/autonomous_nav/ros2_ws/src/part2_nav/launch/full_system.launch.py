import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = '/opt/ros/jazzy/share'

    args = [
        DeclareLaunchArgument(
            'enable_robot',
            default_value='false',
            description='Start ariaNode so cmd_vel reaches the real Pioneer'
        ),
        DeclareLaunchArgument(
            'gps_port',
            default_value='/dev/ttyACM0',
            description='GPS serial port'
        ),
        DeclareLaunchArgument(
            'robot_port',
            default_value='/dev/ttyUSB0',
            description='Pioneer serial port'
        ),
        DeclareLaunchArgument(
            'save_dir',
            default_value='/workspace/autonomous_nav/outputs',
            description='Directory for saved photos and summary files'
        ),
        DeclareLaunchArgument(
            'waypoints_file',
            default_value='/workspace/autonomous_nav/part2_nav/config/waypoints.yaml',
            description='Waypoint YAML file'
        ),
        DeclareLaunchArgument(
            'rgb_topic',
            default_value='/oak/rgb/image_raw',
            description='RGB image topic for cone/object detection'
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/oak/stereo/image_raw',
            description='Depth image topic for object detection'
        ),
    ]

    enable_robot = LaunchConfiguration('enable_robot')
    gps_port = LaunchConfiguration('gps_port')
    robot_port = LaunchConfiguration('robot_port')
    save_dir = LaunchConfiguration('save_dir')
    waypoints_file = LaunchConfiguration('waypoints_file')
    rgb_topic = LaunchConfiguration('rgb_topic')
    depth_topic = LaunchConfiguration('depth_topic')

    depthai_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'depthai_ros_driver_v3', 'launch', 'driver.launch.py')
        )
    )

    nodes = [
        # Camera driver
        depthai_launch,

        # Joystick
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'autorepeat_rate': 20.0}]
        ),

        # Gamepad controller
        Node(
            package='part2_nav',
            executable='gamepad_controller',
            name='gamepad_controller',
            output='screen',
            parameters=[{
                'max_linear': 0.3,
                'max_angular': 0.5,
            }]
        ),

        # GPS
        Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            name='gps_driver',
            output='screen',
            parameters=[{
                'port': gps_port,
                'baud': 9600,
                'frame_id': 'gps_link'
            }]
        ),

        # Waypoint navigation
        Node(
            package='part2_nav',
            executable='waypoint_navigator',
            name='waypoint_navigator',
            output='screen',
            parameters=[{
                'waypoints_file': waypoints_file,
                'arrival_radius': 5.0,
                'max_linear': 0.3,
                'max_angular': 0.5,
                'heading_gain': 1.2,
            }]
        ),

        # Cone detector
        Node(
            package='part2_nav',
            executable='cone_detector',
            name='cone_detector',
            output='screen',
            parameters=[{
                'save_dir': save_dir,
                'min_area': 1200,
                'right_bias': 0.05,
                'image_topic': rgb_topic,
            }]
        ),

        # Cone weaving
        Node(
            package='part2_nav',
            executable='cone_weaver',
            name='cone_weaver',
            output='screen',
            parameters=[{
                'steer_gain': 0.8,
                'max_angular': 0.5,
                'min_cone_area': 500.0,
            }]
        ),

        # Object detector
        Node(
            package='part2_nav',
            executable='object_detector',
            name='object_detector',
            output='screen',
            parameters=[{
                'save_dir': save_dir,
                'rgb_topic': rgb_topic,
                'depth_topic': depth_topic,
            }]
        ),

        # Journey summary
        Node(
            package='part2_nav',
            executable='journey_summary',
            name='journey_summary',
            output='screen',
            parameters=[{
                'save_dir': save_dir,
            }]
        ),

        # Real robot driver (optional)
        Node(
            package='ariaNode',
            executable='ariaNode',
            name='aria_node',
            output='screen',
            arguments=['--rp', robot_port],
            condition=IfCondition(enable_robot)
        ),
    ]

    return LaunchDescription(args + nodes)