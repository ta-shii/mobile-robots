from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            name='gps_driver',
            output='screen',
            parameters=[{
                'port': '/dev/ttyACM0',
                'baud': 9600,
                'frame_id': 'gps_link'
            }]
        ),

        Node(
            package='part2_nav',
            executable='waypoint_navigator',
            name='waypoint_navigator',
            output='screen',
            parameters=[
                '/workspace/autonomous_nav/part2_nav/config/waypoint_nav_params.yaml'
            ]
        ),

        Node(
            package='part2_nav',
            executable='gamepad_controller',
            name='gamepad_controller',
            output='screen',
            parameters=[{
                'max_linear': 0.3,
                'max_angular': 0.5
            }]
        ),
    ])