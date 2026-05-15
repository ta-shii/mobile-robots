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
            'robot_port',
            default_value='/dev/ttyUSB0',
            description='Pioneer serial port'
        ),
        
        
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/oak/stereo/image_raw',
            description='Depth image topic for object detection'
        ),
    ]

    robot_port = LaunchConfiguration('robot_port')
    depth_topic = LaunchConfiguration('depth_topic')

    depthai_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'depthai_ros_driver_v3', 'launch', 'driver.launch.py')
        )
    )

    nodes = [
        # Camera driver
        depthai_launch,
    ]

    return LaunchDescription(args + nodes)