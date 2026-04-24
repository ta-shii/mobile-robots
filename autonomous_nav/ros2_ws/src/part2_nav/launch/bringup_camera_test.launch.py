from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='depthai_ros_driver_v3',
            executable='driver_node',
            name='oak',
            output='screen',
            parameters=['/opt/ros/jazzy/share/depthai_ros_driver_v3/config/driver.yaml']
        ),
        Node(
            package='part2_nav',
            executable='cone_detector',
            name='cone_detector',
            output='screen',
            parameters=[{
                'save_dir': '/tmp/cone_photos',
                'image_topic': '/oak/rgb/image_raw'
            }]
        ),
    ])