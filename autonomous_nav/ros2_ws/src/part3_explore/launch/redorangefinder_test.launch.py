from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction

# Wrap the OAK-D node in a 5 second delay
TimerAction(
    period=5.0,
    actions=[
        Node(
            package='depthai_ros_driver_v3',
            executable='driver_node',
            name='oak',
            output='screen',
            parameters=[
                '/opt/ros/jazzy/share/depthai_ros_driver_v3/config/driver.yaml',
            ]
        )
    ]
),
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lakibeam1',
            executable='lakibeam1_scan_node',
            name='lidar',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('part3_explore'),
                'config', 'lakibeam.yaml'
            )]
        ),
        Node(
            package='part3_explore',
            executable='color_detector',
            name='color_detector',
            output='screen',
            parameters=[{
                'save_dir': '/tmp/object_photos',
                'image_topic': '/oak/rgb/image_raw'
            }]
        ),
    ])  