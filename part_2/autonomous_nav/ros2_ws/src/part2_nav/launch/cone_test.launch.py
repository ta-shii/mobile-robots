from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='part2_nav',
            executable='cone_detector',
            name='cone_detector',
            output='screen',
            parameters=[{
                'save_dir': '/tmp/cone_photos',
                'min_area': 800,
                'right_bias': 0.05,
                'image_topic': '/oak/rgb/image_raw'
            }]
        ),
    ])
