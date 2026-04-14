import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_p3at = get_package_share_directory('p3at')
    pkg_slam = get_package_share_directory('slam_toolbox')

    sdf2_launch = os.path.join(pkg_p3at, 'launch', 'sdf2.launch.py')
    slam_params = os.path.join(pkg_p3at, 'config', 'slam_params.yaml')
    slam_launch = os.path.join(pkg_slam, 'launch', 'online_async_launch.py')

    use_slam_arg = DeclareLaunchArgument('use_slam', default_value='true')
    use_controller_arg = DeclareLaunchArgument('use_controller', default_value='true')
    use_recorder_arg = DeclareLaunchArgument('use_recorder', default_value='false')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')
    use_steering_arg = DeclareLaunchArgument('use_steering', default_value='false')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sdf2_launch),
        launch_arguments={
            'rviz': LaunchConfiguration('use_rviz'),
            'steering': LaunchConfiguration('use_steering'),
        }.items(),
    )

    odom_tf_broadcaster = Node(
        package='p3at',
        executable='odom_tf_broadcaster',
        name='odom_tf_broadcaster',
        output='screen'
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch),
        condition=IfCondition(LaunchConfiguration('use_slam')),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': slam_params,
        }.items(),
    )

    obstacle_controller = Node(
        package='p3at',
        executable='obstacle_aware_waypoint_controller',
        name='obstacle_aware_waypoint_controller',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_controller'))
    )

    lidar_map_recorder = Node(
        package='p3at',
        executable='lidar_map_recorder',
        name='lidar_map_recorder',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_recorder'))
    )

    return LaunchDescription([
        use_slam_arg,
        use_controller_arg,
        use_recorder_arg,
        use_rviz_arg,
        use_steering_arg,
        sim_launch,
        odom_tf_broadcaster,
        slam_toolbox_launch,
        obstacle_controller,
        lidar_map_recorder,
    ])