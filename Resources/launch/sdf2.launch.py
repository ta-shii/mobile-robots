# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Absolute paths inside Resources
    base_dir = '/home/tashi/Desktop/project-mobrob/Resources'
    sdf_file = os.path.join(base_dir, 'worlds', 'basic_urdf.sdf')
    robot_file = os.path.join(base_dir, 'robots', 'pioneer.urdf')
    rviz_config = os.path.join(base_dir, 'rviz', 'pioneer_visuals.rviz')

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    with open(robot_file, 'r') as infp:
        robot_desc = infp.read()

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Open RViz'
    )

    steering_launch_arg = DeclareLaunchArgument(
        'steering',
        default_value='false',
        description='Open rqt_robot_steering'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': sdf_file
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        output='screen',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
        ]
    )

    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        output='screen',
        arguments=[
            '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        ]
    )

    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='imu_bridge',
        output='screen',
        arguments=[
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
        ]
    )

    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='odom_bridge',
        output='screen',
        arguments=[
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        ]
    )

    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        output='screen',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        ]
    )

    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_tf_pub',
        output='screen',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0.1',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'pioneer/base_link/laser'
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': True},
        ]
    )

    robot_steering = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering',
        name='robot_steering',
        output='screen',
        condition=IfCondition(LaunchConfiguration('steering'))
    )

    return LaunchDescription([
        rviz_launch_arg,
        steering_launch_arg,
        gazebo,
        robot_state_publisher,
        joint_state_pub,
        lidar_bridge,
        camera_bridge,
        imu_bridge,
        odom_bridge,
        cmd_vel_bridge,
        laser_tf,
        rviz,
        robot_steering
    ])