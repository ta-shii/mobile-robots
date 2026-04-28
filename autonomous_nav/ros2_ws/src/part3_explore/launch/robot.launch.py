"""
robot.launch.py  –  Week 1 version
Starts the minimum set of nodes needed for:
  - Manual driving via PS4 gamepad
  - SLAM map building from Lakibeam lidar
  - E-stop when obstacle within 1 m

What runs:
  1. ariaNode           – Pioneer motor driver (/cmd_vel → wheels, wheels → /odom)
  2. joy_node           – reads PS4 gamepad, publishes /joy
  3. teleop_twist_joy   – converts /joy → /cmd_vel for manual driving
  4. robot_state_pub    – broadcasts TF tree (base_link → laser etc.)
  5. lakibeam driver    – publishes /scan from Lakibeam lidar
  6. slam_toolbox       – builds /map from /scan + /odom
  7. mission_manager    – state machine, publishes /mission/state
  8. safety_monitor     – monitors /scan, fires /estop/triggered if < 1m

NOT included yet (added in Week 2):
  Nav2, explorer, waypoint_driver, velocity_safety_filter

Usage:
  ros2 launch part3_explore robot.launch.py
  ros2 launch part3_explore robot.launch.py lakibeam_ip:=192.168.198.2
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_p3   = get_package_share_directory('part3_explore')
    pkg_p1   = get_package_share_directory('p3at')
    pkg_slam = get_package_share_directory('slam_toolbox')

    slam_params = os.path.join(pkg_p3, 'config', 'slam_params.yaml')
    p3_params   = os.path.join(pkg_p3, 'config', 'part3_params.yaml')
    urdf_path   = os.path.join(pkg_p1, 'urdf', 'pioneer.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # Launch arguments
    declared_args = [
        DeclareLaunchArgument(
            'lakibeam_ip',
            default_value='192.168.198.2',
            description='IP address of the Lakibeam lidar sensor'
        ),
        DeclareLaunchArgument(
            'robot_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for Pioneer ariaNode'
        ),
    ]

    lakibeam_ip = LaunchConfiguration('lakibeam_ip')
    robot_port  = LaunchConfiguration('robot_port')

    # 1. Pioneer motor driver
    aria_node = Node(
        package='ariaNode',
        executable='ariaNode',
        name='aria_node',
        output='screen',
        arguments=['--rp', robot_port],
    )

    # 2. PS4 gamepad
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'autorepeat_rate': 20.0}],
    )

    # 3. Teleop – converts gamepad to /cmd_vel
    #    Hold L1 to enable driving (deadman switch)
    #    Left stick  = linear velocity
    #    Right stick = angular velocity
    
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        output='screen',
        parameters=[{
            'axis_linear.x':  1,    # left stick up/down
            'axis_angular.yaw': 3,  # right stick left/right
            'scale_linear.x': 0.3,  # max 0.3 m/s forward
            'scale_angular.yaw': 0.5,
            'enable_button': 4,     # L1 = deadman switch
        }],
    )

    # 4. Robot state publisher – TF tree from URDF
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
    )

    # 5. Lakibeam lidar driver
    lakibeam_node = Node(
        package='lakibeam1',
        executable='lakibeam1_scan_node',
        name='lakibeam_node',
        output='screen',
        parameters=[{
            'sensorip':     lakibeam_ip,
            'hostip':       '192.168.198.50',
            'port':         '2368',
            'frame_id':     'laser_frame',
            'output_topic': 'scan',
        }],
    )

    # 6. slam_toolbox – builds map from /scan + /odom
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time':     'false',
            'slam_params_file': slam_params,
        }.items(),
    )

    # 7 & 8. Our Part 3 nodes
    def p3_node(name):
        return Node(
            package='part3_explore',
            executable=name,
            name=name,
            parameters=[p3_params],
            output='screen',
        )

    part3_nodes = [
        p3_node('mission_manager'),
        p3_node('safety_monitor'),
    ]

    return LaunchDescription(
        declared_args + [
            aria_node,
            joy_node,
            teleop_node,
            rsp_node,
            lakibeam_node,
            slam_launch,
        ] + part3_nodes
    )