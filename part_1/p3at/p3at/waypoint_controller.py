#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def normalize_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Convert quaternion to yaw angle."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class WaypointController(Node):
    def __init__(self):
        super().__init__('waypoint_controller')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Goal waypoint
        self.goal_x = 2.0
        self.goal_y = 1.0
        self.goal_yaw = 0.0  # radians

        # Robot state 
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.have_odom = False

        # Control tolerances 
        self.position_tolerance = 0.20
        self.yaw_tolerance = 0.08

        # Control gains / speeds 
        self.max_linear_speed = 0.35
        self.max_angular_speed = 0.8

        self.k_linear = 0.4
        self.k_angular = 1.0

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        self.goal_reached = False

        self.get_logger().info(
            f'Waypoint controller started. Goal = '
            f'({self.goal_x:.2f}, {self.goal_y:.2f}, yaw={self.goal_yaw:.2f})'
        )

    def odom_callback(self, msg: Odometry):
        pose = msg.pose.pose

        self.current_x = pose.position.x
        self.current_y = pose.position.y

        q = pose.orientation
        self.current_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

        self.have_odom = True

    def control_loop(self):
        if not self.have_odom:
            return

        cmd = Twist()

        # Position error
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance_error = math.sqrt(dx * dx + dy * dy)

        # Heading to goal position
        target_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(target_heading - self.current_yaw)

        # Final orientation error
        final_yaw_error = normalize_angle(self.goal_yaw - self.current_yaw)

        # Stage 1 and 2: Go to goal position 
        if distance_error > self.position_tolerance:
            # If robot is facing far away from goal direction, rotate first
            if abs(heading_error) > 0.25:
                cmd.linear.x = 0.0
                cmd.angular.z = max(
                    -self.max_angular_speed,
                    min(self.max_angular_speed, self.k_angular * heading_error)
                )
            else:
                # Move forward and steer a little toward the goal
                cmd.linear.x = min(self.max_linear_speed, self.k_linear * distance_error)
                cmd.angular.z = max(
                    -self.max_angular_speed,
                    min(self.max_angular_speed, self.k_angular * heading_error)
                )

        # Stage 3: Final orientation adjustment 
        elif abs(final_yaw_error) > self.yaw_tolerance:
            cmd.linear.x = 0.0
            cmd.angular.z = max(
                -self.max_angular_speed,
                min(self.max_angular_speed, self.k_angular * final_yaw_error)
            )

        # Goal reached 
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

            if not self.goal_reached:
                self.goal_reached = True
                self.get_logger().info('Goal reached successfully.')

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()