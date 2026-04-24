#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class MultiWaypointController(Node):
    def __init__(self):
        super().__init__('multi_waypoint_controller')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # List of waypoints: (x, y, yaw)
        self.waypoints = [
            (2.0, 1.0, 0.0),
            (4.0, 1.5, 0.0),
            (5.0, -1.0, 1.57),
        ]

        self.current_waypoint_index = 0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.have_odom = False

        self.position_tolerance = 0.20
        self.yaw_tolerance = 0.08

        self.max_linear_speed = 0.35
        self.max_angular_speed = 0.8

        self.k_linear = 0.4
        self.k_angular = 1.0

        self.timer = self.create_timer(0.1, self.control_loop)

        self.all_goals_reached = False

        self.get_logger().info(
            f'Multi-waypoint controller started with {len(self.waypoints)} waypoints.'
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

        if self.all_goals_reached:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

        goal_x, goal_y, goal_yaw = self.waypoints[self.current_waypoint_index]

        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        distance_error = math.sqrt(dx * dx + dy * dy)

        target_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(target_heading - self.current_yaw)

        final_yaw_error = normalize_angle(goal_yaw - self.current_yaw)

        # Stage 1 and 2: move to waypoint position
        if distance_error > self.position_tolerance:
            if abs(heading_error) > 0.25:
                cmd.linear.x = 0.0
                cmd.angular.z = max(
                    -self.max_angular_speed,
                    min(self.max_angular_speed, self.k_angular * heading_error)
                )
            else:
                cmd.linear.x = min(self.max_linear_speed, self.k_linear * distance_error)
                cmd.angular.z = max(
                    -self.max_angular_speed,
                    min(self.max_angular_speed, self.k_angular * heading_error)
                )

        # Stage 3: final orientation at current waypoint
        elif abs(final_yaw_error) > self.yaw_tolerance:
            cmd.linear.x = 0.0
            cmd.angular.z = max(
                -self.max_angular_speed,
                min(self.max_angular_speed, self.k_angular * final_yaw_error)
            )

        # Current waypoint reached -> go to next
        else:
            self.get_logger().info(
                f'Waypoint {self.current_waypoint_index + 1} reached: '
                f'({goal_x:.2f}, {goal_y:.2f}, yaw={goal_yaw:.2f})'
            )

            self.current_waypoint_index += 1

            if self.current_waypoint_index >= len(self.waypoints):
                self.all_goals_reached = True
                self.get_logger().info('All waypoints reached successfully.')
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            else:
                next_goal = self.waypoints[self.current_waypoint_index]
                self.get_logger().info(
                    f'Moving to next waypoint {self.current_waypoint_index + 1}: '
                    f'({next_goal[0]:.2f}, {next_goal[1]:.2f}, yaw={next_goal[2]:.2f})'
                )

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MultiWaypointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()