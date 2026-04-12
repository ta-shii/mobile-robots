#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class SimpleAvoid(Node):
    def __init__(self):
        super().__init__('simple_avoid')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.get_logger().info('Simple obstacle avoidance node started.')

    def scan_callback(self, msg: LaserScan):
        cmd = Twist()

        total = len(msg.ranges)
        mid = total // 2

        # Check a small window in front of the robot
        front_ranges = msg.ranges[mid - 20: mid + 20]

        valid_ranges = [
            r for r in front_ranges
            if not math.isinf(r) and not math.isnan(r)
        ]

        # If nothing valid, rotate slowly just to be safe
        if len(valid_ranges) == 0:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.3
            self.cmd_pub.publish(cmd)
            return

        min_front = min(valid_ranges)

        if min_front < 1.0:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5
        else:
            cmd.linear.x = 0.3
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleAvoid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()