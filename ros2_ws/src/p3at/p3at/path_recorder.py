#!/usr/bin/env python3

import csv
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Convert quaternion to yaw angle."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PathRecorder(Node):
    def __init__(self):
        super().__init__('path_recorder')

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Save file in home directory for simplicity
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.output_file = os.path.expanduser(f'~/path_log_{timestamp}.csv')

        # Minimum movement before writing a new point
        self.min_distance = 0.05  # meters

        self.last_x = None
        self.last_y = None
        self.start_time = None
        self.points_written = 0

        # Open CSV file and write header
        self.csv_file = open(self.output_file, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['time_sec', 'x', 'y', 'yaw'])

        self.get_logger().info(f'Path recorder started.')
        self.get_logger().info(f'Saving path to: {self.output_file}')

    def odom_callback(self, msg: Odometry):
        now_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.start_time is None:
            self.start_time = now_sec

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

        # Always write first point
        if self.last_x is None or self.last_y is None:
            self.write_point(now_sec - self.start_time, x, y, yaw)
            self.last_x = x
            self.last_y = y
            return

        distance = math.sqrt((x - self.last_x) ** 2 + (y - self.last_y) ** 2)

        if distance >= self.min_distance:
            self.write_point(now_sec - self.start_time, x, y, yaw)
            self.last_x = x
            self.last_y = y

    def write_point(self, t: float, x: float, y: float, yaw: float):
        self.csv_writer.writerow([f'{t:.3f}', f'{x:.4f}', f'{y:.4f}', f'{yaw:.4f}'])
        self.csv_file.flush()
        self.points_written += 1

        if self.points_written % 20 == 0:
            self.get_logger().info(f'Recorded {self.points_written} path points...')

    def destroy_node(self):
        self.get_logger().info(
            f'Closing path recorder. Total points saved: {self.points_written}'
        )
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PathRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()