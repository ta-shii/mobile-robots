#!/usr/bin/env python3

import csv
import math
import os
from datetime import datetime

import matplotlib.pyplot as plt
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Convert quaternion to yaw angle."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class LidarMapRecorder(Node):
    def __init__(self):
        super().__init__('lidar_map_recorder')

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Output folder
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.base_dir = os.path.expanduser('~/Desktop/project-mobrob/ros2_ws/output')
        os.makedirs(self.base_dir, exist_ok=True)

        self.path_file = os.path.join(self.base_dir, f'path_log_{timestamp}.csv')
        self.map_file = os.path.join(self.base_dir, f'lidar_map_{timestamp}.csv')
        self.plot_file = os.path.join(self.base_dir, f'lidar_map_{timestamp}_plot.png')

        # CSV writers
        self.path_csv = open(self.path_file, 'w', newline='')
        self.path_writer = csv.writer(self.path_csv)
        self.path_writer.writerow(['time_sec', 'x', 'y', 'yaw'])

        self.map_csv = open(self.map_file, 'w', newline='')
        self.map_writer = csv.writer(self.map_csv)
        self.map_writer.writerow(['map_x', 'map_y'])

        # Robot pose
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.have_odom = False

        # Time / path logging
        self.start_time = None
        self.last_path_x = None
        self.last_path_y = None
        self.min_path_distance = 0.05
        self.path_points_written = 0

        # Path for plotting
        self.path_x = []
        self.path_y = []

        # Map points
        self.map_points = set()
        self.map_x = []
        self.map_y = []

        # Map settings
        self.max_map_range = 6.0      # ignore very far points
        self.beam_step = 4            # use every 4th beam to reduce clutter
        self.map_resolution = 0.05    # 5 cm grid for deduplication

        self.get_logger().info('Lidar map recorder started.')
        self.get_logger().info(f'Saving path CSV to: {self.path_file}')
        self.get_logger().info(f'Saving map CSV to: {self.map_file}')
        self.get_logger().info(f'Final plot will be saved to: {self.plot_file}')

    def odom_callback(self, msg: Odometry):
        now_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.start_time is None:
            self.start_time = now_sec

        pose = msg.pose.pose
        self.current_x = pose.position.x
        self.current_y = pose.position.y

        q = pose.orientation
        self.current_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

        self.have_odom = True

        # Record path point only if moved enough
        if self.last_path_x is None or self.last_path_y is None:
            self.write_path_point(now_sec - self.start_time, self.current_x, self.current_y, self.current_yaw)
            self.last_path_x = self.current_x
            self.last_path_y = self.current_y
            return

        distance = math.sqrt(
            (self.current_x - self.last_path_x) ** 2 +
            (self.current_y - self.last_path_y) ** 2
        )

        if distance >= self.min_path_distance:
            self.write_path_point(now_sec - self.start_time, self.current_x, self.current_y, self.current_yaw)
            self.last_path_x = self.current_x
            self.last_path_y = self.current_y

    def scan_callback(self, msg: LaserScan):
        if not self.have_odom:
            return

        angle = msg.angle_min

        for i, r in enumerate(msg.ranges):
            # Use only some beams to keep it light
            if i % self.beam_step != 0:
                angle += msg.angle_increment
                continue

            # Skip invalid or too-far points
            if math.isinf(r) or math.isnan(r):
                angle += msg.angle_increment
                continue

            if r < msg.range_min or r > min(msg.range_max, self.max_map_range):
                angle += msg.angle_increment
                continue

            # Convert scan point from robot frame to world/odom frame
            global_angle = self.current_yaw + angle
            px = self.current_x + r * math.cos(global_angle)
            py = self.current_y + r * math.sin(global_angle)

            # Grid/deduplicate points
            gx = round(px / self.map_resolution) * self.map_resolution
            gy = round(py / self.map_resolution) * self.map_resolution

            key = (gx, gy)
            if key not in self.map_points:
                self.map_points.add(key)
                self.map_x.append(gx)
                self.map_y.append(gy)
                self.map_writer.writerow([f'{gx:.4f}', f'{gy:.4f}'])

            angle += msg.angle_increment

        self.map_csv.flush()

    def write_path_point(self, t: float, x: float, y: float, yaw: float):
        self.path_writer.writerow([f'{t:.3f}', f'{x:.4f}', f'{y:.4f}', f'{yaw:.4f}'])
        self.path_csv.flush()

        self.path_x.append(x)
        self.path_y.append(y)

        self.path_points_written += 1

        if self.path_points_written % 20 == 0:
            self.get_logger().info(
                f'Recorded {self.path_points_written} path points, '
                f'{len(self.map_points)} map points...'
            )

    def plot_map(self):
        if len(self.path_x) < 2:
            self.get_logger().warn('Not enough path points to plot.')
            return

        plt.figure(figsize=(9, 7))

        # Plot map points first
        if len(self.map_x) > 0:
            plt.scatter(self.map_x, self.map_y, s=4, alpha=0.5, label='Lidar Map')

        # Plot robot path
        plt.plot(self.path_x, self.path_y, linewidth=2, label='Robot Path')

        # Start / end markers
        plt.scatter(self.path_x[0], self.path_y[0], s=80, marker='o', label='Start')
        plt.scatter(self.path_x[-1], self.path_y[-1], s=80, marker='x', label='End')

        plt.xlabel('X position (m)')
        plt.ylabel('Y position (m)')
        plt.title('Lidar-Based Map Overlay with Recorded Robot Path')
        plt.axis('equal')
        plt.grid(True)
        plt.legend()

        plt.savefig(self.plot_file, dpi=200, bbox_inches='tight')
        plt.close()

        self.get_logger().info(f'Map overlay plot saved to: {self.plot_file}')

    def destroy_node(self):
        self.get_logger().info(
            f'Closing recorder. Saved {self.path_points_written} path points '
            f'and {len(self.map_points)} map points.'
        )

        self.path_csv.close()
        self.map_csv.close()

        self.plot_map()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarMapRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()