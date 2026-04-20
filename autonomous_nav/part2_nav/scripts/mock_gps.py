#!/usr/bin/env python3
"""
Mock GPS publisher for personal device testing.

Simulates the robot starting at a home position, then gradually moving toward
each waypoint as soon as auto_mode is enabled. This lets you verify the full
waypoint_navigator logic without any physical hardware.

What it does:
  - Publishes /fix at 5 Hz
  - Starts at HOME position (slightly offset from WP1)
  - When auto_mode = True:  moves the simulated position toward the current
    waypoint at ~0.3 m/s (same speed as the real robot)
  - When auto_mode = False: position stays still
  - Logs distance to current target so you can watch it count down

Run standalone:
  python3 mock_gps.py

Or via docker-compose.laptop.yml (started automatically).
"""

import math
import sys
import time
import yaml
import os

# ── Inline ROS2 — imported after path is ready ─────────────────────────────
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool


# Home position — intentionally ~25 m away from WP1 so you can watch
# the distance count down in the navigator logs.
# These match the placeholder values in waypoints.yaml.
HOME_LAT = -31.98030
HOME_LON = 115.81740

WAYPOINTS_FILE = os.path.join(
    os.path.dirname(__file__), '..', 'config', 'waypoints.yaml'
)

SIM_SPEED_MPS = 0.3    # simulated robot speed (m/s) — matches max_linear param
PUBLISH_HZ    = 5      # GPS publish rate


class MockGPS(Node):

    def __init__(self):
        super().__init__('mock_gps')

        self._lat = HOME_LAT
        self._lon = HOME_LON
        self._auto = False

        # Load waypoints so we can log distance-to-target
        self._waypoints = self._load_waypoints()
        self._wp_idx = 0

        self._pub = self.create_publisher(NavSatFix, 'fix', 10)
        self.create_subscription(Bool, 'auto_mode', self._mode_cb, 10)
        self.create_timer(1.0 / PUBLISH_HZ, self._tick)

        self.get_logger().info(
            f'Mock GPS ready — home=({HOME_LAT}, {HOME_LON})  '
            f'{len(self._waypoints)} waypoints loaded'
        )
        self.get_logger().info('Press X on gamepad (or publish auto_mode=True) to start moving.')

    # ── helpers ────────────────────────────────────────────────────────────

    def _load_waypoints(self):
        try:
            with open(WAYPOINTS_FILE) as f:
                data = yaml.safe_load(f)
            wps = [(float(w['lat']), float(w['lon'])) for w in data.get('waypoints', [])]
            return wps
        except Exception as e:
            self.get_logger().warn(f'Could not load waypoints: {e}')
            return []

    # ── callbacks ──────────────────────────────────────────────────────────

    def _mode_cb(self, msg: Bool):
        prev = self._auto
        self._auto = msg.data
        if self._auto and not prev:
            self.get_logger().info('auto_mode ON — simulated robot now moving')
        elif not self._auto and prev:
            self.get_logger().info('auto_mode OFF — simulated robot stopped')

    def _tick(self):
        if not self._auto:
            return

        all_waypoints_done = self._wp_idx >= len(self._waypoints)

        if not all_waypoints_done:
            # ── Moving toward next waypoint ───────────────────────────────
            target_lat, target_lon = self._waypoints[self._wp_idx]
            dist = self._haversine(self._lat, self._lon, target_lat, target_lon)

            if dist > 1.5:
                self._move_toward(target_lat, target_lon)
                self.get_logger().info(
                    f'Simulating → WP{self._wp_idx+1}  dist={dist:.1f}m  '
                    f'pos=({self._lat:.6f}, {self._lon:.6f})'
                )
            else:
                self.get_logger().info(
                    f'Simulated arrival at WP{self._wp_idx+1}! Moving to next.'
                )
                self._wp_idx += 1

        else:
            # ── All waypoints done — simulate return to home ───────────────
            dist_home = self._haversine(self._lat, self._lon, HOME_LAT, HOME_LON)

            if dist_home > 1.5:
                self._move_toward(HOME_LAT, HOME_LON)
                self.get_logger().info(
                    f'Simulating → HOME  dist={dist_home:.1f}m  '
                    f'pos=({self._lat:.6f}, {self._lon:.6f})'
                )
            else:
                self.get_logger().info(
                    f'Simulated arrival at HOME! Mission complete. dist={dist_home:.1f}m'
                )
                # Stop auto mode so robot halts
                self._auto = False

        # Always publish current position
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_link'
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude  = self._lat
        msg.longitude = self._lon
        msg.altitude  = 10.0
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self._pub.publish(msg)

    def _move_toward(self, target_lat, target_lon):
        """Move simulated position one step toward target."""
        step = SIM_SPEED_MPS / PUBLISH_HZ
        bearing = self._bearing(self._lat, self._lon, target_lat, target_lon)
        R = 6_371_000.0
        self._lat += (step / R) * math.cos(bearing) * (180 / math.pi)
        self._lon += (step / R) * math.sin(bearing) / math.cos(math.radians(self._lat)) * (180 / math.pi)

    # ── geometry ───────────────────────────────────────────────────────────

    @staticmethod
    def _haversine(lat1, lon1, lat2, lon2):
        R = 6_371_000.0
        p1, p2 = math.radians(lat1), math.radians(lat2)
        dp = math.radians(lat2 - lat1)
        dl = math.radians(lon2 - lon1)
        a = math.sin(dp/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    @staticmethod
    def _bearing(lat1, lon1, lat2, lon2):
        p1, p2 = math.radians(lat1), math.radians(lat2)
        dl = math.radians(lon2 - lon1)
        x = math.sin(dl) * math.cos(p2)
        y = math.cos(p1)*math.sin(p2) - math.sin(p1)*math.cos(p2)*math.cos(dl)
        return math.atan2(x, y)


def main():
    rclpy.init()
    node = MockGPS()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
