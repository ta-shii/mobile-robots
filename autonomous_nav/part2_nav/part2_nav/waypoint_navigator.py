#!/usr/bin/env python3
"""
GPS Waypoint Navigator – Part 2, Task 1.

Drives the Pioneer 3-AT through a list of GPS waypoints (each marked by an
orange traffic cone) and returns it to the home position.

Heading is estimated from consecutive GPS fixes (≥ 0.5 m movement).  This
avoids an IMU dependency for task 1, though IMU integration will improve
accuracy for short distances.

Architecture
------------
  /fix          (NavSatFix)  ← GPS driver
  /auto_mode    (Bool)       ← gamepad_controller  (true = robot may move)
  /auto_cmd_vel (Twist)      → gamepad_controller  (mux → cmd_vel → ariaNode)
  /waypoint_status (String)  → UI / summary display
"""

import math
import os
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String


class WaypointNavigator(Node):

    # ── heading estimation ──────────────────────────────────────────────────
    MIN_MOVE_M = 0.5        # minimum GPS movement to update heading estimate
    HEADING_ALPHA = 0.3     # low-pass weight for new heading measurements

    def __init__(self):
        super().__init__('waypoint_navigator')

        # Parameters
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('arrival_radius', 1.5)   # metres
        self.declare_parameter('max_linear', 0.3)       # m/s
        self.declare_parameter('max_angular', 0.5)      # rad/s
        self.declare_parameter('heading_gain', 1.2)     # P gain for angular

        # State
        self._active = False
        self._mission_done = False

        self._lat = None
        self._lon = None
        self._prev_lat = None
        self._prev_lon = None
        self._heading_est = 0.0    # radians, east=0, north=π/2 (ROS convention)

        self._home: tuple[float, float] | None = None
        self._waypoints: list[tuple[float, float]] = []
        self._wp_idx = 0
        self._returning_home = False

        self._load_waypoints()

        # Comms
        self._cmd_pub = self.create_publisher(Twist, 'auto_cmd_vel', 10)
        self._status_pub = self.create_publisher(String, 'waypoint_status', 10)

        self.create_subscription(NavSatFix, 'fix', self._gps_cb, 10)
        self.create_subscription(Bool, 'auto_mode', self._mode_cb, 10)

        self.create_timer(0.2, self._nav_loop)   # 5 Hz

        self.get_logger().info(
            f'Waypoint navigator ready — {len(self._waypoints)} waypoints loaded.'
        )

    # ── setup ──────────────────────────────────────────────────────────────

    def _load_waypoints(self):
        path = self.get_parameter('waypoints_file').value
        if not path or not os.path.exists(path):
            self.get_logger().warn('No waypoints file — set waypoints_file param.')
            return
        with open(path) as f:
            data = yaml.safe_load(f)
        self._waypoints = [
            (float(wp['lat']), float(wp['lon']))
            for wp in data.get('waypoints', [])
        ]

    # ── callbacks ──────────────────────────────────────────────────────────

    def _gps_cb(self, msg: NavSatFix):
        if msg.status.status < 0:   # no fix
            return

        new_lat, new_lon = msg.latitude, msg.longitude

        # Set home on first valid fix
        if self._home is None:
            self._home = (new_lat, new_lon)
            self.get_logger().info(f'Home locked: {self._home}')

        # Update heading estimate from movement
        if self._lat is not None:
            d = self._haversine(self._lat, self._lon, new_lat, new_lon)
            if d >= self.MIN_MOVE_M:
                raw_heading = self._bearing(self._lat, self._lon, new_lat, new_lon)
                # Low-pass filter to smooth GPS noise
                self._heading_est = (
                    self.HEADING_ALPHA * raw_heading
                    + (1 - self.HEADING_ALPHA) * self._heading_est
                )
                self._prev_lat, self._prev_lon = self._lat, self._lon

        self._lat, self._lon = new_lat, new_lon

    def _mode_cb(self, msg: Bool):
        self._active = msg.data
        if not self._active:
            self._cmd_pub.publish(Twist())

    # ── navigation loop ────────────────────────────────────────────────────

    def _nav_loop(self):
        if not self._active or self._lat is None or self._mission_done:
            return

        arrival_r = self.get_parameter('arrival_radius').value

        # Determine current target
        if self._returning_home:
            if self._home is None:
                return
            target = self._home
            label = 'HOME'
        elif self._wp_idx < len(self._waypoints):
            target = self._waypoints[self._wp_idx]
            label = f'WP{self._wp_idx + 1}'
        else:
            self._returning_home = True
            self._publish_status('All waypoints reached — returning home')
            return

        dist = self._haversine(self._lat, self._lon, *target)

        if dist < arrival_r:
            if self._returning_home:
                self._mission_done = True
                self._cmd_pub.publish(Twist())
                self._publish_status('MISSION COMPLETE — back at home')
                self.get_logger().info('Mission complete!')
            else:
                self._publish_status(f'Reached {label} (dist={dist:.2f}m)')
                self.get_logger().info(f'{label} reached at dist={dist:.2f}m')
                self._wp_idx += 1
            return

        self._drive_toward(target, dist, label)

    def _drive_toward(self, target, dist, label):
        max_lin = self.get_parameter('max_linear').value
        max_ang = self.get_parameter('max_angular').value
        kp_ang = self.get_parameter('heading_gain').value

        # Bearing to target (radians)
        target_bearing = self._bearing(self._lat, self._lon, *target)

        # Heading error: difference between where we're going and where we face
        heading_err = self._angle_diff(target_bearing, self._heading_est)

        # Scale linear speed down when:
        #   • turning sharply (don't go fast sideways)
        #   • close to target (gentle arrival)
        turn_scale = max(0.0, 1.0 - abs(heading_err) / math.pi)
        dist_scale = min(1.0, dist / 3.0)
        linear = max_lin * turn_scale * dist_scale

        angular = max(-max_ang, min(max_ang, kp_ang * heading_err))

        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self._cmd_pub.publish(twist)

        self.get_logger().debug(
            f'→ {label}  dist={dist:.1f}m  err={math.degrees(heading_err):.1f}°  '
            f'lin={linear:.2f}  ang={angular:.2f}'
        )

    # ── geometry helpers ───────────────────────────────────────────────────

    @staticmethod
    def _haversine(lat1, lon1, lat2, lon2) -> float:
        R = 6_371_000.0
        p1, p2 = math.radians(lat1), math.radians(lat2)
        dp = math.radians(lat2 - lat1)
        dl = math.radians(lon2 - lon1)
        a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    @staticmethod
    def _bearing(lat1, lon1, lat2, lon2) -> float:
        """Bearing in radians from point 1 to point 2 (north=π/2, east=0)."""
        p1, p2 = math.radians(lat1), math.radians(lat2)
        dl = math.radians(lon2 - lon1)
        x = math.sin(dl) * math.cos(p2)
        y = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
        return math.atan2(x, y)

    @staticmethod
    def _angle_diff(a, b) -> float:
        """Signed shortest angular difference a − b, wrapped to (−π, π]."""
        d = a - b
        while d > math.pi:
            d -= 2 * math.pi
        while d <= -math.pi:
            d += 2 * math.pi
        return d

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)
        self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    try:
        rclpy.spin(node)
    finally:
        node._cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
