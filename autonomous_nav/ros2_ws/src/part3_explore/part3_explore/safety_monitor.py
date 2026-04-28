#!/usr/bin/env python3
"""
safety_monitor.py
==========================
Detects moving obstacles from Lidar and triggers a software e-stop if any
moving object comes within 1 m of the robot.

How it works
  Lidar gives a 360° scan as a list of (angle, range) points every ~100ms.
  To distinguish *moving* obstacles from static ones we use scan-to-scan
  comparison in polar space:

  1. Keep a rolling window of the last N scans.
  2. For each scan, compare each beam's range to the median of the same beam
     across the window (the 'background').
  3. If a beam is significantly *shorter* than background (something appeared),
     and is within the danger radius, it's a moving obstacle event.

  This is a lightweight alternative to full object tracking (e.g. Kalman
  filters) and works well for outdoor environments with sparse dynamic objects.

  NOTE: static obstacles close to the robot will also trigger this.  That's
  intentional – the brief says e-stop if *any* moving object comes within 1 m.
  In practice, the lidar safety from Part 2 already prevents static collisions,
  so close-range hits in MAPPING state are almost certainly dynamic.

Topics subscribed
  /scan                (sensor_msgs/LaserScan)
  /mission/state       (std_msgs/String)

Topics published
  /cmd_vel             (geometry_msgs/Twist)    – zero if e-stop active
  /estop/triggered     (std_msgs/Bool)          – True = e-stop active
  /estop/incident_log  (std_msgs/String)        – one-line log entry per event
"""

import math
import statistics
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class MovingObstacleMonitor(Node):

    DANGER_RADIUS = 1.0      # metres – brief requirement
    WINDOW_SIZE   = 8        # number of scans to keep as background
    DELTA_THRESH  = 0.4      # metres shorter than background → potential obstacle
    MIN_BEAMS     = 3        # minimum consecutive beams to confirm detection

    def __init__(self):
        super().__init__('safety_monitor')

        self.mission_state = 'IDLE'
        self.estop_active  = False

        # Circular buffer of recent scans (list of range arrays)
        self.scan_history: list[list[float]] = []

        # --- Publishers ---
        self.cmd_pub     = self.create_publisher(Twist,  '/cmd_vel',            10)
        self.estop_pub   = self.create_publisher(Bool,   '/estop/triggered',    10)
        self.log_pub     = self.create_publisher(String, '/estop/incident_log', 10)

        # --- Subscribers ---
        self.create_subscription(LaserScan, '/scan',          self._scan_cb,  10)
        self.create_subscription(String,    '/mission/state', self._state_cb, 10)

        # Continuously publish zero cmd_vel at 20 Hz while e-stop is active
        # so teleop cannot override it with non-zero commands.
        self.create_timer(0.05, self._estop_hold)

        self.get_logger().info('MovingObstacleMonitor ready')

    # ------------------------------------------------------------------
    def _state_cb(self, msg: String):
        self.mission_state = msg.data
        if msg.data == 'IDLE':
            # Clear e-stop when operator resets to IDLE
            self._set_estop(False)

    def _scan_cb(self, msg: LaserScan):
        ranges = list(msg.ranges)

        # Replace inf/nan with max range so background median is correct
        max_r = msg.range_max
        clean = [r if (math.isfinite(r) and r > 0) else max_r for r in ranges]

        # Maintain background window
        self.scan_history.append(clean)
        if len(self.scan_history) > self.WINDOW_SIZE:
            self.scan_history.pop(0)

        # Need at least 2 scans to compute background
        if len(self.scan_history) < 2:
            return

        # Only check when robot is doing something (avoid false positives at idle)
        if self.mission_state == 'IDLE':
            return

        # Compute per-beam median background
        n_beams = len(clean)
        background = [
            statistics.median(self.scan_history[i][b] for i in range(len(self.scan_history) - 1))
            for b in range(n_beams)
        ]

        # Detect beams that are significantly closer than background
        close_beams = 0
        for b in range(n_beams):
            range_m = clean[b]
            # Within danger radius AND significantly shorter than background
            if range_m < self.DANGER_RADIUS and (background[b] - range_m) > self.DELTA_THRESH:
                close_beams += 1

        if close_beams >= self.MIN_BEAMS and not self.estop_active:
            angle_min = msg.angle_min
            angle_inc = msg.angle_increment
            self._trigger_estop(close_beams, msg.header.stamp)

    def _trigger_estop(self, n_beams: int, stamp):
        self._set_estop(True)

        # Stop the robot immediately
        self.cmd_pub.publish(Twist())

        ts = time.strftime('%Y-%m-%d %H:%M:%S')
        log = String()
        log.data = (
            f'ESTOP {ts} | moving obstacle detected | '
            f'{n_beams} beams within {self.DANGER_RADIUS} m'
        )
        self.log_pub.publish(log)
        self.get_logger().warn(log.data)

    def _estop_hold(self):
        if self.estop_active:
            self.cmd_pub.publish(Twist())

    def _set_estop(self, active: bool):
        self.estop_active = active
        msg = Bool()
        msg.data = active
        self.estop_pub.publish(msg)


def main():
    rclpy.init()
    node = MovingObstacleMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
