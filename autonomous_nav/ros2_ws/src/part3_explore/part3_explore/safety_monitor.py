#!/usr/bin/env python3
"""
safety_monitor.py - Monitors lidar and triggers e-stop if any obstacle comes within 1m

Uses scan-to-scan comparison to detect moving obstacles:
  1. Keep a rolling window of recent scans as background.
  2. If a beam is significantly shorter than background AND within
     danger radius → moving obstacle detected → e-stop.

Topics subscribed
  /scan            (sensor_msgs/LaserScan)
  /mission/state   (std_msgs/String)

Topics published
  /cmd_vel             (geometry_msgs/Twist)  – zero on e-stop
  /estop/triggered     (std_msgs/Bool)        – True = e-stop active
  /estop/incident_log  (std_msgs/String)      – one line per event
"""

import math
import statistics
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# MovingObstacleMonitor node
class MovingObstacleMonitor(Node):

    DANGER_RADIUS = 1.0   # metres – stop if anything closer than this
    WINDOW_SIZE   = 8     # scans to keep as background reference
    DELTA_THRESH  = 0.4   # metres shorter than background = obstacle appeared
    MIN_BEAMS     = 3     # consecutive beams needed to confirm detection

    def __init__(self):
        super().__init__('safety_monitor')

        self.mission_state = 'IDLE'
        self.estop_active  = False
        self.scan_history: list[list[float]] = []

        # Publishers - need cmd_pub to stop the robot immediately on e-stop
        self.cmd_pub   = self.create_publisher(Twist,  '/cmd_vel',            10)
        self.estop_pub = self.create_publisher(Bool,   '/estop/triggered',    10)
        self.log_pub   = self.create_publisher(String, '/estop/incident_log', 10)

        # Subscribers – monitor mission state to know when to activate obstacle detection
        self.create_subscription(LaserScan, '/scan',          self._scan_cb,  10)
        self.create_subscription(String,    '/mission/state', self._state_cb, 10)

        self.get_logger().info('SafetyMonitor ready')

    # Mission state callback – only monitor for obstacles when mission is active
    def _state_cb(self, msg: String):
        self.mission_state = msg.data
        if msg.data == 'IDLE':
            self._set_estop(False)

    # Scan callback – main logic for detecting moving obstacles
    def _scan_cb(self, msg: LaserScan):
        ranges = list(msg.ranges)
        max_r  = msg.range_max
        clean  = [r if (math.isfinite(r) and r > 0) else max_r for r in ranges]

        self.scan_history.append(clean)
        if len(self.scan_history) > self.WINDOW_SIZE:
            self.scan_history.pop(0)

        if len(self.scan_history) < 2:
            return

        # Only monitor when robot is active
        if self.mission_state == 'IDLE':
            return

        n_beams    = len(clean)
        background = [
            statistics.median(
                self.scan_history[i][b]
                for i in range(len(self.scan_history) - 1)
            )
            for b in range(n_beams)
        ]

        close_beams = 0
        for b in range(n_beams):
            if clean[b] < self.DANGER_RADIUS and (background[b] - clean[b]) > self.DELTA_THRESH:
                close_beams += 1

        if close_beams >= self.MIN_BEAMS and not self.estop_active:
            self._trigger_estop(close_beams)

    # Trigger e-stop and log the event
    def _trigger_estop(self, n_beams: int):
        self._set_estop(True)
        self.cmd_pub.publish(Twist())   # zero velocity immediately

        ts  = time.strftime('%Y-%m-%d %H:%M:%S')
        msg = String()
        msg.data = (
            f'ESTOP {ts} | moving obstacle | '
            f'{n_beams} beams within {self.DANGER_RADIUS} m'
        )
        self.log_pub.publish(msg)
        self.get_logger().warn(msg.data)

    # Helper to set/clear e-stop state
    def _set_estop(self, active: bool):
        self.estop_active = active
        msg      = Bool()
        msg.data = active
        self.estop_pub.publish(msg)

# Entry point
def main():
    rclpy.init()
    node = MovingObstacleMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
