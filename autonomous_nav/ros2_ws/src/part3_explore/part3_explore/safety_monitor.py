#!/usr/bin/env python3
"""
safety_monitor.py
Detects MOVING obstacles from Lidar and triggers a software e-stop if one
comes within DANGER_RADIUS (1 m, per project brief) of the robot.

How it works
  A rolling buffer of the last BACKGROUND_SCANS scans is maintained.
  For each incoming scan, per-beam background values are estimated as the
  median of that beam across recent scans.

  A beam is flagged as "close-moving" only when:
    - Its range is within DANGER_RADIUS, AND
    - It is significantly shorter than its background estimate
      (by more than DELTA_THRESH).

  Static walls always match their own background → never trigger.
  A person walking into the scan plane causes a sudden beam shortening
  relative to background → triggers.

  MIN_BEAMS or more flagged beams → e-stop triggered.

  Auto-clear: once no close-moving beams have been seen for CLEAR_SECS the
  e-stop is lifted so the robot can resume without operator intervention.

  Manual clear: entering IDLE state also clears the e-stop.

Topics subscribed
  /scan                (sensor_msgs/LaserScan)
  /mission/state       (std_msgs/String)

Topics published
  /estop/triggered     (std_msgs/Bool)          – True = e-stop active
  /estop/incident_log  (std_msgs/String)        – one-line log entry per event
"""

import math
import time
import collections
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan


class MovingObstacleMonitor(Node):

    DANGER_RADIUS     = 1.0   # metres – e-stop if moving object within 1 m
    DELTA_THRESH      = 0.30  # metres – beam must be this much shorter than background
    MIN_BEAMS         = 3     # minimum flagged beams to confirm (filters noise)
    BACKGROUND_SCANS  = 10    # rolling window depth for background estimation
    CLEAR_SECS        = 3.0   # seconds of clear scans before auto-clearing e-stop

    def __init__(self):
        super().__init__('safety_monitor')

        self.mission_state  = 'IDLE'
        self.estop_active   = False
        self._last_close_t  = None   # wall-clock time of last triggered scan

        # Rolling buffer: deque of lists, each list is one full scan (cleaned ranges)
        self._scan_buffer: collections.deque = collections.deque(
            maxlen=self.BACKGROUND_SCANS
        )

        # --- Publishers ---
        self.estop_pub = self.create_publisher(Bool,   '/estop/triggered',    10)
        self.log_pub   = self.create_publisher(String, '/estop/incident_log', 10)

        # --- Subscribers ---
        self.create_subscription(LaserScan, '/scan',          self._scan_cb,  10)
        self.create_subscription(String,    '/mission/state', self._state_cb, 10)

        self.get_logger().info('MovingObstacleMonitor ready')

    # ------------------------------------------------------------------
    def _state_cb(self, msg: String):
        prev = self.mission_state
        self.mission_state = msg.data
        if msg.data != prev:
            # Clear stale background so old scan positions don't trigger false positives
            self._scan_buffer.clear()
            self._last_close_t = None
        if msg.data == 'IDLE' and prev != 'IDLE':
            self._set_estop(False)

    # States where the safety monitor is active.
    # MANUAL_MAPPING excluded – operator is in control and can see obstacles.
    # Background comparison breaks when robot moves (walls shift in scan).
    ACTIVE_STATES = frozenset({'MAPPING', 'RAPID_NAV'})

    def _scan_cb(self, msg: LaserScan):
        if self.mission_state not in self.ACTIVE_STATES:
            return

        max_r = msg.range_max
        clean = [r if (math.isfinite(r) and r > 0) else max_r for r in msg.ranges]

        n_beams = len(clean)

        # Build background from rolling buffer (need at least 2 prior scans)
        if len(self._scan_buffer) >= 2:
            # Per-beam median across buffer entries
            background = [
                _median([self._scan_buffer[k][i] for k in range(len(self._scan_buffer))])
                for i in range(n_beams)
            ]

            # Count beams that are close AND significantly shorter than background
            close_moving = sum(
                1
                for i in range(n_beams)
                if (clean[i] < self.DANGER_RADIUS and
                    background[i] - clean[i] > self.DELTA_THRESH)
            )
        else:
            close_moving = 0

        # Add this scan to the buffer AFTER computing (so it doesn't bias itself)
        self._scan_buffer.append(clean)

        now = time.monotonic()

        self.get_logger().debug(
            f'close_moving={close_moving}  buf_depth={len(self._scan_buffer)}'
        )

        if close_moving >= self.MIN_BEAMS:
            self._last_close_t = now
            if not self.estop_active:
                self._trigger_estop(close_moving, msg.header.stamp)
        elif self.estop_active:
            if (self._last_close_t is not None and
                    (now - self._last_close_t) >= self.CLEAR_SECS):
                self.get_logger().info(
                    f'Area clear for {self.CLEAR_SECS} s – auto-clearing e-stop'
                )
                self._set_estop(False)

    # ------------------------------------------------------------------
    def _trigger_estop(self, n_beams: int, stamp):
        self._set_estop(True)

        ts = time.strftime('%Y-%m-%d %H:%M:%S')
        log = String()
        log.data = (
            f'ESTOP {ts} | moving obstacle detected | '
            f'{n_beams} beams within {self.DANGER_RADIUS} m '
            f'(>{self.DELTA_THRESH} m closer than background)'
        )
        self.log_pub.publish(log)
        self.get_logger().warn(log.data)

    def _set_estop(self, active: bool):
        if active == self.estop_active:
            return
        self.estop_active = active
        msg = Bool()
        msg.data = active
        self.estop_pub.publish(msg)


# ---------------------------------------------------------------------------
# Fast median without numpy dependency

def _median(values: list) -> float:
    s = sorted(values)
    n = len(s)
    mid = n // 2
    return s[mid] if n % 2 else (s[mid - 1] + s[mid]) / 2.0


# ---------------------------------------------------------------------------

def main():
    rclpy.init()
    node = MovingObstacleMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
