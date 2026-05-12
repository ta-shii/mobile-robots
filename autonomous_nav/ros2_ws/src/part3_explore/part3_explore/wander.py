"""
wander.py — lawnmower coverage driver for SLAM map-building

Pattern: boustrophedon (back-and-forth strips) across the 15×15 m arena.
  - Drives east across the arena, turns, drives west one strip lower, repeats
  - Strip spacing 1.5 m gives ~20% overlap at the 10 m lidar range
  - Safety layer runs on every tick regardless of state: if anything is
    closer than STOP_DIST the robot stops and waits one cycle before resuming

States:
  DRIVE   – drive straight toward the target X end of the current strip
  TURN1   – rotate 90° toward the next strip
  LATERAL – drive 1.5 m sideways (one strip width)
  TURN2   – rotate back to face the next strip direction (east or west)

Replaced later by Nav2 + proper frontier explorer in Stage 4/6.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


# Arena / pattern constants 
ARENA_MIN_X  = -6.0   # m — stay 1.5 m inside the west wall
ARENA_MAX_X  =  6.0   # m — stay 1.5 m inside the east wall
ARENA_START_Y =  5.5  # m — first strip (near north wall)
ARENA_END_Y  = -5.5   # m — last strip  (near south wall)
STRIP_WIDTH  =  1.5   # m — distance between parallel strips

# Motion constants 
DRIVE_SPEED   = 0.2   # m/s — straight-line cruise
TURN_SPEED    = 0.4   # rad/s — rotation during turns
STOP_DIST     = 0.35  # m — hard stop if obstacle closer than this
GOAL_TOL_XY   = 0.6   # m — declare strip end early, before wall safety stop triggers
GOAL_TOL_YAW  = 0.05  # rad — close enough to target heading

# State machine states
DRIVE   = 'DRIVE'
TURN1   = 'TURN1'
LATERAL = 'LATERAL'
TURN2   = 'TURN2'
DONE    = 'DONE'


class Wander(Node):
    def __init__(self):
        super().__init__('wander')

        self._scan_min_front = float('inf')
        self._x   = 0.0
        self._y   = 0.0
        self._yaw = 0.0

        # Build the strip list: alternating east (+X) and west (-X) sweeps
        self._strips = self._build_strips()
        self._strip_idx = 0
        self._state = DRIVE
        self._blocked = False

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.create_subscription(Odometry,  '/odom', self._odom_cb, 10)
        self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f'wander: lawnmower pattern, {len(self._strips)} strips, '
            f'starting at y={ARENA_START_Y}')

    # Strip list builder 
    def _build_strips(self):
        """Return list of (start_x, end_x, y) tuples for each E/W sweep."""
        strips = []
        y = ARENA_START_Y
        going_east = True
        while y >= ARENA_END_Y - 0.1:
            if going_east:
                strips.append((ARENA_MIN_X, ARENA_MAX_X, y))
            else:
                strips.append((ARENA_MAX_X, ARENA_MIN_X, y))
            y -= STRIP_WIDTH
            going_east = not going_east
        return strips

    # Sensor callbacks

    def _scan_cb(self, msg: LaserScan):
        total = len(msg.ranges)
        rays_per_rad = total / (msg.angle_max - msg.angle_min)
        half_rays = int(math.radians(20) * rays_per_rad)  # 40° cone — avoids reading robot body
        fwd = int((0.0 - msg.angle_min) * rays_per_rad)
        fwd = max(0, min(fwd, total - 1))
        lo, hi = max(0, fwd - half_rays), min(total - 1, fwd + half_rays)
        readings = [r for r in msg.ranges[lo:hi + 1]
                    if msg.range_min < r < msg.range_max]
        self._scan_min_front = min(readings) if readings else float('inf')
        self.get_logger().debug(f'front min range: {self._scan_min_front:.3f} m')

    def _odom_cb(self, msg: Odometry):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # yaw from quaternion — no external package needed
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._yaw = math.atan2(siny, cosy)

    # Main control tick

    def _tick(self):
        cmd = Twist()

        if self._state == DONE:
            self.pub.publish(cmd)
            return

        # Safety layer — only block straight-line motion, never block turns.
        # Turns need to execute even when an obstacle is close (it's the wall
        # we just approached — we need to rotate AWAY from it).
        if self._scan_min_front < STOP_DIST:
            if self._state == DRIVE:
                if not self._blocked:
                    self.get_logger().warn(
                        f'wall at {self._scan_min_front:.2f} m — ending strip, turning')
                    self._blocked = True
                self._state = TURN1
                # fall through — let TURN1 execute this tick
            elif self._state == LATERAL:
                # mid-strip obstacle during lateral move — just pause
                self.pub.publish(cmd)
                return
            # TURN1 / TURN2: ignore safety, allow rotation
        else:
            self._blocked = False

        strip = self._strips[self._strip_idx]
        start_x, end_x, strip_y = strip

        if self._state == DRIVE:
            self._do_drive(cmd, end_x, strip_y)

        elif self._state == TURN1:
            # Face south (−Y direction) to move to next strip
            self._do_turn(cmd, target_yaw=-math.pi / 2)

        elif self._state == LATERAL:
            # Drive south by STRIP_WIDTH to reach next strip's Y
            if self._strip_idx + 1 < len(self._strips):
                next_y = self._strips[self._strip_idx + 1][2]
                self._do_lateral(cmd, next_y)
            else:
                self._state = DONE
                self.get_logger().info('wander: all strips complete')

        elif self._state == TURN2:
            # Face the direction of the next strip (east or west)
            if self._strip_idx + 1 < len(self._strips):
                next_start, next_end, _ = self._strips[self._strip_idx + 1]
                target_yaw = 0.0 if next_end > next_start else math.pi
                self._do_turn(cmd, target_yaw)
            else:
                self._state = DONE

        self.pub.publish(cmd)

    # Motion primitives

    def _do_drive(self, cmd, target_x, target_y):
        """Drive straight until we reach target_x on the current strip."""
        dx = target_x - self._x
        dy = target_y - self._y
        dist = math.hypot(dx, dy)

        if dist < GOAL_TOL_XY:
            self.get_logger().info(
                f'wander: strip {self._strip_idx + 1}/{len(self._strips)} done')
            self._state = TURN1
            return

        # Small heading correction to stay on the strip's Y line
        desired_yaw = math.atan2(dy, dx)
        yaw_err = self._angle_diff(desired_yaw, self._yaw)
        cmd.linear.x  = DRIVE_SPEED
        cmd.angular.z = max(-TURN_SPEED, min(TURN_SPEED, 2.0 * yaw_err))

    def _do_turn(self, cmd, target_yaw):
        """Spin in place until heading matches target_yaw."""
        err = self._angle_diff(target_yaw, self._yaw)
        if abs(err) < GOAL_TOL_YAW:
            # Decide next state after this turn
            if self._state == TURN1:
                self._state = LATERAL
            else:  # TURN2
                self._strip_idx += 1
                self._state = DRIVE
            return
        cmd.angular.z = math.copysign(TURN_SPEED, err)

    def _do_lateral(self, cmd, target_y):
        """Drive south until we reach the next strip's Y coordinate."""
        dy = target_y - self._y
        if abs(dy) < GOAL_TOL_XY:
            self._state = TURN2
            return
        desired_yaw = math.atan2(dy, 0.0)   # pure south = -π/2
        yaw_err = self._angle_diff(desired_yaw, self._yaw)
        cmd.linear.x  = DRIVE_SPEED
        cmd.angular.z = max(-TURN_SPEED, min(TURN_SPEED, 2.0 * yaw_err))

    @staticmethod
    def _angle_diff(target, current):
        """Shortest signed angle from current to target, wrapped to (−π, π]."""
        d = target - current
        while d >  math.pi: d -= 2 * math.pi
        while d < -math.pi: d += 2 * math.pi
        return d


def main(args=None):
    rclpy.init(args=args)
    node = Wander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
