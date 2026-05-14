"""
explorer.py — frontier-based autonomous exploration for SLAM map-building

Replaces the lawnmower pattern.  Instead of a fixed sweep:
  1. Reads /map (OccupancyGrid) from slam_toolbox
  2. Finds frontier cells: free (0) cells adjacent to unknown (-1) cells
  3. Clusters nearby frontier cells → picks the nearest cluster centroid
  4. Publishes that point to /explorer/target_pose (PoseStamped)
  5. waypoint_driver.py receives it and forwards to Nav2's NavigateToPose
  6. Waits for /explorer/wp_reached before choosing the next frontier
  7. Repeats until no frontiers remain → mapping complete

Why frontier over lawnmower?
  The lawnmower generates a fixed grid of waypoints, but Nav2 needs a map
  to exist before it can plan to any of them.  Frontiers are by definition
  on the edge of what has already been mapped, so every goal is reachable
  from the current robot position through explored space.  Nav2's costmap
  also handles obstacle avoidance — we do not need a separate estop during
  the MAPPING phase (safety_monitor is gated to RAPID_NAV only).

Topics published
  /explorer/target_pose   (geometry_msgs/PoseStamped)  – next frontier goal
  /explorer/done          (std_msgs/Bool)               – True when mapping complete
  /explorer/progress      (std_msgs/String)             – human-readable status

Topics subscribed
  /mission/state          (std_msgs/String)      – only act in MAPPING
  /explorer/wp_reached    (std_msgs/Bool)        – waypoint_driver ack
  /map                    (nav_msgs/OccupancyGrid)
  /odom                   (nav_msgs/Odometry)
"""

import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool


CLUSTER_RADIUS         = 1.0   # m  – merge frontier cells within this distance
MIN_FRONTIER_DIST      = 0.8   # m  – skip frontiers closer than this (already nearby)
GOAL_TIMEOUT           = 60.0  # s  – abandon goal if Nav2 hasn't acked within this time
FAILURE_SKIP_RADIUS    = 1.5   # m  – don't re-send goals near a recently-failed position
MAX_FAILED_POSITIONS   = 20    # keep at most this many failed positions in memory
MAX_CONSECUTIVE_FAILS  = 5     # declare mapping done after this many back-to-back failures


class Explorer(Node):

    def __init__(self):
        super().__init__('explorer')

        self._map:      OccupancyGrid | None = None
        self._robot_x   = 0.0
        self._robot_y   = 0.0
        self._mission_state       = 'IDLE'
        self._waiting_for_ack     = False   # True after publishing a goal, before wp_reached
        self._done                = False
        self._goal_count          = 0
        self._goal_sent_time      = 0.0     # wall-clock time when last goal was published
        self._last_goal: tuple | None = None          # (x, y) of the most recent goal
        self._failed_positions: list  = []            # list of (x, y) that Nav2 couldn't reach
        self._consecutive_failures    = 0             # back-to-back Nav2 failures

        # Publishers
        self.target_pub   = self.create_publisher(PoseStamped, '/explorer/target_pose', 10)
        self.done_pub     = self.create_publisher(Bool,        '/explorer/done',        10)
        self.progress_pub = self.create_publisher(String,      '/explorer/progress',    10)

        # Subscribers
        self.create_subscription(OccupancyGrid, '/map',                self._map_cb,     10)
        self.create_subscription(Odometry,      '/odom',               self._odom_cb,    10)
        self.create_subscription(String,        '/mission/state',      self._mission_cb, 10)
        self.create_subscription(Bool,          '/explorer/wp_reached', self._ack_cb,    10)

        # Check for new frontiers every 3 s (light — only runs when not waiting)
        self.create_timer(3.0, self._tick)

        self.get_logger().info('explorer: frontier-based mapping ready')

    # ── Callbacks ──────────────────────────────────────────────────────

    def _map_cb(self, msg: OccupancyGrid):
        self._map = msg

    def _odom_cb(self, msg: Odometry):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y

    def _mission_cb(self, msg: String):
        prev = self._mission_state
        self._mission_state = msg.data
        if prev != msg.data:
            self.get_logger().info(f'explorer: mission state → {msg.data}')
        if msg.data == 'MAPPING' and prev != 'MAPPING':
            # Re-entering MAPPING — reset so we start fresh
            self._waiting_for_ack     = False
            self._done                = False
            self._goal_count          = 0
            self._failed_positions    = []
            self._consecutive_failures = 0

    def _ack_cb(self, msg: Bool):
        if not self._waiting_for_ack:
            return
        self._waiting_for_ack = False
        if msg.data:
            # Nav2 succeeded — reset failure tracking
            self._consecutive_failures = 0
            self.get_logger().info(
                f'explorer: waypoint {self._goal_count} reached — searching next frontier')
        else:
            # Nav2 failed — blacklist this position so we don't immediately retry it
            self._consecutive_failures += 1
            if self._last_goal is not None:
                self._failed_positions.append(self._last_goal)
                if len(self._failed_positions) > MAX_FAILED_POSITIONS:
                    self._failed_positions.pop(0)
            self.get_logger().warn(
                f'explorer: waypoint {self._goal_count} failed '
                f'({self._consecutive_failures} consecutive) — blacklisting position')

    # ── Main tick ──────────────────────────────────────────────────────

    def _tick(self):
        if self._mission_state != 'MAPPING':
            return
        if self._waiting_for_ack:
            elapsed = time.monotonic() - self._goal_sent_time
            if elapsed < GOAL_TIMEOUT:
                return
            self.get_logger().warn(
                f'explorer: goal {self._goal_count} timed out after '
                f'{elapsed:.0f} s — skipping to next frontier')
            self._waiting_for_ack = False
        if self._done:
            return
        if self._map is None:
            self.get_logger().info('explorer: waiting for /map...')
            return

        # Stop if Nav2 has failed too many times in a row — remaining frontiers
        # are likely all ghost frontiers from SLAM drift that are unreachable.
        if self._consecutive_failures >= MAX_CONSECUTIVE_FAILS:
            self.get_logger().info(
                f'explorer: {self._consecutive_failures} consecutive failures '
                f'— no reachable frontiers remain, mapping complete')
            self._done = True
            d = Bool(); d.data = True
            self.done_pub.publish(d)
            return

        frontiers = self._find_frontiers(self._map)

        if not frontiers:
            self.get_logger().info('explorer: no frontiers — mapping complete')
            self._done = True
            d = Bool()
            d.data = True
            self.done_pub.publish(d)
            return

        # Remove frontiers that are too close to recently-failed positions.
        # These are almost certainly phantom frontiers from SLAM drift that
        # Nav2 can't plan to — skip them rather than looping on them.
        if self._failed_positions:
            frontiers = [
                f for f in frontiers
                if all(
                    math.hypot(f[0] - fx, f[1] - fy) > FAILURE_SKIP_RADIUS
                    for fx, fy in self._failed_positions
                )
            ]
            if not frontiers:
                self.get_logger().info(
                    'explorer: all remaining frontiers are near failed positions — '
                    'mapping complete')
                self._done = True
                d = Bool(); d.data = True
                self.done_pub.publish(d)
                return

        # Prefer frontiers far enough away to represent genuinely unexplored area.
        # If the map just started, all frontiers may be very close — in that case
        # fall back to the furthest frontier so the robot drives into unknown space.
        far = [f for f in frontiers
               if math.hypot(f[0] - self._robot_x, f[1] - self._robot_y) >= MIN_FRONTIER_DIST]

        if far:
            nearest = min(far, key=lambda p: math.hypot(p[0] - self._robot_x, p[1] - self._robot_y))
        else:
            # All frontiers are very close — pick the furthest to keep making progress.
            # Only declare done if every cluster is within a tiny radius (truly surrounded).
            nearest = max(frontiers, key=lambda p: math.hypot(p[0] - self._robot_x, p[1] - self._robot_y))
            dist_max = math.hypot(nearest[0] - self._robot_x, nearest[1] - self._robot_y)
            if dist_max < 0.3:
                self.get_logger().info('explorer: all frontiers < 0.3 m — mapping complete')
                self._done = True
                d = Bool()
                d.data = True
                self.done_pub.publish(d)
                return

        dist = math.hypot(nearest[0] - self._robot_x, nearest[1] - self._robot_y)

        self._goal_count += 1
        self.get_logger().info(
            f'explorer: goal {self._goal_count} → '
            f'({nearest[0]:.2f}, {nearest[1]:.2f})  dist={dist:.2f} m  '
            f'clusters={len(frontiers)}')

        self._last_goal       = (nearest[0], nearest[1])
        self._publish_target(nearest[0], nearest[1])
        self._waiting_for_ack = True
        self._goal_sent_time  = time.monotonic()

        p = String()
        p.data = (
            f'Exploring: goal {self._goal_count} '
            f'({nearest[0]:.1f}, {nearest[1]:.1f})  '
            f'dist={dist:.1f} m  frontiers={len(frontiers)}'
        )
        self.progress_pub.publish(p)

    # ── Frontier detection ─────────────────────────────────────────────

    def _find_frontiers(self, msg: OccupancyGrid):
        """Return list of (wx, wy) cluster centroids in the map frame."""
        w   = msg.info.width
        h   = msg.info.height
        res = msg.info.resolution
        ox  = msg.info.origin.position.x
        oy  = msg.info.origin.position.y

        data = np.array(msg.data, dtype=np.int8).reshape(h, w)
        free    = (data == 0)
        unknown = (data == -1)

        # Frontier cell = free AND has at least one unknown 4-neighbour.
        # np.roll wraps at array edges; this can add a thin strip of false
        # frontiers along the map border, which is harmless (Nav2 will reject
        # goals outside the costmap and waypoint_driver moves to the next one).
        adj_unknown = (
            np.roll(unknown,  1, axis=0) |
            np.roll(unknown, -1, axis=0) |
            np.roll(unknown,  1, axis=1) |
            np.roll(unknown, -1, axis=1)
        )
        frontier_mask = free & adj_unknown

        ys, xs = np.where(frontier_mask)
        if len(xs) == 0:
            return []

        wx = ox + (xs + 0.5) * res
        wy = oy + (ys + 0.5) * res
        points = np.column_stack([wx, wy])

        # Greedy clustering: group frontier cells within CLUSTER_RADIUS into
        # one centroid each.  Converts thousands of individual cells into a
        # handful of candidate goal points.
        clusters = []
        used = np.zeros(len(points), dtype=bool)
        for i in range(len(points)):
            if used[i]:
                continue
            dists = np.linalg.norm(points - points[i], axis=1)
            mask = dists < CLUSTER_RADIUS
            clusters.append(tuple(points[mask].mean(axis=0)))
            used[mask] = True

        return clusters

    # ── Helper ─────────────────────────────────────────────────────────

    def _publish_target(self, x: float, y: float):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp.sec = 0      # 0 = use latest available TF
        pose.header.stamp.nanosec = 0
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        self.target_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = Explorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
