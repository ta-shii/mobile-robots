#!/usr/bin/env python3
"""
waypoint_driver.py  –  Part 3, Task 2 (Phase 2: RAPID_NAV).

Reads up to 3 detected Greek-letter waypoints from a JSON file saved by
marker_detector, computes the shortest visit order, then drives the robot
to each location via Nav2 NavigateToPose.

You can edit the waypoints file manually for testing while Greek-letter
recognition is being worked on:

  /workspace/autonomous_nav/outputs/markers/waypoints.json

  Minimum required fields per entry:
    {"letter": "A", "greek_name": "Alpha", "x": 1.2, "y": -0.4}

Shortest-path ordering
----------------------
  With up to 3 waypoints all possible visit orders (at most 3! = 6) are
  tried and the one with the shortest total Euclidean distance from the
  robot's current map position is chosen.  Nav2 then handles the real
  obstacle-avoiding A* path for each individual leg.

Behaviour
---------
  Activates automatically when /mission/state becomes RAPID_NAV.
  Cancels any active goal and resets if the state leaves RAPID_NAV.
  Visits waypoints one by one; on failure logs a clear message and
  continues to the next waypoint rather than stopping entirely.

Topics subscribed
  /mission/state  (std_msgs/String)

Action client
  /navigate_to_pose  (nav2_msgs/NavigateToPose)
"""

import itertools
import json
import math
import os

import rclpy
import rclpy.duration
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Int32, String
from tf2_ros import Buffer, TransformListener

MAX_WAYPOINTS  = 3
DEFAULT_WP_FILE = '/workspace/autonomous_nav/outputs/markers/waypoints.json'


class WaypointDriver(Node):

    def __init__(self):
        super().__init__('waypoint_driver')

        self.declare_parameter('waypoints_file', DEFAULT_WP_FILE)
        self._wp_file = self.get_parameter('waypoints_file').value

        self._state       = 'IDLE'
        self._active      = False
        self._waypoints   = []     # ordered list to visit (computed on RAPID_NAV entry)
        self._current_idx = 0
        self._goal_handle = None
        self._home_pos    = None   # robot position recorded on RAPID_NAV entry

        # Publisher – current waypoint index for display_node
        self._index_pub = self.create_publisher(Int32, '/waypoint_driver/index', 10)

        # TF – to read robot position in map frame for path ordering
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Nav2 action client
        self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.create_subscription(String, '/mission/state', self._state_cb, 10)

        self.get_logger().info(
            f'WaypointDriver ready.  Waypoints file: {self._wp_file}'
        )

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------

    def _state_cb(self, msg: String):
        prev        = self._state
        self._state = msg.data

        if self._state == 'RAPID_NAV' and prev != 'RAPID_NAV':
            self._on_enter_rapid_nav()
        elif self._state != 'RAPID_NAV' and prev == 'RAPID_NAV':
            self._on_leave_rapid_nav()

    def _on_enter_rapid_nav(self):
        self.get_logger().info('Entering RAPID_NAV – loading waypoints.')
        self._active      = True
        self._current_idx = 0
        self._goal_handle = None
        self._home_pos    = self._robot_position()

        waypoints = self._load_waypoints()
        if not waypoints:
            self.get_logger().error(
                f'No waypoints in {self._wp_file}. '
                'Nothing to navigate to.  '
                'Add entries manually for testing, then restart RAPID_NAV.'
            )
            return

        robot_pos = self._robot_position()
        if robot_pos is None:
            self.get_logger().warn(
                'TF map→base_link not ready – using (0, 0) as start for ordering.'
            )
            robot_pos = (0.0, 0.0)

        self._waypoints = _optimal_order(waypoints, robot_pos)

        names = [
            f"{w['greek_name']} ({w['letter']})  ({w['x']:.2f}, {w['y']:.2f})"
            for w in self._waypoints
        ]
        self.get_logger().info(
            f'Visiting {len(self._waypoints)} waypoint(s) in optimised order:\n  '
            + '\n  '.join(f'{i+1}. {n}' for i, n in enumerate(names))
        )

        self._send_next_goal()

    def _on_leave_rapid_nav(self):
        self.get_logger().info('Leaving RAPID_NAV – cancelling any active Nav2 goal.')
        self._active = False
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

    # ------------------------------------------------------------------
    # Navigation
    # ------------------------------------------------------------------

    def _send_next_goal(self):
        if not self._active:
            return

        if self._current_idx >= len(self._waypoints):
            if self._home_pos is not None:
                self.get_logger().info(
                    f'All {len(self._waypoints)} waypoint(s) visited.  '
                    f'Returning to home ({self._home_pos[0]:.2f}, {self._home_pos[1]:.2f}).'
                )
                self._send_home()
            else:
                self.get_logger().info(
                    f'All {len(self._waypoints)} waypoint(s) visited.  RAPID_NAV complete.'
                )
            return

        wp = self._waypoints[self._current_idx]
        self.get_logger().info(
            f'[{self._current_idx + 1}/{len(self._waypoints)}] '
            f'Navigating to {wp["greek_name"]} ({wp["letter"]})  '
            f'map=({wp["x"]:.2f}, {wp["y"]:.2f})'
        )

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                '/navigate_to_pose action server not available after 5 s.  '
                'Is Nav2 running?  Cannot continue waypoint navigation.'
            )
            return

        goal = NavigateToPose.Goal()
        goal.pose = _make_pose(wp['x'], wp['y'])

        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        self._goal_handle = future.result()
        wp    = self._waypoints[self._current_idx]
        label = _wp_label(wp)

        if not self._goal_handle.accepted:
            self.get_logger().error(
                f'Goal REJECTED by Nav2 for {label}.  '
                'Possible reasons: goal outside map bounds, Nav2 not fully active, '
                'costmap not ready.  Skipping to next waypoint.'
            )
            self._advance()
            return

        self.get_logger().info(f'Goal accepted – driving to {label}…')
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        status = future.result().status
        wp     = self._waypoints[self._current_idx]
        label  = _wp_label(wp)

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'[{self._current_idx + 1}/{len(self._waypoints)}] '
                f'Reached {label}'
            )
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(
                f'Navigation to {label} was CANCELLED '
                '(operator pressed O or state changed).  Stopping.'
            )
            return   # don't advance – state already changed
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error(
                f'Navigation to {label} ABORTED by Nav2.  '
                'Possible reasons: path blocked, robot stuck, planner timeout.  '
                'Skipping to next waypoint.'
            )
        else:
            self.get_logger().warn(
                f'Navigation to {label} ended with status {status}.  '
                'Skipping to next waypoint.'
            )

        self._advance()

    def _advance(self):
        self._current_idx += 1
        self._goal_handle  = None
        self._index_pub.publish(Int32(data=self._current_idx))
        self._send_next_goal()

    def _send_home(self):
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 not available for return-to-home.')
            return
        home = self._home_pos
        self._home_pos = None   # clear to prevent re-triggering
        goal = NavigateToPose.Goal()
        goal.pose = _make_pose(home[0], home[1])
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._home_response_cb)

    def _home_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Return-to-home goal rejected by Nav2.')
            return
        self.get_logger().info('Returning home – goal accepted.')
        handle.get_result_async().add_done_callback(self._home_result_cb)

    def _home_result_cb(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Returned to home position.  RAPID_NAV complete.')
        else:
            self.get_logger().warn(f'Return-to-home ended with status {status}.')

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _load_waypoints(self):
        if not os.path.exists(self._wp_file):
            self.get_logger().error(
                f'Waypoints file not found: {self._wp_file}  '
                'Create it with at least one entry: '
                '[{"letter":"A","greek_name":"Alpha","x":1.0,"y":2.0}]'
            )
            return []
        try:
            with open(self._wp_file) as f:
                data = json.load(f)
        except Exception as e:
            self.get_logger().error(f'Failed to parse waypoints file: {e}')
            return []
        if not isinstance(data, list) or len(data) == 0:
            self.get_logger().warn('Waypoints file is empty or not a JSON list.')
            return []
        # Filter: must have required fields and a real (non-zero) position
        valid = []
        skipped_zero = 0
        for i, w in enumerate(data):
            if not all(k in w for k in ('letter', 'greek_name', 'x', 'y')):
                self.get_logger().warn(
                    f'Waypoint {i} missing required fields '
                    '(letter, greek_name, x, y) – skipping.'
                )
                continue
            if w['x'] == 0.0 and w['y'] == 0.0:
                skipped_zero += 1
                continue
            valid.append(w)
        if skipped_zero > 0:
            self.get_logger().warn(
                f'Skipped {skipped_zero} entries with position (0, 0) '
                '– these are junk detections where TF was not ready.'
            )
        if len(valid) > MAX_WAYPOINTS:
            self.get_logger().warn(
                f'Found {len(valid)} valid waypoints – using only the first {MAX_WAYPOINTS}.'
            )
            valid = valid[:MAX_WAYPOINTS]
        return valid

    def _robot_position(self):
        try:
            t = self._tf_buffer.lookup_transform(
                'map', 'base_link',
                Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            return None


# ---------------------------------------------------------------------------
# Module-level helpers
# ---------------------------------------------------------------------------

def _dist(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def _optimal_order(waypoints, start):
    """
    Return the visit ordering with the shortest total Euclidean distance
    from `start`.  With ≤3 waypoints this is exact (≤6 permutations).
    """
    if len(waypoints) == 1:
        return list(waypoints)

    best_order = None
    best_dist  = float('inf')

    for perm in itertools.permutations(waypoints):
        pts   = [start] + [(w['x'], w['y']) for w in perm]
        total = sum(_dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))
        if total < best_dist:
            best_dist  = total
            best_order = list(perm)

    return best_order


def _make_pose(x, y):
    """PoseStamped in the map frame at (x, y) with identity orientation."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    pose.pose.orientation.w = 1.0
    return pose


def _wp_label(wp):
    return f'{wp["greek_name"]} ({wp["letter"]})  map=({wp["x"]:.2f}, {wp["y"]:.2f})'


# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = WaypointDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
