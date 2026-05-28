#!/usr/bin/env python3
"""
explorer_node.py  –  Phase 1 autonomous mapping controller.

Three-stage pipeline:

  Stage 1 – PERIMETER
    Robot drives around the outer boundary of the 15x15 m area so SLAM
    gets the full border mapped immediately.  Uses NavigateToPose exactly
    like waypoint_driver.  Corners are slightly inset (INSET metres) so
    Nav2 doesn't try to reach a point right at the costmap edge.

  Stage 2 – EXPLORE
    Hands off to explore_lite for interior coverage.
    Monitors /explore/status for "stopped" / "traversed".
    Hard timeout (EXPLORE_TIMEOUT_S) forces return home if explore_lite
    runs too long.

  Stage 3 – RETURN HOME
    Sends NavigateToPose to the recorded home position (wherever the robot
    was when MAPPING started – nominally (0, 0)).
    Publishes /explorer/done when complete.

Obstacle avoidance
    Fully handled by Nav2 (costmap + RegulatedPurePursuit controller).
    This node only sends goals – Nav2 decides how to reach them safely.

Topics subscribed
  /mission/state      (std_msgs/String)
  /explore/status     (std_msgs/String)   from explore_lite

Topics published
  /explorer/done      (std_msgs/Bool)     True when home reached
  /explorer/stage     (std_msgs/String)   PERIMETER/EXPLORE/RETURN/DONE
  /explore/resume     (std_msgs/Bool)     tells explore_lite to run
  /explore/pause      (std_msgs/Bool)     tells explore_lite to pause

Action client
  /navigate_to_pose   (nav2_msgs/NavigateToPose)
"""

import math
import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from tf2_ros import Buffer, TransformListener


# ---------------------------------------------------------------------------
# Perimeter parameters
# ---------------------------------------------------------------------------

# Half-width of the area in metres.  Robot starts at centre (0,0).
# 15m area → 7.5m half-width.  We inset by INSET so Nav2 doesn't
# try to reach a point right at the costmap boundary.
AREA_HALF = 7.5
INSET     = 1.0          # metres inside the boundary for corner goals
EDGE      = AREA_HALF - INSET   # = 6.5 m from centre to corner goals

# Perimeter corners in order (counter-clockwise starting from front-right).
# Robot drives to each corner then along each side.
# We add intermediate points on long sides so SLAM gets even coverage.
# Each tuple is (x, y).
def _build_perimeter():
    e = EDGE
    # Corners: NE, NW, SW, SE, back to NE to close the loop
    corners = [
        ( e,  e),   # NE
        (-e,  e),   # NW
        (-e, -e),   # SW
        ( e, -e),   # SE
        ( e,  e),   # close loop back to NE
    ]
    # Insert midpoints on each side for better SLAM coverage
    waypoints = []
    for i in range(len(corners) - 1):
        x0, y0 = corners[i]
        x1, y1 = corners[i + 1]
        waypoints.append((x0, y0))
        # Midpoint
        waypoints.append(((x0 + x1) / 2, (y0 + y1) / 2))
    waypoints.append(corners[-1])   # final close point
    # Return to centre so explore_lite starts from middle
    waypoints.append((0.0, 0.0))
    return waypoints

PERIMETER_WAYPOINTS = _build_perimeter()

# Per-waypoint Nav2 timeout during perimeter drive
PERIMETER_GOAL_TIMEOUT_S = 60.0

# How long to let explore_lite run before forcing return home
EXPLORE_TIMEOUT_S = 240.0   # 4 minutes


# ---------------------------------------------------------------------------

class ExplorerNode(Node):

    def __init__(self):
        super().__init__('explorer_node')

        self._mission       = 'IDLE'
        self._stage         = 'IDLE'
        self._home          = (0.0, 0.0)
        self._goal_handle   = None
        self._goal_active   = False
        self._wp_index      = 0
        self._explore_done  = False
        self._explore_timer = None
        self._wp_timer      = None

        # TF
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Nav2 action client – identical to waypoint_driver
        self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Publishers
        self._done_pub   = self.create_publisher(Bool,   '/explorer/done',  10)
        self._stage_pub  = self.create_publisher(String, '/explorer/stage', 10)
        self._resume_pub = self.create_publisher(Bool,   '/explore/resume', 10)
        self._pause_pub  = self.create_publisher(Bool,   '/explore/pause',  10)

        # Subscribers
        self.create_subscription(String, '/mission/state',  self._mission_cb, 10)
        self.create_subscription(String, '/explore/status', self._explore_cb, 10)

        # 1 Hz tick
        self.create_timer(1.0, self._tick)

        self.get_logger().info(
            f'ExplorerNode ready – {len(PERIMETER_WAYPOINTS)} perimeter waypoints '
            f'(±{EDGE}m, inset {INSET}m from boundary)'
        )

    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------

    def _mission_cb(self, msg: String):
        prev = self._mission
        self._mission = msg.data
        if msg.data == 'MAPPING' and prev != 'MAPPING':
            self._on_start()
        elif msg.data != 'MAPPING' and prev == 'MAPPING':
            self._on_stop()

    def _explore_cb(self, msg: String):
        """Catch explore_lite 'stopped' / 'traversed' signal."""
        text = msg.data.lower()
        if self._stage == 'EXPLORE' and (
            'stopped' in text or 'traversed' in text
        ):
            self.get_logger().info(
                f'explore_lite finished ("{msg.data}") – returning home'
            )
            self._explore_done = True
            self._cancel_explore_timeout()

    # -----------------------------------------------------------------------
    # Start / stop
    # -----------------------------------------------------------------------

    def _on_start(self):
        self._home         = self._robot_position() or (0.0, 0.0)
        self._wp_index     = 0
        self._goal_active  = False
        self._explore_done = False
        self.get_logger().info(
            f'MAPPING started – home={self._home}  '
            f'beginning perimeter drive ({len(PERIMETER_WAYPOINTS)} waypoints)'
        )
        self._pause_explore()
        self._set_stage('PERIMETER')

    def _on_stop(self):
        self.get_logger().info('Left MAPPING – stopping explorer')
        self._cancel_current_goal()
        self._cancel_explore_timeout()
        self._cancel_wp_timer()
        self._pause_explore()
        self._set_stage('IDLE')

    def _set_stage(self, stage: str):
        self._stage = stage
        msg = String()
        msg.data = stage
        self._stage_pub.publish(msg)
        self.get_logger().info(f'Explorer stage → {stage}')

    # -----------------------------------------------------------------------
    # Main tick
    # -----------------------------------------------------------------------

    def _tick(self):
        if self._mission != 'MAPPING':
            return

        if self._stage == 'PERIMETER' and not self._goal_active:
            self._send_perimeter_waypoint()

        elif self._stage == 'EXPLORE' and self._explore_done:
            self._set_stage('RETURN')
            self._pause_explore()
            self._send_home()

    # -----------------------------------------------------------------------
    # Stage 1 – Perimeter drive
    # -----------------------------------------------------------------------

    def _send_perimeter_waypoint(self):
        if self._wp_index >= len(PERIMETER_WAYPOINTS):
            self.get_logger().info(
                'Perimeter complete – handing off to explore_lite'
            )
            self._set_stage('EXPLORE')
            self._resume_explore()
            self._start_explore_timeout()
            return

        x, y = PERIMETER_WAYPOINTS[self._wp_index]
        total = len(PERIMETER_WAYPOINTS)
        self.get_logger().info(
            f'Perimeter waypoint {self._wp_index + 1}/{total}: ({x:.1f}, {y:.1f})'
        )

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Nav2 not ready – retrying next tick')
            return

        goal = NavigateToPose.Goal()
        goal.pose = _make_pose(x, y)

        self._goal_active = True
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._perimeter_accepted_cb)

        # Per-waypoint timeout watchdog
        self._cancel_wp_timer()
        self._wp_timer = self.create_timer(
            PERIMETER_GOAL_TIMEOUT_S, self._perimeter_timeout_cb
        )

    def _perimeter_accepted_cb(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle or not self._goal_handle.accepted:
            self.get_logger().warn(
                f'Perimeter waypoint {self._wp_index + 1} rejected – skipping'
            )
            self._cancel_wp_timer()
            self._wp_index   += 1
            self._goal_active = False
            return
        self._goal_handle.get_result_async().add_done_callback(
            self._perimeter_result_cb
        )

    def _perimeter_result_cb(self, future):
        result = future.result()
        status = result.status if result else None
        self._cancel_wp_timer()

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'Perimeter waypoint {self._wp_index + 1} reached ✓'
            )
        else:
            self.get_logger().warn(
                f'Perimeter waypoint {self._wp_index + 1} status={status} – skipping'
            )

        self._wp_index   += 1
        self._goal_active = False
        self._goal_handle = None

    def _perimeter_timeout_cb(self):
        if not self._goal_active:
            return
        self.get_logger().warn(
            f'Perimeter waypoint {self._wp_index + 1} timed out '
            f'({PERIMETER_GOAL_TIMEOUT_S}s) – skipping'
        )
        self._cancel_current_goal()
        self._cancel_wp_timer()
        self._wp_index   += 1
        self._goal_active = False

    # -----------------------------------------------------------------------
    # Stage 2 – explore_lite control
    # -----------------------------------------------------------------------

    def _resume_explore(self):
        msg = Bool()
        msg.data = True
        self._resume_pub.publish(msg)
        self.get_logger().info('explore_lite resumed')

    def _pause_explore(self):
        msg = Bool()
        msg.data = True
        self._pause_pub.publish(msg)

    def _start_explore_timeout(self):
        self._cancel_explore_timeout()
        self._explore_timer = self.create_timer(
            EXPLORE_TIMEOUT_S, self._explore_timeout_cb
        )
        self.get_logger().info(
            f'explore_lite running – timeout in {EXPLORE_TIMEOUT_S:.0f}s'
        )

    def _cancel_explore_timeout(self):
        if self._explore_timer:
            self._explore_timer.cancel()
            self._explore_timer = None

    def _explore_timeout_cb(self):
        if self._stage != 'EXPLORE':
            return
        self.get_logger().warn(
            f'explore_lite timeout ({EXPLORE_TIMEOUT_S:.0f}s) – forcing return home'
        )
        self._explore_done = True

    # -----------------------------------------------------------------------
    # Stage 3 – Return home (same as waypoint_driver._send_home)
    # -----------------------------------------------------------------------

    def _send_home(self):
        hx, hy = self._home
        self.get_logger().info(f'Returning to home ({hx:.2f}, {hy:.2f})')

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 not available for return home')
            return

        goal = NavigateToPose.Goal()
        goal.pose = _make_pose(hx, hy)
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._home_accepted_cb)

    def _home_accepted_cb(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle or not self._goal_handle.accepted:
            self.get_logger().error('Return home goal rejected')
            self._set_stage('DONE')
            return
        self.get_logger().info('Return home accepted – driving to (0, 0)')
        self._goal_handle.get_result_async().add_done_callback(
            self._home_result_cb
        )

    def _home_result_cb(self, future):
        result = future.result()
        status = result.status if result else None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                'Returned to home – Phase 1 mapping complete ✓'
            )
        else:
            self.get_logger().warn(
                f'Return home ended with status {status}'
            )
        self._set_stage('DONE')
        msg = Bool()
        msg.data = True
        self._done_pub.publish(msg)

    # -----------------------------------------------------------------------
    # Helpers
    # -----------------------------------------------------------------------

    def _cancel_current_goal(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        self._goal_active = False

    def _cancel_wp_timer(self):
        if self._wp_timer:
            self._wp_timer.cancel()
            self._wp_timer = None

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

def _make_pose(x: float, y: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id    = 'map'
    pose.pose.position.x    = float(x)
    pose.pose.position.y    = float(y)
    pose.pose.position.z    = 0.0
    pose.pose.orientation.w = 1.0
    return pose


# ---------------------------------------------------------------------------

def main():
    rclpy.init()
    node = ExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
