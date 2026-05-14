#!/usr/bin/env python3
"""
waypoint_driver.py
==================
Bridges our mission logic to Nav2's NavigateToPose action server.

Why Nav2 instead of a custom controller?
  Our original proportional controller drove in a straight line to each target.
  That works in open fields but fails the moment anything is in the way – it has
  no concept of a map, obstacles, or replanning.  Nav2 gives us all of that for
  free: it runs A* on the SLAM costmap to find a path around obstacles, uses a
  real controller (RegulatedPurePursuit) to follow that path, and automatically
  replans if something new appears.  We just tell it "go here" and it handles
  the rest.

How it works
  Nav2 exposes a ROS2 action server called 'navigate_to_pose'.  An action is
  like a service call that can take a long time – you send a goal, you get
  back a stream of feedback while it's running, and eventually a result.

  This node:
    1. Subscribes to /explorer/target_pose (Phase 1) and /phase2/target_pose (Phase 2)
    2. When a new target arrives, sends it as a NavigateToPose action goal to Nav2
    3. Waits asynchronously (non-blocking) for Nav2 to finish
    4. On success: publishes wp_reached so the caller can send the next target
    5. On e-stop: cancels the active Nav2 goal immediately

  Note: Phase 2 uses a different action (FollowWaypoints) handled in path_planner.py.
  This node handles Phase 1 exploration one-waypoint-at-a-time.

Topics subscribed
  /mission/state           (std_msgs/String)
  /explorer/target_pose    (geometry_msgs/PoseStamped)   – Phase 1
  /phase2/target_pose      (geometry_msgs/PoseStamped)   – Phase 2 fallback
  /estop/triggered         (std_msgs/Bool)

Topics published
  /explorer/wp_reached     (std_msgs/Bool)
  /phase2/wp_reached       (std_msgs/Bool)

Action client
  navigate_to_pose         (nav2_msgs/action/NavigateToPose)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


class WaypointDriver(Node):

    def __init__(self):
        super().__init__('waypoint_driver')

        # ReentrantCallbackGroup lets action callbacks run alongside subscriber
        # callbacks without deadlocking the single-threaded executor.
        cb_group = ReentrantCallbackGroup()

        # --- Nav2 action client ---
        # Nav2 must be running before this node sends any goals.
        # The action server is called 'navigate_to_pose' by default in Nav2.
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=cb_group
        )

        # --- State ---
        self.mission_state   = 'IDLE'
        self._goal_handle    = None    # handle to active Nav2 goal (for cancellation)
        self._goal_in_flight = False   # True while Nav2 is executing a goal
        self._estop          = False
        self._current_phase  = None    # 'MAPPING' or 'RAPID_NAV'
        self._pending_goal: PoseStamped | None = None  # goal queued while cancel is in flight

        # --- Publishers ---
        self.wp_reached_pub  = self.create_publisher(Bool, '/explorer/wp_reached', 10)
        self.p2_reached_pub  = self.create_publisher(Bool, '/phase2/wp_reached',   10)

        # --- Subscribers ---
        self.create_subscription(String,      '/mission/state',        self._state_cb,   10)
        self.create_subscription(PoseStamped, '/explorer/target_pose', self._p1_target_cb, 10)
        self.create_subscription(PoseStamped, '/phase2/target_pose',   self._p2_target_cb, 10)
        self.create_subscription(Bool,        '/estop/triggered',      self._estop_cb,   10)

        self.get_logger().info('WaypointDriver ready – waiting for Nav2 action server...')

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------
    def _state_cb(self, msg: String):
        self.mission_state = msg.data
        if msg.data == 'IDLE':
            self._cancel_active_goal()

    def _p1_target_cb(self, msg: PoseStamped):
        self._current_phase = 'MAPPING'
        self._request_nav(msg)

    def _p2_target_cb(self, msg: PoseStamped):
        self._current_phase = 'RAPID_NAV'
        self._request_nav(msg)

    def _estop_cb(self, msg: Bool):
        self._estop = msg.data
        if msg.data:
            self._cancel_active_goal()

    # ------------------------------------------------------------------
    # Navigation
    # ------------------------------------------------------------------
    def _request_nav(self, pose: PoseStamped):
        if self._estop:
            return
        if self.mission_state not in ('MAPPING', 'RAPID_NAV'):
            return
        if self._goal_in_flight:
            # Already navigating — queue the new goal and cancel the current one.
            # _send_goal will be called once the cancel is acknowledged, avoiding
            # the race condition where Nav2 receives cancel + new goal simultaneously.
            self._pending_goal = pose
            self._cancel_active_goal()
            return

        self._send_goal(pose)

    def _send_goal(self, pose: PoseStamped):
        if not self._nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn(
                'Nav2 navigate_to_pose server not available – is Nav2 running?'
            )
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(
            f'Sending Nav2 goal: ({pose.pose.position.x:.2f}, '
            f'{pose.pose.position.y:.2f})'
        )

        # send_goal_async is non-blocking.  It returns a future that completes
        # when Nav2 either accepts or rejects the goal.
        self._goal_in_flight = True
        send_future = self._nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._on_goal_accepted)

    def _on_goal_accepted(self, future):
        """Called once Nav2 responds to our goal request."""
        self._goal_handle = future.result()

        if not self._goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected the goal')
            self._goal_in_flight = False
            return

        # Nav2 accepted the goal.  Now register a callback for when it finishes.
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future):
        """Called when Nav2 finishes (success, failure, or cancellation)."""
        self._goal_in_flight = False
        self._goal_handle    = None
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Nav2: waypoint reached')
            b = Bool(); b.data = True
            if self._current_phase == 'MAPPING':
                self.wp_reached_pub.publish(b)
            else:
                self.p2_reached_pub.publish(b)

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Nav2: goal cancelled')

        else:
            # Nav2 failed (e.g. no path found, robot stuck).
            # Publish False so the explorer knows this was a failure and can
            # blacklist the goal position — not the same as a successful arrival.
            self.get_logger().warn(f'Nav2 goal failed (status={status}) – skipping waypoint')
            b = Bool(); b.data = False
            if self._current_phase == 'MAPPING':
                self.wp_reached_pub.publish(b)
            else:
                self.p2_reached_pub.publish(b)

    def _cancel_active_goal(self):
        if self._goal_handle is not None:
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._on_cancel_done)
            self._goal_handle    = None
            self._goal_in_flight = False

    def _on_cancel_done(self, future):
        """Called once Nav2 acknowledges the cancel — now safe to send the pending goal."""
        if self._pending_goal is not None:
            pose = self._pending_goal
            self._pending_goal = None
            self._send_goal(pose)


def main():
    rclpy.init()
    node = WaypointDriver()
    # MultiThreadedExecutor needed so action callbacks and topic callbacks
    # can run concurrently without blocking each other.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
