#!/usr/bin/env python3
"""
mission_manager.py  –  Central state machine for Part 3.

States
------
  IDLE           – robot is stationary, waiting for operator input.
  MAPPING        – Phase 1 AUTO: m-explore (explore_lite) drives via Nav2,
                   SLAM Toolbox builds the map.
  MANUAL_MAPPING – Phase 1 MANUAL: operator drives with gamepad, SLAM maps.
                   Use as fallback if autonomous exploration fails – the map
                   produced is identical and feeds Phase 2 the same way.
  RAPID_NAV      – Phase 2: waypoint_driver drives to 3 Greek-letter locations
                   and returns home via Nav2 NavigateToPose.

State transitions
-----------------
  Gamepad X        (btn 0)  IDLE             → MAPPING
  Gamepad △        (btn 3)  IDLE             → MANUAL_MAPPING
  Gamepad □       (btn 2)  MAPPING |
                            MANUAL_MAPPING   → RAPID_NAV
  Gamepad O        (btn 1)  any              → IDLE  (abort)

  /start_mapping_phase   service  IDLE             → MAPPING
  /start_manual_mapping  service  IDLE             → MANUAL_MAPPING
  /start_waypoint_phase  service  MAPPING |
                                  MANUAL_MAPPING   → RAPID_NAV

Nav2 goal management
--------------------
  When MAPPING: m-explore sends NavigateToPose goals to Nav2 freely.
  When entering RAPID_NAV: all active Nav2 goals are cancelled so
    waypoint_driver can take over without fighting m-explore.
  When entering IDLE: all active Nav2 goals are cancelled so the robot
    stops cleanly even if m-explore or waypoint_driver had an active goal.

Topics published
  /mission/state   (std_msgs/String)  – broadcast at 5 Hz to all nodes
  /mission/phase   (std_msgs/Int32)   – 0=IDLE, 1=Phase1, 2=Phase2

Topics subscribed
  /joy             (sensor_msgs/Joy)  – DS4 gamepad
  /estop/triggered (std_msgs/Bool)    – estop events from safety_monitor

Services provided
  /start_mapping_phase   (std_srvs/Trigger) – IDLE → MAPPING
  /start_manual_mapping  (std_srvs/Trigger) – IDLE → MANUAL_MAPPING
  /start_waypoint_phase  (std_srvs/Trigger) – MAPPING|MANUAL_MAPPING → RAPID_NAV
  /mission/start_phase2  (std_srvs/SetBool) – legacy alias
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool, Trigger
from action_msgs.srv import CancelGoal


# ---------------------------------------------------------------------------
# State constants
# ---------------------------------------------------------------------------

class MissionState:
    IDLE           = 'IDLE'
    MAPPING        = 'MAPPING'          # Phase 1 – m-explore drives
    MANUAL_MAPPING = 'MANUAL_MAPPING'   # Phase 1 – operator drives
    RAPID_NAV      = 'RAPID_NAV'        # Phase 2 – waypoint_driver drives

    # Both mapping modes are Phase 1: same SLAM, same map, same Phase 2 entry
    PHASE1_STATES = frozenset({'MAPPING', 'MANUAL_MAPPING'})


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class MissionManager(Node):

    # DS4 button indices  (verified on real controller)
    BTN_X        = 0   # Cross    – IDLE → MAPPING
    BTN_O        = 1   # Circle   – any  → IDLE
    BTN_SQUARE   = 2   # Square   – Phase1 → RAPID_NAV
    BTN_TRIANGLE = 3   # Triangle – IDLE → MANUAL_MAPPING

    def __init__(self):
        super().__init__('mission_manager')

        self.state         = MissionState.IDLE
        self._estop_active = False

        # ── Publishers ──────────────────────────────────────────────────
        self._state_pub = self.create_publisher(String, '/mission/state', 10)
        self._phase_pub = self.create_publisher(Int32,  '/mission/phase', 10)

        # ── Subscribers ─────────────────────────────────────────────────
        self.create_subscription(Joy,  '/joy',             self._joy_cb,    10)
        self.create_subscription(Bool, '/estop/triggered', self._estop_cb,  10)

        # ── Services ────────────────────────────────────────────────────
        self.create_service(Trigger, '/start_mapping_phase',  self._start_mapping_srv)
        self.create_service(Trigger, '/start_manual_mapping', self._start_manual_mapping_srv)
        self.create_service(Trigger, '/start_waypoint_phase', self._start_waypoint_srv)
        self.create_service(SetBool, '/mission/start_phase2', self._start_phase2_srv)  # legacy

        # ── Nav2 goal cancellation ───────────────────────────────────────
        # Used when leaving MAPPING or IDLE so m-explore / waypoint_driver
        # goals don't linger and fight the next phase.
        # The ROS2 action protocol exposes a cancel service on every action
        # server at  /<action_name>/_action/cancel_goal.
        # Sending an empty CancelGoal.Request cancels ALL active goals.
        self._cancel_nav_client = self.create_client(
            CancelGoal,
            '/navigate_to_pose/_action/cancel_goal',
        )

        # ── Broadcast timer ─────────────────────────────────────────────
        # 5 Hz so any node that starts late quickly learns the current state
        self.create_timer(0.2, self._broadcast_state)

        self.get_logger().info(f'MissionManager ready  state={self.state}')

    # -----------------------------------------------------------------------
    # Gamepad
    # -----------------------------------------------------------------------

    def _joy_cb(self, msg: Joy):
        n = len(msg.buttons)

        # Cross – start AUTO mapping (m-explore + Nav2)
        if n > self.BTN_X and msg.buttons[self.BTN_X]:
            if self.state == MissionState.IDLE:
                self._transition(MissionState.MAPPING)

        # Triangle – start MANUAL mapping (gamepad drives, SLAM maps)
        if n > self.BTN_TRIANGLE and msg.buttons[self.BTN_TRIANGLE]:
            if self.state == MissionState.IDLE:
                self._transition(MissionState.MANUAL_MAPPING)

        # Square – move to Phase 2 once mapping is done
        if n > self.BTN_SQUARE and msg.buttons[self.BTN_SQUARE]:
            if self.state in MissionState.PHASE1_STATES:
                self._transition(MissionState.RAPID_NAV)

        # Circle – abort back to IDLE from any active state
        if n > self.BTN_O and msg.buttons[self.BTN_O]:
            if self.state != MissionState.IDLE:
                self._transition(MissionState.IDLE)

    # -----------------------------------------------------------------------
    # E-stop
    # -----------------------------------------------------------------------

    def _estop_cb(self, msg: Bool):
        if msg.data and not self._estop_active:
            self._estop_active = True
            self.get_logger().warn('E-STOP triggered – robot halted')
        elif not msg.data and self._estop_active:
            self._estop_active = False
            self.get_logger().info('E-STOP cleared – robot ready')

    # -----------------------------------------------------------------------
    # Services
    # -----------------------------------------------------------------------

    def _start_mapping_srv(self, request, response):
        """IDLE → MAPPING (m-explore drives autonomously)."""
        if self.state != MissionState.IDLE:
            response.success = False
            response.message = f'Already in state {self.state} – press O to abort first'
            return response
        self._transition(MissionState.MAPPING)
        response.success = True
        response.message = 'AUTO mapping started – m-explore is now exploring'
        return response

    def _start_manual_mapping_srv(self, request, response):
        """IDLE → MANUAL_MAPPING (operator drives, SLAM maps)."""
        if self.state != MissionState.IDLE:
            response.success = False
            response.message = f'Already in state {self.state} – press O to abort first'
            return response
        self._transition(MissionState.MANUAL_MAPPING)
        response.success = True
        response.message = 'MANUAL mapping started – drive with gamepad to build map'
        return response

    def _start_waypoint_srv(self, request, response):
        """MAPPING | MANUAL_MAPPING → RAPID_NAV (Phase 2)."""
        if self.state not in MissionState.PHASE1_STATES:
            response.success = False
            response.message = (
                f'Cannot start Phase 2 from state {self.state}. '
                'Must complete mapping first (MAPPING or MANUAL_MAPPING).'
            )
            return response
        self._estop_active = False
        self._transition(MissionState.RAPID_NAV)
        response.success = True
        response.message = 'Phase 2 started – waypoint_driver is now active'
        return response

    def _start_phase2_srv(self, request, response):
        """Legacy SetBool alias for /start_waypoint_phase."""
        if self.state not in MissionState.PHASE1_STATES:
            response.success = False
            response.message = f'Cannot start Phase 2 from state {self.state}'
            return response
        self._estop_active = False
        self._transition(MissionState.RAPID_NAV)
        response.success = True
        response.message = 'Transitioned to RAPID_NAV'
        return response

    # -----------------------------------------------------------------------
    # Internal helpers
    # -----------------------------------------------------------------------

    def _transition(self, new_state: str):
        prev = self.state
        self.get_logger().info(f'State transition: {prev} → {new_state}')
        self.state = new_state

        if new_state == MissionState.IDLE:
            self._estop_active = False
            # Cancel any Nav2 goal that m-explore or waypoint_driver left running
            self._cancel_all_nav_goals(reason='entering IDLE')

        elif new_state == MissionState.RAPID_NAV:
            # Cancel m-explore's active Nav2 goal so waypoint_driver
            # can take over the navigate_to_pose action server cleanly
            self._cancel_all_nav_goals(reason='handing Nav2 to waypoint_driver')

        self._broadcast_state()

    def _cancel_all_nav_goals(self, reason: str = ''):
        """
        Cancel all active goals on the Nav2 navigate_to_pose action server.
        An empty CancelGoal request = cancel everything.
        Fire-and-forget: we don't wait for the response.
        """
        if not self._cancel_nav_client.service_is_ready():
            self.get_logger().debug(
                'navigate_to_pose cancel service not ready – skipping cancel'
            )
            return
        req = CancelGoal.Request()   # empty → cancel all
        future = self._cancel_nav_client.call_async(req)
        label = f' ({reason})' if reason else ''
        self.get_logger().info(f'Cancelled all Nav2 goals{label}')
        # We intentionally don't await – the state machine must not block

    def _broadcast_state(self):
        s = String()
        s.data = self.state
        self._state_pub.publish(s)

        p = Int32()
        p.data = {
            MissionState.IDLE:           0,
            MissionState.MAPPING:        1,
            MissionState.MANUAL_MAPPING: 1,   # same phase display as MAPPING
            MissionState.RAPID_NAV:      2,
        }.get(self.state, 0)
        self._phase_pub.publish(p)


# ---------------------------------------------------------------------------

def main():
    rclpy.init()
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
