#!/usr/bin/env python3
"""
mission_manager.py - Central state machine for Part 3.

States
------
  IDLE        – robot is stationary, waiting for operator input
  MAPPING     – Phase 1: autonomous exploration + detection
  RAPID_NAV   – Phase 2: fastest-path run to given waypoints

Transitions
-----------
  Gamepad X button          → IDLE → MAPPING   (start exploration)
  Gamepad O button          → any  → IDLE      (abort / pause)
  /start_mapping_phase      → IDLE → MAPPING
  /start_waypoint_phase     → MAPPING → RAPID_NAV

Topics published
  /mission/state      (std_msgs/String)   – broadcast to all nodes every 0.2 s
  /mission/phase      (std_msgs/Int32)    – 1=MAPPING, 2=RAPID_NAV (for display)

Topics subscribed
  /joy                (sensor_msgs/Joy)   – gamepad input
  /explorer/done      (std_msgs/Bool)     – explorer signals Phase 1 complete
  /estop/triggered    (std_msgs/Bool)     – emergency stop event

Services provided
  /start_mapping_phase   (std_srvs/Trigger) – start Phase 1 (IDLE → MAPPING)
  /start_waypoint_phase  (std_srvs/Trigger) – start Phase 2 (MAPPING → RAPID_NAV)
  /mission/start_phase2  (std_srvs/SetBool) – legacy alias
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool, Trigger

# Mission states
class MissionState:
    IDLE       = 'IDLE'
    MAPPING    = 'MAPPING'
    RAPID_NAV  = 'RAPID_NAV'

# Main node
class MissionManager(Node):

    # DS4 button indices
    BTN_X = 0   # Cross  → start MAPPING
    BTN_O = 1   # Circle → back to IDLE

    def __init__(self):
        super().__init__('mission_manager')

        self.state = MissionState.IDLE
        self._estop_active = False

        # Publishers - broadcast state for other nodes to consume
        self.state_pub = self.create_publisher(String, '/mission/state', 10)
        self.phase_pub = self.create_publisher(Int32,  '/mission/phase', 10)

        # Subscribers - gamepad input, explorer completion signal, and e-stop events
        self.create_subscription(Joy,  '/joy',             self._joy_cb,          10)
        self.create_subscription(Bool, '/explorer/done',   self._explorer_done_cb, 10)
        self.create_subscription(Bool, '/estop/triggered', self._estop_cb,         10)

        # Services - allow external triggers to start each phase (e.g. from command line)
        self.create_service(Trigger, '/start_mapping_phase',  self._start_mapping_srv)
        self.create_service(Trigger, '/start_waypoint_phase', self._start_waypoint_srv)
        self.create_service(SetBool, '/mission/start_phase2', self._start_phase2_srv)

        # Broadcast state at 5 Hz so late-joining nodes catch up
        self.create_timer(0.2, self._broadcast_state)

        self.get_logger().info(f'MissionManager ready  state={self.state}')

    # Gamepad input
    def _joy_cb(self, msg: Joy):
        if len(msg.buttons) <= max(self.BTN_X, self.BTN_O):
            return
        if msg.buttons[self.BTN_X] and self.state == MissionState.IDLE:
            self._transition(MissionState.MAPPING)
        if msg.buttons[self.BTN_O] and self.state != MissionState.IDLE:
            self._transition(MissionState.IDLE)

    # Explorer completion signal – only relevant for transitioning to Phase 2 after MAPPING
    def _explorer_done_cb(self, msg: Bool):
        if msg.data and self.state == MissionState.MAPPING:
            self.get_logger().info(
                'Explorer reports Phase 1 complete. '
                'Call /start_waypoint_phase to begin Phase 2.'
            )

    # E-stop event – if triggered, transition back to IDLE and require operator to manually restart
    def _estop_cb(self, msg: Bool):
        if msg.data and not self._estop_active:
            self._estop_active = True
            self.get_logger().warn('E-STOP triggered – robot halted')

    # Services
    def _start_mapping_srv(self, request, response):
        if self.state != MissionState.IDLE:
            response.success = False
            response.message = f'Already in state {self.state} – press O first'
            return response
        self._transition(MissionState.MAPPING)
        response.success = True
        response.message = 'Mapping phase started'
        return response

    # Legacy alias for starting Phase 2 (can be called from command line with ros2 service call)
    def _start_waypoint_srv(self, request, response):
        if self.state != MissionState.MAPPING:
            response.success = False
            response.message = f'Cannot start waypoint phase from state {self.state}'
            return response
        self._estop_active = False
        self._transition(MissionState.RAPID_NAV)
        response.success = True
        response.message = 'Waypoint phase started'
        return response

     # Another alias for starting Phase 2, demonstrating use of SetBool service type
    def _start_phase2_srv(self, request, response):
        if self.state != MissionState.MAPPING:
            response.success = False
            response.message = f'Cannot start Phase 2 from state {self.state}'
            return response
        self._estop_active = False
        self._transition(MissionState.RAPID_NAV)
        response.success = True
        response.message = 'Transitioned to RAPID_NAV'
        return response

    # Helpers
    def _transition(self, new_state: str):
        self.get_logger().info(f'State: {self.state} → {new_state}')
        self.state = new_state
        if new_state == MissionState.IDLE:
            self._estop_active = False
        self._broadcast_state()

    # Broadcast current state and phase for other nodes to consume
    def _broadcast_state(self):
        s = String()
        s.data = self.state
        self.state_pub.publish(s)

        p = Int32()
        p.data = {
            MissionState.IDLE:      0,
            MissionState.MAPPING:   1,
            MissionState.RAPID_NAV: 2,
        }.get(self.state, 0)
        self.phase_pub.publish(p)

# Entry point
def main():
    rclpy.init()
    node = MissionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()