#!/usr/bin/env python3
"""
velocity_safety_filter.py  –  Sole publisher of /cmd_vel.

Priority gate (highest → lowest)
---------------------------------
  1. E-stop active OR mission == IDLE  →  zero (robot must not move)
  2. L1 deadman held on gamepad        →  manual joy input
     (works in any non-IDLE state so the operator can always grab control)
  3. Mission == MAPPING or RAPID_NAV   →  pass through Nav2 output (/cmd_vel_nav)
  4. Default                           →  zero

Why this design
---------------
  Nav2 (controller_server) is configured with  cmd_vel_topic: cmd_vel_nav
  so it never touches /cmd_vel directly.  All velocity arbitration happens
  here in one place:  safety > human > autonomous.

  safety_monitor publishes /estop/triggered (Bool).  When True this node
  zeroes the output immediately and holds zero until the flag clears.

  After Step 3 (safety_monitor cleanup), safety_monitor will no longer
  publish directly to /cmd_vel – this node becomes the single writer.

DS4 manual control axes / buttons
-----------------------------------
  Left stick  Y  (axis 1)  →  linear.x   (forward / back)
  Right stick X  (axis 3)  →  angular.z  (turn left / right)
  L1             (btn  4)  →  deadman switch (must be held to drive manually)

  Scales match the previous teleop_twist_joy config:
    linear  max  0.3 m/s
    angular max  0.5 rad/s

Nav2 prerequisite
-----------------
  nav2_params.yaml must contain:
    controller_server:
      ros__parameters:
        cmd_vel_topic: cmd_vel_nav

Topics subscribed
  /estop/triggered  (std_msgs/Bool)
  /mission/state    (std_msgs/String)
  /joy              (sensor_msgs/Joy)
  /cmd_vel_nav      (geometry_msgs/Twist)  – from Nav2 controller_server

Topics published
  /cmd_vel          (geometry_msgs/Twist)  – to ariaNode / Gazebo
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String

# Active mission states where autonomous navigation is allowed
_AUTO_STATES = frozenset({'MAPPING', 'RAPID_NAV'})


class VelocitySafetyFilter(Node):

    # DS4 axes and buttons (verified on real controller)
    AXIS_LINEAR   = 1   # left stick Y
    AXIS_ANGULAR  = 3   # right stick X
    BTN_DEADMAN   = 4   # L1

    SCALE_LINEAR  = 0.3   # m/s
    SCALE_ANGULAR = 0.5   # rad/s
    PUBLISH_HZ    = 20

    def __init__(self):
        super().__init__('velocity_safety_filter')

        # Cached inputs
        self._estop    = False
        self._mission  = 'IDLE'
        self._joy      = None     # latest Joy message
        self._nav_vel  = Twist()  # latest Nav2 velocity command

        # Sole /cmd_vel publisher
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Input subscribers
        self.create_subscription(Bool,   '/estop/triggered', self._estop_cb,   10)
        self.create_subscription(String, '/mission/state',   self._mission_cb, 10)
        self.create_subscription(Joy,    '/joy',             self._joy_cb,     10)
        self.create_subscription(Twist,  '/cmd_vel_nav',     self._nav_cb,     10)

        # Control loop
        self.create_timer(1.0 / self.PUBLISH_HZ, self._tick)

        self.get_logger().info('VelocitySafetyFilter ready')

    # -----------------------------------------------------------------------
    # Callbacks – just cache the latest value
    # -----------------------------------------------------------------------

    def _estop_cb(self, msg: Bool):
        if msg.data and not self._estop:
            self.get_logger().warn('E-stop received – zeroing /cmd_vel')
        if not msg.data and self._estop:
            self.get_logger().info('E-stop cleared – resuming normal control')
        self._estop = msg.data

    def _mission_cb(self, msg: String):
        self._mission = msg.data

    def _joy_cb(self, msg: Joy):
        self._joy = msg

    def _nav_cb(self, msg: Twist):
        self._nav_vel = msg

    # -----------------------------------------------------------------------
    # Control loop – runs at PUBLISH_HZ
    # -----------------------------------------------------------------------

    def _tick(self):
        out = Twist()  # default: all zeros

        # ── Priority 1: safety hard stop ───────────────────────────────
        if self._estop or self._mission == 'IDLE':
            self._cmd_pub.publish(out)
            return

        # ── Priority 2: manual deadman override ────────────────────────
        if self._joy is not None:
            n_btn  = len(self._joy.buttons)
            n_axis = len(self._joy.axes)
            deadman_held = (n_btn > self.BTN_DEADMAN and
                            bool(self._joy.buttons[self.BTN_DEADMAN]))
            if deadman_held:
                if n_axis > self.AXIS_LINEAR:
                    out.linear.x  = (self._joy.axes[self.AXIS_LINEAR]
                                     * self.SCALE_LINEAR)
                if n_axis > self.AXIS_ANGULAR:
                    out.angular.z = (self._joy.axes[self.AXIS_ANGULAR]
                                     * self.SCALE_ANGULAR)
                self._cmd_pub.publish(out)
                return

        # ── Priority 3: autonomous Nav2 output ─────────────────────────
        if self._mission in _AUTO_STATES:
            self._cmd_pub.publish(self._nav_vel)
            return

        # ── Default: zero (e.g. MANUAL_MAPPING with deadman released) ──
        self._cmd_pub.publish(out)


# ---------------------------------------------------------------------------

def main():
    rclpy.init()
    node = VelocitySafetyFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
