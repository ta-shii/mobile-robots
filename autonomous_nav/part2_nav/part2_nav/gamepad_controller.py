#!/usr/bin/env python3
"""
Gamepad controller / command mux for Pioneer 3-AT.

Modes (toggled via DualShock IV buttons):
  X (btn 0) → AUTO  – navigator commands pass through ONLY while a back
                       trigger (L2 or R2) is held as a dead-man switch.
  O (btn 1) → MANUAL – left stick Y = forward/back, right stick X = rotate.

Publishes:
  cmd_vel        (geometry_msgs/Twist)  → ariaNode
  auto_mode      (std_msgs/Bool)        → waypoint_navigator enable signal

Subscribes:
  joy            (sensor_msgs/Joy)      ← joy_node
  auto_cmd_vel   (geometry_msgs/Twist)  ← waypoint_navigator
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

# ── DualShock IV button / axis indices (joy_node default mapping) ──────────
BTN_X = 0       # cross  → enable auto mode
BTN_O = 1       # circle → enable manual mode

# Analog triggers: resting = 1.0, fully pressed = -1.0
AXIS_L2 = 2
AXIS_R2 = 5

# Joystick axes for manual driving
AXIS_FWD = 1    # left stick Y  (forward positive)
AXIS_ROT = 3    # right stick X (left positive)

DEADMAN_THRESH = 0.0   # trigger value below which dead-man is active


class GamepadController(Node):

    def __init__(self):
        super().__init__('gamepad_controller')

        self.declare_parameter('max_linear', 0.3)   # m/s (→ 150 mm/s in ariaNode)
        self.declare_parameter('max_angular', 0.5)  # rad/s (→ 25 deg/s)

        self._auto_mode = False
        self._deadman_ok = False
        self._last_auto_cmd = Twist()

        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._mode_pub = self.create_publisher(Bool, 'auto_mode', 10)

        self.create_subscription(Joy, 'joy', self._joy_cb, 10)
        self.create_subscription(Twist, 'auto_cmd_vel', self._auto_cmd_cb, 10)

        # Safety: publish stop at 10 Hz so robot never coasts if joy drops out
        self.create_timer(0.1, self._safety_tick)

        self.get_logger().info('Gamepad controller ready  |  X = AUTO  |  O = MANUAL')

    # ── callbacks ──────────────────────────────────────────────────────────

    def _joy_cb(self, msg: Joy):
        buttons = msg.buttons
        axes = msg.axes

        # Mode switching (rising-edge only via simple level check – joy publishes
        # continuously at ~50 Hz so we just act on current level, not edge)
        if len(buttons) > BTN_X and buttons[BTN_X] and not self._auto_mode:
            self._auto_mode = True
            self.get_logger().info('Mode → AUTO (hold L2/R2 dead-man to move)')
            self._publish_mode(True)

        if len(buttons) > BTN_O and buttons[BTN_O] and self._auto_mode:
            self._auto_mode = False
            self.get_logger().info('Mode → MANUAL')
            self._publish_mode(False)

        # Dead-man: L2 OR R2 pressed below threshold
        l2 = axes[AXIS_L2] if len(axes) > AXIS_L2 else 1.0
        r2 = axes[AXIS_R2] if len(axes) > AXIS_R2 else 1.0
        self._deadman_ok = (l2 < DEADMAN_THRESH) or (r2 < DEADMAN_THRESH)

        if self._auto_mode:
            cmd = self._last_auto_cmd if self._deadman_ok else Twist()
            self._cmd_pub.publish(cmd)
        else:
            # Manual: proportional to stick position
            max_lin = self.get_parameter('max_linear').value
            max_ang = self.get_parameter('max_angular').value
            twist = Twist()
            if len(axes) > AXIS_FWD:
                twist.linear.x = axes[AXIS_FWD] * max_lin
            if len(axes) > AXIS_ROT:
                twist.angular.z = axes[AXIS_ROT] * max_ang
            self._cmd_pub.publish(twist)

    def _auto_cmd_cb(self, msg: Twist):
        self._last_auto_cmd = msg

    def _safety_tick(self):
        # If in auto mode but dead-man released, keep robot stopped
        if self._auto_mode and not self._deadman_ok:
            self._cmd_pub.publish(Twist())

    # ── helpers ────────────────────────────────────────────────────────────

    def _publish_mode(self, is_auto: bool):
        msg = Bool()
        msg.data = is_auto
        self._mode_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GamepadController()
    try:
        rclpy.spin(node)
    finally:
        node._cmd_pub.publish(Twist())   # ensure stop on shutdown
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
