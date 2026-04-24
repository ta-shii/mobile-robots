#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class GamepadController(Node):
    """
    O button  → MANUAL mode  (drive with left stick)
    X button  → AUTO mode    (waypoint navigator takes over)

    Deadman: L2 or R2 must be held in BOTH modes or robot stops.
    """

    # DS4 button indices (verify with: ros2 topic echo /joy)
    BTN_X       = 0   # Cross  → AUTO
    BTN_O       = 1   # Circle → MANUAL

    # DS4 axis indices
    AXIS_LEFT_Y  = 1   # left stick up/down  → linear.x
    AXIS_RIGHT_X = 0   # right stick left/right → angular.z
    AXIS_L2      = 4   # deadman
    AXIS_R2      = 5   # deadman (either one works)

    # Speed scaling
    MAX_LINEAR  = 0.5   # m/s
    MAX_ANGULAR = 1.0   # rad/s

    def __init__(self):
        super().__init__('gamepad_controller')

        self.mode      = 'MANUAL'
        self.deadman   = False
        self.auto_cmd  = Twist()

        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel',   10)
        self.mode_pub = self.create_publisher(Bool,  '/auto_mode', 10)

        self.create_subscription(Joy,   '/joy',          self.joy_cb,  10)
        self.create_subscription(Twist, '/auto_cmd_vel', self.auto_cb, 10)

        # Failsafe: keep publishing stop if nothing else does
        self.create_timer(0.1, self.tick)

        self.get_logger().info('GamepadController ready — O=MANUAL  X=AUTO')

    def joy_cb(self, msg: Joy):
        axes    = msg.axes
        buttons = msg.buttons

        # --- Mode switching ---
        if buttons[self.BTN_X] and self.mode != 'AUTO':
            self.mode = 'AUTO'
            self.get_logger().info('→ AUTO mode')

        if buttons[self.BTN_O] and self.mode != 'MANUAL':
            self.mode = 'MANUAL'
            self.get_logger().info('→ MANUAL mode')

        self.mode_pub.publish(Bool(data=(self.mode == 'AUTO')))

        # --- Deadman: L2 or R2 pressed (axes go -1.0 when fully pressed) ---
        l2 = axes[self.AXIS_L2] if len(axes) > self.AXIS_L2 else 1.0
        r2 = axes[self.AXIS_R2] if len(axes) > self.AXIS_R2 else 1.0
        self.deadman = (l2 < 0.0 or r2 < 0.0)

        # --- Build command ---
        cmd = Twist()

        if self.deadman:
            if self.mode == 'MANUAL':
                # Drive directly from sticks
                cmd.linear.x  =  axes[self.AXIS_LEFT_Y]  * self.MAX_LINEAR
                cmd.angular.z = -axes[self.AXIS_RIGHT_X]  * self.MAX_ANGULAR
            else:
                # Pass through whatever the waypoint navigator sent
                cmd = self.auto_cmd

        # If deadman not held: cmd stays zero → robot stops
        self.cmd_pub.publish(cmd)

    def auto_cb(self, msg: Twist):
        self.auto_cmd = msg

    def tick(self):
        """Failsafe: if joy messages stop coming, publish stop."""
        if not self.deadman:
            self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = GamepadController()
    try:
        rclpy.spin(node)
    finally:
        node.cmd_pub.publish(Twist())  # stop on shutdown
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    