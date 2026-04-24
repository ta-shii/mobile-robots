#!/usr/bin/env python3
"""
Cone Weaver — Part 2, Task 3.

Handles the slalom leg between WP1 and WP2 where cones are placed at unknown
intervals. Active ONLY when waypoint_navigator publishes weave_mode=True
(i.e. when the robot is travelling from WP1 → WP2).

How it works
────────────
The waypoint_navigator still handles the FORWARD drive toward WP2 (linear.x).
This node takes over the ANGULAR component to steer around cones.

Strategy — reactive alternating slalom:
  1. Camera (cone_detector) tells us where the nearest cone is in the frame.
  2. We track which side we passed the LAST cone (left or right).
  3. We steer to the OPPOSITE side of each new cone.
     e.g. cone appears on right → steer left to pass it on the left.
  4. After passing (cone disappears or area shrinks back), reset and wait
     for the next cone.
  5. If no cone visible, publish zero angular (let GPS heading take over).

The final published /weave_cmd_vel is picked up by the command mux
(weave_active overrides auto_cmd_vel's angular component only).

Topics
──────
Subscribes:
  weave_mode      (std_msgs/Bool)      ← navigator: True on WP1→WP2 leg
  cone_position   (geometry_msgs/Vector3)  ← cone_detector: x/y/area
  auto_cmd_vel    (geometry_msgs/Twist)    ← navigator: forward speed

Publishes:
  weave_cmd_vel   (geometry_msgs/Twist)   → gamepad_controller mux
  weave_active    (std_msgs/Bool)         → status / UI
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool


# How far the cone must be from centre to trigger a steer (dead-band)
CENTRE_DEADBAND = 0.10   # normalised units (-1 to +1 range)

# When cone area drops below this after being large, we consider it "passed"
PASSED_AREA_RATIO = 0.4   # 40% of peak area → cone is behind us


class ConeWeaver(Node):

    def __init__(self):
        super().__init__('cone_weaver')

        self.declare_parameter('steer_gain', 0.8)       # angular P gain
        self.declare_parameter('max_angular', 0.5)      # rad/s clamp
        self.declare_parameter('min_cone_area', 500.0)  # px² to care about

        self._weave_active = False
        self._last_nav_linear = 0.0    # forward speed from navigator

        # Cone tracking state
        self._cone_visible = False
        self._cone_norm_x = 0.0        # -1=far left, +1=far right
        self._cone_area = 0.0
        self._peak_area = 0.0          # largest area seen for current cone
        self._last_steer = 0.0         # last angular command (direction memory)

        # Publishers
        self._cmd_pub = self.create_publisher(Twist, 'weave_cmd_vel', 10)
        self._active_pub = self.create_publisher(Bool, 'weave_active', 10)

        # Subscribers
        self.create_subscription(Bool, 'weave_mode', self._weave_cb, 10)
        self.create_subscription(Vector3, 'cone_position', self._cone_cb, 10)
        self.create_subscription(Twist, 'auto_cmd_vel', self._nav_cb, 10)

        self.create_timer(0.1, self._control_loop)   # 10 Hz

        self.get_logger().info('Cone weaver ready — waiting for weave_mode=True')

    # ── callbacks ──────────────────────────────────────────────────────────

    def _weave_cb(self, msg: Bool):
        if msg.data and not self._weave_active:
            self.get_logger().info('WEAVE MODE ON — slalom between WP1 and WP2')
            self._peak_area = 0.0
        elif not msg.data and self._weave_active:
            self.get_logger().info('Weave mode OFF — returning to GPS heading')
        self._weave_active = msg.data

    def _cone_cb(self, msg: Vector3):
        """Called by cone_detector when an orange cone is visible."""
        min_area = self.get_parameter('min_cone_area').value
        self._cone_visible = msg.z >= min_area
        self._cone_norm_x = msg.x    # negative=left, positive=right
        self._cone_area = msg.z

        if self._cone_visible and msg.z > self._peak_area:
            self._peak_area = msg.z

    def _nav_cb(self, msg: Twist):
        """Capture the forward speed the GPS navigator wants."""
        self._last_nav_linear = msg.linear.x

    # ── control loop ───────────────────────────────────────────────────────

    def _control_loop(self):
        active_msg = Bool()
        active_msg.data = self._weave_active
        self._active_pub.publish(active_msg)

        if not self._weave_active:
            return

        steer_gain = self.get_parameter('steer_gain').value
        max_ang = self.get_parameter('max_angular').value

        twist = Twist()
        twist.linear.x = self._last_nav_linear   # keep navigator's forward speed

        if self._cone_visible:
            norm_x = self._cone_norm_x

            # Detect if cone has been passed (area shrunk significantly after peak)
            if (self._peak_area > 0
                    and self._cone_area < self._peak_area * PASSED_AREA_RATIO):
                self.get_logger().info(
                    f'Cone passed! (peak={self._peak_area:.0f} → {self._cone_area:.0f})'
                )
                self._peak_area = 0.0
                twist.angular.z = 0.0
            else:
                # Steer AWAY from the cone
                # cone on right (norm_x > 0) → negative angular = steer left
                # cone on left  (norm_x < 0) → positive angular = steer right
                if abs(norm_x) > CENTRE_DEADBAND:
                    angular = -steer_gain * norm_x
                    angular = max(-max_ang, min(max_ang, angular))
                    twist.angular.z = angular
                    self._last_steer = angular

                    side = 'right' if norm_x > 0 else 'left'
                    self.get_logger().debug(
                        f'Cone on {side} (x={norm_x:.2f}) → '
                        f'steering {"left" if angular < 0 else "right"} '
                        f'ang={angular:.2f}'
                    )
                else:
                    twist.angular.z = 0.0   # cone is centred, go straight
        else:
            # No cone visible — straighten up gradually
            twist.angular.z = 0.0

        self._cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ConeWeaver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
