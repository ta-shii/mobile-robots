#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self._active = False
        self._cmd_pub = self.create_publisher(Twist, '/auto_cmd_vel', 10)
        self.create_subscription(Bool, '/auto_mode', self._mode_cb, 10)
        self.create_timer(0.1, self._loop)
        self.get_logger().info('Ready — press X to drive straight')

    def _mode_cb(self, msg: Bool):
        self._active = msg.data

    def _loop(self):
        t = Twist()
        if self._active:
            t.linear.x = 0.5
        self._cmd_pub.publish(t)


def main():
    rclpy.init()
    node = WaypointNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()