#!/usr/bin/env python3
"""
Orange Cone Detector – Part 2, Task 2.

For each orange traffic cone visible in the OAK-D RGB image:
  • Detects it via HSV colour thresholding + contour analysis
  • Verifies the cone is on the robot's RIGHT side (project requirement)
  • Saves an annotated photograph to disk
  • Publishes the cone's normalised image-plane position so the navigator
    can make heading corrections if the cone drifts left

Publishes:
  cone_detected   (std_msgs/Bool)     → waypoint_navigator / UI
  cone_position   (geometry_msgs/Vector3)
                    x = normalised horizontal offset (+1 = far right, -1 = far left)
                    y = normalised vertical offset   (+1 = bottom)
                    z = contour pixel area (proxy for proximity)
  photo_event     (std_msgs/String)   → path of saved photo

Subscribes:
  /oak/rgb/image_raw  (sensor_msgs/Image)  ← OAK-D camera driver
  /waypoint_status    (std_msgs/String)    ← used to embed WP label in filename
"""

import os
from datetime import datetime

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String


# Orange HSV range tuned for traffic cones in outdoor daylight
ORANGE_LOW = np.array([5,  120, 100], dtype=np.uint8)
ORANGE_HIGH = np.array([25, 255, 255], dtype=np.uint8)

# Morphological kernel for noise cleanup
_KERNEL = np.ones((7, 7), np.uint8)


class ConeDetector(Node):

    def __init__(self):
        super().__init__('cone_detector')

        self.declare_parameter('save_dir', '/tmp/cone_photos')
        self.declare_parameter('min_area', 800)       # px² – ignore tiny blobs
        self.declare_parameter('right_bias', 0.05)    # how far right centre must be

        save_dir = self.get_parameter('save_dir').value
        os.makedirs(save_dir, exist_ok=True)

        self._bridge = CvBridge()
        self._photo_count = 0
        self._current_wp_label = 'WP?'
        self._last_saved_area = 0.0   # avoid saving duplicate frames

        # Publishers
        self._detected_pub = self.create_publisher(Bool, 'cone_detected', 10)
        self._pos_pub = self.create_publisher(Vector3, 'cone_position', 10)
        self._photo_pub = self.create_publisher(String, 'photo_event', 10)

        # Subscribers
        self.create_subscription(Image, '/oak/rgb/image_raw', self._image_cb, 10)
        self.create_subscription(String, 'waypoint_status', self._status_cb, 10)

        self.get_logger().info(
            f'Cone detector ready — photos → {save_dir}'
        )

    # ── callbacks ──────────────────────────────────────────────────────────

    def _status_cb(self, msg: String):
        # Extract WP label from navigator status for photo naming
        text = msg.data
        for token in text.split():
            if token.startswith('WP') or token == 'HOME':
                self._current_wp_label = token
                break

    def _image_cb(self, msg: Image):
        frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w = frame.shape[:2]

        # ── colour detection ────────────────────────────────────────────
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, ORANGE_LOW, ORANGE_HIGH)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _KERNEL)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _KERNEL)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        min_area = self.get_parameter('min_area').value
        right_bias = self.get_parameter('right_bias').value

        best_contour = None
        best_area = 0.0
        for c in contours:
            area = cv2.contourArea(c)
            if area > best_area:
                best_area = area
                best_contour = c

        if best_contour is None or best_area < min_area:
            self._detected_pub.publish(Bool(data=False))
            return

        # ── centroid ────────────────────────────────────────────────────
        M = cv2.moments(best_contour)
        if M['m00'] == 0:
            self._detected_pub.publish(Bool(data=False))
            return

        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        # Normalised position: right=+1, left=-1, down=+1, up=-1
        norm_x = (cx - w / 2) / (w / 2)
        norm_y = (cy - h / 2) / (h / 2)

        # ── right-side check ────────────────────────────────────────────
        on_right = norm_x >= right_bias
        side_str = 'RIGHT' if on_right else 'LEFT ⚠'
        if not on_right:
            self.get_logger().warn(
                f'Cone is on the LEFT (norm_x={norm_x:.2f}) — adjust heading right!'
            )

        # ── publish position ────────────────────────────────────────────
        pos = Vector3(x=float(norm_x), y=float(norm_y), z=float(best_area))
        self._pos_pub.publish(pos)
        self._detected_pub.publish(Bool(data=True))

        # ── photograph (save once per significant area change to avoid spam) ──
        area_change = abs(best_area - self._last_saved_area) / max(self._last_saved_area, 1)
        if area_change > 0.20 or self._last_saved_area == 0:
            self._save_photo(frame, best_contour, cx, cy, norm_x, side_str)
            self._last_saved_area = best_area

    # ── photo saving ───────────────────────────────────────────────────────

    def _save_photo(self, frame, contour, cx, cy, norm_x, side_str):
        annotated = frame.copy()

        # Draw contour outline
        cv2.drawContours(annotated, [contour], -1, (0, 255, 0), 2)
        # Centroid marker
        cv2.circle(annotated, (cx, cy), 8, (0, 0, 255), -1)

        # Overlay text
        label = f'{self._current_wp_label}  |  {side_str}  |  x={norm_x:.2f}'
        cv2.putText(annotated, label, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(annotated, label, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 1, cv2.LINE_AA)

        # Vertical guide line at image centre
        h = annotated.shape[0]
        cv2.line(annotated, (annotated.shape[1] // 2, 0),
                 (annotated.shape[1] // 2, h), (200, 200, 0), 1)

        save_dir = self.get_parameter('save_dir').value
        ts = datetime.now().strftime('%H%M%S_%f')[:10]
        filename = os.path.join(
            save_dir,
            f'cone_{self._photo_count:04d}_{self._current_wp_label}_{ts}.jpg'
        )
        cv2.imwrite(filename, annotated)
        self._photo_count += 1

        msg = String()
        msg.data = filename
        self._photo_pub.publish(msg)
        self.get_logger().info(f'Photo saved: {filename}  [{side_str}]')


def main(args=None):
    rclpy.init(args=args)
    node = ConeDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
