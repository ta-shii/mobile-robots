#!/usr/bin/env python3
"""
Object Detector — Part 2, Task 4.

At each waypoint the project brief requires:
  • Identify a coloured object's SHAPE  (circle / square / triangle / rectangle)
  • PHOTOGRAPH it
  • Calculate its DISTANCE from the cone marker

Colour detection
────────────────
Detects non-orange coloured objects using HSV thresholding.
Colours supported: red, blue, green, yellow, purple.
Orange is excluded (that is the cone, handled by cone_detector).

Shape classification
────────────────────
Uses OpenCV contour approximation (cv2.approxPolyDP):
  3 vertices  → triangle
  4 vertices  → square / rectangle
  >8 vertices → circle
  other       → unknown

Distance measurement
────────────────────
Uses the OAK-D stereo depth image. At the object's centroid pixel we read
the depth value (metres). This gives distance from the camera to the object.
Distance from the CONE MARKER = distance_to_object (they are co-located
at each waypoint, so this is the intended measurement).

Topics
──────
Subscribes:
  /oak/rgb/image_raw        (sensor_msgs/Image)   ← OAK-D colour camera
  /oak/stereo/image_raw     (sensor_msgs/Image)   ← OAK-D depth (float32, metres)
  wp_index                  (std_msgs/Int32)       ← which waypoint we're at
  waypoint_status           (std_msgs/String)      ← arrival events

Publishes:
  object_detection          (std_msgs/String)      → journey_summary
  photo_event               (std_msgs/String)      → journey_summary (shared)
"""

import os
from datetime import datetime

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String


# ── Colour HSV ranges (H: 0-179, S: 0-255, V: 0-255 in OpenCV) ────────────
COLOURS = {
    'red':    [(  0,  80, 80), ( 10, 255, 255),   # lower red
               (165,  80, 80), (179, 255, 255)],  # upper red (wraps hue)
    'blue':   [(100,  80, 50), (130, 255, 255)],
    'green':  [( 40,  80, 50), ( 85, 255, 255)],
    'yellow': [( 22, 100, 100), ( 35, 255, 255)],
    'purple': [(130,  50, 50), (165, 255, 255)],
}

MIN_AREA = 1500   # px² — ignore noise
_KERNEL = np.ones((5, 5), np.uint8)


def _hsv_mask(hsv: np.ndarray, colour: str) -> np.ndarray:
    """Return binary mask for a given colour name."""
    ranges = COLOURS[colour]
    if colour == 'red':
        # Red wraps around 0°, needs two ranges
        m1 = cv2.inRange(hsv, np.array(ranges[0]), np.array(ranges[1]))
        m2 = cv2.inRange(hsv, np.array(ranges[2]), np.array(ranges[3]))
        mask = cv2.bitwise_or(m1, m2)
    else:
        mask = cv2.inRange(hsv, np.array(ranges[0]), np.array(ranges[1]))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  _KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _KERNEL)
    return mask


def _classify_shape(contour) -> str:
    """Classify contour as circle / triangle / square / rectangle."""
    peri = cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
    n = len(approx)

    if n == 3:
        return 'triangle'
    elif n == 4:
        # Distinguish square vs rectangle by aspect ratio
        x, y, w, h = cv2.boundingRect(approx)
        ratio = w / float(h)
        return 'square' if 0.85 <= ratio <= 1.15 else 'rectangle'
    else:
        # Use circularity: 4π·area / perimeter²
        area = cv2.contourArea(contour)
        if peri > 0:
            circularity = (4 * np.pi * area) / (peri * peri)
            if circularity > 0.75:
                return 'circle'
        return 'polygon'


class ObjectDetector(Node):

    def __init__(self):
        super().__init__('object_detector')

        self.declare_parameter('save_dir', '/tmp/cone_photos')
        self.declare_parameter('scan_on_arrival', True)

        save_dir = self.get_parameter('save_dir').value
        os.makedirs(save_dir, exist_ok=True)

        self._bridge = CvBridge()
        self._photo_count = 0

        # State
        self._current_wp = -1
        self._scanning = False
        self._latest_frame = None
        self._latest_depth = None
        self._gps_lat = 0.0
        self._gps_lon = 0.0

        # Publishers
        self._detect_pub = self.create_publisher(String, 'object_detection', 10)
        self._photo_pub  = self.create_publisher(String, 'photo_event', 10)

        # Subscribers
        self.declare_parameter('rgb_topic', '/oak/rgb/image_raw')
        self.declare_parameter('depth_topic', '/oak/stereo/image_raw')

        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value

        self.create_subscription(Image, rgb_topic, self._rgb_cb, 10)
        self.create_subscription(Image, depth_topic, self._depth_cb, 10)
        self.create_subscription(Int32,  'wp_index',        self._wpidx_cb, 10)
        self.create_subscription(String, 'waypoint_status', self._status_cb, 10)
        from sensor_msgs.msg import NavSatFix
        self.create_subscription(NavSatFix, '/fix', self._gps_cb, 10)

        self.get_logger().info('Object detector ready — watching for coloured shapes.')

    # ── image callbacks ────────────────────────────────────────────────────

    def _gps_cb(self, msg):
        self._gps_lat = msg.latitude
        self._gps_lon = msg.longitude

    def _rgb_cb(self, msg: Image):
        self._latest_frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        if self._scanning and self._latest_frame is not None:
            self._run_detection()
            self._scanning = False   # detect once per waypoint arrival

    def _depth_cb(self, msg: Image):
        # Depth image arrives as 32FC1 (metres) from OAK-D depthai-ros
        try:
            self._latest_depth = self._bridge.imgmsg_to_cv2(msg, '32FC1')
        except Exception:
            self._latest_depth = None

    # ── nav callbacks ───────────────────────────────────────────────────────

    def _wpidx_cb(self, msg: Int32):
        self._current_wp = msg.data

    def _status_cb(self, msg: String):
        """Trigger a scan when the navigator announces waypoint arrival."""
        text = msg.data
        if 'reached' in text.lower() or 'reached' in text:
            self.get_logger().info(f'Waypoint arrival detected — scanning for object.')
            self._scanning = True

    # ── detection ──────────────────────────────────────────────────────────

    def _run_detection(self):
        frame = self._latest_frame
        if frame is None:
            self.get_logger().warn('No camera frame available for object detection.')
            return

        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        best_result = None   # (colour, shape, area, contour, cx, cy)
        best_area   = 0.0

        for colour in COLOURS:
            mask = _hsv_mask(hsv, colour)
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for c in contours:
                area = cv2.contourArea(c)
                if area < MIN_AREA or area <= best_area:
                    continue
                M = cv2.moments(c)
                if M['m00'] == 0:
                    continue
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                shape = _classify_shape(c)
                best_result = (colour, shape, area, c, cx, cy)
                best_area = area

        if best_result is None:
            self.get_logger().warn('No coloured object found at this waypoint.')
            result_str = f'WP{self._current_wp+1}: no object detected'
            self._detect_pub.publish(String(data=result_str))
            return

        colour, shape, area, contour, cx, cy = best_result

        # ── distance from OAK-D depth ──────────────────────────────────
        distance_m = self._get_depth(cx, cy)
        dist_str = f'{distance_m:.2f}m' if distance_m > 0 else 'unknown'

        # ── annotate and save photo ────────────────────────────────────
        annotated = frame.copy()
        cv2.drawContours(annotated, [contour], -1, (0, 255, 0), 2)
        cv2.circle(annotated, (cx, cy), 8, (0, 0, 255), -1)

        label = (f'WP{self._current_wp+1} | {colour} {shape} | '
                 f'dist={dist_str}')
        cv2.putText(annotated, label, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(annotated, label, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 0, 0), 1, cv2.LINE_AA)

        save_dir = self.get_parameter('save_dir').value
        ts = datetime.now().strftime('%H%M%S')
        filename = os.path.join(
            save_dir,
            f'obj_{self._photo_count:04d}_WP{self._current_wp+1}_{colour}_{shape}_{ts}.jpg'
        )
        cv2.imwrite(filename, annotated)
        self._photo_count += 1

        result_str = (f'WP{self._current_wp+1}: {colour} {shape} '
                      f'at {dist_str} from marker  '
                      f'GPS=({self._gps_lat:.6f},{self._gps_lon:.6f})')
        self.get_logger().info(f'Detected: {result_str}')
        self.get_logger().info(f'Photo: {filename}')

        # GPS sidecar file
        gps_file = filename.replace('.jpg', '_gps.txt')
        with open(gps_file, 'w') as f:
            f.write(f'waypoint:  WP{self._current_wp+1}\n')
            f.write(f'colour:    {colour}\n')
            f.write(f'shape:     {shape}\n')
            f.write(f'distance:  {dist_str}\n')
            f.write(f'lat:       {self._gps_lat:.8f}\n')
            f.write(f'lon:       {self._gps_lon:.8f}\n')
            f.write(f'timestamp: {datetime.now().isoformat()}\n')

        self._detect_pub.publish(String(data=result_str))
        self._photo_pub.publish(String(data=filename))

    def _get_depth(self, cx: int, cy: int) -> float:
        """Sample depth at pixel (cx, cy) and surrounding 5×5 region (median)."""
        if self._latest_depth is None:
            return 0.0
        dh, dw = self._latest_depth.shape[:2]
        if cx >= dw or cy >= dh:
            return 0.0
        # Sample a 5×5 patch for robustness
        y1 = max(0, cy - 2);  y2 = min(dh, cy + 3)
        x1 = max(0, cx - 2);  x2 = min(dw, cx + 3)
        patch = self._latest_depth[y1:y2, x1:x2]
        valid = patch[(patch > 0.1) & (patch < 20.0) & np.isfinite(patch)]
        if valid.size == 0:
            return 0.0
        return float(np.median(valid))


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
