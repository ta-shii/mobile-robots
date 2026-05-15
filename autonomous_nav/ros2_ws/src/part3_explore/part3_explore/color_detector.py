#!/usr/bin/env python3
"""
Object-of-Interest Detector.

Detects any red or orange objects and labels them as objects of interest.
Measures distance via lidar and marks confirmed detections on the robot's
internal map.

External helpers required (not implemented here):
  lidar_helper.py   — lidar_dist(theta: float) -> float
  map_helper.py     — robots_current_position() -> (float, float, float)

Publishes:
  object_detected   (std_msgs/Bool)
  object_position   (geometry_msgs/Vector3)
    x = normalised horizontal offset (+1 = far right, -1 = far left)
    y = normalised vertical offset   (+1 = bottom,   -1 = top)
    z = measured lidar distance (metres)
  photo_event       (std_msgs/String)

Subscribes:
  /oak/rgb/image_raw
"""

import math
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

# ── external helpers ───────────────────────────────────────────────────────────
# lidar_dist(theta)            — defined in lidar_helper.py
#   theta : horizontal angle of the object from the camera's centre axis (degrees)
#   returns: float distance in metres from the robot to the object's centre
#
# robots_current_position()    — defined in map_helper.py
#   returns: 
#            x/y in metres on the robot's internal map frame
#            heading in degrees (0 = map north, clockwise positive)
from part3_explore.lidar_helper import init_lidar_listener, lidar_dist
from part3_explore.map_helper import robots_current_position
# ──────────────────────────────────────────────────────────────────────────────

# Assumed horizontal field of view of the camera in degrees.
# Adjust to match your lens/sensor specification.
CAMERA_HFOV_DEG = 70.0


class ObjectOfInterestDetector(Node):

    def __init__(self):
        super().__init__('object_of_interest_detector')

        self.declare_parameter('save_dir', '/workspace/autonomous_nav/outputs')
        self.declare_parameter('min_area', 1200)
        self.declare_parameter('save_interval_sec', 2.0)
        self.declare_parameter('image_topic', '/oak/rgb/image_raw')

        save_dir = self.get_parameter('save_dir').value
        os.makedirs(save_dir, exist_ok=True)

        self._bridge = CvBridge()
        self._photo_count = 0
        self._last_saved_area = 0.0
        self._last_save_time = self.get_clock().now()

        # Internal map: list of dicts, one entry per confirmed detection.
        # Each entry: { map_x, map_y, colour, distance, timestamp }
        self._internal_map: list[dict] = []

        self._detected_pub = self.create_publisher(Bool,    'object_detected', 10)
        self._pos_pub      = self.create_publisher(Vector3, 'object_position',  10)
        self._photo_pub    = self.create_publisher(String,  'photo_event',      10)

        image_topic = self.get_parameter('image_topic').value
        self.create_subscription(Image, image_topic, self._image_cb, 10)

        self.get_logger().info(f'Object-of-interest detector ready — photos → {save_dir}')
        init_lidar_listener(self)

    # ── callbacks ──────────────────────────────────────────────────────────────

    def _image_cb(self, msg: Image):
        frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w = frame.shape[:2]

        found, contour, centroid, colour_name, annotated = self.detect_object(frame)

        if not found or colour_name not in ('red', 'orange') or contour is None or centroid is None:
            self._detected_pub.publish(Bool(data=False))
            return

        best_area = cv2.contourArea(contour)
        min_area  = int(self.get_parameter('min_area').value)
        if best_area < min_area:
            self._detected_pub.publish(Bool(data=False))
            return

        cx, cy = centroid
        norm_x = (cx - w / 2) / (w / 2)
        norm_y = (cy - h / 2) / (h / 2)

        # Convert normalised horizontal offset to a real angle for lidar lookup.
        # norm_x ∈ [-1, 1]  →  theta ∈ [-HFOV/2, +HFOV/2] degrees
        theta_deg = norm_x * (CAMERA_HFOV_DEG / 2.0)
        distance  = lidar_dist(theta_deg)   # metres — provided by lidar_helper.py

        self._record_on_map(colour_name, theta_deg, distance)

        pos = Vector3(x=float(norm_x), y=float(norm_y), z=float(distance))
        self._pos_pub.publish(pos)
        self._detected_pub.publish(Bool(data=True))

        area_change   = abs(best_area - self._last_saved_area) / max(self._last_saved_area, 1.0)
        save_interval = float(self.get_parameter('save_interval_sec').value)
        now = self.get_clock().now()
        dt  = (now - self._last_save_time).nanoseconds / 1e9

        if (area_change > 0.20 or self._last_saved_area == 0.0) and dt >= save_interval:
            self._save_photo(annotated, contour, cx, cy, norm_x, colour_name, distance)
            self._last_saved_area = best_area
            self._last_save_time  = now

    # ── map recording ──────────────────────────────────────────────────────────

    def _record_on_map(self, colour_name: str, theta_deg: float, distance: float):
        """
        Convert the lidar measurement into map coordinates and store the
        detection in the robot's internal map list.

        robots_current_position() is provided by map_helper.py and returns
        (map_x, map_y, heading_deg) for the robot's current pose.
        """
        robot_x, robot_y, heading_deg = robots_current_position()

        abs_bearing_deg = heading_deg + theta_deg
        abs_bearing_rad = math.radians(abs_bearing_deg)

        obj_map_x = robot_x + distance * math.sin(abs_bearing_rad)
        obj_map_y = robot_y + distance * math.cos(abs_bearing_rad)

        entry = {
            'map_x':     obj_map_x,
            'map_y':     obj_map_y,
            'colour':    colour_name,
            'distance':  distance,
            'timestamp': datetime.now().isoformat(),
        }
        self._internal_map.append(entry)

        self.get_logger().info(
            f'Map updated: object of interest! {colour_name} '
            f'@ ({obj_map_x:.2f} m, {obj_map_y:.2f} m) '
            f'dist={distance:.2f} m'
        )

    # ── photo saving ───────────────────────────────────────────────────────────

    def _save_photo(self, frame, contour, cx, cy, norm_x, colour_name, distance):
        annotated = frame.copy()

        cv2.drawContours(annotated, [contour], -1, (0, 255, 0), 2)
        cv2.circle(annotated, (cx, cy), 8, (0, 0, 255), -1)

        label = f'object of interest! {colour_name} | x={norm_x:.2f} | dist={distance:.2f}m'
        cv2.putText(annotated, label, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(annotated, label, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0),       1, cv2.LINE_AA)

        hh = annotated.shape[0]
        cv2.line(annotated,
                 (annotated.shape[1] // 2, 0),
                 (annotated.shape[1] // 2, hh),
                 (200, 200, 0), 1)

        save_dir = self.get_parameter('save_dir').value
        ts       = datetime.now().strftime('%H%M%S_%f')[:10]
        filename = os.path.join(save_dir, f'object_{self._photo_count:04d}_{ts}.jpg')
        cv2.imwrite(filename, annotated)

        self._photo_count += 1

        out      = String()
        out.data = filename
        self._photo_pub.publish(out)
        self.get_logger().info(f'Photo saved: {filename}')

    # ── detection logic ────────────────────────────────────────────────────────

    def estimate_ground_hsv(self, hsv):
        h, w = hsv.shape[:2]
        strip = hsv[int(h * 0.80):h, int(w * 0.20):int(w * 0.80)]
        valid = strip[(strip[:, :, 1] > 40) & (strip[:, :, 2] > 40)]
        if len(valid) == 0:
            return np.array([60, 100, 100], dtype=np.uint8)
        return np.mean(valid, axis=0).astype(np.uint8)

    def estimate_sky_hsv(self, hsv):
        h, w = hsv.shape[:2]
        strip = hsv[0:int(h * 0.20), int(w * 0.20):int(w * 0.80)]
        valid = strip[strip[:, :, 2] > 100]
        if len(valid) == 0:
            return np.array([100, 50, 200], dtype=np.uint8)
        return np.mean(valid, axis=0).astype(np.uint8)

    def build_mask_around_hsv(self, hsv, mean_hsv, h_tol, s_tol, v_tol):
        h_val, s_val, v_val = [int(x) for x in mean_hsv]
        lower = np.array([max(0,   h_val - h_tol),
                          max(0,   s_val - s_tol),
                          max(0,   v_val - v_tol)], dtype=np.uint8)
        upper = np.array([min(179, h_val + h_tol),
                          min(255, s_val + s_tol),
                          min(255, v_val + v_tol)], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        k    = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        return mask

    def classify_color_from_contour(self, hsv, contour):
        mask   = np.zeros(hsv.shape[:2], dtype=np.uint8)
        cv2.drawContours(mask, [contour], -1, 255, thickness=-1)
        pixels = hsv[mask == 255]
        if len(pixels) == 0:
            return 'unknown'
        mean_h = np.mean(pixels[:, 0])
        mean_s = np.mean(pixels[:, 1])
        mean_v = np.mean(pixels[:, 2])

        if mean_s < 80 or mean_v < 50:
            return 'unknown'
        if mean_h < 10 or mean_h >= 160:
            return 'red'
        if mean_h < 20:
            return 'orange'
        if mean_h < 38:
            return 'yellow'
        if mean_h < 85:
            return 'green'
        if mean_h < 130:
            return 'blue'
        return 'purple'

    def contour_edge_strength(self, edges, contour):
        mask = np.zeros(edges.shape[:2], dtype=np.uint8)
        cv2.drawContours(mask, [contour], -1, 255, thickness=2)
        return float(np.sum(cv2.bitwise_and(edges, edges, mask=mask) > 0))

    def score_contour(self, contour, edge_strength, image_shape):
        h, w  = image_shape[:2]
        area  = cv2.contourArea(contour)
        x, y, cw, ch = cv2.boundingRect(contour)
        cx = x + cw / 2.0
        cy = y + ch / 2.0
        return (
            area
            + 0.5 * cy
            - 0.8 * abs(cx - w / 2.0)
            + 2.0 * edge_strength
            - (500.0  if ch > 0 and cw / ch > 3.0         else 0.0)
            - (600.0  if y < h * 0.15 and cw > w * 0.4    else 0.0)
            - (1000.0 if y + ch >= h - 3 and cw > w * 0.5 else 0.0)
        )

    def detect_object(self, frame):
        annotated = frame.copy()
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ih, iw = frame.shape[:2]

        ground_mask    = self.build_mask_around_hsv(
            hsv, self.estimate_ground_hsv(hsv), h_tol=15, s_tol=80, v_tol=80)
        sky_mask       = self.build_mask_around_hsv(
            hsv, self.estimate_sky_hsv(hsv),    h_tol=15, s_tol=60, v_tol=80)
        candidate_mask = cv2.bitwise_not(cv2.bitwise_or(ground_mask, sky_mask))

        k5    = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        candidate_mask = cv2.morphologyEx(candidate_mask, cv2.MORPH_OPEN,  k5)
        candidate_mask = cv2.morphologyEx(candidate_mask, cv2.MORPH_CLOSE, k5)

        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges   = cv2.Canny(blurred, 50, 150)
        edges   = cv2.bitwise_and(edges, edges, mask=candidate_mask)
        edges   = cv2.dilate(edges,
                             cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)),
                             iterations=1)

        combined = cv2.bitwise_or(candidate_mask, edges)
        combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN,  k5)
        combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, k5)

        contours, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best       = None
        best_score = -1e9
        for c in contours:
            if cv2.contourArea(c) < 400:
                continue
            x, y, w, h = cv2.boundingRect(c)
            if w < 15 or h < 15:
                continue
            s = self.score_contour(c, self.contour_edge_strength(edges, c), frame.shape)
            if s > best_score:
                best_score = s
                best = c

        if best is None:
            return False, None, None, 'unknown', annotated

        best_fill = np.zeros((ih, iw), dtype=np.uint8)
        cv2.drawContours(best_fill, [best], -1, 255, thickness=-1)
        ys, xs = np.where(best_fill == 255)

        if len(xs) > 0:
            rng     = np.random.default_rng(0)
            idx     = rng.choice(len(xs), size=min(200, len(xs)), replace=False)
            med_hsv = np.median(hsv[ys[idx], xs[idx]].astype(float), axis=0)
        else:
            M   = cv2.moments(best)
            cx_ = int(np.clip(M['m10'] / M['m00'], 0, iw - 1)) if M['m00'] > 0 else iw // 2
            cy_ = int(np.clip(M['m01'] / M['m00'], 0, ih - 1)) if M['m00'] > 0 else ih // 2
            med_hsv = hsv[cy_, cx_].astype(float)

        lower = np.array([max(0,   med_hsv[0] - 18),
                          max(0,   med_hsv[1] - 80),
                          max(0,   med_hsv[2] - 90)], dtype=np.uint8)
        upper = np.array([min(179, med_hsv[0] + 18),
                          min(255, med_hsv[1] + 80),
                          min(255, med_hsv[2] + 90)], dtype=np.uint8)

        colour_mask = cv2.inRange(hsv, lower, upper)
        colour_mask = cv2.bitwise_and(colour_mask, colour_mask, mask=candidate_mask)

        ck          = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        colour_mask = cv2.morphologyEx(colour_mask, cv2.MORPH_CLOSE, ck)

        M      = cv2.moments(best)
        seed_x = int(np.clip(M['m10'] / M['m00'], 0, iw - 1)) if M['m00'] > 0 else iw // 2
        seed_y = int(np.clip(M['m01'] / M['m00'], 0, ih - 1)) if M['m00'] > 0 else ih // 2

        n_labels, labels = cv2.connectedComponents(colour_mask)
        seed_label       = labels[seed_y, seed_x]

        if seed_label == 0:
            best_lbl, best_dist = -1, float('inf')
            for lbl in range(1, n_labels):
                lbl_mask = (labels == lbl).astype(np.uint8)
                if np.sum(lbl_mask) < 300:
                    continue
                lys, lxs = np.where(lbl_mask)
                d = (np.mean(lxs) - seed_x) ** 2 + (np.mean(lys) - seed_y) ** 2
                if d < best_dist:
                    best_dist = d
                    best_lbl  = lbl
            clean_mask = ((labels == best_lbl).astype(np.uint8) * 255) if best_lbl != -1 else colour_mask
        else:
            clean_mask = ((labels == seed_label).astype(np.uint8) * 255)

        clean_mask      = cv2.morphologyEx(clean_mask, cv2.MORPH_CLOSE, ck)
        obj_contours, _ = cv2.findContours(clean_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        clean_contour   = max(obj_contours, key=cv2.contourArea) if obj_contours else best

        colour_name = self.classify_color_from_contour(hsv, clean_contour)

        if colour_name in ('red', 'orange'):
            cv2.drawContours(annotated, [clean_contour], -1, (0, 255, 0), 2)

            x, y, w, h = cv2.boundingRect(clean_contour)
            display_label = f'object of interest! {colour_name}'
            font, fs, th  = cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2
            (tw, thh), _  = cv2.getTextSize(display_label, font, fs, th)
            tx = max(5, min(x, iw - tw - 5))
            ty = y - 10 if y - 10 > thh + 5 else min(ih - 5, y + h + thh + 10)
            cv2.putText(annotated, display_label, (tx, ty), font, fs, (0, 255, 0), th)

        M2 = cv2.moments(clean_contour)
        if M2['m00'] > 0:
            cx = int(M2['m10'] / M2['m00'])
            cy = int(M2['m01'] / M2['m00'])
        else:
            x, y, w, h = cv2.boundingRect(clean_contour)
            cx, cy = x + w // 2, y + h // 2
        cv2.circle(annotated, (cx, cy), 5, (0, 200, 255), -1)

        return True, clean_contour, (cx, cy), colour_name, annotated


def main(args=None):
    rclpy.init(args=args)
    node = ObjectOfInterestDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()