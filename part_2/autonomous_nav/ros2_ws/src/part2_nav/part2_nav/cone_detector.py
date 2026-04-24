

#!/usr/bin/env python3
"""
Orange Cone Detector – Part 2, Task 2.

Uses stronger contour + colour + shape logic adapted from the working sim code.

Publishes:
  cone_detected   (std_msgs/Bool)
  cone_position   (geometry_msgs/Vector3)
    x = normalised horizontal offset (+1 = far right, -1 = far left)
    y = normalised vertical offset (+1 = bottom, -1 = top)
    z = contour pixel area (proxy for proximity)
  photo_event     (std_msgs/String)

Subscribes:
  /oak/rgb/image_raw
  /waypoint_status
  /fix
"""

import os
from datetime import datetime

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Bool, String


class ConeDetector(Node):

    def __init__(self):
        super().__init__('cone_detector')

        self.declare_parameter('save_dir', '/workspace/autonomous_nav/outputs')
        self.declare_parameter('min_area', 1200)
        self.declare_parameter('right_bias', 0.05)
        self.declare_parameter('save_interval_sec', 2.0)
        self.declare_parameter('image_topic', '/oak/rgb/image_raw')

        save_dir = self.get_parameter('save_dir').value
        os.makedirs(save_dir, exist_ok=True)

        self._bridge = CvBridge()
        self._photo_count = 0
        self._current_wp_label = 'WP?'
        self._last_saved_area = 0.0
        self._last_save_time = self.get_clock().now()

        self._gps_lat = float('nan')
        self._gps_lon = float('nan')

        self._detected_pub = self.create_publisher(Bool, 'cone_detected', 10)
        self._pos_pub = self.create_publisher(Vector3, 'cone_position', 10)
        self._photo_pub = self.create_publisher(String, 'photo_event', 10)

        image_topic = self.get_parameter('image_topic').value
        self.create_subscription(Image, image_topic, self._image_cb, 10)
        self.create_subscription(String, 'waypoint_status', self._status_cb, 10)
        self.create_subscription(NavSatFix, '/fix', self._gps_cb, 10)

        self.get_logger().info(f'Cone detector ready — photos → {save_dir}')

    # ── callbacks ──────────────────────────────────────────────────────

    def _gps_cb(self, msg: NavSatFix):
        self._gps_lat = msg.latitude
        self._gps_lon = msg.longitude

    def _status_cb(self, msg: String):
        text = msg.data
        for token in text.split():
            if token.startswith('WP') or token == 'HOME':
                self._current_wp_label = token
                break

    def _image_cb(self, msg: Image):
        frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w = frame.shape[:2]

        found, contour, centroid, colour_name, shape_name, annotated = self.detect_object(frame)

        # For real-world cones, allow triangle or rectangle-like views
        valid_cone = found and colour_name == 'orange' and shape_name in ['triangle', 'rectangle']

        if not valid_cone or contour is None or centroid is None:
            self._detected_pub.publish(Bool(data=False))
            return

        best_area = cv2.contourArea(contour)
        min_area = int(self.get_parameter('min_area').value)
        if best_area < min_area:
            self._detected_pub.publish(Bool(data=False))
            return

        cx, cy = centroid
        norm_x = (cx - w / 2) / (w / 2)
        norm_y = (cy - h / 2) / (h / 2)

        right_bias = float(self.get_parameter('right_bias').value)
        on_right = norm_x >= right_bias
        side_str = 'RIGHT' if on_right else 'LEFT ⚠'

        if not on_right:
            self.get_logger().warn(
                f'Cone is on the LEFT (norm_x={norm_x:.2f}) — adjust heading right!'
            )

        pos = Vector3(x=float(norm_x), y=float(norm_y), z=float(best_area))
        self._pos_pub.publish(pos)
        self._detected_pub.publish(Bool(data=True))

        area_change = abs(best_area - self._last_saved_area) / max(self._last_saved_area, 1.0)
        save_interval = float(self.get_parameter('save_interval_sec').value)
        now = self.get_clock().now()
        dt = (now - self._last_save_time).nanoseconds / 1e9

        if (area_change > 0.20 or self._last_saved_area == 0.0) and dt >= save_interval:
            self._save_photo(annotated, contour, cx, cy, norm_x, side_str, colour_name, shape_name)
            self._last_saved_area = best_area
            self._last_save_time = now

    # ── photo saving ───────────────────────────────────────────────────

    def _save_photo(self, frame, contour, cx, cy, norm_x, side_str, colour_name, shape_name):
        annotated = frame.copy()

        cv2.drawContours(annotated, [contour], -1, (0, 255, 0), 2)
        cv2.circle(annotated, (cx, cy), 8, (0, 0, 255), -1)

        label = f'{self._current_wp_label} | {side_str} | {colour_name} {shape_name} | x={norm_x:.2f}'
        cv2.putText(
            annotated, label, (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA
        )
        cv2.putText(
            annotated, label, (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 1, cv2.LINE_AA
        )

        hh = annotated.shape[0]
        cv2.line(
            annotated,
            (annotated.shape[1] // 2, 0),
            (annotated.shape[1] // 2, hh),
            (200, 200, 0), 1
        )

        save_dir = self.get_parameter('save_dir').value
        ts = datetime.now().strftime('%H%M%S_%f')[:10]
        filename = os.path.join(
            save_dir,
            f'cone_{self._photo_count:04d}_{self._current_wp_label}_{ts}.jpg'
        )
        cv2.imwrite(filename, annotated)

        gps_file = filename.replace('.jpg', '_gps.txt')
        with open(gps_file, 'w') as f:
            f.write(f'waypoint:  {self._current_wp_label}\n')
            f.write(f'lat:       {self._gps_lat}\n')
            f.write(f'lon:       {self._gps_lon}\n')
            f.write(f'side:      {side_str}\n')
            f.write(f'colour:    {colour_name}\n')
            f.write(f'shape:     {shape_name}\n')
            f.write(f'timestamp: {datetime.now().isoformat()}\n')

        self._photo_count += 1

        out = String()
        out.data = filename
        self._photo_pub.publish(out)
        self.get_logger().info(
            f'Photo saved: {filename} [{side_str}] GPS=({self._gps_lat}, {self._gps_lon})'
        )

    # ── stronger detection logic adapted from sim ──────────────────────

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
        lower = np.array([
            max(0, h_val - h_tol),
            max(0, s_val - s_tol),
            max(0, v_val - v_tol),
        ], dtype=np.uint8)
        upper = np.array([
            min(179, h_val + h_tol),
            min(255, s_val + s_tol),
            min(255, v_val + v_tol),
        ], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        return mask

    def classify_color_from_contour(self, hsv, contour):
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
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
        elif mean_h < 20:
            return 'orange'
        elif mean_h < 38:
            return 'yellow'
        elif mean_h < 85:
            return 'green'
        elif mean_h < 130:
            return 'blue'
        else:
            return 'purple'

    def classify_shape(self, contour):
        area = cv2.contourArea(contour)
        if area < 100:
            return 'unknown', contour

        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0:
            return 'unknown', contour

        x, y, w, h = cv2.boundingRect(contour)
        if w <= 0 or h <= 0:
            return 'unknown', contour

        aspect = w / float(h)
        bbox_fill = area / float(w * h)

        hull = cv2.convexHull(contour)
        hull_perim = cv2.arcLength(hull, True)
        circularity = 4.0 * np.pi * area / (perimeter * perimeter)

        _, radius = cv2.minEnclosingCircle(contour)
        enc_area = np.pi * radius * radius
        fill_ratio = area / enc_area if enc_area > 0 else 0.0

        if circularity > 0.80 and fill_ratio > 0.80:
            return 'circle', contour

        approx = cv2.approxPolyDP(hull, 0.04 * hull_perim, True)
        v = len(approx)

        if bbox_fill < 0.62:
            for eps in [0.04, 0.06, 0.08, 0.10]:
                tri = cv2.approxPolyDP(hull, eps * hull_perim, True)
                if len(tri) == 3:
                    return 'triangle', tri

        if bbox_fill > 0.75 and 0.7 < aspect < 1.8:
            return 'rectangle', approx

        if v == 3 and bbox_fill < 0.62:
            return 'triangle', approx
        if v == 4:
            return 'rectangle', approx
        if circularity > 0.65 or fill_ratio > 0.75:
            return 'circle', contour

        return 'rectangle', approx

    def contour_edge_strength(self, edges, contour):
        mask = np.zeros(edges.shape[:2], dtype=np.uint8)
        cv2.drawContours(mask, [contour], -1, 255, thickness=2)
        return float(np.sum(cv2.bitwise_and(edges, edges, mask=mask) > 0))

    def score_contour(self, contour, edge_strength, image_shape):
        h, w = image_shape[:2]
        area = cv2.contourArea(contour)
        x, y, cw, ch = cv2.boundingRect(contour)
        cx = x + cw / 2.0
        cy = y + ch / 2.0
        return (
            area
            + 0.5 * cy
            - 0.8 * abs(cx - w / 2.0)
            + 2.0 * edge_strength
            - (500.0 if ch > 0 and cw / ch > 3.0 else 0.0)
            - (600.0 if y < h * 0.15 and cw > w * 0.4 else 0.0)
            - (1000.0 if y + ch >= h - 3 and cw > w * 0.5 else 0.0)
        )

    def classify_shape_from_edges(self, gray, contour):
        x, y, w, h = cv2.boundingRect(contour)
        if w <= 0 or h <= 0:
            return self.classify_shape(contour)

        area = cv2.contourArea(contour)
        bbox_fill = area / float(w * h)
        aspect = w / float(h)

        if bbox_fill > 0.68 and 0.7 < aspect < 1.8:
            return 'rectangle', contour

        pad = 6
        ih, iw = gray.shape[:2]
        x0, y0 = max(0, x - pad), max(0, y - pad)
        x1, y1 = min(iw, x + w + pad), min(ih, y + h + pad)
        crop_gray = gray[y0:y1, x0:x1]
        if crop_gray.size == 0:
            return self.classify_shape(contour)

        obj_mask = np.zeros((ih, iw), dtype=np.uint8)
        border_mask = np.zeros((ih, iw), dtype=np.uint8)
        cv2.drawContours(obj_mask, [contour], -1, 255, thickness=-1)
        cv2.drawContours(border_mask, [contour], -1, 255, thickness=8)
        crop_mask = cv2.bitwise_or(obj_mask, border_mask)[y0:y1, x0:x1]

        blurred = cv2.GaussianBlur(crop_gray, (3, 3), 0)
        crop_edges = cv2.Canny(blurred, 30, 100)
        crop_edges = cv2.bitwise_and(crop_edges, crop_edges, mask=crop_mask)

        k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        crop_edges = cv2.dilate(crop_edges, k, iterations=1)
        crop_edges = cv2.morphologyEx(crop_edges, cv2.MORPH_CLOSE, k)

        edge_contours, _ = cv2.findContours(crop_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not edge_contours:
            return self.classify_shape(contour)

        best_ec = max(edge_contours, key=cv2.contourArea)
        if cv2.contourArea(best_ec) < 200:
            return self.classify_shape(contour)

        best_ec_global = best_ec + np.array([[[x0, y0]]])
        shape, approx = self.classify_shape(best_ec_global)

        if shape == 'triangle' and bbox_fill > 0.68 and 0.7 < aspect < 1.8:
            return 'rectangle', contour

        return shape, approx

    def detect_object(self, frame):
        annotated = frame.copy()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ih, iw = frame.shape[:2]

        ground_mask = self.build_mask_around_hsv(
            hsv, self.estimate_ground_hsv(hsv), h_tol=15, s_tol=80, v_tol=80
        )
        sky_mask = self.build_mask_around_hsv(
            hsv, self.estimate_sky_hsv(hsv), h_tol=15, s_tol=60, v_tol=80
        )
        candidate_mask = cv2.bitwise_not(cv2.bitwise_or(ground_mask, sky_mask))

        k5 = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        candidate_mask = cv2.morphologyEx(candidate_mask, cv2.MORPH_OPEN, k5)
        candidate_mask = cv2.morphologyEx(candidate_mask, cv2.MORPH_CLOSE, k5)

        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        edges = cv2.bitwise_and(edges, edges, mask=candidate_mask)
        edges = cv2.dilate(edges, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)), iterations=1)

        combined = cv2.bitwise_or(candidate_mask, edges)
        combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN, k5)
        combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, k5)

        contours, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
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
            return False, None, None, 'unknown', 'unknown', annotated

        best_fill = np.zeros((ih, iw), dtype=np.uint8)
        cv2.drawContours(best_fill, [best], -1, 255, thickness=-1)
        ys, xs = np.where(best_fill == 255)

        if len(xs) > 0:
            rng = np.random.default_rng(0)
            idx = rng.choice(len(xs), size=min(200, len(xs)), replace=False)
            med_hsv = np.median(hsv[ys[idx], xs[idx]].astype(float), axis=0)
        else:
            M = cv2.moments(best)
            cx_ = int(np.clip(M['m10'] / M['m00'], 0, iw - 1)) if M['m00'] > 0 else iw // 2
            cy_ = int(np.clip(M['m01'] / M['m00'], 0, ih - 1)) if M['m00'] > 0 else ih // 2
            med_hsv = hsv[cy_, cx_].astype(float)

        lower = np.array([
            max(0, med_hsv[0] - 18),
            max(0, med_hsv[1] - 80),
            max(0, med_hsv[2] - 90),
        ], dtype=np.uint8)
        upper = np.array([
            min(179, med_hsv[0] + 18),
            min(255, med_hsv[1] + 80),
            min(255, med_hsv[2] + 90),
        ], dtype=np.uint8)

        colour_mask = cv2.inRange(hsv, lower, upper)
        colour_mask = cv2.bitwise_and(colour_mask, colour_mask, mask=candidate_mask)

        ck = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        colour_mask = cv2.morphologyEx(colour_mask, cv2.MORPH_CLOSE, ck)

        M = cv2.moments(best)
        seed_x = int(np.clip(M['m10'] / M['m00'], 0, iw - 1)) if M['m00'] > 0 else iw // 2
        seed_y = int(np.clip(M['m01'] / M['m00'], 0, ih - 1)) if M['m00'] > 0 else ih // 2

        n_labels, labels = cv2.connectedComponents(colour_mask)
        seed_label = labels[seed_y, seed_x]

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
                    best_lbl = lbl
            clean_mask = ((labels == best_lbl).astype(np.uint8) * 255) if best_lbl != -1 else colour_mask
        else:
            clean_mask = ((labels == seed_label).astype(np.uint8) * 255)

        clean_mask = cv2.morphologyEx(clean_mask, cv2.MORPH_CLOSE, ck)
        obj_contours, _ = cv2.findContours(clean_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        clean_contour = max(obj_contours, key=cv2.contourArea) if obj_contours else best

        colour_name = self.classify_color_from_contour(hsv, clean_contour)
        shape_name, _ = self.classify_shape_from_edges(gray, clean_contour)

        cv2.drawContours(annotated, [clean_contour], -1, (0, 255, 0), 2)

        x, y, w, h = cv2.boundingRect(clean_contour)
        label = f'{colour_name} {shape_name}'
        font, fs, th = cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2
        (tw, thh), _ = cv2.getTextSize(label, font, fs, th)
        tx = max(5, min(x, iw - tw - 5))
        ty = y - 10 if y - 10 > thh + 5 else min(ih - 5, y + h + thh + 10)
        cv2.putText(annotated, label, (tx, ty), font, fs, (0, 255, 0), th)

        M2 = cv2.moments(clean_contour)
        if M2['m00'] > 0:
            cx = int(M2['m10'] / M2['m00'])
            cy = int(M2['m01'] / M2['m00'])
        else:
            cx, cy = x + w // 2, y + h // 2
        cv2.circle(annotated, (cx, cy), 5, (0, 200, 255), -1)

        return True, clean_contour, (cx, cy), colour_name, shape_name, annotated


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
