#!/usr/bin/env python3
"""
marker_detector.py  –  Part 3, Tasks 3 & 4.

Task 3 – Greek letter detection
  Finds white A4 paper in the camera frame, crops and preprocesses it,
  then runs OCR to read the handwritten letter.

  OCR strategy (primary → fallback):
    1. Tesseract  –psm 10 -l ell   (single-char mode, Greek language pack)
       Best for clean white paper with one large character.
    2. EasyOCR  ['en']             (if Tesseract returns nothing/low confidence)
       Better for messy/rotated text but slower (~0.5 s) and weaker on
       isolated Greek characters.

  On a confirmed detection the node looks up the robot's map position
  (TF: map → base_link) and saves:
    • Annotated JPEG  →  outputs/markers/photos/
    • JSON entry      →  outputs/markers/waypoints.json

Task 4 – Red / yellow obstacle logging
  HSV colour thresholding at 2 Hz.  Large blobs → photo + entry in
  outputs/markers/obstacles.json.
  15-second per-colour cooldown prevents flooding the same obstacle.

Active states: MAPPING and MANUAL_MAPPING only.

Topics subscribed
  /oak/rgb/image_raw   (sensor_msgs/Image)
  /mission/state       (std_msgs/String)
  /scan                (sensor_msgs/LaserScan)  – for obstacle ranging

Topics published
  /markers/detected    (std_msgs/String)   – JSON of last detected letter
  /markers/all         (std_msgs/String)   – full waypoints list JSON
"""

import json
import math
import os
import datetime
import time

import cv2
import numpy as np
import rclpy
import rclpy.duration
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

try:
    import torch
    import torch.nn as nn
    import torchvision.models as tv_models
    import torchvision.transforms as T
    from PIL import Image as PILImage

    _CLF_DIR = os.path.join(os.path.dirname(__file__), 'greek_ocr', 'classifier')
    if not os.path.isdir(_CLF_DIR):
        _CLF_DIR = '/workspace/autonomous_nav/ros2_ws/src/part3_explore/part3_explore/greek_ocr/classifier'

    with open(os.path.join(_CLF_DIR, 'classes.json')) as _f:
        _idx_to_class = {int(k): v for k, v in json.load(_f).items()}

    _clf_model = tv_models.mobilenet_v2(weights=None)
    _clf_model.classifier[1] = nn.Linear(_clf_model.last_channel, len(_idx_to_class))
    _clf_model.load_state_dict(
        torch.load(os.path.join(_CLF_DIR, 'mobilenet_best.pth'), map_location='cpu')
    )
    _clf_model.eval()
    _clf_device = 'cuda' if torch.cuda.is_available() else 'cpu'
    _clf_model.to(_clf_device)
    _clf_transform = T.Compose([
        T.Resize((224, 224)),
        T.ToTensor(),
        T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
    ])
    _CLASSIFIER_OK = True
except Exception:
    _CLASSIFIER_OK = False


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

ACTIVE_STATES = frozenset({'MAPPING', 'MANUAL_MAPPING'})

CONF_THRESH = 60   # minimum softmax confidence (0-100) to accept a detection

# Classifier returns the full Greek word ("alpha", "beta" …). Map to letter + canonical name.
LETTER_MAP = {
    'alpha':  ('A', 'Alpha'),   'beta':   ('B', 'Beta'),
    'gamma':  ('G', 'Gamma'),   'delta':  ('D', 'Delta'),
    'zeta':   ('Z', 'Zeta'),    'eta':    ('H', 'Eta'),
    'lambda': ('L', 'Lambda'),  'mu':     ('M', 'Mu'),
    'rho':    ('R', 'Rho'),     'tau':    ('T', 'Tau'),
    'phi':    ('F', 'Phi'),     'psi':    ('Y', 'Psi'),
}

# HSV ranges for obstacle colours  (H: 0-179, S/V: 0-255)
OBSTACLE_COLOURS = {
    'red': [
        ((0,   120, 80), (10,  255, 255)),
        ((165, 120, 80), (179, 255, 255)),
    ],
    'yellow': [
        ((20, 120, 80), (35, 255, 255)),
    ],
}
OBSTACLE_MIN_AREA   = 6000   # px²
OBSTACLE_COOLDOWN_S = 15.0

# OAK-D horizontal field of view (degrees → radians)
OAK_HFOV_RAD = math.radians(69.0)


# ---------------------------------------------------------------------------

class MarkerDetector(Node):

    PROCESS_HZ     = 2.0
    WHITE_MIN_AREA = 3000   # px²  — minimum white blob to be an A4 paper

    def __init__(self):
        super().__init__('marker_detector')

        self.declare_parameter('output_dir', '/workspace/autonomous_nav/outputs')
        out_root = self.get_parameter('output_dir').value

        self._photo_dir      = os.path.join(out_root, 'markers', 'photos')
        self._waypoints_file = os.path.join(out_root, 'markers', 'waypoints.json')
        self._obstacles_file = os.path.join(out_root, 'markers', 'obstacles.json')
        os.makedirs(self._photo_dir, exist_ok=True)

        self._waypoints = _load_json(self._waypoints_file, [])
        self._obstacles = _load_json(self._obstacles_file, [])

        self._bridge          = CvBridge()
        self._state           = 'IDLE'
        self._latest_frame    = None
        self._latest_scan     = None      # cached LaserScan for obstacle ranging
        self._last_obstacle_t: dict = {}
        self._last_marker_t:  dict = {}   # per-letter cooldown

        # TF
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        if _CLASSIFIER_OK:
            self.get_logger().info(f'MobileNetV2 classifier loaded ({len(_idx_to_class)} classes).')
        else:
            self.get_logger().error('Classifier not available — letter detection disabled.')

        # Publishers
        self._detected_pub = self.create_publisher(String, '/markers/detected', 10)
        self._all_pub      = self.create_publisher(String, '/markers/all',      10)

        # Subscribers
        self.create_subscription(Image,     '/oak/rgb/image_raw', self._image_cb, 10)
        self.create_subscription(String,    '/mission/state',     self._state_cb, 10)
        self.create_subscription(LaserScan, '/scan',              self._scan_cb,  10)

        self.create_timer(1.0 / self.PROCESS_HZ, self._tick)
        self.get_logger().info('MarkerDetector ready.')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _state_cb(self, msg: String):
        self._state = msg.data

    def _image_cb(self, msg: Image):
        self._latest_frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _scan_cb(self, msg: LaserScan):
        self._latest_scan = msg

    # ------------------------------------------------------------------
    # Main tick (2 Hz)
    # ------------------------------------------------------------------

    def _tick(self):
        if self._state not in ACTIVE_STATES or self._latest_frame is None:
            return
        frame = self._latest_frame.copy()
        self._detect_letter(frame)
        self._detect_obstacles(frame)

    # ------------------------------------------------------------------
    # Task 3 – Greek letter detection
    # ------------------------------------------------------------------

    def _detect_letter(self, frame: np.ndarray):
        if not _CLASSIFIER_OK:
            return

        paper, bbox = self._find_white_paper(frame)
        if paper is None:
            return

        word, conf = _run_classifier(paper, self.get_logger())

        if word is None or conf < CONF_THRESH:
            return

        match = LETTER_MAP.get(word.strip().lower())
        if match is None:
            self.get_logger().debug(f'Classifier result "{word}" not in LETTER_MAP — ignoring')
            return
        char, greek_name = match

        # Per-letter cooldown: don't spam the same marker
        now = time.monotonic()
        if now - self._last_marker_t.get(char, 0) < 30.0:
            return
        self._last_marker_t[char] = now

        pos = self._map_position()
        if pos is None:
            self.get_logger().warn(
                f'TF not ready — saving "{greek_name} ({char})" with position (0,0)'
            )
            x_map, y_map = 0.0, 0.0
        else:
            x_map, y_map = pos

        # Save annotated photo
        ts        = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        annotated = frame.copy()
        if bbox:
            bx, by, bw, bh = bbox
            cv2.rectangle(annotated, (bx, by), (bx + bw, by + bh), (0, 255, 0), 3)
        _put_label(annotated,
                   f'{greek_name} ({char})  classifier  '
                   f'conf={conf:.0f}  map=({x_map:.2f},{y_map:.2f})')
        photo_path = os.path.join(
            self._photo_dir, f'letter_{char.upper()}_{ts}.jpg'
        )
        cv2.imwrite(photo_path, annotated)

        entry = {
            'letter':     char.upper(),
            'greek_name': greek_name,
            'confidence': round(conf, 1),
            'engine':     'mobilenet',
            'x':          round(x_map, 3),
            'y':          round(y_map, 3),
            'photo':      photo_path,
            'timestamp':  ts,
        }
        self._waypoints.append(entry)
        _save_json(self._waypoints_file, self._waypoints)

        self._detected_pub.publish(String(data=json.dumps(entry)))
        self._all_pub.publish(String(data=json.dumps(self._waypoints)))

        self.get_logger().info(
            f'MARKER DETECTED: {greek_name} ({char.upper()})  '
            f'engine=mobilenet  conf={conf:.0f}  '
            f'map=({x_map:.2f}, {y_map:.2f})'
        )

    # ------------------------------------------------------------------
    # White paper finder
    # ------------------------------------------------------------------

    def _find_white_paper(self, frame: np.ndarray):
        """
        Find the largest white rectangular blob in the lower 80% of frame.
        Returns (cropped_image, bounding_rect) or (None, None).
        """
        h, w   = frame.shape[:2]
        roi_y0 = int(h * 0.20)          # skip top 20% (ceiling)
        roi    = frame[roi_y0:, :]

        hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([0, 0, 180]), np.array([180, 60, 255]))

        k    = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        best, best_area = None, 0
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.WHITE_MIN_AREA:
                continue
            x, y, cw, ch = cv2.boundingRect(c)
            aspect = cw / float(ch) if ch > 0 else 0
            if 0.25 < aspect < 4.0 and area > best_area:
                best_area = area
                best = (x, y, cw, ch)

        if best is None:
            return None, None

        x, y, cw, ch = best
        fy      = y + roi_y0
        cropped = frame[fy:fy + ch, x:x + cw]
        return cropped, (x, fy, cw, ch)

    # ------------------------------------------------------------------
    # Task 4 – Red / yellow obstacle detection
    # ------------------------------------------------------------------

    def _detect_obstacles(self, frame: np.ndarray):
        hsv    = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        kernel = np.ones((9, 9), np.uint8)

        for colour, ranges in OBSTACLE_COLOURS.items():
            now = time.monotonic()
            if now - self._last_obstacle_t.get(colour, 0) < OBSTACLE_COOLDOWN_S:
                continue

            mask = np.zeros(frame.shape[:2], dtype=np.uint8)
            for (lo, hi) in ranges:
                mask = cv2.bitwise_or(
                    mask, cv2.inRange(hsv, np.array(lo), np.array(hi))
                )
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            if not contours:
                continue
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) < OBSTACLE_MIN_AREA:
                continue

            # Compute centroid first — needed for lidar bearing
            M     = cv2.moments(largest)
            cx_px = int(M['m10'] / M['m00']) if M['m00'] > 0 else frame.shape[1] // 2
            cy_px = int(M['m01'] / M['m00']) if M['m00'] > 0 else frame.shape[0] // 2

            # NEW: classify the shape of the detected obstacle
            shape = _detect_shape(largest)

            # old: robot position only
            # pos = self._map_position()
            # if pos is None:
            #     self.get_logger().warn(f'TF not ready — saving {colour} obstacle with position (0,0)')
            #     x_map, y_map = 0.0, 0.0
            # else:
            #     x_map, y_map = pos
            obs = self._obstacle_map_position(cx_px, frame.shape[1])
            if obs is None:
                self.get_logger().warn(f'TF not ready — saving {colour} obstacle with position (0,0)')
                x_map, y_map, obs_dist = 0.0, 0.0, 0.0
            else:
                x_map, y_map, obs_dist = obs

            ts        = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            annotated = frame.copy()
            cv2.drawContours(annotated, [largest], -1, (0, 255, 0), 3)
            if M['m00'] > 0:
                cv2.circle(annotated, (cx_px, cy_px), 10, (0, 0, 255), -1)
            # OLD: _put_label(annotated, f'{colour.upper()} obstacle  map=({x_map:.2f},{y_map:.2f})  dist={obs_dist:.2f}m')
            # NEW: include detected shape in the label
            _put_label(annotated,
                       f'{colour.upper()} {shape}  '
                       f'map=({x_map:.2f},{y_map:.2f})  dist={obs_dist:.2f}m')
            photo_path = os.path.join(
                self._photo_dir, f'{colour}_obstacle_{ts}.jpg'
            )
            cv2.imwrite(photo_path, annotated)

            # OLD: entry did not include shape
            # NEW: 'shape' field added to record detected shape
            entry = {
                'colour':    colour,
                'shape':     shape,
                'x':         round(x_map, 3),
                'y':         round(y_map, 3),
                'distance':  round(obs_dist, 3),
                'photo':     photo_path,
                'timestamp': ts,
            }
            self._obstacles.append(entry)
            _save_json(self._obstacles_file, self._obstacles)
            self._last_obstacle_t[colour] = now

            # OLD: self.get_logger().info(f'OBSTACLE: {colour.upper()}  map=...')
            # NEW: shape included in log output
            self.get_logger().info(
                f'OBSTACLE: {colour.upper()} {shape.upper()}  '
                f'map=({x_map:.2f},{y_map:.2f})  dist={obs_dist:.2f}m  photo={photo_path}'
            )

    # ------------------------------------------------------------------
    # TF / ranging helpers
    # ------------------------------------------------------------------

    def _map_position(self):
        """Returns (x, y) of the robot in map frame. Used for letter detection."""
        try:
            t = self._tf_buffer.lookup_transform(
                'map', 'base_link',
                Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            return None

    def _obstacle_map_position(self, centroid_x: int, frame_width: int):
        """Project blob centroid through the lidar scan to get the obstacle's
        map position.  Returns (x_map, y_map, range_m) or None if TF unavailable.
        Falls back to robot position with range=0 when lidar has no valid return."""
        bearing = (centroid_x / frame_width - 0.5) * OAK_HFOV_RAD

        # Look up the lidar range at this bearing
        r    = None
        scan = self._latest_scan
        if scan is not None and scan.angle_min <= bearing <= scan.angle_max:
            idx = int(round((bearing - scan.angle_min) / scan.angle_increment))
            idx = max(0, min(idx, len(scan.ranges) - 1))
            raw = scan.ranges[idx]
            if math.isfinite(raw) and scan.range_min <= raw <= scan.range_max:
                r = raw

        # Get robot pose from TF
        try:
            t = self._tf_buffer.lookup_transform(
                'map', 'base_link',
                Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
        except Exception:
            return None

        rx  = t.transform.translation.x
        ry  = t.transform.translation.y
        q   = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        if r is None:
            # No valid lidar return — fall back to recording the robot's position
            return rx, ry, 0.0

        obs_x = rx + r * math.cos(yaw + bearing)
        obs_y = ry + r * math.sin(yaw + bearing)
        return obs_x, obs_y, r


# ---------------------------------------------------------------------------
# NEW: Shape detection helper
# ---------------------------------------------------------------------------

def _detect_shape(contour) -> str:
    """Classify a contour as 'circle', 'square', or 'rectangle'.

    Strategy:
      1. Approximate the contour to a polygon (epsilon = 4% of perimeter).
      2. If the approximation has exactly 4 vertices it is a quadrilateral —
         check the bounding-rect aspect ratio to tell square from rectangle.
      3. Otherwise compute circularity (4π·area / perimeter²); values above
         0.72 are treated as circles.  Anything else falls back to 'rectangle'.
    """
    peri   = cv2.arcLength(contour, True)
    # NEW: approximate contour to polygon with 4% tolerance
    approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
    verts  = len(approx)

    if verts == 3:
        # NEW: 3 corners → triangle
        return 'triangle'

    if verts == 4:
        # NEW: 4 corners detected — distinguish square vs rectangle by aspect ratio
        x, y, w, h = cv2.boundingRect(contour)
        aspect = w / float(h) if h > 0 else 1.0
        # NEW: aspect ratio within 15% of 1.0 → square, otherwise rectangle
        if 0.85 <= aspect <= 1.15:
            return 'square'
        return 'rectangle'

    # NEW: not 3 or 4 corners — use circularity to detect circles
    area = cv2.contourArea(contour)
    circularity = (4.0 * math.pi * area / (peri * peri)) if peri > 0 else 0.0
    if circularity > 0.72:
        return 'circle'

    # NEW: fallback for irregular shapes
    return 'rectangle'


# ---------------------------------------------------------------------------
# Classifier helper (module-level)
# ---------------------------------------------------------------------------

def _run_classifier(img_bgr: np.ndarray, logger) -> tuple:
    """MobileNetV2 classifier. Returns (word, confidence 0-100)."""
    try:
        h, w  = img_bgr.shape[:2]
        img   = cv2.resize(img_bgr, (256, int(h * 256 / w)), interpolation=cv2.INTER_AREA)
        gray  = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, bw = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        blurred = cv2.GaussianBlur(bw, (3, 3), 0)
        _, binary = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)
        black = np.where(binary == 0)
        if len(black[0]) > 0:
            p = 6
            binary = binary[
                max(0, black[0].min() - p):min(binary.shape[0], black[0].max() + p),
                max(0, black[1].min() - p):min(binary.shape[1], black[1].max() + p),
            ]
        rgb = cv2.cvtColor(binary, cv2.COLOR_GRAY2RGB)
        pil = PILImage.fromarray(rgb)
        px  = _clf_transform(pil).unsqueeze(0).to(_clf_device)
        with torch.no_grad():
            logits = _clf_model(px)
        probs          = torch.softmax(logits, dim=-1)[0]
        conf_val, pred = probs.max(0)
        word = _idx_to_class[pred.item()]
        conf = round(conf_val.item() * 100, 1)
        return word, conf
    except Exception as e:
        logger.warn(f'Classifier error: {e}')
        return None, 0


# ---------------------------------------------------------------------------
# Misc helpers
# ---------------------------------------------------------------------------

def _put_label(img: np.ndarray, text: str):
    cv2.putText(img, text, (10, 35), cv2.FONT_HERSHEY_SIMPLEX,
                0.9, (0, 0, 0),   4, cv2.LINE_AA)
    cv2.putText(img, text, (10, 35), cv2.FONT_HERSHEY_SIMPLEX,
                0.9, (0, 255, 0), 2, cv2.LINE_AA)


def _load_json(path: str, default):
    if os.path.exists(path):
        try:
            with open(path) as f:
                return json.load(f)
        except Exception:
            pass
    return default


def _save_json(path: str, data):
    with open(path, 'w') as f:
        json.dump(data, f, indent=2)


# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = MarkerDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
