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

Topics published
  /markers/detected    (std_msgs/String)   – JSON of last detected letter
  /markers/all         (std_msgs/String)   – full waypoints list JSON
"""

import json
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
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

try:
    import pytesseract
    _TESS_OK = True
except ImportError:
    _TESS_OK = False


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

ACTIVE_STATES = frozenset({'MAPPING', 'MANUAL_MAPPING'})

# psm 8 = single word mode — best for large handwritten single character
# psm 7 = single line — fallback
_TESS_CONFIGS = [
    '--psm 8 -l eng --oem 1',
    '--psm 7 -l eng --oem 1',
]

# Fix digit/letter lookalikes that Tesseract commonly confuses
_DIGIT_FIX = {'8': 'B', '0': 'O', '1': 'I', '5': 'S', '2': 'Z', '6': 'G'}

TESS_CONF_THRESH = 60   # out of 100  (Tesseract returns 0-100)

# Map OCR output → Greek letter name.
# Covers both Latin-looking capitals AND actual Greek unicode characters.
GREEK_MAP = {
    'A': 'Alpha',   'B': 'Beta',    'G': 'Gamma',   'D': 'Delta',
    'E': 'Epsilon', 'Z': 'Zeta',    'H': 'Eta',     'Q': 'Theta',
    'I': 'Iota',    'K': 'Kappa',   'L': 'Lambda',  'M': 'Mu',
    'N': 'Nu',      'C': 'Xi',      'O': 'Omicron', 'P': 'Pi',
    'R': 'Rho',     'S': 'Sigma',   'T': 'Tau',     'U': 'Upsilon',
    'F': 'Phi',     'X': 'Chi',     'Y': 'Psi',     'W': 'Omega',
    # Actual Greek uppercase
    'Α': 'Alpha',   'Β': 'Beta',    'Γ': 'Gamma',   'Δ': 'Delta',
    'Ε': 'Epsilon', 'Ζ': 'Zeta',    'Η': 'Eta',     'Θ': 'Theta',
    'Ι': 'Iota',    'Κ': 'Kappa',   'Λ': 'Lambda',  'Μ': 'Mu',
    'Ν': 'Nu',      'Ξ': 'Xi',      'Ο': 'Omicron', 'Π': 'Pi',
    'Ρ': 'Rho',     'Σ': 'Sigma',   'Τ': 'Tau',     'Υ': 'Upsilon',
    'Φ': 'Phi',     'Χ': 'Chi',     'Ψ': 'Psi',     'Ω': 'Omega',
    # Lowercase Greek (Tesseract -l ell may return these)
    'α': 'Alpha',   'β': 'Beta',    'γ': 'Gamma',   'δ': 'Delta',
    'ε': 'Epsilon', 'ζ': 'Zeta',    'η': 'Eta',     'θ': 'Theta',
    'ι': 'Iota',    'κ': 'Kappa',   'λ': 'Lambda',  'μ': 'Mu',
    'ν': 'Nu',      'ξ': 'Xi',      'ο': 'Omicron', 'π': 'Pi',
    'ρ': 'Rho',     'σ': 'Sigma',   'τ': 'Tau',     'υ': 'Upsilon',
    'φ': 'Phi',     'χ': 'Chi',     'ψ': 'Psi',     'ω': 'Omega',
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
        self._last_obstacle_t: dict = {}
        self._last_marker_t:  dict = {}   # per-letter cooldown

        # TF
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        if _TESS_OK:
            self.get_logger().info('Tesseract OCR ready.')
        else:
            self.get_logger().error(
                'pytesseract not found — letter detection disabled. '
                'Install: pip install pytesseract --break-system-packages'
            )

        # Publishers
        self._detected_pub = self.create_publisher(String, '/markers/detected', 10)
        self._all_pub      = self.create_publisher(String, '/markers/all',      10)

        # Subscribers
        self.create_subscription(Image,  '/oak/rgb/image_raw', self._image_cb, 10)
        self.create_subscription(String, '/mission/state',     self._state_cb, 10)

        self.create_timer(1.0 / self.PROCESS_HZ, self._tick)
        self.get_logger().info('MarkerDetector ready.')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _state_cb(self, msg: String):
        self._state = msg.data

    def _image_cb(self, msg: Image):
        self._latest_frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')

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
        if not _TESS_OK:
            return

        paper, bbox = self._find_white_paper(frame)
        if paper is None:
            # self.get_logger().debug('No white paper found in frame')
            return

        #bx, by, bw, bh = bbox
        #self.get_logger().info(f'White paper found: bbox=({bx},{by},{bw}x{bh})')

        preprocessed = _preprocess_for_ocr(paper)
        char, conf = _run_tesseract(preprocessed, self.get_logger())
        engine = 'tesseract'

        # self.get_logger().info(
            #f'OCR raw: "{char}"  conf={conf:.0f}  thresh={TESS_CONF_THRESH}'
        #)

        if char is None or conf < TESS_CONF_THRESH:
            return

        # Take the first character that maps to a Greek letter
        char = char.strip()
        greek_char = None
        for c in char:
            if c.upper() in GREEK_MAP or c in GREEK_MAP:
                greek_char = c.upper() if c.upper() in GREEK_MAP else c
                break
        if greek_char is None:
            self.get_logger().debug(f'OCR result "{char}" has no Greek mapping — ignoring')
            return
        char = greek_char

        greek_name = GREEK_MAP[char]

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
                   f'{greek_name} ({char})  {engine}  '
                   f'conf={conf:.0f}  map=({x_map:.2f},{y_map:.2f})')
        photo_path = os.path.join(
            self._photo_dir, f'letter_{char.upper()}_{ts}.jpg'
        )
        cv2.imwrite(photo_path, annotated)

        entry = {
            'letter':     char.upper(),
            'greek_name': greek_name,
            'confidence': round(conf, 1),
            'engine':     engine,
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
            f'engine={engine}  conf={conf:.0f}  '
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

            pos = self._map_position()
            if pos is None:
                self.get_logger().warn(f'TF not ready — saving {colour} obstacle with position (0,0)')
                x_map, y_map = 0.0, 0.0
            else:
                x_map, y_map = pos

            ts        = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            annotated = frame.copy()
            cv2.drawContours(annotated, [largest], -1, (0, 255, 0), 3)
            M = cv2.moments(largest)
            if M['m00'] > 0:
                cv2.circle(annotated,
                           (int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])),
                           10, (0, 0, 255), -1)
            _put_label(annotated,
                       f'{colour.upper()} obstacle  '
                       f'map=({x_map:.2f},{y_map:.2f})')
            photo_path = os.path.join(
                self._photo_dir, f'{colour}_obstacle_{ts}.jpg'
            )
            cv2.imwrite(photo_path, annotated)

            entry = {
                'colour':    colour,
                'x':         round(x_map, 3),
                'y':         round(y_map, 3),
                'photo':     photo_path,
                'timestamp': ts,
            }
            self._obstacles.append(entry)
            _save_json(self._obstacles_file, self._obstacles)
            self._last_obstacle_t[colour] = now

            self.get_logger().info(
                f'OBSTACLE: {colour.upper()}  '
                f'map=({x_map:.2f},{y_map:.2f})  photo={photo_path}'
            )

    # ------------------------------------------------------------------
    # TF helper
    # ------------------------------------------------------------------

    def _map_position(self):
        try:
            t = self._tf_buffer.lookup_transform(
                'map', 'base_link',
                Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            return None


# ---------------------------------------------------------------------------
# OCR helpers (module-level, easily swappable)
# ---------------------------------------------------------------------------

def _preprocess_for_ocr(img: np.ndarray) -> np.ndarray:
    """
    1. Find dark ink contours in the central region (excludes tape/edge noise).
    2. Crop tight to just the letter.
    3. Resize and pad for Tesseract.
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    k      = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, k)

    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return _pad(gray)

    h, w     = binary.shape
    x_margin = int(w * 0.15)
    y_margin = int(h * 0.10)
    central  = [
        c for c in contours
        if cv2.contourArea(c) > 80
        and cv2.boundingRect(c)[0] > x_margin
        and cv2.boundingRect(c)[0] + cv2.boundingRect(c)[2] < w - x_margin
        and cv2.boundingRect(c)[1] > y_margin
        and cv2.boundingRect(c)[1] + cv2.boundingRect(c)[3] < h - y_margin
    ]
    if not central:
        central = contours

    all_pts          = np.vstack([c.reshape(-1, 2) for c in central])
    lx, ly, lw, lh   = cv2.boundingRect(all_pts)
    pad              = int(max(lw, lh) * 0.20)
    x1, y1           = max(0, lx - pad), max(0, ly - pad)
    x2, y2           = min(w, lx + lw + pad), min(h, ly + lh + pad)
    letter_crop      = gray[y1:y2, x1:x2]

    _, clean = cv2.threshold(letter_crop, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return _pad(clean)


def _pad(img: np.ndarray) -> np.ndarray:
    h, w  = img.shape
    scale = 300 / max(h, w)
    img   = cv2.resize(img, (int(w * scale), int(h * scale)),
                       interpolation=cv2.INTER_CUBIC)
    return cv2.copyMakeBorder(img, 60, 60, 60, 60,
                              cv2.BORDER_CONSTANT, value=255)


def _run_tesseract(img: np.ndarray, logger) -> tuple:
    best_text, best_conf = None, 0
    for config in _TESS_CONFIGS:
        try:
            data = pytesseract.image_to_data(
                img, config=config, output_type=pytesseract.Output.DICT
            )
        except Exception as e:
            logger.warn(f'Tesseract error: {e}')
            continue
        for text, conf in zip(data['text'], data['conf']):
            text = text.strip()
            conf = int(conf)
            if text and conf > best_conf:
                best_conf = conf
                best_text = _DIGIT_FIX.get(text, text)
    return best_text, best_conf


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
