#!/usr/bin/env python3
"""
display_node.py  –  Part 3, Task 7: Robot display UI.

Always visible
  Header bar showing current mission state (colour-coded) and e-stop alert.

IDLE
  Gamepad control reference.

MAPPING / MANUAL_MAPPING
  Left panel  – live SLAM map with robot position (red dot) and detected
                marker locations (orange circles).
  Right panel – list of detected markers with coordinates, and the most
                recently saved annotated photo.

RAPID_NAV
  Left panel  – map with Nav2 planned path overlaid (cyan line), robot
                position, and waypoint targets.
  Right panel – waypoint progress list (done/current/pending).

Topics subscribed
  /mission/state         (std_msgs/String)
  /estop/triggered       (std_msgs/Bool)
  /map                   (nav_msgs/OccupancyGrid)
  /plan                  (nav_msgs/Path)           – Nav2 planned path
  /markers/detected      (std_msgs/String)         – latest marker JSON
  /markers/all           (std_msgs/String)         – full marker list JSON
  /waypoint_driver/index (std_msgs/Int32)          – current waypoint index
"""

import json
import os

import cv2
import numpy as np
import rclpy
import rclpy.duration
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32, String
from tf2_ros import Buffer, TransformListener

# ── Window layout ────────────────────────────────────────────────────────────
W, H      = 1280, 800
HEADER_H  = 70
PANEL_H   = H - HEADER_H
MAP_W     = 760
RIGHT_X   = MAP_W + 12
RIGHT_W   = W - MAP_W - 20

# ── Colours (BGR) ────────────────────────────────────────────────────────────
C_BG      = (30,  30,  30)
C_WHITE   = (255, 255, 255)
C_GREY    = (160, 160, 160)
C_YELLOW  = (0,   220, 220)
C_PATH    = (200, 200, 0)
C_ROBOT   = (0,   0,   255)
C_WP      = (0,   165, 255)
C_DONE    = (0,   200, 0)
C_ESTOP   = (0,   0,   200)

STATE_COLOR = {
    'IDLE':           (80,  80,  80),
    'MAPPING':        (0,   160, 0),
    'MANUAL_MAPPING': (0,   160, 120),
    'RAPID_NAV':      (0,   120, 220),
}
STATE_LABEL = {
    'IDLE':           'IDLE  -  Waiting for operator input',
    'MAPPING':        'PHASE 1  -  Autonomous Mapping',
    'MANUAL_MAPPING': 'PHASE 1  -  Manual Mapping',
    'RAPID_NAV':      'PHASE 2  -  Rapid Waypoint Navigation',
}


class DisplayNode(Node):

    DISPLAY_HZ = 2.0

    def __init__(self):
        super().__init__('display_node')

        self._state        = 'IDLE'
        self._estop        = False
        self._map_msg      = None
        self._plan_msg     = None
        self._markers      = []     # list of dicts from /markers/all
        self._latest_photo = None   # cv2 BGR image of last detection
        self._wp_index     = 0      # current waypoint index from waypoint_driver
        self._wp_list      = []     # waypoints loaded for RAPID_NAV display

        self._bridge = CvBridge()
        from sensor_msgs.msg import CompressedImage
        self._img_pub = self.create_publisher(CompressedImage, '/display/image/compressed', 1)

        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(String,        '/mission/state',         self._state_cb,   10)
        self.create_subscription(Bool,          '/estop/triggered',       self._estop_cb,   10)
        self.create_subscription(OccupancyGrid, '/map',                   self._map_cb,      1)
        self.create_subscription(Path,          '/plan',                  self._plan_cb,     1)
        self.create_subscription(String,        '/markers/detected',      self._detected_cb, 10)
        self.create_subscription(String,        '/markers/all',           self._all_cb,      10)
        self.create_subscription(Int32,         '/waypoint_driver/index', self._index_cb,   10)


        self.create_timer(1.0 / self.DISPLAY_HZ, self._tick)
        self.get_logger().info('DisplayNode ready.')

    # ── Subscribers ───────────────────────────────────────────────────────────

    def _state_cb(self, msg: String):
        prev = self._state
        self._state = msg.data
        if self._state == 'RAPID_NAV' and prev != 'RAPID_NAV':
            self._wp_index = 0
            self._load_wp_file()

    def _estop_cb(self, msg: Bool):
        self._estop = msg.data

    def _map_cb(self, msg: OccupancyGrid):
        self._map_msg = msg

    def _plan_cb(self, msg: Path):
        self._plan_msg = msg

    def _detected_cb(self, msg: String):
        try:
            entry = json.loads(msg.data)
            path  = entry.get('photo', '')
            if path and os.path.exists(path):
                img = cv2.imread(path)
                if img is not None:
                    self._latest_photo = img
        except Exception:
            pass

    def _all_cb(self, msg: String):
        try:
            self._markers = json.loads(msg.data)
        except Exception:
            pass

    def _index_cb(self, msg: Int32):
        self._wp_index = msg.data

    # ── Main tick ─────────────────────────────────────────────────────────────

    def _tick(self):
        canvas = np.full((H, W, 3), C_BG, dtype=np.uint8)
        self._draw_header(canvas)

        if self._state in ('MAPPING', 'MANUAL_MAPPING'):
            self._draw_mapping(canvas)
        elif self._state == 'RAPID_NAV':
            self._draw_rapid_nav(canvas)
        else:
            self._draw_idle(canvas)

        # Publish as compressed JPEG for Foxglove (~50KB vs ~3MB raw)
        try:
            from sensor_msgs.msg import CompressedImage
            _, buf = cv2.imencode('.jpg', canvas, [cv2.IMWRITE_JPEG_QUALITY, 70])
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = 'jpeg'
            msg.data   = buf.tobytes()
            self._img_pub.publish(msg)
        except Exception:
            pass


    # ── Header ────────────────────────────────────────────────────────────────

    def _draw_header(self, canvas):
        color = STATE_COLOR.get(self._state, (80, 80, 80))
        cv2.rectangle(canvas, (0, 0), (W, HEADER_H), color, -1)
        label = STATE_LABEL.get(self._state, self._state)
        _text(canvas, label, 20, 46, scale=1.0, thickness=2)

        if self._estop:
            cv2.rectangle(canvas, (W - 200, 8), (W - 8, HEADER_H - 8), C_ESTOP, -1)
            _text(canvas, 'E-STOP', W - 185, 46, scale=0.95, thickness=2)

    # ── Idle view ─────────────────────────────────────────────────────────────

    def _draw_idle(self, canvas):
        y = HEADER_H + 80
        for line in [
            'X  (Cross)     ->  Start autonomous mapping',
            '/\\  (Triangle) ->  Start manual mapping',
            '[] (Square)    ->  Start Phase 2 (after mapping)',
            'O  (Circle)    ->  Abort / return to IDLE',
            '',
            'L1 + sticks    ->  Manual drive override (any state)',
        ]:
            _text(canvas, line, 80, y, color=C_WHITE, scale=0.75)
            y += 46

    # ── Mapping view ──────────────────────────────────────────────────────────

    def _draw_mapping(self, canvas):
        # Left: map
        canvas[HEADER_H:, :MAP_W] = self._render_map(MAP_W, PANEL_H)
        cv2.line(canvas, (MAP_W, HEADER_H), (MAP_W, H), (60, 60, 60), 2)

        # Right: marker list
        y = HEADER_H + 18
        _text(canvas, 'Detected Markers', RIGHT_X, y, color=C_YELLOW, scale=0.7, thickness=2)
        y += 32

        if not self._markers:
            _text(canvas, 'None yet', RIGHT_X, y, color=C_GREY, scale=0.6)
            y += 28
        else:
            for wp in self._markers[:7]:
                line = (f"{wp.get('greek_name','?')} ({wp.get('letter','?')})  "
                        f"({wp.get('x', 0):.2f}, {wp.get('y', 0):.2f})")
                _text(canvas, line, RIGHT_X, y, color=C_WHITE, scale=0.55)
                y += 26

        # Latest photo
        if self._latest_photo is not None:
            ph_h = H - y - 16
            ph_w = RIGHT_W
            if ph_h > 60 and ph_w > 60:
                photo = cv2.resize(self._latest_photo, (ph_w, ph_h))
                canvas[y:y + ph_h, RIGHT_X:RIGHT_X + ph_w] = photo

    # ── Rapid nav view ────────────────────────────────────────────────────────

    def _draw_rapid_nav(self, canvas):
        canvas[HEADER_H:, :MAP_W] = self._render_map(MAP_W, PANEL_H, draw_path=True)
        cv2.line(canvas, (MAP_W, HEADER_H), (MAP_W, H), (60, 60, 60), 2)

        y = HEADER_H + 18
        _text(canvas, 'Waypoint Progress', RIGHT_X, y, color=C_YELLOW, scale=0.7, thickness=2)
        y += 40

        wps = self._wp_list[:3]
        if not wps:
            _text(canvas, 'No waypoints loaded', RIGHT_X, y, color=C_GREY, scale=0.6)
        else:
            for i, wp in enumerate(wps):
                if i < self._wp_index:
                    prefix = 'DONE  '
                    color  = C_DONE
                elif i == self._wp_index:
                    prefix = '>>>   '
                    color  = (0, 200, 255)
                else:
                    prefix = f'  {i+1}.  '
                    color  = C_GREY
                line = (f"{prefix}{wp.get('greek_name','?')} ({wp.get('letter','?')})  "
                        f"({wp.get('x',0):.2f}, {wp.get('y',0):.2f})")
                _text(canvas, line, RIGHT_X, y, color=color, scale=0.65, thickness=2)
                y += 52

    # ── Map renderer ──────────────────────────────────────────────────────────

    def _render_map(self, w, h, draw_path=False):
        img = np.full((h, w, 3), (50, 50, 50), dtype=np.uint8)

        if self._map_msg is None:
            _text(img, 'Waiting for map...', 20, h // 2, color=C_GREY, scale=0.8)
            return img

        gw  = self._map_msg.info.width
        gh  = self._map_msg.info.height
        res = self._map_msg.info.resolution
        ox  = self._map_msg.info.origin.position.x
        oy  = self._map_msg.info.origin.position.y

        data = np.array(self._map_msg.data, dtype=np.int8).reshape(gh, gw)
        raw  = np.zeros((gh, gw, 3), dtype=np.uint8)
        raw[data == -1] = (100, 100, 100)   # unknown  – grey
        raw[data ==  0] = (210, 210, 210)   # free     – light grey
        raw[data >   0] = (20,  20,  20)    # occupied – dark
        raw = cv2.flip(raw, 0)

        scale  = min(w / gw, h / gh)
        disp_w = int(gw * scale)
        disp_h = int(gh * scale)
        map_img = cv2.resize(raw, (disp_w, disp_h), interpolation=cv2.INTER_NEAREST)

        off_x = (w - disp_w) // 2
        off_y = (h - disp_h) // 2
        img[off_y:off_y + disp_h, off_x:off_x + disp_w] = map_img

        def to_px(wx, wy):
            px = int((wx - ox) / res * scale) + off_x
            py = disp_h - int((wy - oy) / res * scale) + off_y
            return (px, py)

        # Planned path
        if draw_path and self._plan_msg and len(self._plan_msg.poses) > 1:
            pts = [to_px(p.pose.position.x, p.pose.position.y)
                   for p in self._plan_msg.poses]
            for i in range(len(pts) - 1):
                cv2.line(img, pts[i], pts[i + 1], C_PATH, 2, cv2.LINE_AA)

        # Waypoint markers
        wps = self._wp_list[:3] if self._state == 'RAPID_NAV' else self._markers[:8]
        for i, wp in enumerate(wps):
            try:
                px, py = to_px(float(wp['x']), float(wp['y']))
                done   = (self._state == 'RAPID_NAV' and i < self._wp_index)
                color  = C_DONE if done else C_WP
                cv2.circle(img, (px, py), 7, color, -1)
                cv2.circle(img, (px, py), 10, color, 2)
                _text(img, wp.get('letter', '?'), px + 12, py + 5,
                      color=color, scale=0.5, thickness=2)
            except Exception:
                pass

        # Robot position
        pos = self._robot_position()
        if pos:
            rx, ry = to_px(pos[0], pos[1])
            cv2.circle(img, (rx, ry), 7,  C_ROBOT, -1)
            cv2.circle(img, (rx, ry), 12, C_ROBOT, 2)

        return img

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _robot_position(self):
        try:
            t = self._tf_buffer.lookup_transform(
                'map', 'base_link', Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            return None

    def _load_wp_file(self):
        path = '/workspace/autonomous_nav/outputs/markers/waypoints.json'
        try:
            with open(path) as f:
                self._wp_list = json.load(f)
        except Exception:
            self._wp_list = []


# ── Helpers ───────────────────────────────────────────────────────────────────

def _text(img, text, x, y, color=(255, 255, 255), scale=0.65, thickness=1):
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                scale, color, thickness, cv2.LINE_AA)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = DisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
