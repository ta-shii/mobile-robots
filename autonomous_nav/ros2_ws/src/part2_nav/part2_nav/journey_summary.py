#!/usr/bin/env python3
"""
Journey Summary — Part 2, Task 5.

Collects ALL mission events and on MISSION COMPLETE:
  • Prints a formatted summary to the terminal
  • Saves a .txt report with every event + GPS coordinates
  • Generates a PNG map of the driven GPS path using matplotlib

The map shows:
  - Blue line  = GPS path driven
  - Green star = HOME position
  - Red dots   = waypoints (with WP labels)
  - Orange X   = cone photo locations
  - Purple diamond = object detection locations

Topics subscribed
─────────────────
  waypoint_status   (std_msgs/String)
  photo_event       (std_msgs/String)
  object_detection  (std_msgs/String)
  weave_active      (std_msgs/Bool)
  /fix              (sensor_msgs/NavSatFix)
"""

import os
import math
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String


class JourneySummary(Node):

    def __init__(self):
        super().__init__('journey_summary')

        self.declare_parameter('save_dir', '/workspace/autonomous_nav/outputs')
        save_dir = self.get_parameter('save_dir').value
        os.makedirs(save_dir, exist_ok=True)

        self._start_time = self.get_clock().now()
        self._events = []
        self._photos = []
        self._detections = []
        self._wp_arrivals = {}
        self._weave_logged = False
        self._mission_done = False

        # GPS path recording
        self._gps_path = []
        self._home_pos = None
        self._wp_positions = {}
        self._photo_positions = []
        self._detect_positions = []
        self._current_lat = 0.0
        self._current_lon = 0.0
        self._gps_recording = False

        self.create_subscription(String, 'waypoint_status', self._status_cb, 10)
        self.create_subscription(String, 'photo_event', self._photo_cb, 10)
        self.create_subscription(String, 'object_detection', self._detection_cb, 10)
        self.create_subscription(Bool, 'weave_active', self._weave_cb, 10)
        self.create_subscription(NavSatFix, '/fix', self._gps_cb, 10)

        self.get_logger().info(f'Journey summary recorder started. Saving to {save_dir}')

    # ── GPS ────────────────────────────────────────────────────────────────

    def _gps_cb(self, msg: NavSatFix):
        if msg.status.status < 0:
            return

        self._current_lat = msg.latitude
        self._current_lon = msg.longitude

        if self._home_pos is None:
            self._home_pos = (msg.latitude, msg.longitude)

        if self._gps_recording and not self._mission_done:
            if self._gps_path:
                last = self._gps_path[-1]
                if self._haversine(last[0], last[1], msg.latitude, msg.longitude) < 0.5:
                    return
            self._gps_path.append((msg.latitude, msg.longitude))

    # ── event callbacks ────────────────────────────────────────────────────

    def _status_cb(self, msg: String):
        text = msg.data
        ts = self._elapsed()
        self._log(ts, f'NAV: {text}')

        if 'Home locked' in text:
            self._gps_recording = True

        for wp in ['WP1', 'WP2', 'WP3', 'WP4', 'WP5', 'HOME']:
            if wp in text and 'reached' in text.lower():
                self._wp_arrivals[wp] = ts
                self._wp_positions[wp] = (self._current_lat, self._current_lon)

        if 'MISSION COMPLETE' in text and not self._mission_done:
            self._mission_done = True
            self._gps_recording = False
            self._print_summary()

    def _photo_cb(self, msg: String):
        self._photos.append(msg.data)
        self._photo_positions.append((self._current_lat, self._current_lon))
        self._log(self._elapsed(), f'PHOTO: {os.path.basename(msg.data)}')

    def _detection_cb(self, msg: String):
        self._detections.append(msg.data)
        self._detect_positions.append((self._current_lat, self._current_lon))
        self._log(self._elapsed(), f'OBJECT: {msg.data}')

    def _weave_cb(self, msg: Bool):
        if msg.data and not self._weave_logged:
            self._log(self._elapsed(), 'Slalom weave mode STARTED (WP1→WP2 leg)')
            self._weave_logged = True
        elif not msg.data and self._weave_logged:
            self._log(self._elapsed(), 'Slalom weave mode ENDED')

    # ── helpers ────────────────────────────────────────────────────────────

    def _elapsed(self) -> str:
        ns = (self.get_clock().now() - self._start_time).nanoseconds
        secs = ns / 1e9
        m = int(secs // 60)
        s = secs % 60
        return f'{m:02d}:{s:05.2f}'

    def _log(self, ts: str, text: str):
        entry = f'[{ts}] {text}'
        self._events.append(entry)
        self.get_logger().info(entry)

    @staticmethod
    def _haversine(lat1, lon1, lat2, lon2) -> float:
        R = 6_371_000.0
        p1 = math.radians(lat1)
        p2 = math.radians(lat2)
        dp = math.radians(lat2 - lat1)
        dl = math.radians(lon2 - lon1)
        a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    @staticmethod
    def _to_metres(home_lat, home_lon, lat, lon):
        R = 6_371_000.0
        dy = math.radians(lat - home_lat) * R
        dx = math.radians(lon - home_lon) * R * math.cos(math.radians(home_lat))
        return dx, dy

    # ── summary ────────────────────────────────────────────────────────────

    def _print_summary(self):
        total = self._elapsed()
        save_dir = self.get_parameter('save_dir').value
        ts_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        lines = []

        lines += [
            '=' * 62,
            '    PIONEER 3-AT  ──  PART 2 MISSION SUMMARY',
            '=' * 62,
            f'  Date/Time  : {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}',
            f'  Total time : {total}',
            f'  GPS points : {len(self._gps_path)} recorded',
            '',
            '  WAYPOINTS VISITED',
            '  ' + '-' * 44,
        ]

        for wp, arrival in sorted(self._wp_arrivals.items()):
            pos = self._wp_positions.get(wp, (0, 0))
            lines.append(f'    {wp}  →  t={arrival}  GPS=({pos[0]:.6f}, {pos[1]:.6f})')

        if not self._wp_arrivals:
            lines.append('    (none reached)')

        lines += [
            '',
            '  OBJECT DETECTIONS',
            '  ' + '-' * 44,
        ]

        if self._detections:
            for d in self._detections:
                lines.append(f'    {d}')
        else:
            lines.append('    (none detected)')

        lines += [
            '',
            f'  PHOTOS SAVED  ({len(self._photos)} total)',
            '  ' + '-' * 44,
        ]

        for p in self._photos:
            lines.append(f'    {os.path.basename(p)}')

        lines += [
            '',
            '  SLALOM LEG (WP1 → WP2)',
            '  ' + '-' * 44,
            f'    Activated: {"yes" if self._weave_logged else "no"}',
            '',
            '=' * 62,
        ]

        for line in lines:
            self.get_logger().info(line)

        txt_path = os.path.join(save_dir, f'summary_{ts_str}.txt')
        try:
            with open(txt_path, 'w') as f:
                f.write('\n'.join(lines))
                f.write('\n\n--- FULL EVENT LOG ---\n')
                f.write('\n'.join(self._events))
            self.get_logger().info(f'Summary saved → {txt_path}')
        except Exception as e:
            self.get_logger().error(f'Could not save summary: {e}')

        self._save_map(save_dir, ts_str)

    def _save_map(self, save_dir: str, ts_str: str):
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            import matplotlib.patches as mpatches
        except ImportError:
            self.get_logger().warn(
                'matplotlib not installed — skipping map generation. '
                'Install with: pip3 install matplotlib'
            )
            return

        if not self._gps_path and not self._home_pos:
            self.get_logger().warn('No GPS path recorded — skipping map.')
            return

        home = self._home_pos or (self._current_lat, self._current_lon)

        def to_m(lat, lon):
            return self._to_metres(home[0], home[1], lat, lon)

        fig, ax = plt.subplots(figsize=(10, 10))

        if len(self._gps_path) > 1:
            xs = [to_m(p[0], p[1])[0] for p in self._gps_path]
            ys = [to_m(p[0], p[1])[1] for p in self._gps_path]
            ax.plot(xs, ys, 'b-', linewidth=2, label='Driven path', zorder=2)
            ax.plot(xs[0], ys[0], 'go', markersize=12, zorder=5)
            ax.plot(xs[-1], ys[-1], 'rs', markersize=10, zorder=5)

        ax.plot(0, 0, 'g*', markersize=18, label='Home', zorder=6)

        for wp, pos in self._wp_positions.items():
            x, y = to_m(pos[0], pos[1])
            ax.plot(x, y, 'r^', markersize=12, zorder=5)
            ax.annotate(
                wp, (x, y),
                textcoords='offset points',
                xytext=(6, 6),
                fontsize=11,
                color='red',
                fontweight='bold'
            )

        for pos in self._photo_positions:
            x, y = to_m(pos[0], pos[1])
            ax.plot(x, y, 'x', color='orange', markersize=10, markeredgewidth=2, zorder=4)

        for pos in self._detect_positions:
            x, y = to_m(pos[0], pos[1])
            ax.plot(x, y, 'D', color='purple', markersize=10, zorder=4)

        legend_items = [
            mpatches.Patch(color='blue', label='Driven path'),
            mpatches.Patch(color='green', label='Home / Start'),
            mpatches.Patch(color='red', label='Waypoints'),
            mpatches.Patch(color='orange', label='Cone photos'),
            mpatches.Patch(color='purple', label='Object detections'),
        ]
        ax.legend(handles=legend_items, loc='upper right', fontsize=10)

        ax.set_xlabel('East  (metres from home)', fontsize=12)
        ax.set_ylabel('North (metres from home)', fontsize=12)
        ax.set_title('Pioneer 3-AT — Part 2 Mission Path', fontsize=14)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.axhline(0, color='gray', linewidth=0.5)
        ax.axvline(0, color='gray', linewidth=0.5)

        map_path = os.path.join(save_dir, f'mission_map_{ts_str}.png')
        plt.tight_layout()
        plt.savefig(map_path, dpi=150)
        plt.close()
        self.get_logger().info(f'Mission map saved → {map_path}')


def main(args=None):
    rclpy.init(args=args)
    node = JourneySummary()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()