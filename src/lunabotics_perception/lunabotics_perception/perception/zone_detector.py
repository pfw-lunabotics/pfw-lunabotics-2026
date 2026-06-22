#!/usr/bin/env python3
"""
Zone Detector for Lunabotics 2026 — UCF Arena
================================================

Determines which arena zone the robot currently occupies by reading
its position from TF and comparing against known zone boundaries.

Published on /perception/current_zone at 5 Hz.

UCF Arena 2026 (single-team half, origin at center):
  Total: 8.10m (X) × 4.57m (Y)
  X: [-4.05, 4.05]   (-X = judges/construction, +X = ingress/excavation)
  Y: [-2.285, 2.285]  (-Y = back wall, +Y = divider)

  Zones (horizontal bands, full Y height):
    Excavation:   X [0.05, 4.05]    4.0m wide (right side, digging area)
    Starting:     X [2.05, 4.05]    2.0m wide (subset of excavation, random start)
    Obstacle:     X [-4.05, 0.05]   4.1m wide (center-left, rocks & craters)
    Construction: X [-4.05, -1.45]  2.6m wide (leftmost, berm zone inside)

  The obstacle zone (4.1m) INCLUDES the construction zone (2.6m).
  Construction is a higher-priority match, so when the robot is in
  X [-4.05, -1.45] it reports "construction", not "obstacle".

  Zone priority (first match wins):
    starting > construction > excavation > obstacle > unknown

TF lookup order:
  1. map → base_footprint   (full stack: Point-LIO + EKF running)
  2. map → base_link        (full stack, no footprint frame)
  3. odom → base_footprint  (fallback: EKF only, no global map)
  4. odom → base_link       (minimal fallback)
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String
import tf2_ros


# ---------------------------------------------------------------------------
# Zone boundary defaults — UCF Arena 2026
# 8.10m × 4.57m, centered at origin
# ---------------------------------------------------------------------------
ZONE_BOUNDARIES = {
    'starting': {
        'x': (2.05, 4.05),
        'y': (-2.285, 2.285),
    },
    'excavation': {
        'x': (0.05, 4.05),
        'y': (-2.285, 2.285),
    },
    'construction': {
        'x': (-4.05, -1.45),
        'y': (-2.285, 2.285),
    },
    'obstacle': {
        'x': (-4.05, 0.05),
        'y': (-2.285, 2.285),
    },
}

# Zones checked in this order — first match wins when zones overlap
ZONE_PRIORITY = ['starting', 'construction', 'excavation', 'obstacle']

# TF frames to try, in order of preference
# 'arena' frame = arena center (published by mission_controller from start pose)
# 'map' frame = Point-LIO origin (fallback if arena TF not yet available)
TF_CANDIDATES = [
    ('arena', 'base_footprint'),
    ('arena', 'base_link'),
    ('map',   'base_footprint'),
    ('map',   'base_link'),
    ('odom',  'base_footprint'),
    ('odom',  'base_link'),
]


class ZoneDetector(Node):
    """
    Classifies the robot's current arena zone and publishes it as a string.
    """

    def __init__(self):
        super().__init__('zone_detector')

        # --- Parameters ---
        self.declare_parameters(
            namespace='',
            parameters=[
                ('publish_rate', 5.0),

                # Zone boundaries — overridable via YAML for different arenas
                ('zones.starting.x_min',      2.05),
                ('zones.starting.x_max',      4.05),
                ('zones.starting.y_min',     -2.285),
                ('zones.starting.y_max',      2.285),

                ('zones.excavation.x_min',    0.05),
                ('zones.excavation.x_max',    4.05),
                ('zones.excavation.y_min',   -2.285),
                ('zones.excavation.y_max',    2.285),

                ('zones.construction.x_min', -4.05),
                ('zones.construction.x_max', -1.45),
                ('zones.construction.y_min', -2.285),
                ('zones.construction.y_max',  2.285),

                ('zones.obstacle.x_min',     -4.05),
                ('zones.obstacle.x_max',      0.05),
                ('zones.obstacle.y_min',     -2.285),
                ('zones.obstacle.y_max',      2.285),
            ]
        )

        # Load zone boundaries from parameters
        self._zones = {}
        for zone in ZONE_PRIORITY:
            self._zones[zone] = {
                'x': (
                    self.get_parameter(f'zones.{zone}.x_min').value,
                    self.get_parameter(f'zones.{zone}.x_max').value,
                ),
                'y': (
                    self.get_parameter(f'zones.{zone}.y_min').value,
                    self.get_parameter(f'zones.{zone}.y_max').value,
                ),
            }

        publish_rate = self.get_parameter('publish_rate').value

        # --- TF ---
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # --- Publisher ---
        self._zone_pub = self.create_publisher(String, '/perception/current_zone', 10)

        # --- Timer ---
        self._timer = self.create_timer(1.0 / publish_rate, self._timer_callback)

        # State
        self._current_zone = 'unknown'
        self._active_tf_pair = None

        self.get_logger().info('Zone Detector initialized (UCF Arena 8.10m × 4.57m)')
        self.get_logger().info(f'Publishing /perception/current_zone at {publish_rate} Hz')
        for name, bounds in self._zones.items():
            self.get_logger().info(
                f'  {name:14s}: x{list(bounds["x"])}, y{list(bounds["y"])}'
            )

    def _get_robot_position(self):
        timeout = Duration(seconds=0.5)

        candidates = list(TF_CANDIDATES)
        if self._active_tf_pair and self._active_tf_pair in candidates:
            candidates.remove(self._active_tf_pair)
            candidates.insert(0, self._active_tf_pair)

        for parent, child in candidates:
            try:
                tf_stamped = self._tf_buffer.lookup_transform(
                    parent, child, rclpy.time.Time(), timeout
                )
                t = tf_stamped.transform.translation
                self._active_tf_pair = (parent, child)
                return t.x, t.y
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                continue

        return None

    def _classify_zone(self, x, y):
        for zone in ZONE_PRIORITY:
            bounds = self._zones[zone]
            x_min, x_max = bounds['x']
            y_min, y_max = bounds['y']
            if x_min <= x <= x_max and y_min <= y <= y_max:
                return zone
        return 'unknown'

    def _timer_callback(self):
        pos = self._get_robot_position()

        if pos is None:
            if self._current_zone == 'unknown':
                self.get_logger().warn(
                    'ZoneDetector: TF not available yet',
                    throttle_duration_sec=5.0
                )
        else:
            x, y = pos
            new_zone = self._classify_zone(x, y)

            if new_zone != self._current_zone:
                self.get_logger().info(
                    f'Zone: {self._current_zone} → {new_zone} '
                    f'(x={x:.2f}, y={y:.2f})'
                )
                self._current_zone = new_zone

        msg = String()
        msg.data = self._current_zone
        self._zone_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ZoneDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
