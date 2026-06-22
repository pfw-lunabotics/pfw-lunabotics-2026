#!/usr/bin/env python3
"""
Localization Quality Monitor for Lunabotics 2026
=================================================

Watches two signals to decide how reliable the robot's position estimate is:

  1. EKF odometry covariance  (/odometry/filtered, nav_msgs/Odometry)
     - Diagonal elements [0] and [7] are x- and y-position variances (m²).
     - Summed and compared against configurable thresholds.

  2. TF freshness  (map → base_link or odom → base_link)
     - If the newest available transform is older than `tf_timeout_sec`,
       the stack is considered lost regardless of the covariance.

Quality levels published on /perception/localization_quality (std_msgs/String):
  "good"      – covariance low, TF fresh
  "degraded"  – covariance elevated OR TF mildly stale
  "lost"      – no odometry / TF for > tf_timeout_sec, or covariance huge

A Float32 confidence score (0.0 – 1.0) is also published on
/perception/localization_confidence for BT nodes that need a numeric value.

Topic wired to EKF output:
  /odometry/filtered  (robot_localization EKF node output)

TF frames checked (first available wins):
  map  → base_footprint
  map  → base_link
  odom → base_link

Author: PFW Lunabotics Team
Date: February 2026
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32
import tf2_ros


# ---------------------------------------------------------------------------
# Default thresholds (can be overridden via YAML / launch params)
# ---------------------------------------------------------------------------
# Sum of x- and y-position variances that trigger quality degradation
_COV_GOOD_THRESH    = 0.04   # < 0.04 m² → good   (≈ ±14 cm 1-sigma each)
_COV_DEGRADED_THRESH = 0.25  # < 0.25 m² → degraded; ≥ 0.25 → lost

# TF pairs to try in priority order
TF_CANDIDATES = [
    ('map',  'base_footprint'),
    ('map',  'base_link'),
    ('odom', 'base_footprint'),
    ('odom', 'base_link'),
]


class LocalizationQualityMonitor(Node):
    """
    Publishes a human-readable quality label and a numeric confidence score
    derived from EKF covariance and TF freshness.
    """

    def __init__(self):
        super().__init__('localization_quality_monitor')

        # --- Parameters ---
        self.declare_parameters(
            namespace='',
            parameters=[
                ('publish_rate',        2.0),    # Hz
                ('odom_timeout_sec',    2.0),    # sec before odom is "stale"
                ('tf_timeout_sec',      2.0),    # sec before TF is "stale"
                ('cov_good_thresh',     _COV_GOOD_THRESH),
                ('cov_degraded_thresh', _COV_DEGRADED_THRESH),
            ]
        )

        self._odom_timeout    = self.get_parameter('odom_timeout_sec').value
        self._tf_timeout      = self.get_parameter('tf_timeout_sec').value
        self._cov_good        = self.get_parameter('cov_good_thresh').value
        self._cov_degraded    = self.get_parameter('cov_degraded_thresh').value
        publish_rate          = self.get_parameter('publish_rate').value

        # --- TF ---
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # --- Subscribers ---
        self._odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self._odom_callback,
            10,
        )

        # --- Publishers ---
        self._quality_pub = self.create_publisher(
            String, '/perception/localization_quality', 10
        )
        self._confidence_pub = self.create_publisher(
            Float32, '/perception/localization_confidence', 10
        )

        # --- Timer ---
        self._timer = self.create_timer(1.0 / publish_rate, self._timer_callback)

        # --- State ---
        self._last_odom_time: float | None = None   # ROS time in seconds
        self._last_cov_sum: float = float('inf')    # x+y position variance sum
        self._last_quality  = 'unknown'

        self.get_logger().info('Localization Quality Monitor initialized')
        self.get_logger().info(
            f'Thresholds — cov_good: {self._cov_good:.3f} m², '
            f'cov_degraded: {self._cov_degraded:.3f} m², '
            f'odom_timeout: {self._odom_timeout:.1f} s, '
            f'tf_timeout: {self._tf_timeout:.1f} s'
        )

    # -----------------------------------------------------------------------

    def _odom_callback(self, msg: Odometry):
        """Cache the latest EKF odometry covariance and arrival time."""
        self._last_odom_time = self.get_clock().now().nanoseconds * 1e-9
        # Position covariance: row-major 6×6; indices [0] = xx, [7] = yy
        cov = msg.pose.covariance
        self._last_cov_sum = cov[0] + cov[7]   # x-var + y-var (m²)

    def _check_tf_fresh(self) -> bool:
        """Return True if any TF candidate has a transform newer than tf_timeout."""
        now = rclpy.time.Time()
        timeout = Duration(seconds=0.1)
        for parent, child in TF_CANDIDATES:
            try:
                tf_stamped = self._tf_buffer.lookup_transform(
                    parent, child, now, timeout
                )
                age = (
                    self.get_clock().now()
                    - rclpy.time.Time.from_msg(tf_stamped.header.stamp)
                ).nanoseconds * 1e-9
                if age < self._tf_timeout:
                    return True
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                continue
        return False

    def _compute_quality(self) -> tuple[str, float]:
        """
        Return (quality_label, confidence_score).

        confidence_score: 1.0 = perfect, 0.0 = totally lost.
        Derived by mapping cov_sum through the two thresholds and combining
        with TF freshness.
        """
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        # ---- Check odometry recency ----
        odom_ok = (
            self._last_odom_time is not None
            and (now_sec - self._last_odom_time) < self._odom_timeout
        )

        # ---- Check TF freshness ----
        tf_ok = self._check_tf_fresh()

        if not odom_ok and not tf_ok:
            return 'lost', 0.0

        # ---- Covariance-based score ----
        cov = self._last_cov_sum
        if math.isinf(cov) or cov >= self._cov_degraded:
            cov_score = 0.0
        elif cov >= self._cov_good:
            # Linear interpolation between good and degraded thresholds
            span = self._cov_degraded - self._cov_good
            cov_score = 0.5 * (1.0 - (cov - self._cov_good) / span)
        else:
            # Interpolate 0.5–1.0 as cov approaches 0 from cov_good
            cov_score = 0.5 + 0.5 * (1.0 - cov / self._cov_good)

        # TF freshness bonus: losing TF halves the score
        confidence = cov_score if tf_ok else cov_score * 0.5

        if not odom_ok:
            # Odom stale — cap at degraded territory
            confidence = min(confidence, 0.4)

        # ---- Label ----
        if confidence >= 0.7:
            label = 'good'
        elif confidence >= 0.3:
            label = 'degraded'
        else:
            label = 'lost'

        return label, round(confidence, 3)

    def _timer_callback(self):
        label, confidence = self._compute_quality()

        if label != self._last_quality:
            self.get_logger().info(
                f'Localization quality: {self._last_quality} → {label} '
                f'(confidence={confidence:.2f}, cov_sum={self._last_cov_sum:.4f} m²)'
            )
            self._last_quality = label

        q_msg = String()
        q_msg.data = label
        self._quality_pub.publish(q_msg)

        c_msg = Float32()
        c_msg.data = confidence
        self._confidence_pub.publish(c_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationQualityMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
