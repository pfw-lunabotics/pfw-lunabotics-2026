#!/usr/bin/env python3
"""
reactive_wander.py — Reactive obstacle avoidance with LiDAR visualization
==========================================================================
Reads /unilidar/cloud, splits into angular sectors, drives toward the
most open direction. LiDAR is centered on the robot (permanent front mount).

Publishes to /cmd_vel by default (goes through cmd_vel_relay for safety limits
and E-stop enforcement). Use output_topic:=/cmd_vel_safe to bypass relay
when running standalone without the full bringup.

Usage:
  python3 reactive_wander.py --ros-args -p duration:=60.0 -p debug_viz:=true
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
import struct
import math
import time


# LiDAR is centered on robot (permanent front mount, URDF: x=0.62, y=0.0)
LIDAR_Y_OFFSET = 0.0     # LiDAR centered on robot
ROBOT_WIDTH = 0.66        # chassis width in meters
ROBOT_HALF_W = ROBOT_WIDTH / 2.0  # 0.33m from center to edge

# Clearance from LiDAR to each chassis edge (symmetric since centered)
LEFT_CLEARANCE = ROBOT_HALF_W    # 0.33m
RIGHT_CLEARANCE = ROBOT_HALF_W   # 0.33m



class ReactiveWander(Node):

    def __init__(self):
        super().__init__('reactive_wander')

        # Parameters
        self.declare_parameter('duration', 60.0)
        self.declare_parameter('forward_speed', 0.25)
        self.declare_parameter('turn_speed', 0.5)
        self.declare_parameter('turn_speed_max', 1.2)  # adaptive max when obstacle very close
        self.declare_parameter('stop_dist', 0.55)
        self.declare_parameter('slow_dist', 1.0)
        self.declare_parameter('height_min', -0.60)
        self.declare_parameter('height_max', 0.80)
        self.declare_parameter('range_max', 3.0)
        self.declare_parameter('debug_viz', False)
        self.declare_parameter('side_buffer', 0.08)  # extra buffer beyond chassis edge
        self.declare_parameter('use_perception', False)  # use perception pipeline output
        self.declare_parameter('output_topic', '/cmd_vel')  # /cmd_vel = through relay, /cmd_vel_safe = bypass
        # Reverse-scan behavior: when front 180° is blocked, back up then scan
        self.declare_parameter('reverse_speed', 0.15)
        self.declare_parameter('reverse_duration', 2.0)  # seconds to reverse
        self.declare_parameter('scan_speed', 0.6)         # rad/s rotation during scan
        self.declare_parameter('scan_duration', 4.0)      # seconds to scan (~220° at 0.6 rad/s)
        self.declare_parameter('blocked_fraction', 0.6)   # fraction of front 180° sectors blocked to trigger
        # Anti-sink: minimum linear velocity while rotating (sand sinks wheels during in-place turns)
        self.declare_parameter('min_turn_linear', 0.15)   # m/s creep while turning

        self.duration = self.get_parameter('duration').value
        self.fwd_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.turn_speed_max = self.get_parameter('turn_speed_max').value
        self.stop_dist = self.get_parameter('stop_dist').value
        self.slow_dist = self.get_parameter('slow_dist').value
        self.h_min = self.get_parameter('height_min').value
        self.h_max = self.get_parameter('height_max').value
        self.r_max = self.get_parameter('range_max').value
        self.debug_viz = self.get_parameter('debug_viz').value
        self.use_perception = self.get_parameter('use_perception').value
        side_buf = self.get_parameter('side_buffer').value
        self.reverse_speed = self.get_parameter('reverse_speed').value
        self.reverse_duration = self.get_parameter('reverse_duration').value
        self.scan_speed = self.get_parameter('scan_speed').value
        self.scan_duration = self.get_parameter('scan_duration').value
        self.blocked_fraction = self.get_parameter('blocked_fraction').value
        self.min_turn_linear = self.get_parameter('min_turn_linear').value

        # Side clearance thresholds (distance from LiDAR to trigger side avoidance)
        self.left_side_thresh = LEFT_CLEARANCE + side_buf    # ~0.67m
        self.right_side_thresh = RIGHT_CLEARANCE + side_buf  # ~0.23m

        # 36 sectors of 10° each
        # Sector 0 = -180° (behind), sector 18 = 0° (forward)
        # Sector 9 = -90° (RIGHT in LiDAR frame), sector 27 = +90° (LEFT in LiDAR frame)
        self.n_sectors = 36
        self.sector_width = 2 * math.pi / self.n_sectors
        self.sector_min_dist = [self.r_max] * self.n_sectors

        # State
        self.start_time = None
        self.cloud_count = 0
        self.last_cloud_time = 0.0
        self.estopped = False
        self.last_viz_time = 0.0

        # Reverse-scan state machine: 'normal' → 'reversing' → 'scanning' → 'normal'
        self._maneuver_state = 'normal'
        self._maneuver_start = 0.0
        self._best_scan_idx = 0       # best sector found during scan
        self._best_scan_dist = 0.0    # distance of best sector during scan
        self._scan_direction = 1.0    # +1 or -1, chosen at scan start

        # Publishers / subscribers
        output_topic = self.get_parameter('output_topic').value
        self.cmd_pub = self.create_publisher(Twist, output_topic, 10)
        if self.use_perception:
            # Subscribe to perception pipeline output (rocks + inverted craters)
            cloud_topic = '/perception/unified_obstacles'
            # Also subscribe to raw cloud just for heartbeat (perception publishes at 2Hz)
            self.create_subscription(PointCloud2, '/unilidar/cloud', self._heartbeat_cb, 5)
        else:
            cloud_topic = '/unilidar/cloud'
        self.create_subscription(PointCloud2, cloud_topic, self._cloud_cb, 5)
        self.create_subscription(Bool, '/estop', self._estop_cb, 10)
        self.create_subscription(Bool, '/safety/estop', self._estop_cb, 10)

        # Control loop at 10 Hz
        self.create_timer(0.1, self._control_loop)

        mode = 'PERCEPTION (rocks+craters)' if self.use_perception else 'RAW LiDAR'
        self.get_logger().info(
            f'Reactive wander [{mode}]: duration={self.duration}s, '
            f'fwd={self.fwd_speed}m/s, turn={self.turn_speed}-{self.turn_speed_max}rad/s, '
            f'stop={self.stop_dist}m, slow={self.slow_dist}m'
        )
        self.get_logger().info(
            f'LiDAR offset: y={LIDAR_Y_OFFSET}m | '
            f'left_thresh={self.left_side_thresh:.2f}m, right_thresh={self.right_side_thresh:.2f}m'
        )
        self.get_logger().info(f'debug_viz={self.debug_viz}')
        self.get_logger().info(f'Listening on: {cloud_topic}')
        self.get_logger().info(f'Publishing to: {output_topic}')
        self.get_logger().info('Press "e" in teleop terminal to E-STOP')

    def _heartbeat_cb(self, msg: PointCloud2):
        """Keep last_cloud_time updated from raw LiDAR (perception publishes slower)."""
        self.last_cloud_time = time.time()
        if self.start_time is None:
            self.start_time = time.time()
            self.get_logger().info('Raw LiDAR heartbeat received. Waiting for perception...')

    def _estop_cb(self, msg: Bool):
        self.estopped = msg.data
        if self.estopped:
            self.get_logger().warn('E-STOP received — wander paused')
            self._stop()
        else:
            self.get_logger().info('E-STOP released — wander resumed')

    def _cloud_cb(self, msg: PointCloud2):
        """Parse PointCloud2 into sector-based min distances."""
        if self.start_time is None:
            self.start_time = time.time()
            self.get_logger().info('LiDAR data received. Wandering!')

        self.cloud_count += 1
        self.last_cloud_time = time.time()

        x_off = y_off = z_off = None
        for field in msg.fields:
            if field.name == 'x':
                x_off = field.offset
            elif field.name == 'y':
                y_off = field.offset
            elif field.name == 'z':
                z_off = field.offset

        if x_off is None or y_off is None or z_off is None:
            return

        step = msg.point_step
        data = msg.data
        sectors = [self.r_max] * self.n_sectors

        for i in range(0, len(data) - step + 1, step):
            x = struct.unpack_from('<f', data, i + x_off)[0]
            y = struct.unpack_from('<f', data, i + y_off)[0]
            z = struct.unpack_from('<f', data, i + z_off)[0]

            if math.isnan(x) or math.isnan(y) or math.isnan(z):
                continue
            if math.isinf(x) or math.isinf(y) or math.isinf(z):
                continue

            # Skip height filter in perception mode — already ground-segmented
            if not self.use_perception:
                if z < self.h_min or z > self.h_max:
                    continue

            dist = math.sqrt(x * x + y * y)
            if dist < 0.05 or dist > self.r_max:
                continue

            angle = math.atan2(y, x)  # -pi to pi, 0 = forward
            sector_idx = int((angle + math.pi) / self.sector_width) % self.n_sectors
            if dist < sectors[sector_idx]:
                sectors[sector_idx] = dist

        self.sector_min_dist = sectors

    def _print_viz(self, cmd, front_min, left_min, right_min, action):
        """Print ASCII top-down radar view of sector distances."""
        now = time.time()
        if now - self.last_viz_time < 2.0:
            return
        self.last_viz_time = now
        elapsed = now - self.start_time

        s = self.sector_min_dist
        n = self.n_sectors
        # Sector indices for key directions (36 sectors, 10° each)
        # 0=-180°(back), 9=-90°(right), 18=0°(front), 27=+90°(left)

        def d(idx):
            """Format distance for display."""
            v = s[idx % n]
            if v >= self.r_max:
                return " -- "
            return f"{v:.1f}".rjust(4)

        # Gather sector groups for display
        # Back: sectors 34,35,0,1,2 (-60° to -140° from back)
        # Front: sectors 16,17,18,19,20
        # Left: sectors 25,26,27,28,29
        # Right: sectors 7,8,9,10,11

        lines = [
            f"",
            f"  ┌─────── LiDAR Radar [{elapsed:.0f}s] ── action: {action} ───┐",
            f"  │           FRONT                          │",
            f"  │      {d(16)} {d(17)} {d(18)} {d(19)} {d(20)}            │",
            f"  │ L  {d(25)}                    {d(11)}  R   │",
            f"  │    {d(26)}   [==ROBOT==]      {d(10)}      │",
            f"  │    {d(27)}   [ L  *  R ]      {d(9)}       │",
            f"  │    {d(28)}                    {d(8)}       │",
            f"  │      {d(34)} {d(35)} {d(0)}  {d(1)}  {d(2)}            │",
            f"  │           BACK                           │",
            f"  ├────────────────────────────────────────────┤",
            f"  │ front={front_min:.2f}m  left={left_min:.2f}m  right={right_min:.2f}m │",
            f"  │ speed={cmd.linear.x:.2f}m/s  turn={cmd.angular.z:.2f}rad/s        │",
            f"  │ *=LiDAR (centered, front mount)            │",
            f"  └────────────────────────────────────────────┘",
        ]
        print('\n'.join(lines))

    def _adaptive_turn(self, obstacle_dist, threshold_dist):
        """Scale turn speed: normal when far, max when very close.

        At threshold_dist → turn_speed (normal cap)
        At 0 → turn_speed_max (emergency dodge)
        Linear interpolation between them.
        """
        if obstacle_dist >= threshold_dist:
            return self.turn_speed
        ratio = max(0.0, obstacle_dist / threshold_dist)  # 1.0=far, 0.0=touching
        # Invert: closer → higher speed
        return self.turn_speed + (1.0 - ratio) * (self.turn_speed_max - self.turn_speed)

    def _turn_creep(self, front_min):
        """Return linear velocity to use while rotating (anti-sink).

        On sand, pure in-place rotation digs wheels in. Always creep:
          - Forward if front is clear (> stop_dist)
          - Backward if front is blocked
        """
        if front_min > self.stop_dist:
            return self.min_turn_linear
        else:
            return -self.min_turn_linear

    def _is_front_hemisphere_blocked(self, sectors, front_idx, n):
        """Check if the front 180° (±90°) is mostly blocked."""
        front_hemi = [(front_idx + i) % n for i in range(-9, 10)]  # ±90°
        blocked_count = sum(1 for i in front_hemi if sectors[i] < self.stop_dist)
        return blocked_count >= len(front_hemi) * self.blocked_fraction

    def _control_loop(self):
        """Decide velocity based on sector distances with side protection.

        State machine:
          normal    — reactive avoidance (cruise/slow/stop/side)
          reversing — front 180° blocked, backing up to create space
          scanning  — rotating to find the most open direction
          turning   — turning to face the best direction found during scan
        """
        now = time.time()

        if self.start_time is not None and (now - self.start_time) >= self.duration:
            self.get_logger().info(
                f'Wander complete. {self.cloud_count} clouds in {self.duration:.0f}s.'
            )
            self._stop()
            raise SystemExit(0)

        if self.estopped:
            self._stop()
            return

        # Wait for first cloud before starting (don't stop during initial boot)
        if self.start_time is None:
            self._stop()
            return

        # Perception publishes at 2Hz, so allow longer gap before stopping
        cloud_timeout = 5.0 if self.use_perception else 2.0
        if (now - self.last_cloud_time) > cloud_timeout:
            self._stop()
            return

        sectors = self.sector_min_dist
        n = self.n_sectors
        front_idx = n // 2  # sector 18 = 0° (forward)

        # === FRONT CHECK ===
        # Symmetric: LiDAR is centered, check ±40° (sectors 14-22)
        front_left_indices = [(front_idx + i) % n for i in range(0, 5)]    # 0° to +40° (left)
        front_right_indices = [(front_idx + i) % n for i in range(-4, 0)]  # -40° to 0° (right)
        front_indices = front_right_indices + front_left_indices
        front_min = min(sectors[i] for i in front_indices)

        # === SIDE CHECKS ===
        # LiDAR is centered (x=+0.62, y=0.0), symmetric side arcs
        # Left side: +50° to +130°
        left_indices = [(front_idx + i) % n for i in range(5, 14)]
        left_min = min(sectors[i] for i in left_indices)

        # Right side: -50° to -130°
        right_indices = [(front_idx + i) % n for i in range(-13, -4)]
        right_min = min(sectors[i] for i in right_indices)

        cmd = Twist()
        action = ""

        # =============================================================
        # REVERSE-SCAN STATE MACHINE
        # When front 180° is heavily blocked, back up then scan for the
        # best direction instead of just spinning in place.
        # =============================================================
        maneuver_elapsed = now - self._maneuver_start

        if self._maneuver_state == 'reversing':
            if maneuver_elapsed >= self.reverse_duration:
                # Done reversing — start scanning
                self._maneuver_state = 'scanning'
                self._maneuver_start = now
                self._best_scan_idx = max(range(n), key=lambda i: sectors[i])
                self._best_scan_dist = sectors[self._best_scan_idx]
                # Pick scan direction: rotate toward the side with more space
                self._scan_direction = 1.0 if left_min >= right_min else -1.0
                self.get_logger().info('Reverse done — scanning for best direction...')
            else:
                cmd.linear.x = -self.reverse_speed
                cmd.angular.z = 0.0
                action = f"REVERSE({maneuver_elapsed:.1f}s/{self.reverse_duration:.0f}s)"
                self.cmd_pub.publish(cmd)
                if self.debug_viz:
                    self._print_viz(cmd, front_min, left_min, right_min, action)
                return

        if self._maneuver_state == 'scanning':
            # Track the most open sector seen during rotation
            current_best = max(range(n), key=lambda i: sectors[i])
            if sectors[current_best] > self._best_scan_dist:
                self._best_scan_idx = current_best
                self._best_scan_dist = sectors[current_best]

            if maneuver_elapsed >= self.scan_duration:
                # Done scanning — turn to face the best direction
                self._maneuver_state = 'turning'
                self._maneuver_start = now
                self.get_logger().info(
                    f'Scan done — best sector {self._best_scan_idx} '
                    f'at {self._best_scan_dist:.1f}m')
            else:
                cmd.linear.x = self._turn_creep(front_min)
                cmd.angular.z = self.scan_speed * self._scan_direction
                action = (f"SCAN({maneuver_elapsed:.1f}s/{self.scan_duration:.0f}s "
                          f"best={self._best_scan_dist:.1f}m)")
                self.cmd_pub.publish(cmd)
                if self.debug_viz:
                    self._print_viz(cmd, front_min, left_min, right_min, action)
                return

        if self._maneuver_state == 'turning':
            # Turn until the best sector is in front (sector 18 ± 2)
            best_angle = -math.pi + (self._best_scan_idx + 0.5) * self.sector_width
            angle_to_best = abs(best_angle)
            # Best sector is "in front" when it's within ±20° of forward
            if angle_to_best < 0.35 or front_min > self.slow_dist or maneuver_elapsed > 5.0:
                # Aligned or front is clear — resume normal avoidance
                self._maneuver_state = 'normal'
                self.get_logger().info(
                    f'Turn complete — resuming (front={front_min:.1f}m)')
            else:
                cmd.linear.x = self._turn_creep(front_min)
                cmd.angular.z = self.turn_speed_max if best_angle >= 0 else -self.turn_speed_max
                action = f"TURNING(angle={math.degrees(best_angle):.0f}°)"
                self.cmd_pub.publish(cmd)
                if self.debug_viz:
                    self._print_viz(cmd, front_min, left_min, right_min, action)
                return

        # =============================================================
        # NORMAL REACTIVE AVOIDANCE
        # =============================================================
        left_danger = left_min < self.left_side_thresh    # <0.41m from LiDAR = chassis will clip
        right_danger = right_min < self.right_side_thresh  # <0.41m from LiDAR = chassis will clip

        # Check if front hemisphere is blocked → trigger reverse-scan
        front_hemi_blocked = self._is_front_hemisphere_blocked(sectors, front_idx, n)
        if front_hemi_blocked and (front_min < self.stop_dist or (left_danger and right_danger)):
            self._maneuver_state = 'reversing'
            self._maneuver_start = now
            self.get_logger().info(
                f'Front 180° blocked (front={front_min:.2f}m) — reversing')
            cmd.linear.x = -self.reverse_speed
            cmd.angular.z = 0.0
            action = "REVERSE(start)"
            self.cmd_pub.publish(cmd)
            if self.debug_viz:
                self._print_viz(cmd, front_min, left_min, right_min, action)
            return

        # === PRIORITY 1: SIDE COLLISION ===
        if left_danger and right_danger:
            # Boxed in on both sides — creep while turning toward most open
            best_idx = max(range(n), key=lambda i: sectors[i])
            best_angle = -math.pi + (best_idx + 0.5) * self.sector_width
            box_min = min(left_min, right_min)
            t = self._adaptive_turn(box_min, max(self.left_side_thresh, self.right_side_thresh))
            # If best direction is behind, reverse; otherwise creep forward (anti-sink)
            if abs(best_angle) > math.pi / 2:
                cmd.linear.x = -self.min_turn_linear
            else:
                cmd.linear.x = self.min_turn_linear
            cmd.angular.z = t if best_angle >= 0 else -t
            action = "SIDE:BOXED"
        elif left_danger:
            # Left chassis edge too close — steer right, slow down
            cmd.linear.x = self.fwd_speed * 0.3
            t = self._adaptive_turn(left_min, self.left_side_thresh)
            cmd.angular.z = -t  # steer right (negative = clockwise)
            action = f"SIDE:LEFT({left_min:.2f}<{self.left_side_thresh:.2f})"
        elif right_danger:
            # Right chassis edge too close — steer left
            cmd.linear.x = self.fwd_speed * 0.3
            t = self._adaptive_turn(right_min, self.right_side_thresh)
            cmd.angular.z = t  # steer left
            action = f"SIDE:RIGHT({right_min:.2f}<{self.right_side_thresh:.2f})"

        # === PRIORITY 2: FRONT COLLISION ===
        elif front_min < self.stop_dist:
            best_idx = max(range(n), key=lambda i: sectors[i])
            best_angle = -math.pi + (best_idx + 0.5) * self.sector_width
            t = self._adaptive_turn(front_min, self.stop_dist)
            # Front blocked — creep backward while turning (anti-sink)
            cmd.linear.x = -self.min_turn_linear
            cmd.angular.z = t if best_angle >= 0 else -t
            action = f"FRONT:STOP({front_min:.2f})"

        # === PRIORITY 3: FRONT SLOW ZONE ===
        elif front_min < self.slow_dist:
            speed_factor = (front_min - self.stop_dist) / (self.slow_dist - self.stop_dist)
            cmd.linear.x = self.fwd_speed * max(0.2, speed_factor)

            # Steer toward more open side — adaptive turn in slow zone too
            t = self._adaptive_turn(front_min, self.slow_dist)
            steer_left = min(sectors[i] for i in front_left_indices)
            steer_right = min(sectors[i] for i in front_right_indices)
            if steer_right > steer_left:
                cmd.angular.z = -t * 0.6  # scale down since still moving forward
            elif steer_left > steer_right:
                cmd.angular.z = t * 0.6
            action = f"FRONT:SLOW({front_min:.2f})"

        # === PRIORITY 4: CRUISE ===
        else:
            cmd.linear.x = self.fwd_speed
            cmd.angular.z = 0.0
            action = f"CRUISE({front_min:.2f})"

        self.cmd_pub.publish(cmd)

        # ASCII viz or regular log
        if self.debug_viz:
            self._print_viz(cmd, front_min, left_min, right_min, action)
        elif self.cloud_count % 20 == 0:
            elapsed = now - self.start_time
            self.get_logger().info(
                f'[{elapsed:.0f}s] front={front_min:.2f} left={left_min:.2f} '
                f'right={right_min:.2f} | {action}'
            )

    def _stop(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveWander()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            stop = Twist()
            node.cmd_pub.publish(stop)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
