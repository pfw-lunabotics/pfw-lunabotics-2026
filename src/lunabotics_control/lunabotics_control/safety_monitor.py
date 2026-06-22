#!/usr/bin/env python3
"""
Safety Monitor for Lunabotics 2026
====================================
Monitors critical system health and publishes E-stop when failures detected.

Monitors:
  1. LiDAR timeout — no /unilidar/cloud for N seconds
  2. IMU timeout — no /unilidar/imu for N seconds
  3. Localization quality — CRITICAL status from /localization/quality
  4. Odometry timeout — no /odom for N seconds

Publishes:
  /safety/estop (std_msgs/Bool) — True = STOP, False = OK
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


class SafetyMonitor(Node):

    def __init__(self):
        super().__init__('safety_monitor')

        # Parameters
        self.declare_parameter('lidar_timeout', 3.0)
        self.declare_parameter('imu_timeout', 3.0)
        self.declare_parameter('odom_timeout', 3.0)
        self.declare_parameter('require_odom', True)   # True = wheel encoders are connected
        self.declare_parameter('require_sensors', True)  # False = skip LiDAR/IMU checks (proof of life)
        self.declare_parameter('require_localization', True)  # False = skip localization quality check
        self.declare_parameter('check_rate', 5.0)
        self.declare_parameter('startup_grace_period', 15.0)
        self.declare_parameter('stall_estop', False)   # If True, encoder stall triggers E-stop

        self.lidar_timeout = self.get_parameter('lidar_timeout').value
        self.imu_timeout = self.get_parameter('imu_timeout').value
        self.odom_timeout = self.get_parameter('odom_timeout').value
        self.require_odom = self.get_parameter('require_odom').value
        self.require_sensors = self.get_parameter('require_sensors').value
        self.require_localization = self.get_parameter('require_localization').value
        self.stall_estop = self.get_parameter('stall_estop').value
        check_rate = self.get_parameter('check_rate').value
        self.startup_grace = self.get_parameter('startup_grace_period').value

        # Timestamps of last received messages
        now = self.get_clock().now()
        self.last_lidar = now
        self.last_imu = now
        self.last_odom = now
        self.localization_critical = False
        self.is_stalled = False
        self.boot_time = now

        # Subscribers
        self.create_subscription(PointCloud2, '/unilidar/cloud', self._lidar_cb, 5)
        self.create_subscription(Imu, '/unilidar/imu', self._imu_cb, 5)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 5)
        self.create_subscription(Bool, '/pico/stalled', self._stall_cb, 5)
        self.create_subscription(
            String, '/perception/localization_quality', self._loc_cb, 5
        )

        # Publisher
        self.estop_pub = self.create_publisher(Bool, '/safety/estop', 10)

        # Check timer
        self.create_timer(1.0 / check_rate, self._check_health)

        self.estop_active = False
        self.get_logger().info(
            f'Safety Monitor initialized (grace period: {self.startup_grace:.0f}s, '
            f'require_sensors={self.require_sensors}, '
            f'require_localization={self.require_localization})'
        )

    def _lidar_cb(self, msg):
        self.last_lidar = self.get_clock().now()

    def _imu_cb(self, msg):
        self.last_imu = self.get_clock().now()

    def _odom_cb(self, msg):
        self.last_odom = self.get_clock().now()

    def _stall_cb(self, msg):
        self.is_stalled = msg.data

    def _loc_cb(self, msg):
        self.localization_critical = (msg.data == 'lost')

    def _check_health(self):
        now = self.get_clock().now()
        reasons = []

        # Startup grace period — don't E-STOP while nodes are still launching
        uptime = (now - self.boot_time).nanoseconds / 1e9
        if uptime < self.startup_grace:
            msg = Bool()
            msg.data = False
            self.estop_pub.publish(msg)
            return

        # Re-read params so runtime changes via ros2 param set work
        self.require_sensors = self.get_parameter('require_sensors').value
        self.require_odom = self.get_parameter('require_odom').value
        self.require_localization = self.get_parameter('require_localization').value
        self.stall_estop = self.get_parameter('stall_estop').value

        if self.require_sensors:
            lidar_age = (now - self.last_lidar).nanoseconds / 1e9
            if lidar_age > self.lidar_timeout:
                reasons.append(f'LiDAR timeout ({lidar_age:.1f}s)')

            imu_age = (now - self.last_imu).nanoseconds / 1e9
            if imu_age > self.imu_timeout:
                reasons.append(f'IMU timeout ({imu_age:.1f}s)')

        if self.require_odom:
            odom_age = (now - self.last_odom).nanoseconds / 1e9
            if odom_age > self.odom_timeout:
                reasons.append(f'Odom timeout ({odom_age:.1f}s)')

        if self.require_localization and self.localization_critical:
            reasons.append('Localization CRITICAL')

        if self.stall_estop and self.is_stalled:
            reasons.append('Encoder STALL detected')

        should_estop = len(reasons) > 0

        # Publish
        msg = Bool()
        msg.data = should_estop
        self.estop_pub.publish(msg)

        # Log state changes
        if should_estop and not self.estop_active:
            self.get_logger().error(f'E-STOP ACTIVATED: {", ".join(reasons)}')
        elif not should_estop and self.estop_active:
            self.get_logger().info('E-STOP cleared — all sensors healthy')

        self.estop_active = should_estop


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
