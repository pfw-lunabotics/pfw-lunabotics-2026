#!/usr/bin/env python3
"""
System Watchdog — Lunabotics 2026
====================================
Monitors all subsystems and auto-recovers from known failure modes.

Unlike safety_monitor (which just E-stops), this node actively fixes problems:
  - Re-adds LiDAR IP if it drops from the ethernet interface
  - Restarts crashed nodes (LiDAR driver, Point-LIO, perception, EKF)
  - Detects Nav2 stuck states
  - Monitors Pico serial health
  - Publishes aggregate /system/health for the operator

Recovery has cooldowns — won't spam restarts. If recovery fails after
max attempts, escalates to E-stop.

Topics published:
  /system/health (std_msgs/String) — OK / WARN / ERROR / RECOVERING
  /system/watchdog_log (std_msgs/String) — human-readable event log

Topics monitored:
  /unilidar/cloud, /unilidar/imu, /point_lio/odom, /odometry/filtered,
  /perception/unified_obstacles, /perception/localization_quality,
  /pico/status, /mission/state, /cmd_vel_safe
"""

import subprocess
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, Float32
from geometry_msgs.msg import Twist


class SubsystemState:
    """Tracks health of a single subsystem."""

    def __init__(self, name, timeout, max_recovery_attempts=3, recovery_cooldown=30.0):
        self.name = name
        self.timeout = timeout
        self.max_recovery_attempts = max_recovery_attempts
        self.recovery_cooldown = recovery_cooldown

        self.last_seen = None  # wall-clock time.time()
        self.healthy = True
        self.recovery_attempts = 0
        self.last_recovery_time = 0.0
        self.failed = False  # True = gave up recovering

    def touch(self):
        """Mark as alive."""
        self.last_seen = time.time()
        if not self.healthy:
            self.healthy = True
            self.recovery_attempts = 0
            self.failed = False

    def check(self):
        """Returns True if healthy, False if timed out."""
        if self.last_seen is None:
            return True  # never received = still initializing
        age = time.time() - self.last_seen
        self.healthy = age < self.timeout
        return self.healthy

    def can_recover(self):
        """Check if recovery is allowed (cooldown + attempt limit)."""
        if self.failed:
            return False
        if self.recovery_attempts >= self.max_recovery_attempts:
            self.failed = True
            return False
        elapsed = time.time() - self.last_recovery_time
        return elapsed >= self.recovery_cooldown

    def mark_recovery(self):
        self.recovery_attempts += 1
        self.last_recovery_time = time.time()


class SystemWatchdog(Node):

    def __init__(self):
        super().__init__('system_watchdog')

        # --- Parameters ---
        self.declare_parameter('check_rate', 1.0)  # Hz
        self.declare_parameter('lidar_ip', '192.168.1.62')
        self.declare_parameter('jetson_ip', '192.168.1.2')
        self.declare_parameter('network_interface', 'enP8p1s0')
        self.declare_parameter('enable_network_recovery', True)
        self.declare_parameter('enable_node_restart', True)
        self.declare_parameter('enable_nav_recovery', True)
        self.declare_parameter('startup_grace_period', 15.0)  # seconds

        self.lidar_ip = self.get_parameter('lidar_ip').value
        self.jetson_ip = self.get_parameter('jetson_ip').value
        self.net_iface = self.get_parameter('network_interface').value
        self.enable_network = self.get_parameter('enable_network_recovery').value
        self.enable_restart = self.get_parameter('enable_node_restart').value
        self.enable_nav = self.get_parameter('enable_nav_recovery').value
        self.startup_grace = self.get_parameter('startup_grace_period').value
        check_rate = self.get_parameter('check_rate').value

        self.start_time = time.time()

        # --- Subsystem trackers ---
        # Timeouts: how long before we consider the subsystem dead
        self.subsystems = {
            'lidar': SubsystemState('LiDAR cloud', timeout=5.0, max_recovery_attempts=5),
            'imu': SubsystemState('IMU', timeout=5.0, max_recovery_attempts=3),
            'point_lio': SubsystemState('Point-LIO odom', timeout=8.0, max_recovery_attempts=3),
            'ekf': SubsystemState('EKF odom', timeout=8.0, max_recovery_attempts=3),
            'perception': SubsystemState('Perception obstacles', timeout=10.0, max_recovery_attempts=3),
            'cmd_vel': SubsystemState('cmd_vel_safe', timeout=30.0),  # can be idle legitimately
        }

        # Extra state tracking
        self.pico_status = 'UNKNOWN'
        self.localization_quality = 'unknown'
        self.localization_confidence = 1.0
        self.mission_state = 'IDLE'
        self.mission_state_time = time.time()
        self.last_network_check = 0.0
        self.network_ok = True
        self.nav_stuck_alerted = False

        # --- Subscribers ---
        self.create_subscription(PointCloud2, '/unilidar/cloud', self._lidar_cb, 5)
        self.create_subscription(Imu, '/unilidar/imu', self._imu_cb, 5)
        self.create_subscription(Odometry, '/point_lio/odom', self._plio_cb, 5)
        self.create_subscription(Odometry, '/odometry/filtered', self._ekf_cb, 5)
        self.create_subscription(PointCloud2, '/perception/unified_obstacles', self._percep_cb, 5)
        self.create_subscription(Twist, '/cmd_vel_safe', self._cmdvel_cb, 5)
        self.create_subscription(String, '/pico/status', self._pico_cb, 5)
        self.create_subscription(String, '/perception/localization_quality', self._locqual_cb, 5)
        self.create_subscription(Float32, '/perception/localization_confidence', self._locconf_cb, 5)
        self.create_subscription(String, '/mission/state', self._mission_cb, 5)

        # --- Publishers ---
        self.health_pub = self.create_publisher(String, '/system/health', 10)
        self.log_pub = self.create_publisher(String, '/system/watchdog_log', 10)
        self.estop_pub = self.create_publisher(Bool, '/estop', 10)

        # --- Main check loop ---
        self.create_timer(1.0 / check_rate, self._check_all)

        # --- Periodic network check (every 10s, not every tick) ---
        self.create_timer(10.0, self._check_network)

        self._log('System Watchdog started — grace period {:.0f}s'.format(self.startup_grace))

    # ------------------------------------------------------------------ #
    # Subscriber callbacks — just touch the subsystem tracker
    # ------------------------------------------------------------------ #
    def _lidar_cb(self, msg):
        self.subsystems['lidar'].touch()

    def _imu_cb(self, msg):
        self.subsystems['imu'].touch()

    def _plio_cb(self, msg):
        self.subsystems['point_lio'].touch()

    def _ekf_cb(self, msg):
        self.subsystems['ekf'].touch()

    def _percep_cb(self, msg):
        self.subsystems['perception'].touch()

    def _cmdvel_cb(self, msg):
        self.subsystems['cmd_vel'].touch()

    def _pico_cb(self, msg):
        self.pico_status = msg.data

    def _locqual_cb(self, msg):
        self.localization_quality = msg.data

    def _locconf_cb(self, msg):
        self.localization_confidence = msg.data

    def _mission_cb(self, msg):
        new_state = msg.data
        if new_state != self.mission_state:
            self.mission_state = new_state
            self.mission_state_time = time.time()
            self.nav_stuck_alerted = False

    # ------------------------------------------------------------------ #
    # Network check (runs on separate timer, every 10s)
    # ------------------------------------------------------------------ #
    def _check_network(self):
        if not self.enable_network:
            return

        # Check if LiDAR is reachable
        try:
            result = subprocess.run(
                ['ping', '-c', '1', '-W', '1', self.lidar_ip],
                capture_output=True, timeout=3
            )
            if result.returncode == 0:
                if not self.network_ok:
                    self._log('LiDAR network recovered — {} reachable'.format(self.lidar_ip))
                self.network_ok = True
                return
        except (subprocess.TimeoutExpired, OSError):
            pass

        self.network_ok = False

        # Check if our IP is even on the interface
        try:
            result = subprocess.run(
                ['ip', 'addr', 'show', self.net_iface],
                capture_output=True, text=True, timeout=3
            )
            if self.jetson_ip not in result.stdout:
                self._log('RECOVERING: {} IP missing from {} — re-adding'.format(
                    self.jetson_ip, self.net_iface))
                self._run_recovery_cmd([
                    'sudo', 'ip', 'addr', 'add',
                    '{}/24'.format(self.jetson_ip), 'dev', self.net_iface
                ])
                # Also bring interface up in case it went down
                self._run_recovery_cmd([
                    'sudo', 'ip', 'link', 'set', self.net_iface, 'up'
                ])
            else:
                self._log('WARNING: {} on {} but LiDAR {} unreachable — cable?'.format(
                    self.jetson_ip, self.net_iface, self.lidar_ip))
        except (subprocess.TimeoutExpired, OSError) as e:
            self._log('Network check error: {}'.format(e))

    # ------------------------------------------------------------------ #
    # Main health check (runs at check_rate Hz)
    # ------------------------------------------------------------------ #
    def _check_all(self):
        elapsed = time.time() - self.start_time
        if elapsed < self.startup_grace:
            # During grace period, just publish OK and don't recover
            self._publish_health('STARTING')
            return

        issues = []
        recovering = False

        # --- Check each subsystem ---
        for key, sub in self.subsystems.items():
            if not sub.check():
                if sub.failed:
                    issues.append('{} FAILED (gave up after {} attempts)'.format(
                        sub.name, sub.max_recovery_attempts))
                elif sub.can_recover():
                    success = self._recover_subsystem(key)
                    if success:
                        recovering = True
                    else:
                        issues.append('{} down, recovery attempted'.format(sub.name))
                else:
                    cooldown_remaining = sub.recovery_cooldown - (time.time() - sub.last_recovery_time)
                    if cooldown_remaining > 0:
                        issues.append('{} down (retry in {:.0f}s)'.format(
                            sub.name, cooldown_remaining))
                    else:
                        issues.append('{} down'.format(sub.name))

        # --- Network health ---
        if not self.network_ok:
            issues.append('LiDAR network unreachable')

        # --- Pico serial health ---
        if 'DISCONNECTED' in self.pico_status.upper():
            issues.append('Pico serial disconnected (auto-retrying)')

        # --- Localization quality ---
        if self.localization_quality == 'lost':
            issues.append('Localization LOST (confidence={:.2f})'.format(
                self.localization_confidence))

        # --- Nav2 stuck detection ---
        if self.enable_nav and self.mission_state.startswith('NAVIGATE'):
            nav_duration = time.time() - self.mission_state_time
            if nav_duration > 120.0 and not self.nav_stuck_alerted:
                self.nav_stuck_alerted = True
                issues.append('Nav2 stuck in {} for {:.0f}s'.format(
                    self.mission_state, nav_duration))
                self._log('WARNING: Navigation stuck — consider manual intervention')

        # --- Determine overall health ---
        if not issues:
            self._publish_health('OK')
        elif recovering:
            self._publish_health('RECOVERING')
        else:
            # Check if any subsystem has permanently failed
            any_failed = any(s.failed for s in self.subsystems.values()
                           if s.name != 'cmd_vel_safe')  # cmd_vel can be idle
            if any_failed:
                self._publish_health('ERROR')
                # E-stop if critical subsystems (LiDAR/IMU) have given up
                if self.subsystems['lidar'].failed or self.subsystems['imu'].failed:
                    self._escalate_estop('Sensor recovery exhausted')
            else:
                self._publish_health('WARN')

    # ------------------------------------------------------------------ #
    # Recovery actions per subsystem
    # ------------------------------------------------------------------ #
    def _recover_subsystem(self, key):
        """Attempt to recover a subsystem. Returns True if action was taken."""
        sub = self.subsystems[key]
        sub.mark_recovery()

        if key == 'lidar':
            # LiDAR driver is launched by bringup with ethernet params —
            # watchdog cannot restart it correctly (would launch in serial mode)
            self._log('LiDAR not publishing — bringup manages driver, skipping restart')
            return False

        elif key == 'imu':
            # IMU comes from the same LiDAR driver — same reason, skip
            self._log('IMU not publishing — bringup manages LiDAR driver, skipping restart')
            return False

        elif key == 'point_lio':
            self._log('RECOVERING: Point-LIO — restarting node (attempt {}/{})'.format(
                sub.recovery_attempts, sub.max_recovery_attempts))
            # Point-LIO needs its config file
            return self._restart_node_with_params(
                'point_lio', 'pointlio_mapping', 'point_lio',
                extra_note='Point-LIO restart — may need 5-10s to initialize')

        elif key == 'ekf':
            self._log('RECOVERING: EKF — restarting node (attempt {}/{})'.format(
                sub.recovery_attempts, sub.max_recovery_attempts))
            return self._restart_node('robot_localization', 'ekf_node',
                                      node_name='ekf_filter_node')

        elif key == 'perception':
            self._log('RECOVERING: Perception — restarting node (attempt {}/{})'.format(
                sub.recovery_attempts, sub.max_recovery_attempts))
            return self._restart_node('lunabotics_perception', 'unified_obstacle_detector',
                                      node_name='unified_obstacle_detector')

        return False

    # ------------------------------------------------------------------ #
    # Node restart helpers
    # ------------------------------------------------------------------ #
    def _restart_node(self, package, executable, node_name=None):
        """Kill a node if running, then start it fresh."""
        # Kill existing
        if node_name:
            self._kill_node(node_name)

        # Start new instance
        cmd = ['ros2', 'run', package, executable]
        try:
            subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,  # detach from our process group
            )
            self._log('Started: {}'.format(' '.join(cmd)))
            return True
        except OSError as e:
            self._log('Failed to start {}: {}'.format(executable, e))
            return False

    def _restart_node_with_params(self, package, executable, node_name, extra_note=''):
        """Restart node — for nodes that need params, just do basic restart.
        Config comes from the parameter server / launch defaults."""
        self._kill_node(node_name)
        if extra_note:
            self._log(extra_note)

        cmd = ['ros2', 'run', package, executable]
        try:
            subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
            self._log('Started: {}'.format(' '.join(cmd)))
            return True
        except OSError as e:
            self._log('Failed to start {}: {}'.format(executable, e))
            return False

    def _kill_node(self, node_name):
        """Best-effort kill of a ROS2 node by name."""
        try:
            # pkill by the node name in the process cmdline
            subprocess.run(
                ['pkill', '-f', node_name],
                timeout=3, capture_output=True
            )
        except (subprocess.TimeoutExpired, OSError):
            pass

    def _run_recovery_cmd(self, cmd):
        """Run a system command for recovery."""
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            if result.returncode != 0:
                self._log('Recovery cmd failed: {} — {}'.format(
                    ' '.join(cmd), result.stderr.strip()))
            return result.returncode == 0
        except (subprocess.TimeoutExpired, OSError) as e:
            self._log('Recovery cmd error: {}'.format(e))
            return False

    # ------------------------------------------------------------------ #
    # Escalation
    # ------------------------------------------------------------------ #
    def _escalate_estop(self, reason):
        """Last resort — trigger E-stop."""
        self._log('E-STOP ESCALATION: {}'.format(reason))
        msg = Bool()
        msg.data = True
        self.estop_pub.publish(msg)

    # ------------------------------------------------------------------ #
    # Publishing helpers
    # ------------------------------------------------------------------ #
    def _publish_health(self, status):
        msg = String()
        msg.data = status
        self.health_pub.publish(msg)

    def _log(self, message):
        """Log to both ROS logger and the watchdog_log topic."""
        self.get_logger().info('[WATCHDOG] {}'.format(message))
        msg = String()
        msg.data = '[{:.0f}s] {}'.format(
            time.time() - self.start_time, message)
        self.log_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SystemWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
