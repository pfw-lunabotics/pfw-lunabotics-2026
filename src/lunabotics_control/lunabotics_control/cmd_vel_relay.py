#!/usr/bin/env python3
"""
cmd_vel_relay.py — Safe cmd_vel bridge for Lunabotics 2026
===========================================================
Sits between Nav2 (or teleop) and the motor driver. Enforces:
  - Hard velocity/angular limits (tuned for the robot + lunar terrain)
  - Emergency-stop: zero output when /estop is True
  - Watchdog: stops the robot if no cmd_vel arrives within timeout_sec

Subscribers:
  /cmd_vel        (geometry_msgs/Twist) — raw command from Nav2 or teleop
  /estop          (std_msgs/Bool)       — True = motors stopped immediately

Publishers:
  /cmd_vel_safe   (geometry_msgs/Twist) — rate-limited, safety-checked output
  /control/status (std_msgs/String)     — relay state: RUNNING / ESTOP / WATCHDOG

Parameters:
  max_linear_vel   (float, 0.4)   m/s   — competition-safe forward speed
  max_angular_vel  (float, 0.8)   rad/s — safe rotation speed
  max_linear_accel (float, 0.5)   m/s²  — max speed-up per second
  max_angular_accel(float, 1.0)   rad/s² — max angular ramp per second
  publish_rate     (float, 20.0)  Hz    — output publish frequency
  watchdog_timeout (float, 0.5)   s     — silence before zeroing output
  output_topic     (str)          — topic to write safe commands to
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
import time

try:
    from luna_msgs.msg import ExcavationStatus
except ImportError:  # luna_msgs not built yet — relay still works without turn-block
    ExcavationStatus = None


class CmdVelRelay(Node):

    def __init__(self):
        super().__init__('cmd_vel_relay')

        # Parameters
        self.declare_parameter('max_linear_vel',    0.4)
        self.declare_parameter('max_angular_vel',   0.8)
        self.declare_parameter('max_linear_accel',  0.5)
        self.declare_parameter('max_angular_accel', 1.0)
        self.declare_parameter('publish_rate',      20.0)
        self.declare_parameter('watchdog_timeout',  0.5)
        self.declare_parameter('output_topic',      '/cmd_vel_safe')
        self.declare_parameter('software_estop_enabled', True)

        self._max_lv   = self.get_parameter('max_linear_vel').value
        self._max_av   = self.get_parameter('max_angular_vel').value
        self._max_la   = self.get_parameter('max_linear_accel').value
        self._max_aa   = self.get_parameter('max_angular_accel').value
        self._timeout  = self.get_parameter('watchdog_timeout').value
        out_topic      = self.get_parameter('output_topic').value

        # State
        self._target_lv   = 0.0
        self._target_av   = 0.0
        self._current_lv  = 0.0
        self._current_av  = 0.0
        self._estop       = False
        self._sw_estop_enabled = self.get_parameter('software_estop_enabled').value
        self._last_cmd_t  = time.monotonic()
        # Excavation interlock — when belt is running we MUST NOT turn the
        # drivetrain. Power budget can't sustain both; spinning wheels while
        # the belt is loaded causes overcurrent on the BLD-510B drivers.
        # The bridge already pauses the belt on detected turns; this relay-
        # side guard adds defense-in-depth: any angular command from any
        # source (nav2, teleop) is zeroed while the excavation is digging.
        self._belt_active = False

        # I/O
        self._pub_safe   = self.create_publisher(Twist,  out_topic,        10)
        self._pub_status = self.create_publisher(String, '/control/status', 10)

        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb,   10)
        self.create_subscription(Bool,  '/safety/estop', self._estop_cb, 10)
        self.create_subscription(Bool,  '/estop', self._estop_cb, 10)
        if ExcavationStatus is not None:
            self.create_subscription(
                ExcavationStatus, '/excavation/status',
                self._exc_status_cb, 10
            )
        else:
            self.get_logger().warn(
                'luna_msgs not available — excavation turn-block disabled')

        rate = self.get_parameter('publish_rate').value
        self._dt = 1.0 / rate
        self.create_timer(self._dt, self._publish_cb)

        self.get_logger().info(
            f'cmd_vel_relay ready — max_lv={self._max_lv} m/s, '
            f'max_av={self._max_av} rad/s, watchdog={self._timeout} s'
        )

    # ------------------------------------------------------------------

    def _cmd_cb(self, msg: Twist):
        self._last_cmd_t = time.monotonic()
        # Clamp incoming targets to limits
        self._target_lv = max(-self._max_lv,
                              min(self._max_lv, msg.linear.x))
        self._target_av = max(-self._max_av,
                              min(self._max_av, msg.angular.z))

    def _estop_cb(self, msg: Bool):
        if not self._sw_estop_enabled:
            if msg.data:
                self.get_logger().warn(
                    'Software E-STOP received but IGNORED (disabled for competition)')
            return
        self._estop = msg.data
        if self._estop:
            self._target_lv = self._target_av = 0.0
            self.get_logger().warn('E-STOP active — motors zeroed')

    def _exc_status_cb(self, msg):
        # DIGGING = belt running. We forbid simultaneous drivetrain rotation
        # (power-budget constraint + BLD-510B overcurrent risk).
        was_active = self._belt_active
        self._belt_active = (msg.state == 'DIGGING')
        if self._belt_active and not was_active:
            self.get_logger().info(
                'Excavation DIGGING — turn commands now blocked')
        elif not self._belt_active and was_active:
            self.get_logger().info(
                'Excavation idle — turn commands re-enabled')

    def _publish_cb(self):
        now = time.monotonic()
        watchdog_tripped = (now - self._last_cmd_t) > self._timeout

        if self._estop or watchdog_tripped:
            # Hard stop — ramp to zero quickly
            self._target_lv = self._target_av = 0.0

        # Excavation interlock — zero angular target while belt is running.
        # This is a HARD block: any turn command is dropped at the relay.
        effective_target_av = 0.0 if self._belt_active else self._target_av

        # Ramp current velocity toward target
        self._current_lv = self._ramp(
            self._current_lv, self._target_lv, self._max_la * self._dt)
        self._current_av = self._ramp(
            self._current_av, effective_target_av, self._max_aa * self._dt)

        out = Twist()
        out.linear.x  = self._current_lv
        out.angular.z = self._current_av
        self._pub_safe.publish(out)

        status = String()
        if self._estop:
            status.data = 'ESTOP'
        elif watchdog_tripped:
            status.data = 'WATCHDOG'
        elif self._belt_active:
            status.data = (f'RUNNING+BELT_LOCK lv={self._current_lv:.2f} '
                           f'av={self._current_av:.2f} (turn blocked)')
        else:
            status.data = f'RUNNING lv={self._current_lv:.2f} av={self._current_av:.2f}'
        self._pub_status.publish(status)

    @staticmethod
    def _ramp(current: float, target: float, max_step: float) -> float:
        diff = target - current
        if abs(diff) <= max_step:
            return target
        return current + max_step * (1.0 if diff > 0 else -1.0)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
