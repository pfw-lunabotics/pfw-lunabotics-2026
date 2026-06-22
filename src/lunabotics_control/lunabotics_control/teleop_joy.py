#!/usr/bin/env python3
"""
teleop_joy.py — Gamepad manual control for Lunabotics 2026
==========================================================
Full manual control via gamepad (Switch Pro / Xbox controller).
Replaces teleop_twist_joy with support for ALL robot subsystems.

Requires: ros-humble-joy (sudo apt install ros-humble-joy)

Connection:
  USB:       Plug in, appears as /dev/input/js0 automatically
  Bluetooth: bluetoothctl → scan on → pair <MAC> → trust → connect

Default button mapping (Switch Pro Controller via hid-nintendo):

  Left Stick              — drive forward/back + turn (proportional)
  LB / L (btn 4)          — deadman switch (HOLD to drive)

  A (btn 0)               — toggle MANUAL / OBSERVER mode
  B (btn 1)               — toggle E-STOP
  X (btn 2)               — toggle excavation ON/OFF
  Y (btn 3)               — FULL STOP (all motors)

  D-pad Up / Down         — actuator UP / DOWN (hold)
  D-pad Left / Right      — excavation speed down / up

  ZL / LT (btn 6)         — dump servo (deposition)
  ZR / RT (btn 7)         — stow servo (retract)

  - / Back (btn 8)        — print diagnostics
  L-stick press (btn 10)  — stop actuator only

All button/axis indices configurable via ROS2 parameters.
To find your controller's mapping: ros2 topic echo /joy

Usage:
  # Launch everything (joy_node + teleop_joy):
  ros2 launch lunabotics_control control.launch.py use_joystick:=true

  # Or run standalone:
  ros2 run joy joy_node --ros-args -p dev:=/dev/input/js0 -p deadzone:=0.1 -p autorepeat_rate:=20.0
  ros2 run lunabotics_control teleop_joy
"""

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import Bool, String, Int32

PWM_MAX = 65535

BANNER = r"""
=============================================
  Lunabotics 2026 -- Gamepad Teleop
=============================================
  Mode: OBSERVER (autonomous runs freely)

  Left Stick        : drive (hold deadman)
  LB / L            : deadman switch (hold)

  A                 : toggle MANUAL / OBSERVER
  B                 : toggle E-STOP
  X                 : toggle excavation ON/OFF
  Y                 : FULL STOP (all motors)

  D-pad Up/Down     : actuator UP / DOWN
  D-pad Left/Right  : excavation speed -/+

  ZL / LT           : dump servo
  ZR / RT           : stow servo

  Back / -          : diagnostics
  L-stick press     : stop actuator
=============================================
"""


class TeleopJoy(Node):

    def __init__(self):
        super().__init__('teleop_joy')

        # --- Axis indices ---
        self.declare_parameter('axis_linear', 1)
        self.declare_parameter('axis_angular', 0)
        self.declare_parameter('axis_dpad_x', 6)
        self.declare_parameter('axis_dpad_y', 7)

        # --- Velocity scaling ---
        # Sign convention: stick-up (axis 1 = -1) * negative scale = positive linear.x = forward
        # pico_bridge handles motor inversion at the hardware boundary
        self.declare_parameter('linear_scale', -0.3)
        self.declare_parameter('angular_scale', -0.8)

        # --- Button indices (Switch Pro / generic defaults) ---
        self.declare_parameter('btn_deadman', 4)         # LB / L
        self.declare_parameter('btn_manual_toggle', 0)   # A
        self.declare_parameter('btn_estop', 1)           # B
        self.declare_parameter('btn_excavation', 2)      # X
        self.declare_parameter('btn_full_stop', 3)       # Y
        self.declare_parameter('btn_dump', 6)            # ZL / LT
        self.declare_parameter('btn_stow', 7)            # ZR / RT
        self.declare_parameter('btn_diagnostics', 8)     # - / Back
        self.declare_parameter('btn_actuator_stop', 10)  # L-stick press

        # --- Excavation parameters ---
        self.declare_parameter('dig_pwm_default', 6000)
        self.declare_parameter('dig_pwm_step', 1500)
        self.declare_parameter('dig_pwm_min', 5000)
        self.declare_parameter('dig_pwm_max', PWM_MAX)

        # --- Actuator parameters ---
        self.declare_parameter('actuator_up_val', 100)
        self.declare_parameter('actuator_down_val', -100)

        # --- Deposition servo parameters ---
        self.declare_parameter('dump_angle', 52.0)   # open: door opens at ~+52 deg from closed
        self.declare_parameter('stow_angle', -10.0)  # close + preload against hard stop (firm lock)
        self.declare_parameter('servo_move_time_ms', 4500)

        # --- Health monitoring ---
        self.declare_parameter('pico_timeout', 3.0)
        self.declare_parameter('imu_timeout', 3.0)
        self.declare_parameter('joy_timeout', 5.0)

        # ---- Read all params ----
        self._axis_lin = self.get_parameter('axis_linear').value
        self._axis_ang = self.get_parameter('axis_angular').value
        self._axis_dpad_x = self.get_parameter('axis_dpad_x').value
        self._axis_dpad_y = self.get_parameter('axis_dpad_y').value

        self._lin_scale = self.get_parameter('linear_scale').value
        self._ang_scale = self.get_parameter('angular_scale').value

        self._btn_deadman = self.get_parameter('btn_deadman').value
        self._btn_manual = self.get_parameter('btn_manual_toggle').value
        self._btn_estop = self.get_parameter('btn_estop').value
        self._btn_ex = self.get_parameter('btn_excavation').value
        self._btn_stop = self.get_parameter('btn_full_stop').value
        self._btn_dump = self.get_parameter('btn_dump').value
        self._btn_stow = self.get_parameter('btn_stow').value
        self._btn_diag = self.get_parameter('btn_diagnostics').value
        self._btn_ac_stop = self.get_parameter('btn_actuator_stop').value

        self._dig_pwm_default = self.get_parameter('dig_pwm_default').value
        self._dig_pwm_step = self.get_parameter('dig_pwm_step').value
        self._dig_pwm_min = self.get_parameter('dig_pwm_min').value
        self._dig_pwm_max = self.get_parameter('dig_pwm_max').value

        self._ac_up = self.get_parameter('actuator_up_val').value
        self._ac_down = self.get_parameter('actuator_down_val').value

        self._dump_angle = self.get_parameter('dump_angle').value
        self._stow_angle = self.get_parameter('stow_angle').value
        self._servo_time = self.get_parameter('servo_move_time_ms').value

        self._pico_timeout = self.get_parameter('pico_timeout').value
        self._imu_timeout = self.get_parameter('imu_timeout').value
        self._joy_timeout = self.get_parameter('joy_timeout').value

        # ---- Operator state ----
        self._lv = 0.0
        self._av = 0.0
        self._estop = False
        self._manual = False
        self._deadman_held = False

        self._ex_speed = self._dig_pwm_default
        self._ex_on = False
        self._ex_pwm = 0

        self._ac_val = 0

        # Edge detection: previous button/axis state
        self._prev_buttons = []
        self._prev_dpad_x = 0.0
        self._prev_dpad_y = 0.0

        # ---- Health monitoring state ----
        self._pico_status_str = 'UNKNOWN'
        self._pico_last_t = 0.0
        self._imu_last_t = 0.0
        self._joy_last_t = 0.0
        self._joy_warned = False

        # ---- Publishers ----
        self._pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self._pub_cmd_safe = self.create_publisher(Twist, '/cmd_vel_safe', 10)
        self._pub_estop = self.create_publisher(Bool, '/estop', 10)
        self._pub_info = self.create_publisher(String, '/teleop/status', 10)
        self._pub_ex = self.create_publisher(Int32, '/excavation/motor', 10)
        self._pub_ac = self.create_publisher(Int32, '/actuator/command', 10)
        self._pub_servo = self.create_publisher(String, '/deposition/tilt', 10)

        # ---- Subscribers ----
        self.create_subscription(Joy, '/joy', self._joy_cb, 10)
        self.create_subscription(String, '/pico/status', self._pico_cb, 10)
        self.create_subscription(Imu, '/unilidar/imu', self._imu_cb, 5)

        # ---- Timer: continuous publish at 20 Hz ----
        self.create_timer(0.05, self._timer_cb)

        print(BANNER)
        self.get_logger().info(
            'Gamepad teleop started. Waiting for /joy messages...')
        self.get_logger().info(
            'If no controller detected, check: ls /dev/input/js*')

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------

    def _pico_cb(self, msg):
        self._pico_status_str = msg.data
        self._pico_last_t = time.monotonic()

    def _imu_cb(self, _msg):
        self._imu_last_t = time.monotonic()

    # ------------------------------------------------------------------
    # Joy callback — process gamepad input
    # ------------------------------------------------------------------

    def _joy_cb(self, msg):
        self._joy_last_t = time.monotonic()
        if self._joy_warned:
            self._joy_warned = False
            print('\n  Controller connected!')
            self._print_status()

        buttons = list(msg.buttons)
        axes = list(msg.axes)

        # -- helpers --
        def pressed(idx):
            """Rising edge: button was 0, now 1."""
            if idx >= len(buttons):
                return False
            if idx >= len(self._prev_buttons):
                return buttons[idx] == 1
            return buttons[idx] == 1 and self._prev_buttons[idx] == 0

        def held(idx):
            if idx >= len(buttons):
                return False
            return buttons[idx] == 1

        def axis(idx):
            if idx >= len(axes):
                return 0.0
            return axes[idx]

        # ---- Global controls (any mode) ----

        # Toggle manual / observer
        if pressed(self._btn_manual):
            self._manual = not self._manual
            self._stop_all()
            if self._manual:
                print('\n  >>> MANUAL MODE — gamepad active')
            else:
                print('\n  >>> OBSERVER MODE — autonomous resumed')
            self._print_status()

        # Toggle E-stop
        if pressed(self._btn_estop):
            self._estop = not self._estop
            estop_msg = Bool()
            estop_msg.data = self._estop
            self._pub_estop.publish(estop_msg)
            if self._estop:
                self._stop_all()
                print('\n  *** E-STOP ACTIVATED — all motors zeroed ***')
            else:
                print('\n  E-stop released')
            self._print_status()

        # Diagnostics
        if pressed(self._btn_diag):
            self._print_diagnostics()

        # ---- Manual-mode controls (only when manual and not e-stopped) ----

        if self._manual and not self._estop:

            # Full stop
            if pressed(self._btn_stop):
                self._stop_all()
                print('\n  ** FULL STOP **')
                self._print_status()

            # -- Driving (proportional, requires deadman) --
            self._deadman_held = held(self._btn_deadman)
            if self._deadman_held:
                self._lv = axis(self._axis_lin) * self._lin_scale
                self._av = axis(self._axis_ang) * self._ang_scale
            else:
                if abs(self._lv) > 0.001 or abs(self._av) > 0.001:
                    self._lv = 0.0
                    self._av = 0.0

            # -- Excavation toggle --
            if pressed(self._btn_ex):
                self._ex_on = not self._ex_on
                if self._ex_on:
                    self._ex_pwm = self._ex_speed
                    pct = self._ex_pwm * 100 // PWM_MAX
                    print(f'\n  EXCAVATION ON — PWM {self._ex_pwm} ({pct}%)'
                          f'{self._pico_warn()}')
                else:
                    self._ex_pwm = 0
                    self._publish_int(self._pub_ex, 0)
                    print('\n  EXCAVATION OFF')
                self._print_status()

            # -- D-pad Y: actuator up/down (hold to move, release to stop) --
            dpad_y = axis(self._axis_dpad_y)
            if dpad_y > 0.5:
                self._ac_val = self._ac_up
            elif dpad_y < -0.5:
                self._ac_val = self._ac_down
            elif abs(self._prev_dpad_y) > 0.5:
                # D-pad released — stop actuator
                self._ac_val = 0
                self._publish_int(self._pub_ac, 0)

            # -- D-pad X: excavation speed (edge-triggered) --
            dpad_x = axis(self._axis_dpad_x)
            if dpad_x > 0.5 and self._prev_dpad_x <= 0.5:
                self._ex_speed = min(self._dig_pwm_max,
                                     self._ex_speed + self._dig_pwm_step)
                if self._ex_on:
                    self._ex_pwm = self._ex_speed
                pct = self._ex_speed * 100 // PWM_MAX
                print(f'\n  Excavation speed: {self._ex_speed} ({pct}%)')
                self._print_status()
            elif dpad_x < -0.5 and self._prev_dpad_x >= -0.5:
                self._ex_speed = max(self._dig_pwm_min,
                                     self._ex_speed - self._dig_pwm_step)
                if self._ex_on:
                    self._ex_pwm = self._ex_speed
                pct = self._ex_speed * 100 // PWM_MAX
                print(f'\n  Excavation speed: {self._ex_speed} ({pct}%)')
                self._print_status()

            # -- Actuator stop button --
            if pressed(self._btn_ac_stop):
                self._ac_val = 0
                self._publish_int(self._pub_ac, 0)
                print('\n  ACTUATOR STOPPED')
                self._print_status()

            # -- Dump servo --
            if pressed(self._btn_dump):
                servo_msg = String()
                servo_msg.data = f'{self._dump_angle},{self._servo_time}'
                self._pub_servo.publish(servo_msg)
                print(f'\n  DUMP — servo to {self._dump_angle:.0f} deg'
                      f' over {self._servo_time}ms')
                self._print_status()

            # -- Stow servo --
            if pressed(self._btn_stow):
                servo_msg = String()
                servo_msg.data = f'{self._stow_angle},{self._servo_time}'
                self._pub_servo.publish(servo_msg)
                print(f'\n  STOW — servo to {self._stow_angle:.0f} deg')
                self._print_status()

        # ---- Save previous state for edge detection ----
        self._prev_buttons = buttons
        self._prev_dpad_x = axis(self._axis_dpad_x)
        self._prev_dpad_y = axis(self._axis_dpad_y)

    # ------------------------------------------------------------------
    # Timer callback — continuous publishing + health checks
    # ------------------------------------------------------------------

    def _timer_cb(self):
        now = time.monotonic()

        # Warn if no joy messages
        if self._joy_last_t == 0.0:
            if not self._joy_warned and now > 5.0:
                self._joy_warned = True
                print('\n  !!! No /joy messages — is joy_node running?')
                print('      ros2 run joy joy_node --ros-args '
                      '-p dev:=/dev/input/js0')
                self._print_status()
        elif (now - self._joy_last_t) > self._joy_timeout:
            if not self._joy_warned:
                self._joy_warned = True
                print('\n  !!! Controller disconnected — no /joy for '
                      f'{now - self._joy_last_t:.0f}s')
                self._stop_all()
                self._print_status()

        if not self._manual:
            return

        if self._estop:
            zero = Twist()
            self._pub_cmd.publish(zero)
            self._pub_cmd_safe.publish(zero)
            self._publish_int(self._pub_ex, 0)
            self._publish_int(self._pub_ac, 0)
            return

        # Driving
        twist = Twist()
        twist.linear.x = self._lv
        twist.angular.z = self._av
        self._pub_cmd.publish(twist)
        self._pub_cmd_safe.publish(twist)

        # Excavation (continuous for Pico watchdog)
        if self._ex_pwm != 0:
            self._publish_int(self._pub_ex, -self._ex_pwm)

        # Actuator (continuous for Sabertooth)
        if self._ac_val != 0:
            self._publish_int(self._pub_ac, self._ac_val)

        # Status topic
        self._publish_status_topic()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _stop_all(self):
        self._lv = 0.0
        self._av = 0.0
        self._ex_pwm = 0
        self._ex_on = False
        self._ac_val = 0
        zero = Twist()
        self._pub_cmd.publish(zero)
        self._pub_cmd_safe.publish(zero)
        self._publish_int(self._pub_ex, 0)
        self._publish_int(self._pub_ac, 0)

    @staticmethod
    def _publish_int(pub, val):
        msg = Int32()
        msg.data = val
        pub.publish(msg)

    def _pico_alive(self):
        if self._pico_last_t == 0.0:
            return False
        return (time.monotonic() - self._pico_last_t) < self._pico_timeout

    def _imu_alive(self):
        if self._imu_last_t == 0.0:
            return False
        return (time.monotonic() - self._imu_last_t) < self._imu_timeout

    def _pico_warn(self):
        if not self._pico_alive():
            return ' [!PICO OFFLINE!]'
        return ''

    # ------------------------------------------------------------------
    # Status display
    # ------------------------------------------------------------------

    def _print_status(self):
        mode = 'MANUAL ' if self._manual else 'OBSERVE'
        parts = [f'[{mode}]']
        parts.append(f'lv={self._lv:+.2f} av={self._av:+.2f}')

        if self._deadman_held:
            parts.append('DM:ON')

        if self._ex_on:
            pct = self._ex_pwm * 100 // PWM_MAX
            parts.append(f'EX:{self._ex_pwm}({pct}%)')

        if self._ac_val > 0:
            parts.append('AC:UP')
        elif self._ac_val < 0:
            parts.append('AC:DN')

        if self._estop:
            parts.append('E-STOP')

        warnings = []
        if not self._pico_alive() and self._pico_last_t > 0:
            warnings.append('PICO:OFFLINE')
        if not self._imu_alive() and self._imu_last_t > 0:
            warnings.append('IMU:OFFLINE')
        if warnings:
            parts.append('|')
            parts.extend(warnings)

        line = '  ' + '  '.join(parts)
        print(f'\r{line:<110}', end='', flush=True)

    def _publish_status_topic(self):
        msg = String()
        msg.data = (
            f'{"MANUAL" if self._manual else "OBSERVER"} '
            f'lv={self._lv:.2f} av={self._av:.2f} '
            f'ex={self._ex_pwm} ac={self._ac_val} '
            f'estop={self._estop} deadman={self._deadman_held}'
        )
        self._pub_info.publish(msg)

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------

    def _print_diagnostics(self):
        now = time.monotonic()
        print('\n  ====== DIAGNOSTIC SNAPSHOT ======')
        print(f'  Mode:       {"MANUAL" if self._manual else "OBSERVER"}')
        print(f'  E-Stop:     {"ON" if self._estop else "OFF"}')
        print(f'  Deadman:    {"HELD" if self._deadman_held else "released"}')
        print(f'  Driving:    lv={self._lv:+.2f} m/s  av={self._av:+.2f} rad/s')

        if self._ex_on:
            pct = self._ex_pwm * 100 // PWM_MAX
            print(f'  Excavation: ON, PWM {self._ex_pwm} ({pct}%)')
        else:
            print(f'  Excavation: OFF (stored speed: {self._ex_speed})')

        if self._ac_val > 0:
            print(f'  Actuator:   UP (+{self._ac_val})')
        elif self._ac_val < 0:
            print(f'  Actuator:   DOWN ({self._ac_val})')
        else:
            print('  Actuator:   STOPPED')

        # Controller
        if self._joy_last_t > 0:
            age = now - self._joy_last_t
            print(f'  Controller: CONNECTED (last: {age:.1f}s ago)')
        else:
            print('  Controller: NO DATA')
            print('    Action:   ls /dev/input/js*')
            print('              ros2 run joy joy_node')

        # Pico
        if self._pico_alive():
            age = now - self._pico_last_t
            print(f'  Pico:       CONNECTED ({age:.1f}s ago)')
            print(f'    Raw:      {self._pico_status_str}')
        elif self._pico_last_t > 0:
            print(f'  Pico:       OFFLINE ({now - self._pico_last_t:.0f}s)')
        else:
            print('  Pico:       NO DATA (pico_bridge not running?)')

        # IMU
        if self._imu_alive():
            print(f'  LiDAR/IMU:  ALIVE ({now - self._imu_last_t:.1f}s ago)')
        else:
            print('  LiDAR/IMU:  OFFLINE')

        print('  ================================')
        self._print_status()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopJoy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_all()
        node.destroy_node()
        rclpy.shutdown()
        print('\nGamepad teleop stopped — all motors zeroed.')


if __name__ == '__main__':
    main()
