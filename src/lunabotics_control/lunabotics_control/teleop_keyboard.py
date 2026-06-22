#!/usr/bin/env python3
"""
teleop_keyboard.py — Full manual control for Lunabotics 2026
=============================================================
Two operating modes:

  OBSERVER mode (default):
    - E-stop works (press 'e')
    - NO commands published -- autonomous system runs uninterrupted
    - Press 'm' to take manual control

  MANUAL mode (press 'm' to enter):
    - Full driving, excavation, actuator, and deposition control
    - All commands published at ~20Hz, override autonomous system
    - Press 'm' to release back to autonomous
    - Press SPACE for FULL STOP (all motors)

Self-diagnosis (runs automatically in MANUAL mode):
  - Drive stall: detects commanding velocity but no IMU motion,
    shows escalating guidance (wait -> check cable -> replug -> power cycle)
  - Excavation verify: cross-checks pico_bridge status to confirm
    motor commands are actually being forwarded
  - Pico offline: time-based escalating action items
  - Press '1' for full diagnostic snapshot at any time

Controls:
  m            -- toggle MANUAL/OBSERVER mode
  e            -- toggle emergency stop (works in BOTH modes)
  q            -- quit (stops everything, safe shutdown)
  1            -- cycle self-test / diagnostic snapshot
                   (OFF → PICO_OFFLINE → DRIVE_STALL → EX_MISMATCH → IMU_OFFLINE → OFF)

  --- Driving (MANUAL only) ---
  w / s        -- increase / decrease linear speed
  a / d        -- turn left / right
  SPACE        -- FULL STOP (all: driving + excavation + actuator)

  --- Excavation Belt (MANUAL only) ---
  r            -- toggle excavation motor ON/OFF
  f / v        -- increase / decrease belt speed (+/- 1500 PWM)

  --- Actuator (MANUAL only) ---
  u            -- actuator UP (raise, +100 continuous)
  j            -- actuator DOWN (lower, -100 continuous)
  h            -- stop actuator only

  --- Deposition Servo (MANUAL only) ---
  t            -- dump (tilt servo to dump angle)
  y            -- stow (retract servo to 0 deg)

Usage:
  ros2 run lunabotics_control teleop_keyboard
"""

import sys
import tty
import termios
import select
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String, Int32

PWM_MAX = 65535

BANNER = r"""
=============================================
  Lunabotics 2026 -- Keyboard Teleop
=============================================
  Mode: OBSERVER (autonomous runs freely)

  m       : toggle MANUAL / OBSERVER
  e       : toggle E-STOP (all modes)
  1       : cycle self-test / diagnostic snapshot
  q       : quit

  --- Driving (MANUAL) ---
  w / s   : forward / backward
  a / d   : turn left / right

  --- Excavation Belt ---
  r       : toggle belt ON/OFF
  f / v   : speed up / down

  --- Actuator ---
  u       : UP (raise)
  j       : DOWN (lower)
  h       : stop actuator

  --- Deposition Servo ---
  t       : dump (tilt)
  y       : stow (retract)

  SPACE   : FULL STOP (all motors)
=============================================
"""

SELFTEST_MODES = ['OFF', 'PICO_OFFLINE', 'DRIVE_STALL', 'EX_MISMATCH', 'IMU_OFFLINE']

DRIVE_KEYS = {
    'w': ( 1,  0),
    's': (-1,  0),
    'a': ( 0,  1),
    'd': ( 0, -1),
}


def _get_key_nonblocking(settings):
    """Return a key if one is available, else None."""
    tty.setraw(sys.stdin.fileno())
    ready, _, _ = select.select([sys.stdin], [], [], 0.05)  # 50ms timeout
    key = None
    if ready:
        key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class TeleopKeyboard(Node):

    def __init__(self):
        super().__init__('teleop_keyboard')

        # --- Driving parameters ---
        self.declare_parameter('linear_step',  0.03)
        self.declare_parameter('angular_step', 0.08)
        self.declare_parameter('max_linear',   0.3)
        self.declare_parameter('max_angular',  0.8)

        # --- Excavation parameters ---
        self.declare_parameter('dig_pwm_default', 6000)
        self.declare_parameter('dig_pwm_step',    1500)
        self.declare_parameter('dig_pwm_min',     5000)
        self.declare_parameter('dig_pwm_max',     PWM_MAX)

        # --- Actuator parameters ---
        self.declare_parameter('actuator_up_val',   100)
        self.declare_parameter('actuator_down_val', -100)

        # --- Deposition parameters ---
        self.declare_parameter('dump_angle',         52.0)   # open: door opens at ~+52 deg from closed
        self.declare_parameter('stow_angle',         -10.0)  # close + preload against hard stop (firm lock)
        self.declare_parameter('servo_move_time_ms', 4500)

        # --- Health monitoring thresholds ---
        self.declare_parameter('pico_timeout', 3.0)
        self.declare_parameter('imu_timeout',  3.0)
        self.declare_parameter('stall_timeout', 3.0)

        # ---- Read all params ----
        self._lin_step = self.get_parameter('linear_step').value
        self._ang_step = self.get_parameter('angular_step').value
        self._max_lin  = self.get_parameter('max_linear').value
        self._max_ang  = self.get_parameter('max_angular').value

        self._dig_pwm_default = self.get_parameter('dig_pwm_default').value
        self._dig_pwm_step    = self.get_parameter('dig_pwm_step').value
        self._dig_pwm_min     = self.get_parameter('dig_pwm_min').value
        self._dig_pwm_max     = self.get_parameter('dig_pwm_max').value

        self._ac_up   = self.get_parameter('actuator_up_val').value
        self._ac_down = self.get_parameter('actuator_down_val').value

        self._dump_angle   = self.get_parameter('dump_angle').value
        self._stow_angle   = self.get_parameter('stow_angle').value
        self._servo_time   = self.get_parameter('servo_move_time_ms').value

        self._pico_timeout = self.get_parameter('pico_timeout').value
        self._imu_timeout  = self.get_parameter('imu_timeout').value
        self._stall_timeout = self.get_parameter('stall_timeout').value

        # ---- Operator state ----
        self._lv       = 0.0     # linear velocity target
        self._av       = 0.0     # angular velocity target
        self._estop    = False
        self._manual   = False   # OBSERVER mode by default

        self._ex_speed = self._dig_pwm_default   # excavation speed setting
        self._ex_on    = False                    # excavation belt toggle
        self._ex_pwm   = 0                        # actual PWM being sent

        self._ac_val   = 0       # actuator command: +100 / 0 / -100

        # ---- Health monitoring state ----
        self._pico_status_str = 'UNKNOWN'
        self._pico_last_t     = 0.0
        self._imu_last_t      = 0.0
        self._sys_health      = 'UNKNOWN'

        # ---- Self-diagnosis state ----
        # IMU motion detection
        self._imu_has_motion = False
        self._imu_gyro_z     = 0.0
        self._imu_vibration  = 0.0
        self._last_motion_t  = time.monotonic()

        # Stall tracking
        self._drive_stall_since = 0.0   # 0 = no stall
        self._pico_offline_since = 0.0  # 0 = not offline
        self._ex_mismatch_since = 0.0   # 0 = no mismatch

        # Escalation dedup: category -> (last_msg, last_time)
        self._last_escalation = {}
        self._last_diag_t = 0.0

        # Self-test: cycles through simulated failure modes (0 = OFF)
        self._selftest_mode = 0

        # ---- Publishers ----
        # Publish to BOTH /cmd_vel (for cmd_vel_relay when full stack is running)
        # and /cmd_vel_safe (direct to pico_bridge for standalone manual teleop)
        self._pub_cmd   = self.create_publisher(Twist,  '/cmd_vel',          10)
        self._pub_cmd_safe = self.create_publisher(Twist, '/cmd_vel_safe',   10)
        self._pub_estop = self.create_publisher(Bool,   '/estop',            10)
        self._pub_info  = self.create_publisher(String, '/teleop/status',    10)
        self._pub_ex    = self.create_publisher(Int32,  '/excavation/motor', 10)
        self._pub_ac    = self.create_publisher(Int32,  '/actuator/command', 10)
        self._pub_servo = self.create_publisher(String, '/deposition/tilt',  10)

        # ---- Subscribers (health monitoring -- callbacks processed via spin_once) ----
        self.create_subscription(String, '/pico/status',    self._pico_cb,    10)
        self.create_subscription(Imu,    '/unilidar/imu',   self._imu_cb,      5)
        self.create_subscription(String, '/system/health',  self._health_cb,   5)

        print(BANNER)

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------

    def _pico_cb(self, msg):
        self._pico_status_str = msg.data
        self._pico_last_t = time.monotonic()

    def _imu_cb(self, msg):
        self._imu_last_t = time.monotonic()
        # Track actual motion for stall detection
        self._imu_gyro_z = abs(msg.angular_velocity.z)
        gyro_xy = (msg.angular_velocity.x ** 2
                   + msg.angular_velocity.y ** 2) ** 0.5
        accel_mag = (msg.linear_acceleration.x ** 2
                     + msg.linear_acceleration.y ** 2
                     + msg.linear_acceleration.z ** 2) ** 0.5
        self._imu_vibration = abs(accel_mag - 9.81)
        self._imu_has_motion = (
            self._imu_gyro_z > 0.08
            or gyro_xy > 0.15
            or self._imu_vibration > 0.5
        )
        if self._imu_has_motion:
            self._last_motion_t = time.monotonic()

    def _health_cb(self, msg):
        self._sys_health = msg.data

    # ------------------------------------------------------------------
    # Pico status parsing
    # ------------------------------------------------------------------

    def _parse_pico_vals(self):
        """Parse pico status like 'OK L:123 R:-456 EX:-6000 AC:100 KICK'.

        Returns dict with integer values for keys found (L, R, EX, AC, etc.)
        and boolean flags for special states.
        """
        s = self._pico_status_str
        vals = {}
        for token in s.split():
            if ':' in token:
                parts = token.split(':', 1)
                try:
                    vals[parts[0]] = int(parts[1])
                except ValueError:
                    pass
        vals['_stall_recovery'] = 'STALL_RECOVERY' in s
        vals['_kick'] = 'KICK' in s
        return vals

    # ------------------------------------------------------------------
    # Health checks (passive)
    # ------------------------------------------------------------------

    def _pico_alive(self):
        if self._selftest_mode == 1:  # PICO_OFFLINE
            return False
        if self._selftest_mode == 3:  # EX_MISMATCH needs Pico "alive" to trigger
            return True
        if self._pico_last_t == 0.0:
            return False
        return (time.monotonic() - self._pico_last_t) < self._pico_timeout

    def _pico_connected(self):
        return self._pico_alive() and 'DISCONNECT' not in self._pico_status_str.upper()

    def _imu_alive(self):
        if self._selftest_mode == 4:  # IMU_OFFLINE
            return False
        if self._selftest_mode == 2:  # DRIVE_STALL needs IMU "alive" to trigger
            return True
        if self._imu_last_t == 0.0:
            return False
        return (time.monotonic() - self._imu_last_t) < self._imu_timeout

    def _health_warnings(self):
        warnings = []
        if not self._pico_alive():
            warnings.append('PICO:NO_DATA')
        elif not self._pico_connected():
            warnings.append('PICO:DISCONN')
        if not self._imu_alive():
            warnings.append('LIDAR:NO_IMU')
        if self._sys_health not in ('OK', 'STARTING', 'UNKNOWN'):
            warnings.append('SYS:' + self._sys_health)
        if self._drive_stall_since > 0:
            warnings.append('DRIVE:STALL')
        return warnings

    def _pico_warn_str(self):
        if not self._pico_connected():
            return ' [!PICO OFFLINE!]'
        return ''

    # ------------------------------------------------------------------
    # Self-diagnosis — runs every ~1 second in manual mode
    # ------------------------------------------------------------------

    def _run_diagnostics(self):
        """Periodic self-diagnosis. Detects failure patterns and prints
        escalating action items so the operator knows what to do."""
        now = time.monotonic()
        if (now - self._last_diag_t) < 1.0:
            return
        self._last_diag_t = now

        if not self._manual:
            return

        self._diag_pico_offline(now)
        self._diag_drive_stall(now)
        self._diag_excavation_verify(now)

    def _diag_pico_offline(self, now):
        """Escalating guidance when Pico bridge stops responding."""
        if not self._pico_alive():
            if self._pico_offline_since == 0.0:
                self._pico_offline_since = now
            dur = now - self._pico_offline_since

            if dur > 30:
                self._escalate('pico',
                    '!!! PICO OFFLINE {:.0f}s — power cycle Pico. '
                    'Check /dev/ttyACM*. Re-flash main.py if REPL (>>>) showing.'.format(dur))
            elif dur > 15:
                self._escalate('pico',
                    '!!! PICO OFFLINE {:.0f}s — unplug/replug Pico USB cable'.format(dur))
            elif dur > 5:
                self._escalate('pico',
                    '!!! PICO OFFLINE {:.0f}s — check USB cable to Pico'.format(dur))
            elif dur > 2:
                self._escalate('pico',
                    '!!! PICO OFFLINE — waiting for pico_bridge auto-reconnect...')
        else:
            if self._pico_offline_since > 0:
                dur = now - self._pico_offline_since
                self._escalate('pico',
                    'Pico RECONNECTED after {:.0f}s'.format(dur))
                self._pico_offline_since = 0.0

    def _diag_drive_stall(self, now):
        """Detect driving commands with no IMU motion."""
        # Self-test mode 2: pretend we're commanding but not moving
        if self._selftest_mode == 2:
            commanding = True
        else:
            commanding = abs(self._lv) > 0.01 or abs(self._av) > 0.01

        if not commanding or self._estop:
            self._drive_stall_since = 0.0
            return

        if not self._imu_alive():
            # Can't detect stall without IMU — separate warning already shown
            return

        stall_dur = now - self._last_motion_t
        if stall_dur < self._stall_timeout:
            if self._drive_stall_since > 0:
                self._escalate('drive',
                    'Drive stall cleared — motion detected')
            self._drive_stall_since = 0.0
            return

        # Stall confirmed
        if self._drive_stall_since == 0.0:
            self._drive_stall_since = now

        # Check if pico_bridge is already doing its own recovery
        pv = self._parse_pico_vals()
        if pv.get('_kick'):
            self._escalate('drive',
                '!!! DRIVE STALL — Pico bridge PWM kick in progress, wait...')
            return
        if pv.get('_stall_recovery'):
            self._escalate('drive',
                '!!! DRIVE STALL — Pico bridge reconnecting serial (attempt {})...'.format(
                    pv.get('STALL_RECOVERY', '?')))
            return

        dur = now - self._drive_stall_since
        if dur > 20:
            self._escalate('drive',
                '!!! DRIVE STALL {:.0f}s — unplug/replug Pico USB. '
                'Check motor EN wiring. Check BLD-510B power.'.format(dur))
        elif dur > 10:
            self._escalate('drive',
                '!!! DRIVE STALL {:.0f}s — try SPACE then re-command. '
                'Check Pico serial output.'.format(dur))
        else:
            disp_lv = self._lv if self._selftest_mode != 2 else 0.15
            disp_av = self._av if self._selftest_mode != 2 else 0.0
            self._escalate('drive',
                '!!! DRIVE STALL — commanding lv={:.2f} av={:.2f} but no IMU motion'.format(
                    disp_lv, disp_av))

    def _diag_excavation_verify(self, now):
        """Cross-check: teleop commanding excavation vs what Pico confirms."""
        # Self-test mode 3: pretend excavation is on but Pico shows EX:0
        if self._selftest_mode == 3:
            if not self._pico_connected():
                return
            pico_ex = 0
            commanding = 6000
            # Skip the normal early-exit checks, jump to mismatch logic below
            if self._ex_mismatch_since == 0.0:
                self._ex_mismatch_since = now
            dur = now - self._ex_mismatch_since
            if dur > 10:
                self._escalate('ex_verify',
                    '!!! EXCAVATION NOT REACHING MOTOR {:.0f}s — '
                    'commanding PWM {} but Pico shows EX:0. '
                    'Check pico_bridge node. Try: r (off), r (on).'.format(dur, commanding))
            elif dur > 3:
                self._escalate('ex_verify',
                    '!!! EXCAVATION MISMATCH — commanding PWM {} but Pico shows EX:0. '
                    'Pico may not be forwarding.'.format(commanding))
            return

        if not self._ex_on or self._ex_pwm == 0:
            self._ex_mismatch_since = 0.0
            return

        if not self._pico_connected():
            # Pico offline — that warning is already being shown
            return

        pv = self._parse_pico_vals()
        pico_ex = abs(pv.get('EX', 0))
        commanding = self._ex_pwm  # always positive in teleop state

        # If Pico confirms non-zero EX, commands are getting through
        if pico_ex > 0:
            if self._ex_mismatch_since > 0:
                self._escalate('ex_verify',
                    'Excavation confirmed — Pico forwarding EX:{}'.format(pv.get('EX', 0)))
                self._ex_mismatch_since = 0.0
            return

        # Pico shows EX:0 but we're commanding non-zero
        if self._ex_mismatch_since == 0.0:
            self._ex_mismatch_since = now

        dur = now - self._ex_mismatch_since
        if dur > 10:
            self._escalate('ex_verify',
                '!!! EXCAVATION NOT REACHING MOTOR {:.0f}s — '
                'commanding PWM {} but Pico shows EX:0. '
                'Check pico_bridge node. Try: r (off), r (on).'.format(dur, commanding))
        elif dur > 3:
            self._escalate('ex_verify',
                '!!! EXCAVATION MISMATCH — commanding PWM {} but Pico shows EX:0. '
                'Pico may not be forwarding.'.format(commanding))

    def _escalate(self, category, msg):
        """Print an escalation message, deduplicated: only prints if the
        message changed or 10+ seconds passed since same message."""
        now = time.monotonic()
        last = self._last_escalation.get(category)
        if last is not None and last[0] == msg and (now - last[1]) < 10.0:
            return  # Dedup — same message within 10s
        self._last_escalation[category] = (msg, now)
        print('\n  ' + msg)
        self._print_status()

    # ------------------------------------------------------------------
    # Diagnostic snapshot (press '1')
    # ------------------------------------------------------------------

    def _cycle_selftest(self):
        """Advance self-test mode and reset injected state from previous mode."""
        prev = self._selftest_mode
        self._selftest_mode = (self._selftest_mode + 1) % len(SELFTEST_MODES)

        # Clean up state from previous test mode
        if prev == 1:  # PICO_OFFLINE
            self._pico_offline_since = 0.0
        elif prev == 2:  # DRIVE_STALL
            self._drive_stall_since = 0.0
        elif prev == 3:  # EX_MISMATCH
            self._ex_mismatch_since = 0.0

        # Seed initial timestamps for new test mode so escalation starts
        now = time.monotonic()
        if self._selftest_mode == 1:  # PICO_OFFLINE
            self._pico_offline_since = now
        elif self._selftest_mode == 2:  # DRIVE_STALL
            self._last_motion_t = now - self._stall_timeout - 1.0
            self._drive_stall_since = now
        elif self._selftest_mode == 3:  # EX_MISMATCH
            self._ex_mismatch_since = now

        # Clear escalation dedup so messages show immediately
        self._last_escalation.clear()

        mode_name = SELFTEST_MODES[self._selftest_mode]
        if self._selftest_mode == 0:
            print('\n  >> SELF-TEST OFF — back to normal operation')
        else:
            print('\n  >> SELF-TEST: {} — diagnostics will fire for this failure'.format(
                mode_name))
            print('     (No real commands affected. Press 1 again to cycle.)')

    def _print_diagnostics(self):
        """Full system state dump for troubleshooting at competition."""
        now = time.monotonic()

        # Cycle self-test mode first
        self._cycle_selftest()

        print('\n  ====== DIAGNOSTIC SNAPSHOT ======')
        if self._selftest_mode > 0:
            print('  SELF-TEST:  ** {} ** (simulated failure)'.format(
                SELFTEST_MODES[self._selftest_mode]))
        print('  Mode:       {}'.format('MANUAL' if self._manual else 'OBSERVER'))
        print('  E-Stop:     {}'.format('ON' if self._estop else 'OFF'))
        print('  Driving:    lv={:+.2f} m/s  av={:+.2f} rad/s'.format(
            self._lv, self._av))

        if self._ex_on:
            pct = self._ex_pwm * 100 // PWM_MAX
            print('  Excavation: ON, PWM {} ({}%), wire: -{}'.format(
                self._ex_pwm, pct, self._ex_pwm))
        else:
            print('  Excavation: OFF (stored speed: {})'.format(self._ex_speed))

        if self._ac_val > 0:
            print('  Actuator:   UP (+{})'.format(self._ac_val))
        elif self._ac_val < 0:
            print('  Actuator:   DOWN ({})'.format(self._ac_val))
        else:
            print('  Actuator:   STOPPED')

        # -- Pico bridge --
        if self._pico_alive():
            age = now - self._pico_last_t
            print('  Pico:       CONNECTED (last msg: {:.1f}s ago)'.format(age))
            print('    Raw:      {}'.format(self._pico_status_str))
            pv = self._parse_pico_vals()
            confirmed = []
            for k in ('L', 'R', 'EX', 'AC'):
                if k in pv:
                    confirmed.append('{}:{}'.format(k, pv[k]))
            if confirmed:
                print('    Confirmed: {}'.format('  '.join(confirmed)))
            if pv.get('_stall_recovery'):
                print('    ** Pico stall recovery in progress **')
            if pv.get('_kick'):
                print('    ** PWM kick active **')
        elif self._pico_offline_since > 0:
            dur = now - self._pico_offline_since
            print('  Pico:       OFFLINE ({:.0f}s)'.format(dur))
            print('    Action:   unplug/replug Pico USB, check /dev/ttyACM*')
        else:
            print('  Pico:       NO DATA (never received /pico/status)')
            print('    Action:   is pico_bridge node running?')
            print('              ros2 run lunabotics_control pico_bridge')

        # -- IMU / LiDAR --
        if self._imu_alive():
            age = now - self._imu_last_t
            motion_str = 'YES' if self._imu_has_motion else 'NO'
            print('  LiDAR/IMU:  ALIVE (last: {:.1f}s ago)'.format(age))
            print('    Motion:   {} (gyro_z={:.3f}, vibration={:.2f})'.format(
                motion_str, self._imu_gyro_z, self._imu_vibration))
        else:
            print('  LiDAR/IMU:  OFFLINE')
            print('    Action:   check ethernet to L2 (192.168.1.62)')
            print('              ip addr show enP8p1s0  (need 192.168.1.2)')

        # -- System --
        print('  System:     {}'.format(self._sys_health))

        # -- Active issues --
        issues = []
        if self._drive_stall_since > 0:
            issues.append('Drive stall: {:.0f}s'.format(
                now - self._drive_stall_since))
        if self._pico_offline_since > 0:
            issues.append('Pico offline: {:.0f}s'.format(
                now - self._pico_offline_since))
        if self._ex_mismatch_since > 0:
            issues.append('Excavation mismatch: {:.0f}s'.format(
                now - self._ex_mismatch_since))

        if issues:
            print('  Issues:     {}'.format(', '.join(issues)))
        else:
            print('  Issues:     none')

        print('  ================================')
        self._print_status()

    # ------------------------------------------------------------------
    # Motor stop helpers
    # ------------------------------------------------------------------

    def _stop_all(self):
        """Immediately zero ALL motor outputs."""
        self._lv     = 0.0
        self._av     = 0.0
        self._ex_pwm = 0
        self._ex_on  = False
        self._ac_val = 0
        # Clear stall tracking (we stopped commanding)
        self._drive_stall_since = 0.0
        self._ex_mismatch_since = 0.0
        # Publish zeros right now (don't wait for next loop tick)
        zero = Twist()
        self._pub_cmd.publish(zero)
        self._pub_cmd_safe.publish(zero)
        self._publish_int(self._pub_ex, 0)
        self._publish_int(self._pub_ac, 0)

    def _stop_actuator(self):
        """Stop only the actuator."""
        self._ac_val = 0
        self._publish_int(self._pub_ac, 0)

    @staticmethod
    def _publish_int(pub, val):
        msg = Int32()
        msg.data = val
        pub.publish(msg)

    # ------------------------------------------------------------------
    # Status display
    # ------------------------------------------------------------------

    def _print_status(self):
        mode  = 'MANUAL ' if self._manual else 'OBSERVE'
        estop = 'E-STOP' if self._estop else ''

        parts = [f'[{mode}]']
        parts.append(f'lv={self._lv:+.2f} av={self._av:+.2f}')

        if self._ex_on:
            pct = self._ex_pwm * 100 // PWM_MAX
            parts.append(f'EX:{self._ex_pwm}({pct}%)')

        if self._ac_val > 0:
            parts.append('AC:UP')
        elif self._ac_val < 0:
            parts.append('AC:DN')

        if estop:
            parts.append(estop)

        if self._selftest_mode > 0:
            parts.append('TEST:' + SELFTEST_MODES[self._selftest_mode])

        warnings = self._health_warnings()
        if warnings:
            parts.append('|')
            parts.extend(warnings)

        line = '  ' + '  '.join(parts)
        print(f'\r{line:<110}', end='', flush=True)

        # Publish machine-readable status topic
        status_msg = String()
        status_msg.data = (
            f'{"MANUAL" if self._manual else "OBSERVER"} '
            f'lv={self._lv:.2f} av={self._av:.2f} '
            f'ex={self._ex_pwm} ac={self._ac_val} '
            f'estop={self._estop}'
        )
        self._pub_info.publish(status_msg)

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        try:
            while rclpy.ok():
                key = _get_key_nonblocking(settings)

                # Process subscriber callbacks (health monitoring)
                rclpy.spin_once(self, timeout_sec=0)

                # ---- Global keys (work in any mode) ----

                if key == 'q':
                    self._stop_all()
                    break

                elif key == 'e':
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

                elif key == 'm':
                    self._manual = not self._manual
                    if self._manual:
                        self._stop_all()
                        print('\n  >>> MANUAL MODE — wasd + excavation/actuator/servo')
                    else:
                        self._stop_all()
                        print('\n  >>> OBSERVER MODE — autonomous system resumed')
                    self._print_status()

                elif key == '1':
                    self._print_diagnostics()

                elif self._manual and not self._estop:
                    self._handle_manual_key(key)

                # ---- Continuous publishing (runs every ~50ms) ----
                if self._manual:
                    self._publish_continuous()

                # ---- Self-diagnosis (runs every ~1s in manual mode) ----
                self._run_diagnostics()

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            self._stop_all()
            print('\nTeleop stopped — all motors zeroed.')

    # ------------------------------------------------------------------
    # Manual-mode key handler
    # ------------------------------------------------------------------

    def _handle_manual_key(self, key):
        if key is None:
            return

        # ---- Driving ----
        if key in DRIVE_KEYS:
            dl, da = DRIVE_KEYS[key]
            self._lv = max(-self._max_lin,
                           min(self._max_lin,
                               self._lv + dl * self._lin_step))
            self._av = max(-self._max_ang,
                           min(self._max_ang,
                               self._av + da * self._ang_step))
            self._print_status()

        elif key == ' ':
            self._stop_all()
            print('\n  ** FULL STOP **')
            self._print_status()

        # ---- Excavation belt ----
        elif key == 'r':
            self._ex_on = not self._ex_on
            if self._ex_on:
                self._ex_pwm = self._ex_speed
                pct = self._ex_pwm * 100 // PWM_MAX
                print(f'\n  EXCAVATION ON — PWM {self._ex_pwm} ({pct}%){self._pico_warn_str()}')
            else:
                self._ex_pwm = 0
                self._ex_mismatch_since = 0.0
                self._publish_int(self._pub_ex, 0)
                print('\n  EXCAVATION OFF')
            self._print_status()

        elif key == 'f':
            self._ex_speed = min(self._dig_pwm_max,
                                 self._ex_speed + self._dig_pwm_step)
            if self._ex_on:
                self._ex_pwm = self._ex_speed
            pct = self._ex_speed * 100 // PWM_MAX
            print(f'\n  Excavation speed: {self._ex_speed} ({pct}%)')
            self._print_status()

        elif key == 'v':
            self._ex_speed = max(self._dig_pwm_min,
                                 self._ex_speed - self._dig_pwm_step)
            if self._ex_on:
                self._ex_pwm = self._ex_speed
            pct = self._ex_speed * 100 // PWM_MAX
            print(f'\n  Excavation speed: {self._ex_speed} ({pct}%)')
            self._print_status()

        # ---- Actuator ----
        elif key == 'u':
            self._ac_val = self._ac_up
            print(f'\n  ACTUATOR UP (+{self._ac_up}){self._pico_warn_str()}')
            self._print_status()

        elif key == 'j':
            self._ac_val = self._ac_down
            print(f'\n  ACTUATOR DOWN ({self._ac_down}){self._pico_warn_str()}')
            self._print_status()

        elif key == 'h':
            self._stop_actuator()
            print('\n  ACTUATOR STOPPED')
            self._print_status()

        # ---- Deposition servo ----
        elif key == 't':
            servo_msg = String()
            servo_msg.data = f'{self._dump_angle},{self._servo_time}'
            self._pub_servo.publish(servo_msg)
            print(f'\n  DUMP — servo to {self._dump_angle:.0f} deg over {self._servo_time}ms')
            self._print_status()

        elif key == 'y':
            servo_msg = String()
            servo_msg.data = f'{self._stow_angle},{self._servo_time}'
            self._pub_servo.publish(servo_msg)
            print(f'\n  STOW — servo to {self._stow_angle:.0f} deg')
            self._print_status()

    # ------------------------------------------------------------------
    # Continuous command publishing (~20Hz from the 50ms poll loop)
    # ------------------------------------------------------------------

    def _publish_continuous(self):
        """Publish all motor commands every loop tick.

        Pico bridge has a 500ms watchdog — if it stops receiving excavation
        or actuator messages, it zeros them. We must keep publishing while
        active. Actuator (Sabertooth) also needs continuous pulse updates.
        """
        if self._estop:
            zero = Twist()
            self._pub_cmd.publish(zero)
            self._pub_cmd_safe.publish(zero)
            self._publish_int(self._pub_ex, 0)
            self._publish_int(self._pub_ac, 0)
            return

        # Driving — publish to both /cmd_vel (for relay) and /cmd_vel_safe (direct to pico)
        twist = Twist()
        twist.linear.x  = self._lv
        twist.angular.z = self._av
        self._pub_cmd.publish(twist)
        self._pub_cmd_safe.publish(twist)

        # Excavation motor — publish continuously while on (Pico 500ms watchdog)
        if self._ex_pwm != 0:
            self._publish_int(self._pub_ex, -self._ex_pwm)

        # Actuator — publish continuously while moving (Sabertooth needs pulse)
        if self._ac_val != 0:
            self._publish_int(self._pub_ac, self._ac_val)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
