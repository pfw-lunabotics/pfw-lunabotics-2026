#!/usr/bin/env python3
"""
pico_bridge.py — ROS2 <-> Raspberry Pi Pico serial bridge for Lunabotics 2026
==============================================================================
Subscribes to /cmd_vel_safe (Twist), converts to 4-wheel skid-steer
motor commands, and sends them over USB serial to the Pico.

Also handles excavation motor, actuator, and deposition servo commands.

ENCODER ODOMETRY:
  Pico reports encoder delta counts every 50ms as [ENC:FL,FR,RL,RR].
  Bridge computes differential drive odometry and publishes /odom.
  PPR = 1098 (measured 2026-05-10).

STALL DETECTION:
  Primary: encoder-based (commanding PWM but no encoder pulses for 1s).
  Fallback: IMU-based (no angular velocity/vibration for stall_timeout).
  Recovery: serial reconnect → DTR reset → PWM kick → alternate port scan.

Serial protocol (Pico expects these):
  Drivetrain: <FL:+/-PWM,FR:+/-PWM,RL:+/-PWM,RR:+/-PWM>\n  (16-bit, -65535..+65535)
  Excavation: <EX:+/-PWM>\n                                   (16-bit, -65535..+65535)
  Actuator:   <AC:val>\n                                       (-100..+100)

Serial feedback (Pico sends these):
  Encoder:    [ENC:fl_delta,fr_delta,rl_delta,rr_delta]\n  (signed int, 50ms intervals)
  Status:     [OK], [WATCHDOG], [READY], [SERVO:OK/NOT_FOUND]

Subscribers:
  /cmd_vel_safe       (geometry_msgs/Twist) — safety-checked velocity command
  /excavation/motor   (std_msgs/Int32)      — excavation belt motor PWM
  /actuator/command   (std_msgs/Int32)      — actuator position (-100..+100)
  /unilidar/imu       (sensor_msgs/Imu)     — for fallback stall detection

Publishers:
  /odom               (nav_msgs/Odometry)   — wheel encoder odometry
  /pico/status        (std_msgs/String)     — bridge status + diagnostics
  /pico/encoders      (std_msgs/String)     — raw encoder feedback for debugging
"""

import glob
import math
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Int32, Bool, Float32
from tf2_ros import TransformBroadcaster
import serial
import time

# Pico hardware PWM is 16-bit (0-65535)
PWM_MAX = 65535


class PicoBridge(Node):

    def __init__(self):
        super().__init__('pico_bridge')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_separation', 0.65)
        self.declare_parameter('wheel_radius', 0.15)
        self.declare_parameter('max_motor_speed', 0.6)
        self.declare_parameter('publish_rate', 20.0)
        # Encoder parameters
        self.declare_parameter('encoder_ppr', 1098)         # Pulses per revolution (measured)
        self.declare_parameter('publish_odom', True)         # Publish /odom from encoders
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        # Stall detection parameters
        self.declare_parameter('stall_timeout', 1.0)        # seconds — encoder-based (was 5s IMU-only)
        self.declare_parameter('stall_min_pwm', 3000)       # PWM below this = not really trying to move
        self.declare_parameter('kick_extra_pwm', 20000)     # extra PWM added during kick
        self.declare_parameter('kick_duration', 0.5)        # seconds of kick pulse
        self.declare_parameter('imu_motion_threshold', 0.08)  # rad/s angular velocity = moving
        # Wheel slip detection: wheels spinning but robot not moving (sinking in sand)
        self.declare_parameter('slip_detection', True)       # Enable slip detection
        self.declare_parameter('slip_timeout', 1.0)          # seconds of slip before boosting
        self.declare_parameter('slip_boost_step', 0.15)      # PWM multiplier increase per step
        self.declare_parameter('slip_boost_max', 1.8)        # max PWM multiplier (1.0 = no boost)
        self.declare_parameter('slip_boost_interval', 0.5)   # seconds between boost increments
        self.declare_parameter('slip_accel_threshold', 0.3)  # m/s² — IMU accel deviation indicating actual motion
        # HX711 weight sensor (raw counts → kg calibration)
        self.declare_parameter('weight_zero', 1700)         # raw ADC at zero load (tare)
        self.declare_parameter('weight_scale', 28000.0)     # raw counts per kg (calibrate with known weight)
        # Drivetrain direction inversion (per-axis, runtime-toggleable).
        # Set whichever is wrong on the day. `false` means upstream sign is
        # forwarded unchanged; `true` means negated at the hardware boundary.
        self.declare_parameter('invert_linear', False)      # W=forward (was True, caused reversal)
        self.declare_parameter('invert_angular', True)      # A=left turn (matches existing wiring)

        self._port = self.get_parameter('serial_port').value
        self._baud = self.get_parameter('baud_rate').value
        self._track = self.get_parameter('wheel_separation').value
        self._radius = self.get_parameter('wheel_radius').value
        self._max_speed = self.get_parameter('max_motor_speed').value
        self._invert_linear = self.get_parameter('invert_linear').value
        self._invert_angular = self.get_parameter('invert_angular').value
        rate = self.get_parameter('publish_rate').value

        self._ppr = self.get_parameter('encoder_ppr').value
        self._publish_odom = self.get_parameter('publish_odom').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value

        self._stall_timeout = self.get_parameter('stall_timeout').value
        self._stall_min_pwm = self.get_parameter('stall_min_pwm').value
        self._kick_extra_pwm = self.get_parameter('kick_extra_pwm').value
        self._kick_duration = self.get_parameter('kick_duration').value
        self._imu_motion_thresh = self.get_parameter('imu_motion_threshold').value

        # Slip detection params
        self._slip_detection = self.get_parameter('slip_detection').value
        self._slip_timeout = self.get_parameter('slip_timeout').value
        self._slip_boost_step = self.get_parameter('slip_boost_step').value
        self._slip_boost_max = self.get_parameter('slip_boost_max').value
        self._slip_boost_interval = self.get_parameter('slip_boost_interval').value
        self._slip_accel_threshold = self.get_parameter('slip_accel_threshold').value

        # Encoder → distance conversion
        # distance_per_pulse = (2 * pi * wheel_radius) / PPR
        self._dist_per_pulse = (2.0 * math.pi * self._radius) / self._ppr

        # State -- drivetrain
        self._linear = 0.0
        self._angular = 0.0
        self._last_cmd_t = time.monotonic()

        # State -- excavation motor
        self._ex_pwm = 0
        self._last_ex_t = 0.0

        # State -- actuator
        self._ac_val = 0
        self._last_ac_t = 0.0
        self._ac_changed = False

        # Serial
        self._serial = None
        self._last_reconnect_t = 0.0
        self._serial_buf = ""  # Buffer for incoming serial lines

        # Odometry state (integrated position from encoders)
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_theta = 0.0
        self._odom_vx = 0.0
        self._odom_wz = 0.0
        self._last_enc_time = time.monotonic()
        self._enc_alive = False  # True once we receive first encoder data

        # Stall detection state (encoder-based primary, IMU fallback)
        self._last_motion_time = time.monotonic()
        self._last_enc_motion_time = time.monotonic()
        self._imu_alive = False
        self._stall_recovery_count = 0
        self._last_recovery_time = 0.0
        self._recovery_cooldown = 8.0      # seconds between recovery attempts
        self._in_kick = False
        self._kick_end_time = 0.0
        self._kick_pwm_left = 0
        self._kick_pwm_right = 0

        # Slip detection state: wheels spinning but robot not moving
        self._slip_boost = 1.0              # Current PWM multiplier (1.0 = normal)
        self._slip_start_time = 0.0         # When slip was first detected
        self._last_boost_time = 0.0         # When we last increased the boost
        self._is_slipping = False           # Currently in slip condition
        self._imu_accel_deviation = 0.0     # Current acceleration deviation from gravity
        self._last_imu_moving = time.monotonic()  # Last time IMU showed real robot motion

        # Weight sensor state (HX711 via Pico serial)
        self._weight_zero = self.get_parameter('weight_zero').value
        self._weight_scale = self.get_parameter('weight_scale').value

        # Publishers
        self._pub_status = self.create_publisher(String, '/pico/status', 10)
        self._pub_encoders = self.create_publisher(String, '/pico/encoders', 10)
        self._pub_odom = self.create_publisher(Odometry, '/odom', 50)
        self._pub_stall = self.create_publisher(Bool, '/pico/stalled', 10)
        self._pub_slip = self.create_publisher(Bool, '/pico/wheel_slip', 10)
        self._pub_weight = self.create_publisher(Float32, '/deposition/weight', 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.create_subscription(Twist, '/cmd_vel_safe', self._cmd_cb, 10)
        self.create_subscription(Int32, '/excavation/motor', self._ex_cb, 10)
        self.create_subscription(Int32, '/actuator/command', self._ac_cb, 10)
        self.create_subscription(Imu, '/unilidar/imu', self._imu_cb, 5)

        # Connect serial
        self._connect_serial()

        # Timer to send commands at fixed rate
        self.create_timer(1.0 / rate, self._send_cb)

        # Stall check at 2 Hz (faster with encoder feedback)
        self.create_timer(0.5, self._check_stall)

        self.get_logger().info(
            f'Pico bridge ready -- port={self._port}, baud={self._baud}, '
            f'track={self._track}m, radius={self._radius}m, '
            f'PPR={self._ppr}, dist/pulse={self._dist_per_pulse:.6f}m, '
            f'stall_timeout={self._stall_timeout}s'
        )

    # ------------------------------------------------------------------
    # Serial connection
    # ------------------------------------------------------------------

    def _connect_serial(self):
        """Attempt to open serial port. Non-fatal if it fails."""
        try:
            self._serial = serial.Serial(
                self._port, self._baud, timeout=0.01
            )
            self.get_logger().info(f'Serial connected: {self._port}')
            self._publish_status('CONNECTED')
        except serial.SerialException as e:
            self._serial = None
            self.get_logger().warn(
                f'Serial port {self._port} not available: {e}. '
                'Will retry on next send cycle.'
            )
            self._publish_status('DISCONNECTED')

    def _reconnect_serial(self):
        """Close, optionally reset Pico via DTR toggle, and reopen."""
        if self._serial is not None:
            try:
                # Toggle DTR to reset Pico (MicroPython resets on DTR low)
                self._serial.dtr = False
                time.sleep(0.1)
                self._serial.dtr = True
                time.sleep(0.1)
                self._serial.close()
            except Exception:
                pass
            self._serial = None

        time.sleep(0.3)
        self._connect_serial()

    def _scan_alternate_ports(self):
        """Try to find Pico on another /dev/ttyACM* port."""
        # Check stable symlink first
        pico_symlink = '/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e6647c156730ab24-if00'
        candidates = []
        try:
            if os.path.exists(pico_symlink):
                real_path = os.path.realpath(pico_symlink)
                if real_path != self._port:
                    candidates.insert(0, real_path)
        except Exception:
            pass

        # Then try all ttyACM ports
        for port in sorted(glob.glob('/dev/ttyACM*')):
            if port != self._port and port not in candidates:
                candidates.append(port)

        for alt_port in candidates:
            try:
                self._serial = serial.Serial(alt_port, self._baud, timeout=0.01)
                self._port = alt_port
                self.get_logger().warn(f'STALL RECOVERY: switched to alternate port {alt_port}')
                self._publish_status(f'RECONNECTED:{alt_port}')
                return True
            except serial.SerialException:
                continue

        return False

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------

    def _cmd_cb(self, msg: Twist):
        # All ROS nodes (Nav2, teleop, mission_controller) use standard
        # convention: +linear.x = forward, +angular.z = left turn (CCW).
        # If the motor wiring is reversed on either axis, set the matching
        # invert_* param to true. Runtime-toggleable via:
        #   ros2 param set /pico_bridge invert_linear  true|false
        #   ros2 param set /pico_bridge invert_angular true|false
        # (re-read live so the field can flip without restart)
        self._invert_linear = self.get_parameter('invert_linear').value
        self._invert_angular = self.get_parameter('invert_angular').value
        self._linear = -msg.linear.x if self._invert_linear else msg.linear.x
        self._angular = -msg.angular.z if self._invert_angular else msg.angular.z
        self._last_cmd_t = time.monotonic()

    def _ex_cb(self, msg: Int32):
        self._ex_pwm = max(-PWM_MAX, min(PWM_MAX, msg.data))
        self._last_ex_t = time.monotonic()

    def _ac_cb(self, msg: Int32):
        self._ac_val = max(-100, min(100, msg.data))
        self._last_ac_t = time.monotonic()
        self._ac_changed = True

    def _imu_cb(self, msg: Imu):
        """Detect motion from IMU angular velocity and linear acceleration."""
        self._imu_alive = True
        # Angular velocity (turning) is the most reliable motion signal
        gyro_z = abs(msg.angular_velocity.z)
        gyro_xy = math.sqrt(msg.angular_velocity.x**2 + msg.angular_velocity.y**2)
        # Linear acceleration deviation from gravity (vibration from driving on sand)
        accel_mag = math.sqrt(
            msg.linear_acceleration.x**2 +
            msg.linear_acceleration.y**2 +
            msg.linear_acceleration.z**2
        )
        # Gravity is ~9.81; driving on sand adds vibration (magnitude fluctuates)
        accel_deviation = abs(accel_mag - 9.81)
        self._imu_accel_deviation = accel_deviation

        # Robot is actually moving if: turning, tilting, or experiencing vibrations
        now = time.monotonic()
        if gyro_z > self._imu_motion_thresh or gyro_xy > 0.15 or accel_deviation > self._slip_accel_threshold:
            self._last_motion_time = now
            self._last_imu_moving = now

    # ------------------------------------------------------------------
    # Stall detection and recovery
    # ------------------------------------------------------------------

    def _check_stall(self):
        """Called at 2 Hz. Detect 'commanding but not moving' and recover.

        Primary detection: encoder-based (no pulses while commanding PWM).
        Fallback: IMU-based (if encoders not available).
        """
        now = time.monotonic()

        # End kick if active
        if self._in_kick and now >= self._kick_end_time:
            self._in_kick = False

        # Compute current commanded PWM magnitude
        left_speed = self._linear - (self._track / 2.0) * self._angular
        right_speed = self._linear + (self._track / 2.0) * self._angular
        max_commanded = max(abs(self._speed_to_pwm(left_speed)),
                           abs(self._speed_to_pwm(right_speed)))

        # Not commanding significant motion? Reset stall timer.
        if max_commanded < self._stall_min_pwm:
            self._last_motion_time = now
            self._last_enc_motion_time = now
            return

        # Determine stall duration based on available sensors
        if self._enc_alive:
            # Primary: encoder-based (fast, 1s timeout)
            stall_duration = now - self._last_enc_motion_time
        elif self._imu_alive:
            # Fallback: IMU-based (slower, use stall_timeout)
            stall_duration = now - self._last_motion_time
        else:
            # No sensors available, can't detect stall
            return

        # Publish stall state
        stall_msg = Bool()
        stall_msg.data = stall_duration >= self._stall_timeout
        self._pub_stall.publish(stall_msg)

        if stall_duration < self._stall_timeout:
            return  # Not stalled yet

        # Cooldown between recovery attempts
        if (now - self._last_recovery_time) < self._recovery_cooldown:
            return

        # === STALL DETECTED ===
        self._stall_recovery_count += 1
        self._last_recovery_time = now
        source = "ENCODER" if self._enc_alive else "IMU"
        self.get_logger().error(
            f'STALL DETECTED ({source}): commanding PWM {max_commanded} for '
            f'{stall_duration:.1f}s but no motion. Recovery #{self._stall_recovery_count}')

        # Step 1: Serial reconnect (fixes buffer corruption, firmware hang)
        self.get_logger().warn('STALL RECOVERY step 1: serial reconnect + DTR reset')
        self._reconnect_serial()

        # Step 2: If reconnect failed, try alternate ports
        if self._serial is None:
            self.get_logger().warn('STALL RECOVERY step 2: scanning alternate ports')
            self._scan_alternate_ports()

        # Step 3: PWM kick (overcome static friction / driver deadband)
        if self._serial is not None:
            sign_l = 1 if left_speed >= 0 else -1
            sign_r = 1 if right_speed >= 0 else -1
            self._kick_pwm_left = sign_l * min(PWM_MAX, abs(self._speed_to_pwm(left_speed)) + self._kick_extra_pwm)
            self._kick_pwm_right = sign_r * min(PWM_MAX, abs(self._speed_to_pwm(right_speed)) + self._kick_extra_pwm)
            self._in_kick = True
            self._kick_end_time = now + self._kick_duration
            self.get_logger().warn(
                f'STALL RECOVERY step 3: PWM kick L:{self._kick_pwm_left} R:{self._kick_pwm_right} '
                f'for {self._kick_duration}s')

        # Increase cooldown after each attempt (back off: 8s, 15s, 25s, ...)
        self._recovery_cooldown = min(30.0, self._recovery_cooldown + 5.0)

        self._publish_status(
            f'STALL_RECOVERY:{self._stall_recovery_count} '
            f'port={self._port}')

        # Reset motion timers to give recovery time to work
        self._last_motion_time = now
        self._last_enc_motion_time = now

    # ------------------------------------------------------------------
    # Wheel slip detection and velocity boost
    # ------------------------------------------------------------------

    def _check_slip(self, now, left_pwm, right_pwm):
        """Detect wheels spinning but robot not moving (sinking in sand).

        Condition: encoders show motion (wheels turning) BUT IMU shows no
        forward progress (no vibration/acceleration). Response: gradually
        increase PWM multiplier to power out of the rut.
        """
        if not self._slip_detection or not self._imu_alive:
            return

        # Are we commanding significant motion?
        max_pwm = max(abs(left_pwm), abs(right_pwm))
        if max_pwm < self._stall_min_pwm:
            # Not commanding motion — reset slip state
            self._slip_boost = 1.0
            self._is_slipping = False
            self._slip_start_time = 0.0
            return

        # Are wheels actually turning? (encoder motion detected recently)
        enc_moving = self._enc_alive and (now - self._last_enc_motion_time) < 0.3

        # Is the robot actually moving? (IMU detects real motion)
        imu_moving = (now - self._last_imu_moving) < 0.5

        # SLIP = wheels turning BUT robot not moving
        if enc_moving and not imu_moving:
            if not self._is_slipping:
                # Just started slipping
                self._is_slipping = True
                self._slip_start_time = now
                self.get_logger().warn('WHEEL SLIP detected: wheels spinning but robot not moving')

            slip_duration = now - self._slip_start_time

            # After slip_timeout, start boosting
            if slip_duration >= self._slip_timeout:
                if (now - self._last_boost_time) >= self._slip_boost_interval:
                    self._last_boost_time = now
                    old_boost = self._slip_boost
                    self._slip_boost = min(self._slip_boost_max,
                                           self._slip_boost + self._slip_boost_step)
                    if self._slip_boost != old_boost:
                        self.get_logger().warn(
                            f'SLIP BOOST: {old_boost:.2f} → {self._slip_boost:.2f} '
                            f'(slip for {slip_duration:.1f}s)')
        else:
            # Not slipping — decay boost back to 1.0
            if self._is_slipping:
                self.get_logger().info(
                    f'Slip resolved (boost was {self._slip_boost:.2f})')
                self._is_slipping = False
            # Gradual decay back to normal (don't snap back instantly)
            if self._slip_boost > 1.0:
                self._slip_boost = max(1.0, self._slip_boost - 0.05)

        # Publish slip state
        slip_msg = Bool()
        slip_msg.data = self._is_slipping
        self._pub_slip.publish(slip_msg)

    # ------------------------------------------------------------------
    # Serial send loop (20 Hz)
    # ------------------------------------------------------------------

    def _send_cb(self):
        now = time.monotonic()

        # Watchdog: if no cmd_vel_safe for 500ms, zero drivetrain
        if (now - self._last_cmd_t) > 0.5:
            self._linear = 0.0
            self._angular = 0.0

        # Watchdog: if no excavation command for 500ms, zero it
        if self._last_ex_t > 0 and (now - self._last_ex_t) > 0.5:
            self._ex_pwm = 0

        # Skid-steer kinematics: left/right wheel surface speeds
        left_speed = self._linear - (self._track / 2.0) * self._angular
        right_speed = self._linear + (self._track / 2.0) * self._angular

        # Convert to PWM (-65535 to +65535)
        left_pwm = self._speed_to_pwm(left_speed)
        right_pwm = self._speed_to_pwm(right_speed)

        # --- Slip detection and boost ---
        # Wheels spinning (encoder alive + commanding motion) but IMU shows no movement
        self._check_slip(now, left_pwm, right_pwm)
        if self._slip_boost > 1.0 and not self._in_kick:
            left_pwm = int(min(PWM_MAX, max(-PWM_MAX, left_pwm * self._slip_boost)))
            right_pwm = int(min(PWM_MAX, max(-PWM_MAX, right_pwm * self._slip_boost)))

        # Override with kick PWM if active
        if self._in_kick:
            left_pwm = self._kick_pwm_left
            right_pwm = self._kick_pwm_right

        if self._serial is None:
            if (now - self._last_reconnect_t) >= 3.0:
                self._last_reconnect_t = now
                self._connect_serial()
            return

        try:
            # Read and parse incoming data from Pico (encoder reports, status)
            if self._serial.in_waiting:
                raw = self._serial.read(self._serial.in_waiting)
                try:
                    self._serial_buf += raw.decode('ascii', errors='ignore')
                except Exception:
                    self._serial_buf = ""
                # Process complete lines
                while '\n' in self._serial_buf:
                    line, self._serial_buf = self._serial_buf.split('\n', 1)
                    line = line.strip()
                    if line.startswith('[ENC:') and line.endswith(']'):
                        self._process_encoder(line)
                    elif line.startswith('[WT:') and line.endswith(']'):
                        self._process_weight(line)
                # Prevent buffer runaway
                if len(self._serial_buf) > 512:
                    self._serial_buf = self._serial_buf[-128:]

            # Always send drivetrain
            cmd = f'<FL:{left_pwm},FR:{right_pwm},RL:{left_pwm},RR:{right_pwm}>\n'
            self._serial.write(cmd.encode('ascii'))

            # Always send excavation (Pico needs explicit EX:0 to stop motor)
            ex_cmd = f'<EX:{self._ex_pwm}>\n'
            self._serial.write(ex_cmd.encode('ascii'))

            # Send actuator when changed
            if self._ac_changed:
                ac_cmd = f'<AC:{self._ac_val}>\n'
                self._serial.write(ac_cmd.encode('ascii'))
                self._ac_changed = False

            kick_str = ' KICK' if self._in_kick else ''
            slip_str = f' SLIP_BOOST:{self._slip_boost:.2f}' if self._slip_boost > 1.0 else ''
            self._publish_status(
                f'OK L:{left_pwm} R:{right_pwm} EX:{self._ex_pwm} AC:{self._ac_val}{kick_str}{slip_str}'
            )
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write failed: {e}')
            self._serial = None
            self._publish_status('DISCONNECTED')

    # ------------------------------------------------------------------
    # Encoder processing and odometry
    # ------------------------------------------------------------------

    def _process_encoder(self, line: str):
        """Parse [ENC:fl,fr,rl,rr] and compute odometry."""
        try:
            # Parse: [ENC:fl,fr,rl,rr]
            inner = line[5:-1]  # strip [ENC: and ]
            parts = inner.split(',')
            if len(parts) != 4:
                return
            fl, fr, rl, rr = int(parts[0]), int(parts[1]), int(parts[2]), int(parts[3])
        except (ValueError, IndexError):
            return

        self._enc_alive = True

        # Publish raw encoder data for debugging
        enc_msg = String()
        enc_msg.data = f'FL:{fl} FR:{fr} RL:{rl} RR:{rr}'
        self._pub_encoders.publish(enc_msg)

        # Average left and right sides for differential drive
        left_pulses = (fl + rl) / 2.0
        right_pulses = (fr + rr) / 2.0

        # Convert pulses to distance (meters)
        left_dist = left_pulses * self._dist_per_pulse
        right_dist = right_pulses * self._dist_per_pulse

        # Time delta
        now = time.monotonic()
        dt = now - self._last_enc_time
        self._last_enc_time = now

        if dt <= 0.0 or dt > 1.0:
            # Skip bogus time deltas (first iteration or after long gap)
            return

        # Differential drive forward kinematics
        linear_dist = (left_dist + right_dist) / 2.0
        angular_dist = (right_dist - left_dist) / self._track

        # Update pose (integrate)
        self._odom_theta += angular_dist
        # Normalize theta to [-pi, pi]
        self._odom_theta = math.atan2(
            math.sin(self._odom_theta), math.cos(self._odom_theta))
        self._odom_x += linear_dist * math.cos(self._odom_theta)
        self._odom_y += linear_dist * math.sin(self._odom_theta)

        # Compute velocities
        self._odom_vx = linear_dist / dt
        self._odom_wz = angular_dist / dt

        # Stall detection: update motion time if wheels are turning
        total_pulses = abs(fl) + abs(fr) + abs(rl) + abs(rr)
        if total_pulses > 2:  # More than noise threshold
            self._last_enc_motion_time = now
            self._last_motion_time = now  # Also update global motion time

        # Publish odometry
        if self._publish_odom:
            self._publish_odometry()

    def _process_weight(self, line: str):
        """Parse [WT:raw_value] and publish calibrated weight in kg."""
        try:
            raw = int(line[4:-1])  # strip [WT: and ]
            kg = (raw - self._weight_zero) / self._weight_scale
            if kg < 0.0:
                kg = 0.0
            msg = Float32()
            msg.data = kg
            self._pub_weight.publish(msg)
        except (ValueError, IndexError):
            pass

    def _publish_odometry(self):
        """Publish nav_msgs/Odometry and odom→base_footprint TF."""
        now = self.get_clock().now()

        # Quaternion from yaw
        cy = math.cos(self._odom_theta / 2.0)
        sy = math.sin(self._odom_theta / 2.0)

        # Odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame

        odom.pose.pose.position.x = self._odom_x
        odom.pose.pose.position.y = self._odom_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = sy
        odom.pose.pose.orientation.w = cy

        # Covariance — moderate confidence (encoder drift accumulates)
        # [x, y, z, roll, pitch, yaw] — diagonal
        odom.pose.covariance[0] = 0.01   # x
        odom.pose.covariance[7] = 0.01   # y
        odom.pose.covariance[35] = 0.03  # yaw

        odom.twist.twist.linear.x = self._odom_vx
        odom.twist.twist.angular.z = self._odom_wz

        odom.twist.covariance[0] = 0.005   # vx
        odom.twist.covariance[35] = 0.01   # wz

        self._pub_odom.publish(odom)

    def _speed_to_pwm(self, speed_mps: float) -> int:
        """Convert wheel surface speed (m/s) to PWM value (-65535 to 65535)."""
        ratio = speed_mps / self._max_speed
        ratio = max(-1.0, min(1.0, ratio))
        return int(round(ratio * PWM_MAX))

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self._pub_status.publish(msg)

    def destroy_node(self):
        if self._serial is not None:
            try:
                self._serial.write(b'<FL:0,FR:0,RL:0,RR:0>\n')
                self._serial.write(b'<EX:0>\n')
                self._serial.write(b'<AC:0>\n')
                self._serial.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PicoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
