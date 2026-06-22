#!/usr/bin/env python3
"""
Excavation Bridge for Lunabotics 2026
=======================================
Bridges mission controller <-> excavation/deposition hardware.

In simulation mode (simulate: true):
  - Simulates timing for dig/dump/stow
  - No real motor/actuator/servo control

On hardware (simulate: false):
  - Sends real commands to Pico bridge and servo driver:
    * /excavation/motor  (Int32)  — belt motor PWM (-65535..+65535)
    * /actuator/command   (Int32)  — actuator position (-100..+100)
    * /deposition/tilt    (String) — servo "angle,duration_ms"

Hardware DIG sequence (Scenario 2 final flow — actuator-first):
  1. Begin lowering actuator immediately (belt OFF).
  2. When actuator passes actuator_belt_engage_pct (60% depth):
     - Start belt at dig_pwm_start (10500).
     - Begin belt PWM ramp toward dig_pwm_target (21000).
  3. Actuator continues lowering to actuator_lower_cap_pct (85% depth).
  4. At 85%: actuator holds; bridge publishes /excavation/needs_nudge=True.
  5. Mission controller orchestrates step-and-dig from here: dwell, advance
     0.4m, dwell, ... — see mission_controller.py.
  6. If mission calls /excavation/dig_deeper (weight stall at 85%):
     - Cap extends to actuator_deeper_cap_pct (92%).
     - Actuator descends to new cap, then signals needs_nudge again.
  7. If mission calls /excavation/belt_pause (before any turn):
     - Belt motor → 0, actuator stepping halts (HOLDS position).
     - State stays DIGGING so mission can resume.
  8. /excavation/belt_resume restarts belt at last ramped PWM.
  9. Belt keeps running at target until mission controller calls STOW.

Motor direction is inverted (positive config = negative PWM on wire)
because the belt wiring runs backward at positive PWM values.

Hardware STOW sequence:
  1. Stop excavation belt motor (publish 0)
  2. Raise actuator (publish actuator_raise_val)
  3. Wait actuator_settle_sec
  4. Retract deposition servo to stowed position
  5. Return to IDLE

Hardware DUMP sequence:
  1. Tilt deposition servo to dump angle
  2. Mission controller waits dump_wait_sec
  3. Mission controller calls STOW to retract

Services:
  /excavation/dig          (Trigger) — start actuator descent + auto-engage belt
  /excavation/dump         (Trigger) — tilt deposition servo to dump
  /excavation/stow         (Trigger) — stop belt + raise actuator + retract servo
  /excavation/belt_pause   (Trigger) — stop belt, HOLD actuator (mission turn safety)
  /excavation/belt_resume  (Trigger) — restart belt at last ramped PWM
  /excavation/dig_deeper   (Trigger) — extend actuator cap from 85% to 92%

Publishers (in addition to /excavation/status, /excavation/command):
  /excavation/needs_nudge   (Bool)    — actuator at cap, mission should advance
  /excavation/actuator_pct  (Float32) — estimated actuator depth percentage (0–100)
  /excavation/belt_pwm      (Int32)   — current belt PWM target
"""

import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32, Float32, String as StringMsg
from luna_msgs.msg import ExcavationCommand, ExcavationStatus


class ExcavationBridge(Node):

    def __init__(self):
        super().__init__('excavation_bridge')

        self.declare_parameters(namespace='', parameters=[
            ('simulate', True),
            ('sim_dig_duration', 30.0),
            ('sim_dump_duration', 5.0),
            ('sim_stow_duration', 3.0),
            ('command_topic', '/excavation/command'),
            ('status_topic', '/excavation/status'),
            # Hardware motor parameters (Scenario 2 final flow)
            ('dig_pwm_start', 10500),            # Belt PWM at 60% actuator depth
            ('dig_pwm_target', 21000),           # Belt PWM cap (safe per latest tuning)
            ('dig_pwm_ramp_step', 1500),         # PWM increase per interval
            ('dig_pwm_ramp_interval', 1.0),      # Seconds between ramp steps
            # Hardware actuator parameters (actuator-first)
            ('actuator_lower_val', -100),         # Actuator full extend (-100)
            ('actuator_lower_cap_pct', 0.85),     # Default cap at 85% depth (1in ground clearance)
            ('actuator_deeper_cap_pct', 0.92),    # /excavation/dig_deeper extends cap to 92%
            ('actuator_belt_engage_pct', 0.60),   # Belt engages at this depth
            ('actuator_raise_val', 100),          # Actuator full retract (+100)
            ('actuator_settle_sec', 26.0),        # Wait time for actuator full travel
            ('actuator_step_val', -15),           # Actuator increment per step (negative = lower)
            ('actuator_step_interval', 2.0),      # Seconds between actuator steps
            ('actuator_initial_delay', 0.5),      # Seconds before first actuator step
            # Hardware deposition parameters
            ('deposition_dump_angle', 52.0),      # Servo tilt angle for dumping (mechanical open at +52.8 deg from closed)
            ('deposition_stow_angle', -10.0),     # Stow angle (deg); -10 preloads servo against closed hard stop (firm lock)
            ('deposition_move_time_ms', 4500),    # Servo movement time (ms)
            # Weight sensor feedback for smart lowering
            ('weight_topic', '/deposition/weight'),
            ('weight_gain_threshold', 0.02),      # kg gain to consider "digging effectively"
            ('weight_check_interval', 3.0),       # seconds between weight checks during lowering
            ('weight_stall_resume_sec', 5.0),     # seconds to wait before lowering more if weight stalls
            # Turn guard — pause excavation during turns to save power
            ('turn_pause_threshold', 0.1),        # rad/s — pause if |angular.z| exceeds this
            # Software E-stop — disable for competition (hands-free, can't clear it)
            ('software_estop_enabled', True),
        ])

        self._simulate = self.get_parameter('simulate').value
        self._dig_dur = self.get_parameter('sim_dig_duration').value
        self._dump_dur = self.get_parameter('sim_dump_duration').value
        self._stow_dur = self.get_parameter('sim_stow_duration').value

        # Hardware params — excavation ramp
        self._dig_pwm_start = self.get_parameter('dig_pwm_start').value
        self._dig_pwm_target = self.get_parameter('dig_pwm_target').value
        self._dig_ramp_step = self.get_parameter('dig_pwm_ramp_step').value
        self._dig_ramp_interval = self.get_parameter('dig_pwm_ramp_interval').value
        self._current_dig_pwm = 0
        self._ramp_timer = None

        self._ac_lower = self.get_parameter('actuator_lower_val').value
        self._ac_lower_cap_pct = self.get_parameter('actuator_lower_cap_pct').value
        self._ac_deeper_cap_pct = self.get_parameter('actuator_deeper_cap_pct').value
        self._ac_belt_engage_pct = self.get_parameter('actuator_belt_engage_pct').value
        self._ac_raise = self.get_parameter('actuator_raise_val').value
        # Cap formula: raise + pct*(lower-raise).
        # E.g. raise=100, lower=-100, cap_pct=0.85 → cap = 100 + 0.85*(-200) = -70
        self._ac_lower_cap = int(
            self._ac_raise + self._ac_lower_cap_pct * (self._ac_lower - self._ac_raise))
        self._ac_deeper_cap = int(
            self._ac_raise + self._ac_deeper_cap_pct * (self._ac_lower - self._ac_raise))
        # Current active cap (switches to deeper on dig_deeper service)
        self._ac_active_cap = self._ac_lower_cap
        self._ac_settle = self.get_parameter('actuator_settle_sec').value
        self._ac_step_val = self.get_parameter('actuator_step_val').value
        self._ac_step_interval = self.get_parameter('actuator_step_interval').value
        self._ac_initial_delay = self.get_parameter('actuator_initial_delay').value
        self._dump_angle = self.get_parameter('deposition_dump_angle').value
        self._stow_angle = self.get_parameter('deposition_stow_angle').value
        self._servo_time_ms = self.get_parameter('deposition_move_time_ms').value

        # Weight sensor feedback
        self._weight_gain_thresh = self.get_parameter('weight_gain_threshold').value
        self._weight_check_interval = self.get_parameter('weight_check_interval').value
        self._weight_stall_resume = self.get_parameter('weight_stall_resume_sec').value

        # Turn guard
        self._turn_threshold = self.get_parameter('turn_pause_threshold').value
        self._is_turning = False

        # Software E-stop
        self._sw_estop_enabled = self.get_parameter('software_estop_enabled').value

        # State
        self._state = 'IDLE'
        self._gate = 'CLOSED'
        self._active_timer = None
        self._ac_lower_timer = None
        self._current_ac_val = 0       # current actuator command being sent
        self._actuator_fully_down = False
        # Scenario-2 actuator-first state
        self._dig_start_time = 0.0          # set when /excavation/dig fires
        self._belt_engaged = False          # True once belt started during DIG
        self._belt_paused = False           # /excavation/belt_pause toggles this
        self._belt_pwm_before_pause = 0     # remember PWM so resume can restore

        # Weight sensor state
        self._current_weight = 0.0
        self._last_weight_time = 0.0
        self._weight_available = False
        self._weight_at_last_check = 0.0
        self._last_weight_check_time = 0.0
        self._ac_paused_for_weight = False
        self._ac_pause_time = 0.0

        # Abstract command/status publishers
        self._cmd_pub = self.create_publisher(
            ExcavationCommand,
            self.get_parameter('command_topic').value, 10
        )
        self._status_pub = self.create_publisher(
            ExcavationStatus,
            self.get_parameter('status_topic').value, 10
        )

        # Hardware publishers
        self._hw_heartbeat_timer = None
        self._target_motor_pwm = 0
        self._target_actuator_val = 0
        # Signal to mission controller: actuator at cap + weight not increasing → need nudge
        self._pub_needs_nudge = self.create_publisher(Bool, '/excavation/needs_nudge', 10)
        # Telemetry for mission controller's step-and-dig state machine
        self._pub_actuator_pct = self.create_publisher(Float32, '/excavation/actuator_pct', 10)
        self._pub_belt_pwm = self.create_publisher(Int32, '/excavation/belt_pwm', 10)

        if not self._simulate:
            self._motor_pub = self.create_publisher(Int32, '/excavation/motor', 10)
            self._actuator_pub = self.create_publisher(Int32, '/actuator/command', 10)
            self._servo_pub = self.create_publisher(StringMsg, '/deposition/tilt', 10)
            # Heartbeat at 10Hz keeps Pico watchdog alive (500ms timeout)
            self._hw_heartbeat_timer = self.create_timer(0.1, self._hw_heartbeat)
            # Weight sensor subscription
            weight_topic = self.get_parameter('weight_topic').value
            self.create_subscription(Float32, weight_topic, self._weight_cb, 10)

        # Monitor cmd_vel_safe for turning detection (power guard)
        self.create_subscription(Twist, '/cmd_vel_safe', self._cmd_vel_cb, 10)

        # Status heartbeat at 5 Hz
        self.create_timer(0.2, self._publish_status)

        # E-stop subscription
        self._estopped = False
        self.create_subscription(Bool, '/estop', self._estop_cb, 10)
        self.create_subscription(Bool, '/safety/estop', self._estop_cb, 10)

        # Services
        self.create_service(Trigger, '/excavation/dig', self._dig_cb)
        self.create_service(Trigger, '/excavation/dump', self._dump_cb)
        self.create_service(Trigger, '/excavation/stow', self._stow_cb)
        # Step-and-dig support services
        self.create_service(Trigger, '/excavation/belt_pause', self._belt_pause_cb)
        self.create_service(Trigger, '/excavation/belt_resume', self._belt_resume_cb)
        self.create_service(Trigger, '/excavation/dig_deeper', self._dig_deeper_cb)

        mode = 'SIMULATION' if self._simulate else 'HARDWARE'
        self.get_logger().info(f'Excavation Bridge initialized ({mode} mode)')
        if not self._simulate:
            self.get_logger().info(
                f'  Belt ramp: {self._dig_pwm_start}->{self._dig_pwm_target} '
                f'(+{self._dig_ramp_step}/{self._dig_ramp_interval}s)')
            self.get_logger().info(
                f'  Actuator: step={self._ac_step_val} every {self._ac_step_interval}s, '
                f'full range=[{self._ac_lower}, {self._ac_raise}], '
                f'initial delay={self._ac_initial_delay}s')

    # ------------------------------------------------------------------
    # Weight sensor callback
    # ------------------------------------------------------------------

    def _weight_cb(self, msg):
        self._current_weight = msg.data
        self._last_weight_time = time.time()
        self._weight_available = True

    def _has_weight_data(self):
        if not self._weight_available:
            return False
        return (time.time() - self._last_weight_time) < 10.0

    # ------------------------------------------------------------------
    # Turn guard — pause excavation when drivetrain is turning
    # ------------------------------------------------------------------

    def _cmd_vel_cb(self, msg):
        """Monitor cmd_vel_safe for angular velocity. Pause excavation during turns."""
        was_turning = self._is_turning
        self._is_turning = abs(msg.angular.z) > self._turn_threshold

        if self._is_turning and not was_turning and self._state == 'DIGGING':
            self.get_logger().info(
                f'TURNING detected (az={msg.angular.z:.2f}) — '
                f'pausing belt + actuator (power guard)')
            if not self._simulate:
                # Send motor 0 immediately (heartbeat will maintain 0 while turning)
                motor_msg = Int32()
                motor_msg.data = 0
                self._motor_pub.publish(motor_msg)
        elif not self._is_turning and was_turning and self._state == 'DIGGING':
            self.get_logger().info(
                f'Turn complete — resuming belt at PWM {self._current_dig_pwm}')
            if not self._simulate:
                self._publish_motor(self._current_dig_pwm)

    # ------------------------------------------------------------------
    # E-stop handler
    # ------------------------------------------------------------------

    def _estop_cb(self, msg):
        if not self._sw_estop_enabled:
            if msg.data:
                self.get_logger().warn(
                    'Software E-STOP received but IGNORED (disabled for competition)')
            return
        if msg.data and not self._estopped:
            self._estopped = True
            self.get_logger().error('E-STOP — killing all excavation motors')
            self._cancel_all_timers()
            if not self._simulate:
                self._publish_motor(0)
                self._publish_actuator(self._ac_raise)
                self._publish_servo(self._stow_angle)
            self._state = 'ESTOP'
            self._gate = 'CLOSED'
        elif not msg.data and self._estopped:
            self._estopped = False
            self._state = 'IDLE'
            self.get_logger().info('E-stop released — excavation bridge IDLE')

    def _cancel_all_timers(self):
        for timer in [self._active_timer, self._ramp_timer, self._ac_lower_timer]:
            if timer is not None:
                timer.cancel()
        self._active_timer = None
        self._ramp_timer = None
        self._ac_lower_timer = None

    # ------------------------------------------------------------------
    # Service handlers
    # ------------------------------------------------------------------

    def _dig_cb(self, _req, res):
        if self._estopped:
            res.success = False
            res.message = 'Cannot dig: E-STOP active'
            return res
        if self._state != 'IDLE':
            res.success = False
            res.message = f'Cannot dig: state is {self._state}'
            return res

        self.get_logger().info('DIG requested')
        self._send_command('DIG_START', self._dig_dur)

        if self._simulate:
            self._state = 'DIGGING'
            self._active_timer = self.create_timer(
                self._dig_dur, lambda: self._finish_action('IDLE', 'DIG_STOP')
            )
            res.message = f'[SIM] Digging for {self._dig_dur:.0f}s'
        else:
            # Hardware (Scenario 2 actuator-first): lower actuator FIRST,
            # belt engages when actuator passes 60% depth.
            self._state = 'DIGGING'
            self._actuator_fully_down = False
            self._current_ac_val = 0
            self._ac_paused_for_weight = False
            self._belt_engaged = False
            self._belt_paused = False
            self._belt_pwm_before_pause = 0
            self._current_dig_pwm = 0  # belt OFF until 60% depth
            # Reset active cap to default (in case last dig used DEEPER)
            self._ac_active_cap = self._ac_lower_cap
            self._dig_start_time = time.time()

            belt_engage_sec = self._ac_settle * self._ac_belt_engage_pct
            cap_reach_sec = self._ac_settle * self._ac_lower_cap_pct
            self.get_logger().info(
                f'DIG (actuator-first): belt engages at {self._ac_belt_engage_pct*100:.0f}%% depth '
                f'(~{belt_engage_sec:.1f}s), actuator caps at {self._ac_lower_cap_pct*100:.0f}%% '
                f'(~{cap_reach_sec:.1f}s)')
            self.get_logger().info(
                f'Belt plan: start {self._dig_pwm_start} → target {self._dig_pwm_target} '
                f'(+{self._dig_ramp_step}/{self._dig_ramp_interval}s)')

            # 1. Belt motor stays OFF until actuator passes 60% depth.
            self._publish_motor(0)

            # 2. Start gradual actuator lowering after initial delay.
            self._weight_at_last_check = self._current_weight
            self._last_weight_check_time = time.time()
            self._ac_lower_timer = self.create_timer(
                self._ac_initial_delay, self._start_actuator_lowering)

            res.message = (
                f'Actuator lowering in {self._ac_initial_delay}s; '
                f'belt engages at {self._ac_belt_engage_pct*100:.0f}%% depth')

        res.success = True
        return res

    def _dump_cb(self, _req, res):
        if self._estopped:
            res.success = False
            res.message = 'Cannot dump: E-STOP active'
            return res
        if self._state != 'IDLE':
            res.success = False
            res.message = f'Cannot dump: state is {self._state}'
            return res

        self.get_logger().info('DUMP requested')
        self._send_command('DUMP_OPEN')
        self._state = 'DUMPING'
        self._gate = 'OPEN'

        if self._simulate:
            self._active_timer = self.create_timer(
                self._dump_dur, self._finish_dump)
            res.message = f'[SIM] Dumping for {self._dump_dur:.0f}s'
        else:
            self._publish_servo(self._dump_angle)
            self.get_logger().info(
                f'Deposition servo tilting to {self._dump_angle:.0f}deg')
            res.message = f'Servo tilting to {self._dump_angle:.0f}deg'

        res.success = True
        return res

    def _stow_cb(self, _req, res):
        self.get_logger().info('STOW requested')
        self._send_command('STOW')
        self._cancel_all_timers()

        if self._simulate:
            self._state = 'STOWING'
            self._active_timer = self.create_timer(
                self._stow_dur, lambda: self._finish_action('IDLE', None))
            res.message = '[SIM] Stowing'
        else:
            self._state = 'STOWING'
            self._publish_motor(0)
            self.get_logger().info('Belt motor stopped')
            self._publish_actuator(self._ac_raise)
            self.get_logger().info(f'RAISING actuator to {self._ac_raise}')
            self._publish_servo(self._stow_angle)
            self.get_logger().info(f'Servo retracting to {self._stow_angle:.0f}deg')
            self._gate = 'CLOSED'
            self._active_timer = self.create_timer(
                self._ac_settle, self._hw_stow_complete)
            res.message = 'Stowing: motor off, actuator raising, servo retracting'

        res.success = True
        return res

    # ------------------------------------------------------------------
    # Hardware dig sequence — belt-first, gradual actuator lowering
    # ------------------------------------------------------------------

    def _start_actuator_lowering(self):
        """Called after initial delay. Begin gradual actuator lowering."""
        if self._ac_lower_timer:
            self._ac_lower_timer.cancel()
            self._ac_lower_timer = None

        if self._state != 'DIGGING':
            return

        self.get_logger().info('Starting gradual actuator lowering...')
        # Replace with periodic lowering timer
        self._ac_lower_timer = self.create_timer(
            self._ac_step_interval, self._actuator_step_tick)

    def _actuator_step_tick(self):
        """Lower actuator one step toward the active cap.

        In Scenario 2 actuator-first mode:
          - Belt-engage gate (60% depth): if reached, start belt at dig_pwm_start
            and kick off the PWM ramp toward dig_pwm_target.
          - Active cap (85% default; 92% after /excavation/dig_deeper): stop
            stepping and signal needs_nudge → mission controller takes over.
          - Pauses while turning (power guard) or while explicitly paused via
            /excavation/belt_pause.
        """
        if self._state != 'DIGGING':
            if self._ac_lower_timer:
                self._ac_lower_timer.cancel()
                self._ac_lower_timer = None
            return

        # Skip actuator lowering while turning (power guard)
        if self._is_turning:
            return

        # Skip while belt is explicitly paused (mission is repositioning)
        if self._belt_paused:
            return

        # Engage belt once actuator passes the belt-engage threshold.
        # Belt-engage is a one-shot: starts belt at dig_pwm_start and kicks
        # off the PWM ramp toward dig_pwm_target.
        if (not self._belt_engaged
                and self._get_actuator_depth_pct() >= self._ac_belt_engage_pct * 100.0):
            self._engage_belt()

        # Already at the active cap → done lowering
        if self._actuator_fully_down:
            if self._ac_lower_timer:
                self._ac_lower_timer.cancel()
                self._ac_lower_timer = None
            return

        # Lower one step toward the active cap.
        self._current_ac_val = max(
            self._ac_active_cap,
            self._current_ac_val + self._ac_step_val)
        self._publish_actuator(self._current_ac_val)
        self.get_logger().info(
            f'Actuator step: {self._current_ac_val} '
            f'(depth ≈ {self._get_actuator_depth_pct():.0f}%, '
            f'cap = {self._ac_active_cap})')

        # Reached the active cap?
        if self._current_ac_val <= self._ac_active_cap:
            self._actuator_fully_down = True
            cap_pct = (self._ac_raise - self._ac_active_cap) / float(
                self._ac_raise - self._ac_lower) * 100.0
            self.get_logger().info(
                f'Actuator at active cap ({self._ac_active_cap}, '
                f'~{cap_pct:.0f}% depth) — signaling mission controller')
            if self._ac_lower_timer:
                self._ac_lower_timer.cancel()
                self._ac_lower_timer = None
            # Signal mission controller: at cap, ready for step-and-dig
            nudge_msg = Bool()
            nudge_msg.data = True
            self._pub_needs_nudge.publish(nudge_msg)

    def _engage_belt(self):
        """One-shot: actuator passed 60% depth → start belt + begin ramp.

        Idempotent: only runs the first time we cross the gate during a dig.
        Does not run if belt is currently paused (mission is dodging).
        """
        if self._belt_engaged:
            return
        self._belt_engaged = True
        if self._belt_paused:
            self.get_logger().info(
                'Belt-engage gate reached but belt is PAUSED — '
                'will start on resume')
            return
        self._current_dig_pwm = self._dig_pwm_start
        self._publish_motor(self._current_dig_pwm)
        self.get_logger().info(
            f'Belt ENGAGED at PWM {self._dig_pwm_start} '
            f'(actuator at ~{self._ac_belt_engage_pct*100:.0f}% depth) — '
            f'ramping to {self._dig_pwm_target}')
        # Start PWM ramp timer if not already running
        if self._ramp_timer is None:
            self._ramp_timer = self.create_timer(
                self._dig_ramp_interval, self._ramp_dig_speed)

    def _ramp_dig_speed(self):
        """Increase dig speed by ramp_step until target reached."""
        if self._state != 'DIGGING':
            if self._ramp_timer:
                self._ramp_timer.cancel()
                self._ramp_timer = None
            return

        # Skip ramp while turning (power guard)
        if self._is_turning:
            return

        if self._current_dig_pwm >= self._dig_pwm_target:
            if self._ramp_timer:
                self._ramp_timer.cancel()
                self._ramp_timer = None
            return

        self._current_dig_pwm = min(
            self._dig_pwm_target,
            self._current_dig_pwm + self._dig_ramp_step)
        self._publish_motor(self._current_dig_pwm)
        self.get_logger().info(f'Dig ramp: PWM {self._current_dig_pwm}')

        if self._current_dig_pwm >= self._dig_pwm_target:
            self.get_logger().info(
                f'Dig ramp complete — holding at {self._dig_pwm_target}')
            if self._ramp_timer:
                self._ramp_timer.cancel()
                self._ramp_timer = None

    def _hw_stow_complete(self):
        """Called after actuator has settled in raised position."""
        if self._active_timer:
            self._active_timer.cancel()
            self._active_timer = None
        self._state = 'IDLE'
        # Reset dig-flow flags so next dig starts clean
        self._belt_engaged = False
        self._belt_paused = False
        self._ac_active_cap = self._ac_lower_cap
        self.get_logger().info('Stow complete — IDLE')

    # ------------------------------------------------------------------
    # Actuator depth estimation (no encoder — commanded value)
    # ------------------------------------------------------------------

    def _get_actuator_depth_pct(self):
        """Return current estimated actuator depth as a percentage [0, 100].

        0% = fully retracted (ac_raise = +100)
        100% = fully extended (ac_lower = -100)

        Uses the commanded value, which is a good proxy for the actual
        position because step rate (~7.5 units/s) matches the actuator's
        physical max speed (~7.7 units/s).
        """
        span = float(self._ac_raise - self._ac_lower)
        if span <= 0.0:
            return 0.0
        pct = (self._ac_raise - self._current_ac_val) / span * 100.0
        return max(0.0, min(100.0, pct))

    # ------------------------------------------------------------------
    # Step-and-dig support services
    # ------------------------------------------------------------------

    def _belt_pause_cb(self, _req, res):
        """Stop the belt and HOLD actuator position. Used by mission_controller
        before any turn (rock dodge) so we never turn with the belt running."""
        if self._simulate:
            self._belt_paused = True
            res.success = True
            res.message = '[SIM] belt paused (state held)'
            return res

        if self._state != 'DIGGING':
            res.success = False
            res.message = f'Cannot pause belt: state is {self._state}'
            return res

        # Remember current PWM so resume can restore it
        self._belt_pwm_before_pause = self._current_dig_pwm
        self._belt_paused = True
        self._publish_motor(0)
        self.get_logger().info(
            f'BELT PAUSED (was {self._belt_pwm_before_pause} PWM) — actuator held')
        # Cancel ramp timer (it'll restart in resume if needed)
        if self._ramp_timer is not None:
            self._ramp_timer.cancel()
            self._ramp_timer = None
        res.success = True
        res.message = f'Belt paused at PWM {self._belt_pwm_before_pause}'
        return res

    def _belt_resume_cb(self, _req, res):
        """Restart the belt at the last ramped PWM. No actuator change."""
        if self._simulate:
            self._belt_paused = False
            res.success = True
            res.message = '[SIM] belt resumed'
            return res

        if self._state != 'DIGGING':
            res.success = False
            res.message = f'Cannot resume belt: state is {self._state}'
            return res

        self._belt_paused = False
        resume_pwm = self._belt_pwm_before_pause
        if resume_pwm <= 0:
            # We never engaged before pause? Engage now if we're past the gate.
            if self._belt_engaged:
                resume_pwm = self._dig_pwm_target
            elif self._get_actuator_depth_pct() >= self._ac_belt_engage_pct * 100.0:
                self._engage_belt()
                return self._success(res, f'Belt engaged on resume')
            else:
                res.success = True
                res.message = 'Resume noop (actuator not yet at engage depth)'
                return res
        self._current_dig_pwm = resume_pwm
        self._publish_motor(self._current_dig_pwm)
        self.get_logger().info(f'BELT RESUMED at PWM {self._current_dig_pwm}')
        # Resume ramp if not at target yet
        if self._current_dig_pwm < self._dig_pwm_target and self._ramp_timer is None:
            self._ramp_timer = self.create_timer(
                self._dig_ramp_interval, self._ramp_dig_speed)
        res.success = True
        res.message = f'Belt resumed at PWM {self._current_dig_pwm}'
        return res

    def _dig_deeper_cb(self, _req, res):
        """Extend the actuator cap from 85% to 92% (one-shot per dig).

        After mission_controller's DWELL phase sees stalled weight, it tries
        pushing the actuator a bit deeper before resorting to ADVANCE.
        Resets _actuator_fully_down so stepping resumes toward the new cap.
        """
        if self._simulate:
            res.success = True
            res.message = '[SIM] dig_deeper accepted'
            return res

        if self._state != 'DIGGING':
            res.success = False
            res.message = f'Cannot dig deeper: state is {self._state}'
            return res

        if self._ac_active_cap <= self._ac_deeper_cap:
            res.success = True
            res.message = 'Already at deeper cap'
            return res

        self._ac_active_cap = self._ac_deeper_cap
        self._actuator_fully_down = False
        # Make sure stepping is running
        if self._ac_lower_timer is None:
            self._ac_lower_timer = self.create_timer(
                self._ac_step_interval, self._actuator_step_tick)
        self.get_logger().info(
            f'DIG DEEPER: cap extended to {self._ac_active_cap} '
            f'(~{self._ac_deeper_cap_pct*100:.0f}% depth)')
        res.success = True
        res.message = f'Cap extended to {self._ac_active_cap}'
        return res

    def _success(self, res, msg):
        res.success = True
        res.message = msg
        return res

    # ------------------------------------------------------------------
    # Hardware publish helpers
    # ------------------------------------------------------------------

    def _hw_heartbeat(self):
        """Republish motor/actuator commands at 10Hz for Pico watchdog."""
        if self._estopped:
            if self._target_motor_pwm != 0:
                self._target_motor_pwm = 0
                msg = Int32()
                msg.data = 0
                self._motor_pub.publish(msg)
            return

        # Turn guard OR explicit belt-pause: send 0 motor, hold actuator
        if self._state == 'DIGGING' and (self._is_turning or self._belt_paused):
            msg = Int32()
            msg.data = 0
            self._motor_pub.publish(msg)
            return

        if self._target_motor_pwm != 0:
            msg = Int32()
            msg.data = self._target_motor_pwm
            self._motor_pub.publish(msg)

        if self._target_actuator_val != 0:
            msg = Int32()
            msg.data = self._target_actuator_val
            self._actuator_pub.publish(msg)

    def _publish_motor(self, pwm):
        """Set and publish excavation belt motor PWM."""
        self._target_motor_pwm = -int(pwm)
        msg = Int32()
        msg.data = self._target_motor_pwm
        self._motor_pub.publish(msg)

    def _publish_actuator(self, val):
        """Publish actuator position (-100 to +100)."""
        self._target_actuator_val = int(val)
        msg = Int32()
        msg.data = self._target_actuator_val
        self._actuator_pub.publish(msg)

    def _publish_servo(self, angle):
        """Publish deposition servo tilt command."""
        msg = StringMsg()
        msg.data = f'{angle:.1f},{self._servo_time_ms}'
        self._servo_pub.publish(msg)

    # ------------------------------------------------------------------
    # Abstract command/status
    # ------------------------------------------------------------------

    def _send_command(self, cmd, param=0.0):
        msg = ExcavationCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.command = cmd
        msg.parameter = float(param)
        self._cmd_pub.publish(msg)

    def _finish_action(self, new_state, stop_cmd):
        if self._active_timer:
            self._active_timer.cancel()
            self._active_timer = None
        if stop_cmd:
            self._send_command(stop_cmd)
        self._state = new_state
        self.get_logger().info(f'Action complete -> {new_state}')

    def _finish_dump(self):
        if self._active_timer:
            self._active_timer.cancel()
            self._active_timer = None
        self._send_command('DUMP_CLOSE')
        self._gate = 'CLOSED'
        self._state = 'IDLE'
        self.get_logger().info('Dump complete -> IDLE')

    def _publish_status(self):
        msg = ExcavationStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = self._state
        msg.motor_current = 0.0
        msg.gate_position = self._gate
        msg.error_code = 'NONE'
        self._status_pub.publish(msg)

        # Telemetry for step-and-dig: actuator depth pct + belt PWM
        depth_msg = Float32()
        depth_msg.data = float(self._get_actuator_depth_pct())
        self._pub_actuator_pct.publish(depth_msg)

        belt_msg = Int32()
        # Report POSITIVE PWM (target_motor_pwm is negated on the wire);
        # report 0 while turning/paused so mission sees the actual state.
        if (self._state == 'DIGGING'
                and (self._is_turning or self._belt_paused)):
            belt_msg.data = 0
        else:
            belt_msg.data = int(self._current_dig_pwm)
        self._pub_belt_pwm.publish(belt_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ExcavationBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
