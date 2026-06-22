#!/usr/bin/env python3
"""
Mission Controller for Lunabotics 2026 — UCF Arena
=====================================================
Fully autonomous state-machine controller for the UCF competition.

STARTING POSE CALIBRATION:
  Point-LIO's map frame starts at wherever the robot is placed.
  The operator tells the controller where that is:
    mission> setpose 3.0 1.0 90
  The controller publishes arena→map TF and transforms all waypoints.

SMART EXCAVATION:
  During digging, monitors the deposition box weight sensor. If weight
  gain stalls (sand level dropped at current spot), the robot nudges
  forward to fresh sand while the belt keeps running. Falls back to
  pure time-based excavation if no weight sensor data is available.

Mission flow:
  IDLE → WAIT_LOCALIZATION → ALIGN → MAP_AHEAD → [NAVIGATE_TO_EXCAVATION →
  EXCAVATE → NAVIGATE_TO_BERM → DEPOSIT] × N → DONE
"""

import math
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, Bool, Float32
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster, Buffer, TransformListener
from tf2_ros import TransformException
try:
    from luna_msgs.msg import HazardSummary
except ImportError:
    HazardSummary = None


class MissionController(Node):

    def __init__(self):
        super().__init__('mission_controller')

        # --- Parameters ---
        self.declare_parameters(namespace='', parameters=[
            ('num_cycles', 10),
            ('mission_timeout_sec', 1740.0),
            ('return_reserve_sec', 120.0),
            ('dig_duration_sec', 120.0),
            ('dump_wait_sec', 10.0),         # legacy: fallback when no weight sensor
            ('dump_max_wait_sec', 20.0),     # max time to wait for material to fall
            ('dump_baseline_threshold_kg', 1.0),  # stow when weight - baseline <= this
            ('nav_timeout_sec', 180.0),
            ('min_localization_confidence', 0.3),
            ('localization_wait_timeout', 30.0),
            # Arena waypoints
            ('waypoint_excavation_x', 2.0),
            ('waypoint_excavation_y', 0.0),
            ('waypoint_excavation_yaw', 180.0),
            # Cycle 2+ excavation arrival (faces EAST so dig + reverse-to-berm
            # don't need 180° flips)
            ('waypoint_excavation_return_x', 2.0),
            ('waypoint_excavation_return_y', 0.0),
            ('waypoint_excavation_return_yaw', 0.0),
            ('waypoint_berm_x', -2.0),
            ('waypoint_berm_y', -1.3),
            ('waypoint_berm_yaw', 0.0),
            # Starting pose
            ('start_x', 3.0),
            ('start_y', 0.0),
            ('start_yaw_deg', 180.0),
            # Smart excavation — weight-aware digging with forward nudge
            ('weight_topic', '/deposition/weight'),
            ('weight_check_interval', 15.0),   # seconds between weight checks
            ('min_weight_gain_kg', 0.05),       # minimum gain to consider "making progress"
            ('nudge_speed', 0.05),              # m/s forward during nudge (slow crawl)
            ('nudge_duration', 4.0),            # seconds per nudge (~20cm at 0.05 m/s)
            ('max_nudges', 6),                  # max nudges per excavation (total ~1.2m)
            ('max_load_kg', 10.0),              # stop digging if weight GAIN exceeds this (kg)
            ('weight_timeout', 10.0),           # if no weight data for this long, ignore sensor
            # Alignment — minimal rotation to face excavation waypoint
            ('align_speed', 0.4),               # rad/s rotation speed
            ('align_tolerance_deg', 15.0),       # skip rotation if within this tolerance
            # Mapping dwell — after facing forward, pause to let LiDAR map all
            # obstacles ahead. Perception at 2Hz + costmap at 2Hz = 4s gives
            # ~8 perception frames and ~8 costmap updates for a solid map.
            ('map_dwell_sec', 4.0),              # seconds to dwell after aligning
            # Berm approach — reverse into berm with beam-pair detection
            #   Cycle 1: pass front beams, stop near rear beams (deep deposit)
            #   Cycle 2+: stop at front beams (rear aligned with berm edge)
            # Reverse_duration_* are MAX timeouts (safety fallback if beams not seen).
            ('berm_reverse_speed', -0.08),
            ('berm_reverse_duration', 12.0),         # max time cycle 2+ (~96cm fallback)
            ('berm_reverse_duration_first', 22.0),   # max time cycle 1 (~176cm fallback)
            # Berm beam-pair detection — closed-loop reverse stop
            ('lidar_to_rear_offset', 1.10),          # m, LiDAR forward of rear bumper
            ('berm_rear_cone_half_width', 0.55),     # m, lateral half-width of rear cone
            ('berm_rear_cone_max_lookback', 2.5),    # m, how far behind rear to look
            ('berm_rear_cone_min_z', -0.40),         # m, drop ground (LiDAR ~0.50m above)
            ('berm_rear_cone_max_z', 0.60),          # m, drop overhead noise
            ('berm_rear_min_points', 3),             # min cluster size to trust as obstacle
            ('berm_pair_detect_threshold', 1.0),     # m, distance to trigger "pair approaching"
            ('berm_pair_release_threshold', 0.50),   # m, distance to declare "pair passed"
            ('berm_cycle1_stop_distance', 0.30),     # m, stop cycle 1 at this dist on 2nd pair
            ('berm_cycle2plus_stop_distance', 0.05), # m, stop cycle 2+ at this dist on 1st pair
            ('berm_emergency_stop_distance', 0.05),  # m, hard collision stop (any cycle, any pair)
            ('berm_cloud_stale_timeout', 1.0),       # s, ignore cloud if older than this
            # Pre-reverse lateral alignment check
            ('berm_lateral_check_half_width', 0.80), # m, wider cone for centroid detection
            ('berm_lateral_tolerance', 0.05),        # m, abort deposit if |offset| > this
            # Beam-corrected re-approach: if lateral check is 'off' but the
            # offset is within this bound, send a small Nav2 fix-up goal at
            # the beam-computed corrected position (per cycle, 1 attempt max).
            # Use beams as fiducials so a wrong setpose doesn't doom deposits.
            ('berm_max_beam_correction', 0.5),       # m, max |offset| we'll try to fix via Nav2
            # Forward exit after stow (clear inflation zone before Nav2)
            ('berm_exit_forward_speed', 0.08),       # m/s (positive = forward)
            ('berm_exit_forward_duration', 5.0),     # s (~40cm at 0.08 m/s)
            # Software E-stop — disable for competition (hands-free, can't clear it)
            # Physical E-STOP button on the robot handles real emergencies.
            ('software_estop_enabled', True),
            # Arena layout — A (default) or B (mirror, berm Y flipped)
            # See: project_arena_mirror_may13 memory, scenario2 docs
            ('arena_layout', 'A'),
            # Dig at spawn — skip cycle-1 nav-to-excavation and dig in place.
            # Starting zone is a subset of the excavation zone (UCF spec), so
            # digging at spawn is legal. ALIGN + MAP_AHEAD still run so the
            # robot faces the right direction and the costmap is built before
            # the nav-to-berm leg.
            ('dig_at_start_zone', False),
            # Step-and-dig (Scenario 2 final flow)
            ('dig_dwell_sec_min', 6.0),
            ('dig_dwell_sec_max', 10.0),
            ('dig_advance_distance_m', 0.4),
            ('dig_advance_speed', 0.05),
            ('dig_weight_window_sec', 2.0),
            ('dig_weight_stall_threshold_kg', 0.2),
            ('dig_zone_x_min', 0.5),
            ('dig_zone_x_max', 3.7),
            ('dig_deeper_settle_sec', 2.5),
            ('dig_descend_settle_sec', 10.0),    # wait for physical actuator after needs_nudge
            ('dig_obstacle_block_distance', 0.85),
            ('dodge_belt_settle_sec', 1.5),
            ('dodge_turn_rad', 0.52),
            ('dodge_turn_speed', 0.5),
            ('dodge_forward_m', 0.6),
            ('dodge_forward_speed', 0.08),
            ('dodge_max_attempts', 2),
            # Cycle 2+ open-loop reverse-to-berm
            ('reverse_to_berm_speed', -0.10),
            ('reverse_to_berm_target_x', -1.0),
            ('reverse_to_berm_max_sec', 50.0),
            ('reverse_to_berm_rear_clearance', 0.40),
            ('reverse_to_berm_min_pts', 5),
        ])

        self._num_cycles = self.get_parameter('num_cycles').value
        self._mission_timeout = self.get_parameter('mission_timeout_sec').value
        self._return_reserve = self.get_parameter('return_reserve_sec').value
        self._dig_duration = self.get_parameter('dig_duration_sec').value
        self._dump_wait = self.get_parameter('dump_wait_sec').value
        self._dump_max_wait = self.get_parameter('dump_max_wait_sec').value
        self._dump_baseline_threshold = self.get_parameter('dump_baseline_threshold_kg').value
        self._nav_timeout = self.get_parameter('nav_timeout_sec').value
        self._min_loc_conf = self.get_parameter('min_localization_confidence').value
        self._loc_wait_timeout = self.get_parameter('localization_wait_timeout').value

        # Smart excavation params
        self._weight_check_interval = self.get_parameter('weight_check_interval').value
        self._min_weight_gain = self.get_parameter('min_weight_gain_kg').value
        self._nudge_speed = self.get_parameter('nudge_speed').value
        self._nudge_duration = self.get_parameter('nudge_duration').value
        self._max_nudges = self.get_parameter('max_nudges').value
        self._max_load = self.get_parameter('max_load_kg').value
        self._weight_timeout = self.get_parameter('weight_timeout').value

        # Alignment params
        self._align_speed = self.get_parameter('align_speed').value
        self._align_tolerance = math.radians(self.get_parameter('align_tolerance_deg').value)
        self._map_dwell = self.get_parameter('map_dwell_sec').value

        # Berm reverse alignment params
        self._berm_reverse_speed = self.get_parameter('berm_reverse_speed').value
        self._berm_reverse_duration = self.get_parameter('berm_reverse_duration').value
        self._berm_reverse_duration_first = self.get_parameter('berm_reverse_duration_first').value

        # Berm beam-pair detection params
        self._lidar_to_rear_offset = self.get_parameter('lidar_to_rear_offset').value
        self._rear_cone_half_width = self.get_parameter('berm_rear_cone_half_width').value
        self._rear_cone_max_lookback = self.get_parameter('berm_rear_cone_max_lookback').value
        self._rear_cone_min_z = self.get_parameter('berm_rear_cone_min_z').value
        self._rear_cone_max_z = self.get_parameter('berm_rear_cone_max_z').value
        self._rear_min_points = self.get_parameter('berm_rear_min_points').value
        self._pair_detect_thresh = self.get_parameter('berm_pair_detect_threshold').value
        self._pair_release_thresh = self.get_parameter('berm_pair_release_threshold').value
        self._cycle1_stop_dist = self.get_parameter('berm_cycle1_stop_distance').value
        self._cycle2plus_stop_dist = self.get_parameter('berm_cycle2plus_stop_distance').value
        self._emergency_stop_dist = self.get_parameter('berm_emergency_stop_distance').value
        self._cloud_stale_timeout = self.get_parameter('berm_cloud_stale_timeout').value
        self._lateral_check_half_width = self.get_parameter('berm_lateral_check_half_width').value
        self._lateral_tolerance = self.get_parameter('berm_lateral_tolerance').value
        self._max_beam_correction = self.get_parameter('berm_max_beam_correction').value
        self._beam_correction_attempted = False  # reset per cycle in _begin_cycle
        self._exit_forward_speed = self.get_parameter('berm_exit_forward_speed').value
        self._exit_forward_duration = self.get_parameter('berm_exit_forward_duration').value

        # Software E-stop
        self._sw_estop_enabled = self.get_parameter('software_estop_enabled').value

        # Cycle-1 dig-in-place (skip nav-to-excavation on first cycle only)
        self._dig_at_start_zone = self.get_parameter('dig_at_start_zone').value

        # Step-and-dig params
        self._dig_dwell_min = self.get_parameter('dig_dwell_sec_min').value
        self._dig_dwell_max = self.get_parameter('dig_dwell_sec_max').value
        self._dig_advance_dist = self.get_parameter('dig_advance_distance_m').value
        self._dig_advance_speed = self.get_parameter('dig_advance_speed').value
        self._dig_weight_window = self.get_parameter('dig_weight_window_sec').value
        self._dig_weight_stall = self.get_parameter('dig_weight_stall_threshold_kg').value
        self._dig_zone_x_min = self.get_parameter('dig_zone_x_min').value
        self._dig_zone_x_max = self.get_parameter('dig_zone_x_max').value
        self._dig_deeper_settle = self.get_parameter('dig_deeper_settle_sec').value
        self._dig_descend_settle = self.get_parameter('dig_descend_settle_sec').value
        self._dig_obstacle_block = self.get_parameter('dig_obstacle_block_distance').value
        self._dodge_belt_settle = self.get_parameter('dodge_belt_settle_sec').value
        self._dodge_turn_rad = self.get_parameter('dodge_turn_rad').value
        self._dodge_turn_speed = self.get_parameter('dodge_turn_speed').value
        self._dodge_forward_m = self.get_parameter('dodge_forward_m').value
        self._dodge_forward_speed = self.get_parameter('dodge_forward_speed').value
        self._dodge_max_attempts = self.get_parameter('dodge_max_attempts').value
        # Cycle 2+ open-loop reverse-to-berm
        self._rev_berm_speed = self.get_parameter('reverse_to_berm_speed').value
        self._rev_berm_target_x = self.get_parameter('reverse_to_berm_target_x').value
        self._rev_berm_max_sec = self.get_parameter('reverse_to_berm_max_sec').value
        self._rev_berm_rear_clr = self.get_parameter('reverse_to_berm_rear_clearance').value
        self._rev_berm_min_pts = self.get_parameter('reverse_to_berm_min_pts').value
        # Set when an open-loop reverse-to-berm is in progress
        self._rev_berm_timer = None
        self._rev_berm_start = 0.0
        self._rev_berm_obs_pause_until = 0.0
        self._rev_berm_last_log = 0.0

        # Arena-frame waypoints — populated by _apply_arena_layout()
        self._arena_waypoints = {}
        self._arena_layout = 'A'
        self._apply_arena_layout()

        # Starting pose
        self._start_x = self.get_parameter('start_x').value
        self._start_y = self.get_parameter('start_y').value
        self._start_yaw = math.radians(self.get_parameter('start_yaw_deg').value)

        # --- State ---
        self._state = 'IDLE'
        self._cycle = 0
        self._mission_start_time = None
        self._goal_handle = None
        self._estop_active = False
        self._loc_confidence = 0.0
        self._loc_quality = 'unknown'
        self._current_zone = 'unknown'
        self._loc_wait_start = None
        self._dig_timer = None
        self._nav_retry_count = 0
        self._nav_timeout_timer = None
        self._nav_done_callback = None

        # Weight sensor state
        self._current_weight = 0.0
        self._last_weight_time = 0.0
        self._weight_available = False

        # Excavation monitoring state (legacy nudge fields, unused but kept
        # so any operator GUI introspecting parameters still works)
        self._excavation_start = None
        self._last_check_weight = 0.0
        self._last_check_time = 0.0
        self._nudge_count = 0
        self._excavation_timer = None
        self._nudge_timer = None
        self._is_nudging = False

        # Step-and-dig state machine
        self._excavation_phase = None       # 'DESCEND' | 'SETTLE' | 'DWELL' | 'ADVANCE'
                                            # | 'DEEPER' | 'DODGE' | 'RETRACT'
        self._phase_start = 0.0             # time the current phase began
        self._dwell_count = 0               # dwells completed in this excavation
        self._advance_start_pose = None     # (ax, ay) when ADVANCE began
        self._dig_deeper_used = False       # one-shot per excavation
        self._dig_descend_settle_start = 0.0
        self._dig_actuator_pct = 0.0        # latest /excavation/actuator_pct value
        self._dig_belt_pwm = 0              # latest /excavation/belt_pwm value
        # Rolling weight history: list[(time, weight)]
        self._weight_history = deque(maxlen=200)
        # Hazard summary state (perception)
        self._hazard_path_blocked = False
        self._hazard_min_distance = -1.0
        self._hazard_last_time = 0.0
        # Dodge sub-state machine
        self._dodge_step = None             # 'STOP' | 'BELT_OFF' | 'TURN' | 'FORWARD'
                                            # | 'TURN_BACK' | 'BELT_ON'
        self._dodge_step_start = 0.0
        self._dodge_attempt = 0
        self._dodge_direction = 1.0         # +1 = left/CCW, -1 = right/CW (alternates)

        # Berm reverse beam-pair detection state
        self._latest_cloud_msg = None
        self._latest_cloud_recv_time = 0.0
        self._reverse_pairs_passed = 0
        self._reverse_obs_state = 'waiting'   # 'waiting' | 'observing'
        self._reverse_min_in_pair = float('inf')
        self._reverse_last_log_time = 0.0
        self._berm_reverse_timer = None
        self._current_berm_reverse_duration = 0.0
        # Berm forward-exit state (drives out of beam inflation zone post-dump)
        self._exit_timer = None
        self._exit_start_time = 0.0
        # Dump-completion monitor (weight-based)
        self._dump_monitor_timer = None
        self._dump_start_time = 0.0
        self._dump_start_weight = 0.0
        self._dump_last_log_time = 0.0
        self._baseline_weight = 0.0  # set in _begin_excavation, reused in dump monitor

        # --- TF broadcaster + listener ---
        self._tf_broadcaster = StaticTransformBroadcaster(self)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # --- Callback group ---
        self._cb_group = ReentrantCallbackGroup()

        # --- Nav2 action client ---
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self._cb_group
        )

        # --- cmd_vel publisher for excavation nudges ---
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Excavation service clients ---
        self._dig_client = self.create_client(
            Trigger, '/excavation/dig', callback_group=self._cb_group
        )
        self._dump_client = self.create_client(
            Trigger, '/excavation/dump', callback_group=self._cb_group
        )
        self._stow_client = self.create_client(
            Trigger, '/excavation/stow', callback_group=self._cb_group
        )
        # Step-and-dig support
        self._belt_pause_client = self.create_client(
            Trigger, '/excavation/belt_pause', callback_group=self._cb_group
        )
        self._belt_resume_client = self.create_client(
            Trigger, '/excavation/belt_resume', callback_group=self._cb_group
        )
        self._dig_deeper_client = self.create_client(
            Trigger, '/excavation/dig_deeper', callback_group=self._cb_group
        )

        # --- Publishers ---
        self._status_pub = self.create_publisher(String, '/mission/status', 10)
        self._state_pub = self.create_publisher(String, '/mission/state', 10)
        # Defensive e-stop clearer — published once at mission start so any
        # latched True from safety_monitor / system_watchdog / stale teleop
        # cannot block /cmd_vel_safe at the relay.
        self._estop_clear_pub = self.create_publisher(Bool, '/estop', 10)
        self._safety_estop_clear_pub = self.create_publisher(Bool, '/safety/estop', 10)

        # --- Subscribers ---
        self.create_subscription(Bool, '/estop', self._estop_cb, 10)
        self.create_subscription(Bool, '/safety/estop', self._estop_cb, 10)
        self.create_subscription(
            Float32, '/perception/localization_confidence',
            self._loc_conf_cb, 10
        )
        self.create_subscription(
            String, '/perception/localization_quality',
            self._loc_qual_cb, 10
        )
        self.create_subscription(
            String, '/perception/current_zone',
            self._zone_cb, 10
        )
        # Weight sensor
        weight_topic = self.get_parameter('weight_topic').value
        self.create_subscription(Float32, weight_topic, self._weight_cb, 10)
        # Encoder stall detection (from pico_bridge)
        self._is_stalled = False
        self.create_subscription(Bool, '/pico/stalled', self._stall_cb, 10)
        # Excavation bridge signals: actuator at cap, needs nudge
        self._actuator_at_cap = False
        self._nudge_direction = 1.0  # 1.0 = forward, -1.0 = backward (alternates)
        self.create_subscription(Bool, '/excavation/needs_nudge', self._needs_nudge_cb, 10)
        # LiDAR cloud — used during berm reverse for beam-pair detection.
        # Callback just stashes the latest message; expensive parsing happens
        # only inside _berm_reverse_tick (when state == NAVIGATE_TO_BERM/DEPOSIT).
        self.create_subscription(
            PointCloud2, '/unilidar/cloud', self._lidar_cb, 10
        )

        # Step-and-dig telemetry from excavation_bridge
        self.create_subscription(
            Float32, '/excavation/actuator_pct',
            self._actuator_pct_cb, 10
        )
        from std_msgs.msg import Int32 as _Int32
        self.create_subscription(
            _Int32, '/excavation/belt_pwm',
            self._belt_pwm_cb, 10
        )
        self.create_subscription(
            Float32, '/deposition/weight',
            self._weight_history_cb, 10
        )
        # Hazard summary from perception (path_blocked_ahead is the key field)
        if HazardSummary is not None:
            self.create_subscription(
                HazardSummary, '/perception/hazard_summary',
                self._hazard_cb, 10
            )

        # --- Services ---
        self.create_service(Trigger, '/mission/start', self._start_cb)
        self.create_service(Trigger, '/mission/stop', self._stop_cb)
        self.create_service(Trigger, '/mission/reset', self._reset_cb)

        # --- State publish timer ---
        self.create_timer(0.5, self._publish_state)

        self.get_logger().info(
            f'Mission Controller ready — {self._mission_timeout:.0f}s timeout, '
            f'{self._dig_duration:.0f}s dig, weight-aware nudge enabled'
        )
        self.get_logger().info(
            f'Start pose: ({self._start_x:.1f}, {self._start_y:.1f}, '
            f'{math.degrees(self._start_yaw):.0f}°)')

    # ------------------------------------------------------------------
    # Arena layout (mirror handling)
    # ------------------------------------------------------------------

    def _apply_arena_layout(self):
        """Read arena_layout param and (re)populate self._arena_waypoints.

        Layout 'A' = default (matches mission_params.yaml as authored).
        Layout 'B' = mirror pit. Berm Y is flipped to the opposite side of
        the construction zone. Operator's start_y still uses the same +X
        convention (start zone at +X, construction at -X) — only the berm
        Y-side differs between the two physical pits.
        """
        layout_raw = self.get_parameter('arena_layout').value
        layout = (layout_raw or 'A').strip().upper()
        if layout not in ('A', 'B'):
            self.get_logger().warn(
                f'Unknown arena_layout "{layout_raw}" — defaulting to A'
            )
            layout = 'A'
        self._arena_layout = layout

        base_berm_y = self.get_parameter('waypoint_berm_y').value
        berm_y = -base_berm_y if layout == 'B' else base_berm_y

        self._arena_waypoints = {
            'excavation': (
                self.get_parameter('waypoint_excavation_x').value,
                self.get_parameter('waypoint_excavation_y').value,
                self.get_parameter('waypoint_excavation_yaw').value,
            ),
            'excavation_return': (
                self.get_parameter('waypoint_excavation_return_x').value,
                self.get_parameter('waypoint_excavation_return_y').value,
                self.get_parameter('waypoint_excavation_return_yaw').value,
            ),
            'berm': (
                self.get_parameter('waypoint_berm_x').value,
                berm_y,
                self.get_parameter('waypoint_berm_yaw').value,
            ),
        }

    # ------------------------------------------------------------------
    # Coordinate transform: arena ↔ Point-LIO map
    # ------------------------------------------------------------------

    def _arena_to_map(self, ax, ay, ayaw_deg):
        dx = ax - self._start_x
        dy = ay - self._start_y
        cos_a = math.cos(-self._start_yaw)
        sin_a = math.sin(-self._start_yaw)
        mx = dx * cos_a - dy * sin_a
        my = dx * sin_a + dy * cos_a
        myaw_deg = ayaw_deg - math.degrees(self._start_yaw)
        return mx, my, myaw_deg

    def _publish_arena_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'arena'
        t.child_frame_id = 'map'
        t.transform.translation.x = self._start_x
        t.transform.translation.y = self._start_y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self._start_yaw / 2.0)
        t.transform.rotation.w = math.cos(self._start_yaw / 2.0)
        self._tf_broadcaster.sendTransform(t)
        self._log(f'Published arena→map TF (start: '
                  f'{self._start_x:.1f}, {self._start_y:.1f}, '
                  f'{math.degrees(self._start_yaw):.0f}°)')

    # ------------------------------------------------------------------
    # Pose helper
    # ------------------------------------------------------------------

    def _make_pose(self, waypoint_name):
        arena_x, arena_y, arena_yaw = self._arena_waypoints[waypoint_name]
        mx, my, myaw_deg = self._arena_to_map(arena_x, arena_y, arena_yaw)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        yaw = math.radians(myaw_deg)
        pose.pose.position.x = float(mx)
        pose.pose.position.y = float(my)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    # ------------------------------------------------------------------
    # Safety / sensor callbacks
    # ------------------------------------------------------------------

    def _estop_cb(self, msg):
        if not self._sw_estop_enabled:
            if msg.data:
                self.get_logger().warn(
                    'Software E-STOP received but IGNORED (disabled for competition)')
            return
        if msg.data and not self._estop_active:
            self._estop_active = True
            self.get_logger().error('E-STOP — mission halted')
            self._cancel_nav()
            self._stop_excavation()
            self._set_state('ESTOP')
        elif not msg.data and self._estop_active:
            self._estop_active = False
            self.get_logger().info('E-stop released')

    def _stall_cb(self, msg):
        self._is_stalled = msg.data

    def _needs_nudge_cb(self, msg):
        self._actuator_at_cap = msg.data

    def _loc_conf_cb(self, msg):
        self._loc_confidence = msg.data

    def _loc_qual_cb(self, msg):
        self._loc_quality = msg.data

    def _zone_cb(self, msg):
        self._current_zone = msg.data

    def _weight_cb(self, msg):
        self._current_weight = msg.data
        self._last_weight_time = time.time()
        self._weight_available = True

    def _lidar_cb(self, msg):
        self._latest_cloud_msg = msg
        self._latest_cloud_recv_time = time.time()

    def _actuator_pct_cb(self, msg):
        self._dig_actuator_pct = float(msg.data)

    def _belt_pwm_cb(self, msg):
        self._dig_belt_pwm = int(msg.data)

    def _weight_history_cb(self, msg):
        """Stash (time, weight) for rolling weight-rate calc.

        Keep ~10s of history at the expected ~20 Hz publish rate.
        deque maxlen=200 = ~10s buffer.
        """
        self._weight_history.append((time.time(), float(msg.data)))

    def _hazard_cb(self, msg):
        self._hazard_path_blocked = bool(msg.path_blocked_ahead)
        self._hazard_min_distance = float(msg.min_distance_to_obstacle)
        self._hazard_last_time = time.time()

    def _weight_gain_in_window(self, window_sec):
        """Return weight change over the last `window_sec` seconds.

        Uses oldest sample within the window vs latest sample. Returns 0.0 if
        we don't have enough samples spanning the window.
        """
        if len(self._weight_history) < 2:
            return 0.0
        now = time.time()
        cutoff = now - window_sec
        # Find oldest sample on or after cutoff
        oldest = None
        for t, w in self._weight_history:
            if t >= cutoff:
                oldest = (t, w)
                break
        if oldest is None:
            return 0.0
        _, latest_w = self._weight_history[-1]
        # Only meaningful if oldest is at least 80% of the window old
        actual_window = self._weight_history[-1][0] - oldest[0]
        if actual_window < 0.8 * window_sec:
            return 0.0
        return latest_w - oldest[1]

    def _path_blocked_ahead(self):
        """Whether a forward-corridor obstacle is close enough to halt advance.

        Uses min_distance_to_obstacle (XY distance from base_link origin) as
        a finer trigger than the binary path_blocked_ahead. If hazard data is
        stale (>1s), don't block (don't freeze on missing perception).
        """
        if HazardSummary is None:
            return False
        if (time.time() - self._hazard_last_time) > 1.5:
            return False
        if self._hazard_path_blocked and 0.0 < self._hazard_min_distance <= self._dig_obstacle_block:
            return True
        return False

    def _get_robot_arena_pose(self):
        """Look up base_footprint in map frame, convert to arena coords.

        Returns (ax, ay, ayaw_rad) or None if TF lookup fails.
        """
        try:
            t = self._tf_buffer.lookup_transform(
                'map', 'base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2)
            )
        except (TransformException, Exception):
            return None
        mx = t.transform.translation.x
        my = t.transform.translation.y
        # Inverse of _arena_to_map: rotate by +start_yaw then add start offset
        cos_a = math.cos(self._start_yaw)
        sin_a = math.sin(self._start_yaw)
        ax = mx * cos_a - my * sin_a + self._start_x
        ay = mx * sin_a + my * cos_a + self._start_y
        # Yaw — from quaternion, then add start_yaw to get arena yaw
        qz = t.transform.rotation.z
        qw = t.transform.rotation.w
        map_yaw = 2.0 * math.atan2(qz, qw)
        arena_yaw = map_yaw + self._start_yaw
        return ax, ay, arena_yaw

    # ------------------------------------------------------------------
    # Berm rear-cone distance (used during reverse for beam-pair detection)
    # ------------------------------------------------------------------

    def _parse_latest_xyz(self):
        """Return Nx3 numpy array of finite (x,y,z) points from latest cloud,
        or None if no fresh data / parse failure.
        Cloud is in unilidar_lidar frame (LiDAR at front of robot)."""
        msg = self._latest_cloud_msg
        if msg is None:
            return None
        if (time.time() - self._latest_cloud_recv_time) > self._cloud_stale_timeout:
            return None

        n_points = msg.width * msg.height
        if n_points == 0:
            return None

        field_map = {f.name: f.offset for f in msg.fields}
        if 'x' not in field_map or 'y' not in field_map or 'z' not in field_map:
            return None

        point_step = msg.point_step
        if (field_map.get('x') == 0 and field_map.get('y') == 4
                and field_map.get('z') == 8 and point_step == 12):
            xyz = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 3)
        else:
            try:
                raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n_points, point_step)
                xyz = np.empty((n_points, 3), dtype=np.float32)
                for i, axis in enumerate(('x', 'y', 'z')):
                    offset = field_map[axis]
                    xyz[:, i] = raw[:, offset:offset + 4].view(np.float32).flatten()
            except Exception:
                return None

        valid = np.isfinite(xyz).all(axis=1)
        xyz = xyz[valid]
        return xyz if xyz.shape[0] > 0 else None

    def _compute_rear_cone_dist(self):
        """Min distance (5th percentile) from rear bumper to obstacle in rear cone.
        Returns (dist_m, n_points) or None if no data / no obstacles in cone."""
        xyz = self._parse_latest_xyz()
        if xyz is None:
            return None

        rear_x = -self._lidar_to_rear_offset
        cone = (
            (xyz[:, 0] < rear_x) &
            (xyz[:, 0] > rear_x - self._rear_cone_max_lookback) &
            (np.abs(xyz[:, 1]) < self._rear_cone_half_width) &
            (xyz[:, 2] > self._rear_cone_min_z) &
            (xyz[:, 2] < self._rear_cone_max_z)
        )
        cone_pts = xyz[cone]
        if cone_pts.shape[0] < self._rear_min_points:
            return None

        distances = -cone_pts[:, 0] - self._lidar_to_rear_offset
        min_dist = float(np.percentile(distances, 5))
        return min_dist, int(cone_pts.shape[0])

    def _check_lateral_alignment(self):
        """Detect the front beam pair behind the robot and compute its Y-centroid.

        Approach:
          1. Use a WIDER lateral window than the reverse-monitor cone, so we can
             see beams even if we're somewhat off-center.
          2. Filter to the closest cluster in X (5th percentile + 0.30m band)
             — that's the front beam pair (rear pair is much further back).
          3. Require points on BOTH +Y and -Y sides of the robot centerline
             (otherwise we're seeing only one beam, can't trust centroid).
          4. Y-centroid in robot frame = berm_center_Y - robot_Y, so robot's
             lateral offset from berm center = -centroid_Y.

        Returns (status, lateral_offset_m, n_points):
            status = 'ok'      — within ±lateral_tolerance, safe to reverse
                   = 'off'     — outside tolerance, abort deposit
                   = 'no_data' — cannot determine (LiDAR stale, single beam,
                                  or too few points); caller decides what to do
        """
        xyz = self._parse_latest_xyz()
        if xyz is None:
            return 'no_data', 0.0, 0

        rear_x = -self._lidar_to_rear_offset
        wide_cone = (
            (xyz[:, 0] < rear_x) &
            (xyz[:, 0] > rear_x - self._rear_cone_max_lookback) &
            (np.abs(xyz[:, 1]) < self._lateral_check_half_width) &
            (xyz[:, 2] > self._rear_cone_min_z) &
            (xyz[:, 2] < self._rear_cone_max_z)
        )
        cone_pts = xyz[wide_cone]
        if cone_pts.shape[0] < 2 * self._rear_min_points:
            return 'no_data', 0.0, int(cone_pts.shape[0])

        # Closest X (front pair) — take points within 0.30m of the closest
        closest_x = float(np.percentile(cone_pts[:, 0], 5))
        front_band = cone_pts[cone_pts[:, 0] <= closest_x + 0.30]
        if front_band.shape[0] < 2 * self._rear_min_points:
            return 'no_data', 0.0, int(front_band.shape[0])

        # Need points on both sides to compute centroid reliably
        pos_y = front_band[front_band[:, 1] > 0.10]   # +Y side beam
        neg_y = front_band[front_band[:, 1] < -0.10]  # -Y side beam
        if (pos_y.shape[0] < self._rear_min_points
                or neg_y.shape[0] < self._rear_min_points):
            # Only one beam visible — can't compute centroid
            return 'no_data', 0.0, int(front_band.shape[0])

        # Median Y of each beam (robust to outliers within a beam)
        pos_y_center = float(np.median(pos_y[:, 1]))
        neg_y_center = float(np.median(neg_y[:, 1]))
        centroid_y = (pos_y_center + neg_y_center) / 2.0

        # Robot's lateral offset from berm center = -centroid_y.
        # If centroid_y > 0 (beams biased to +Y in robot frame), robot is
        # at -Y side of berm → offset is negative.
        lateral_offset = -centroid_y
        n_pts = int(front_band.shape[0])

        if abs(lateral_offset) <= self._lateral_tolerance:
            return 'ok', lateral_offset, n_pts
        return 'off', lateral_offset, n_pts

    def _has_weight_data(self):
        """Check if weight sensor data is fresh enough to use."""
        if not self._weight_available:
            return False
        return (time.time() - self._last_weight_time) < self._weight_timeout

    def _time_remaining(self):
        if self._mission_start_time is None:
            return self._mission_timeout
        elapsed = (self.get_clock().now() - self._mission_start_time).nanoseconds * 1e-9
        return self._mission_timeout - elapsed

    def _safety_ok(self):
        if self._estop_active:
            return False, 'E-stop active'
        if self._time_remaining() <= 0:
            return False, 'Mission timeout'
        return True, ''

    # ------------------------------------------------------------------
    # Service handlers
    # ------------------------------------------------------------------

    def _start_cb(self, _req, res):
        if self._state != 'IDLE':
            res.success = False
            res.message = f'Cannot start: state is {self._state}'
            return res

        # Re-read starting pose params
        self._start_x = self.get_parameter('start_x').value
        self._start_y = self.get_parameter('start_y').value
        self._start_yaw = math.radians(self.get_parameter('start_yaw_deg').value)

        # Re-read arena layout in case operator changed it after launch
        self._apply_arena_layout()

        self._mission_start_time = self.get_clock().now()
        self._cycle = 0

        # Force-clear any latched e-stop before motion can begin. Downstream
        # nodes (cmd_vel_relay, excavation_bridge) have software_estop_enabled
        # disabled in YAML and ignore True, but they DO act on False, so this
        # also resets the relay's internal _estop flag in case YAML didn't
        # load. Publish once on both topics so the relay's two subscriptions
        # both receive a clean False.
        clear = Bool()
        clear.data = False
        self._estop_clear_pub.publish(clear)
        self._safety_estop_clear_pub.publish(clear)
        self._estop_active = False
        self._log('E-stop topics force-cleared (defense in depth)')

        self._log('=== MISSION STARTED ===')
        self._log(f'Arena layout: {self._arena_layout} '
                  f'(berm at Y={self._arena_waypoints["berm"][1]:.2f})')
        self._log(f'Start pose: arena({self._start_x:.2f}, {self._start_y:.2f}, '
                  f'{math.degrees(self._start_yaw):.0f}°)')

        self._publish_arena_tf()

        for name, (ax, ay, ayaw) in self._arena_waypoints.items():
            mx, my, myaw = self._arena_to_map(ax, ay, ayaw)
            self._log(f'  {name}: arena({ax:.1f},{ay:.1f},{ayaw:.0f}°) → '
                      f'map({mx:.1f},{my:.1f},{myaw:.0f}°)')

        self._begin_localization_wait()
        res.success = True
        res.message = 'Mission started'
        return res

    def _stop_cb(self, _req, res):
        self._cancel_nav()
        self._stop_excavation()
        self._cancel_berm_motion()
        self._set_state('IDLE')
        self._log('Mission stopped by operator')
        res.success = True
        res.message = 'Mission stopped'
        return res

    def _reset_cb(self, _req, res):
        self._cancel_nav()
        self._stop_excavation()
        self._cancel_berm_motion()
        self._cycle = 0
        self._mission_start_time = None
        self._set_state('IDLE')
        self._log('Mission reset')
        res.success = True
        res.message = 'Mission reset'
        return res

    def _cancel_berm_motion(self):
        """Cancel any in-progress berm reverse, dump monitor, or exit timer."""
        if self._berm_reverse_timer is not None:
            self._berm_reverse_timer.cancel()
            self._berm_reverse_timer = None
        if self._dump_monitor_timer is not None:
            self._dump_monitor_timer.cancel()
            self._dump_monitor_timer = None
        if self._exit_timer is not None:
            self._exit_timer.cancel()
            self._exit_timer = None
        if self._rev_berm_timer is not None:
            self._rev_berm_timer.cancel()
            self._rev_berm_timer = None
        self._cmd_vel_pub.publish(Twist())  # zero velocity

    # ------------------------------------------------------------------
    # Phase 0: Wait for localization
    # ------------------------------------------------------------------

    def _begin_localization_wait(self):
        self._set_state('WAIT_LOCALIZATION')
        self._loc_wait_start = time.time()
        self._log('Waiting for localization...')
        self._loc_check_timer = self.create_timer(0.5, self._check_localization)

    def _check_localization(self):
        elapsed = time.time() - self._loc_wait_start

        if self._loc_confidence >= self._min_loc_conf and self._loc_quality != 'lost':
            self._loc_check_timer.cancel()
            self._log(f'Localization OK (conf={self._loc_confidence:.2f}) — {elapsed:.1f}s')
            self._begin_alignment()
            return

        if elapsed > self._loc_wait_timeout:
            self._loc_check_timer.cancel()
            self._log(f'Localization timeout — proceeding (conf={self._loc_confidence:.2f})')
            self._begin_alignment()
            return

    # ------------------------------------------------------------------
    # Phase 0b: Align toward excavation waypoint + map obstacles ahead
    # ------------------------------------------------------------------

    def _begin_alignment(self):
        """Align to face the excavation waypoint, then dwell to map obstacles.

        The starting zone is bounded by walls on 2 sides. All obstacles
        (rocks, craters) are ahead in the excavation/obstacle zone direction.
        After aligning to face that direction, the LiDAR's FOV covers all
        obstacles ahead at up to 6m range. A brief dwell lets perception
        populate the costmap so SmacPlanner2D plans the optimal path from
        the first navigation goal — no last-second dodging.
        """
        exc_x, exc_y, _exc_yaw = self._arena_waypoints['excavation']
        dx = exc_x - self._start_x
        dy = exc_y - self._start_y
        desired_heading = math.atan2(dy, dx)

        # Angular difference (normalized to [-pi, pi])
        diff = desired_heading - self._start_yaw
        diff = math.atan2(math.sin(diff), math.cos(diff))

        self._log(f'=== ALIGNMENT ===')
        self._log(f'Current heading: {math.degrees(self._start_yaw):.0f}° | '
                  f'Target heading: {math.degrees(desired_heading):.0f}° | '
                  f'Diff: {math.degrees(diff):.0f}°')

        if abs(diff) <= self._align_tolerance:
            self._log(f'Within tolerance ({math.degrees(self._align_tolerance):.0f}°) '
                      f'— skipping rotation')
            self._begin_map_dwell()
            return

        # Rotate the minimum amount
        self._set_state('ALIGN')
        self._align_direction = 1.0 if diff > 0 else -1.0
        rotation_duration = abs(diff) / self._align_speed
        self._align_end_time = time.time() + rotation_duration
        self._log(f'Rotating {math.degrees(abs(diff)):.0f}° '
                  f'({"CCW" if diff > 0 else "CW"}, '
                  f'{rotation_duration:.1f}s at {self._align_speed:.1f} rad/s)')
        self._align_timer = self.create_timer(0.1, self._align_tick)

    def _align_tick(self):
        if self._estop_active:
            self._cmd_vel_pub.publish(Twist())
            self._align_timer.cancel()
            self._align_timer = None
            return

        if time.time() >= self._align_end_time:
            self._cmd_vel_pub.publish(Twist())
            self._align_timer.cancel()
            self._align_timer = None
            self._log('Rotation complete')
            self._begin_map_dwell()
            return

        twist = Twist()
        twist.linear.x = 0.05  # tiny forward creep to prevent sinking
        twist.angular.z = self._align_speed * self._align_direction
        self._cmd_vel_pub.publish(twist)

    # ------------------------------------------------------------------
    # Phase 0c: Map obstacles ahead before navigating
    # ------------------------------------------------------------------

    def _begin_map_dwell(self):
        """Dwell facing forward to let LiDAR map all obstacles ahead.

        Robot is now facing the excavation/obstacle zone. LiDAR sees up to
        6m — from the starting zone that covers most of the arena ahead.
        Perception at 2Hz + costmap at 2Hz means 4s gives ~8 frames of
        obstacle data. SmacPlanner2D then plans the optimal path on first try.
        """
        self._set_state('MAP_AHEAD')
        self._log(f'=== MAPPING AHEAD ===')
        self._log(f'Dwelling {self._map_dwell:.0f}s — LiDAR mapping obstacles ahead')
        self._active_timer = self.create_timer(
            self._map_dwell, self._on_map_dwell_complete)

    def _on_map_dwell_complete(self):
        if self._active_timer:
            self._active_timer.cancel()
            self._active_timer = None
        self._log('Obstacle map built — starting navigation.')
        self._begin_cycle()

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------

    def _begin_cycle(self):
        self._cycle += 1

        ok, reason = self._safety_ok()
        if not ok:
            self._log(f'Cannot begin cycle {self._cycle}: {reason}')
            self._set_state('DONE')
            return

        remaining = self._time_remaining()
        if remaining < self._return_reserve:
            self._log(f'Only {remaining:.0f}s left — DONE')
            self._set_state('DONE')
            return

        self._nav_retry_count = 0
        self._beam_correction_attempted = False
        self._log(f'')
        self._log(f'====== CYCLE {self._cycle} ======  ({remaining:.0f}s remaining)')

        if self._cycle == 1 and self._dig_at_start_zone:
            self._log('dig_at_start_zone=true — skipping nav to excavation, digging in place')
            self._begin_excavation()
            return

        # Cycle 2+ uses excavation_return (east-facing arrival) so the next
        # dig advances east, leaving rear toward berm for straight reverse.
        target = 'excavation_return' if self._cycle >= 2 else 'excavation'
        self._log(f'Cycle {self._cycle}: navigating to {target} waypoint')
        self._navigate_to(target, self._on_reached_excavation)

    def _on_reached_excavation(self, success):
        if not success:
            self._nav_retry_count += 1
            if self._nav_retry_count <= 1:
                self._log('Nav to excavation failed — retrying')
                self._navigate_to('excavation', self._on_reached_excavation)
                return
            self._log('Retry failed — excavating at current position')

        # Start smart excavation
        self._begin_excavation()

    # ------------------------------------------------------------------
    # Step-and-Dig Excavation (Scenario 2 final flow)
    # ------------------------------------------------------------------
    # Phase machine (substate inside the outer 'EXCAVATE' state):
    #
    #   DESCEND  — actuator lowering, belt auto-engages at 60% depth,
    #              transitions to SETTLE when bridge signals needs_nudge.
    #   SETTLE   — physical actuator catches up to commanded cap (~10s).
    #              Then → DWELL.
    #   DWELL    — staying in place 6–10s; weight-rate gates the next move.
    #              If Δm < stall_threshold over the window AND min dwell met:
    #                first try DEEPER (one-shot per excavation),
    #                otherwise ADVANCE.
    #              If max dwell exceeded: ADVANCE anyway.
    #              If load reached or zone boundary: RETRACT.
    #   DEEPER   — called /excavation/dig_deeper; wait dig_deeper_settle_sec,
    #              then → DWELL.
    #   ADVANCE  — drive 0.4m forward at slow crawl. Path-blocked → DODGE.
    #              On arrival → DWELL.
    #   DODGE    — sub-machine: belt_pause → settle → turn → forward → turn-back →
    #              belt_resume → DWELL.
    #   RETRACT  — call /excavation/stow → wait → NAV_TO_BERM.
    # ------------------------------------------------------------------

    def _begin_excavation(self):
        """Kick off step-and-dig: call /excavation/dig (actuator-first), then
        run the substate tick at 5 Hz."""
        self._set_state('EXCAVATE_DESCEND')
        self._excavation_phase = 'DESCEND'
        self._phase_start = time.time()
        self._excavation_start = time.time()
        self._baseline_weight = self._current_weight
        self._dwell_count = 0
        self._actuator_at_cap = False
        self._dig_deeper_used = False
        self._dodge_attempt = 0
        self._dig_actuator_pct = 0.0
        self._weight_history.clear()

        has_sensor = self._has_weight_data()
        if has_sensor:
            self._log(
                f'STEP-AND-DIG begin (baseline={self._baseline_weight:.2f}kg, '
                f'stop at +{self._max_load:.1f}kg). Cycle: descend → settle → '
                f'dwell {self._dig_dwell_min:.0f}-{self._dig_dwell_max:.0f}s '
                f'→ advance {self._dig_advance_dist*100:.0f}cm → repeat.')
        else:
            self._log(
                'STEP-AND-DIG begin (no weight sensor — time-based dwells, '
                f'{self._dig_dwell_max:.0f}s each).')

        self._call_service(self._dig_client, self._on_dig_started)

    def _on_dig_started(self, success):
        if not success:
            self._log('Dig service failed — proceeding with timer anyway')
        # 5 Hz substate tick
        if self._excavation_timer is not None:
            self._excavation_timer.cancel()
        self._excavation_timer = self.create_timer(0.2, self._excavation_tick)

    # ------------------------------------------------------------------
    # Top-level excavation tick (dispatches to phase handlers)
    # ------------------------------------------------------------------

    def _excavation_tick(self):
        # Safety
        ok, reason = self._safety_ok()
        if not ok:
            self._log(f'Excavation aborted: {reason}')
            self._enter_retract(emergency=True)
            return

        # Time budget
        elapsed = time.time() - self._excavation_start
        if elapsed >= self._dig_duration:
            self._log(f'Dig time budget reached ({elapsed:.0f}s) — retracting')
            self._enter_retract()
            return

        # Box-full
        if self._has_weight_data():
            gain = self._current_weight - self._baseline_weight
            if gain >= self._max_load:
                self._log(f'Box FULL (+{gain:.1f}kg ≥ {self._max_load:.1f}kg) '
                          f'— retracting early at {elapsed:.0f}s')
                self._enter_retract()
                return

        # Excavation-zone safety boundary — cycle 1 advances west toward x_min,
        # cycle 2+ advances east toward x_max. Both bounds checked.
        pose = self._get_robot_arena_pose()
        if pose is not None:
            ax, ay, _ = pose
            if ax < self._dig_zone_x_min:
                self._log(f'Reached west boundary (arena X={ax:.2f} < '
                          f'{self._dig_zone_x_min:.2f}) — retracting')
                self._enter_retract()
                return
            if ax > self._dig_zone_x_max:
                self._log(f'Reached east boundary (arena X={ax:.2f} > '
                          f'{self._dig_zone_x_max:.2f}) — retracting')
                self._enter_retract()
                return

        # Dispatch to phase handler
        phase = self._excavation_phase
        if phase == 'DESCEND':
            self._phase_descend()
        elif phase == 'SETTLE':
            self._phase_settle()
        elif phase == 'DWELL':
            self._phase_dwell()
        elif phase == 'DEEPER':
            self._phase_deeper()
        elif phase == 'ADVANCE':
            self._phase_advance()
        elif phase == 'DODGE':
            self._phase_dodge()
        elif phase == 'RETRACT':
            pass  # retract is timer-driven (stow service), no tick logic

    # ------------------------------------------------------------------
    # Phase: DESCEND  (waiting for bridge to signal actuator at cap)
    # ------------------------------------------------------------------

    def _phase_descend(self):
        # Bridge sets self._actuator_at_cap=True via /excavation/needs_nudge
        # when commanded value reaches the active cap.
        if self._actuator_at_cap:
            self._log(
                f'Actuator at commanded cap (~{self._dig_actuator_pct:.0f}%% '
                f'reported) — settling {self._dig_descend_settle:.0f}s for '
                f'physical catch-up')
            self._excavation_phase = 'SETTLE'
            self._set_state('EXCAVATE_SETTLE')
            self._phase_start = time.time()
            self._dig_descend_settle_start = time.time()

    # ------------------------------------------------------------------
    # Phase: SETTLE  (let physical actuator finish reaching cap)
    # ------------------------------------------------------------------

    def _phase_settle(self):
        elapsed = time.time() - self._phase_start
        if elapsed >= self._dig_descend_settle:
            self._log(f'Physical settle done ({elapsed:.1f}s) — entering DWELL')
            self._enter_dwell()

    # ------------------------------------------------------------------
    # Phase: DWELL  (stay in place, watch weight rate)
    # ------------------------------------------------------------------

    def _enter_dwell(self):
        self._excavation_phase = 'DWELL'
        self._set_state('EXCAVATE_DWELL')
        self._phase_start = time.time()
        self._dwell_count += 1
        # Stop forward motion in case ADVANCE was running
        self._cmd_vel_pub.publish(Twist())

    def _phase_dwell(self):
        elapsed = time.time() - self._phase_start
        weight_gain = self._weight_gain_in_window(self._dig_weight_window)
        # Periodic log
        if int(elapsed * 5) % 10 == 0:  # ~ every 2s
            self._log(
                f'DWELL #{self._dwell_count}: t={elapsed:.1f}s, '
                f'Δw({self._dig_weight_window:.0f}s)={weight_gain:+.3f}kg, '
                f'total={self._current_weight:.2f}kg, '
                f'actuator={self._dig_actuator_pct:.0f}%%')

        # Don't decide before we've collected enough data
        if elapsed < self._dig_dwell_min:
            return

        stalled = (self._has_weight_data()
                   and weight_gain < self._dig_weight_stall)

        if stalled:
            if not self._dig_deeper_used:
                self._log(
                    f'Weight stalled (+{weight_gain:.3f}kg in '
                    f'{self._dig_weight_window:.0f}s < '
                    f'{self._dig_weight_stall:.2f}kg) — pushing actuator '
                    f'DEEPER (92%)')
                self._dig_deeper_used = True
                self._enter_deeper()
                return
            else:
                self._log(
                    f'Weight still stalled after deeper — ADVANCING '
                    f'{self._dig_advance_dist*100:.0f}cm forward')
                self._enter_advance()
                return

        # Max dwell exceeded — advance even if still loading
        if elapsed >= self._dig_dwell_max:
            self._log(
                f'Max dwell reached ({elapsed:.1f}s, +{weight_gain:.3f}kg) '
                f'— ADVANCING for fresh sand')
            self._enter_advance()
            return

    # ------------------------------------------------------------------
    # Phase: DEEPER  (push actuator to 92% then short re-dwell)
    # ------------------------------------------------------------------

    def _enter_deeper(self):
        self._excavation_phase = 'DEEPER'
        self._set_state('EXCAVATE_DEEPER')
        self._phase_start = time.time()
        # Fire the dig_deeper service (best-effort, async)
        if self._dig_deeper_client.wait_for_service(timeout_sec=0.5):
            self._dig_deeper_client.call_async(Trigger.Request())
            self._log('Sent /excavation/dig_deeper')
        else:
            self._log('dig_deeper service unavailable — skipping straight to ADVANCE')
            self._enter_advance()

    def _phase_deeper(self):
        elapsed = time.time() - self._phase_start
        if elapsed >= self._dig_deeper_settle:
            self._log(f'Deeper settle done ({elapsed:.1f}s) — re-DWELLing')
            # Don't reset _dwell_count; this is part of the same dwell attempt
            self._excavation_phase = 'DWELL'
            self._set_state('EXCAVATE_DWELL')
            self._phase_start = time.time()
            # Clear weight history so the next stall detection is fresh
            self._weight_history.clear()

    # ------------------------------------------------------------------
    # Phase: ADVANCE  (drive 0.4m forward, watching for obstacles)
    # ------------------------------------------------------------------

    def _enter_advance(self):
        self._excavation_phase = 'ADVANCE'
        self._set_state('EXCAVATE_ADVANCE')
        self._phase_start = time.time()
        pose = self._get_robot_arena_pose()
        if pose is None:
            self._advance_start_pose = None
            self._log('ADVANCE: TF lookup failed — will advance time-based '
                      f'(~{self._dig_advance_dist/self._dig_advance_speed:.1f}s)')
        else:
            self._advance_start_pose = (pose[0], pose[1])
            self._log(f'ADVANCE: start arena=({pose[0]:.2f}, {pose[1]:.2f}), '
                      f'target +{self._dig_advance_dist*100:.0f}cm fwd')

    def _phase_advance(self):
        elapsed = time.time() - self._phase_start

        # Path-blocked → DODGE
        if self._path_blocked_ahead():
            self._log(
                f'Obstacle ahead at {self._hazard_min_distance:.2f}m during '
                f'ADVANCE — DODGE')
            self._cmd_vel_pub.publish(Twist())  # stop
            self._enter_dodge()
            return

        # Distance check via TF (preferred) or time fallback
        traveled = 0.0
        pose = self._get_robot_arena_pose()
        if pose is not None and self._advance_start_pose is not None:
            dx = pose[0] - self._advance_start_pose[0]
            dy = pose[1] - self._advance_start_pose[1]
            traveled = math.sqrt(dx * dx + dy * dy)
        else:
            traveled = elapsed * self._dig_advance_speed  # time-based fallback

        # Safety: stop early if we'd cross either X boundary
        if pose is not None:
            if pose[0] < self._dig_zone_x_min + 0.05:
                self._log(f'ADVANCE approaching west boundary (X={pose[0]:.2f}) '
                          f'— ending advance early')
                self._cmd_vel_pub.publish(Twist())
                self._enter_dwell()
                return
            if pose[0] > self._dig_zone_x_max - 0.05:
                self._log(f'ADVANCE approaching east boundary (X={pose[0]:.2f}) '
                          f'— ending advance early')
                self._cmd_vel_pub.publish(Twist())
                self._enter_dwell()
                return

        if traveled >= self._dig_advance_dist:
            self._cmd_vel_pub.publish(Twist())
            self._log(f'ADVANCE complete ({traveled:.2f}m in {elapsed:.1f}s)')
            self._enter_dwell()
            return

        # Time-out safety (advance shouldn't take more than 2x expected)
        max_advance_time = (self._dig_advance_dist / self._dig_advance_speed) * 2.0
        if elapsed > max_advance_time:
            self._cmd_vel_pub.publish(Twist())
            self._log(f'ADVANCE time-out ({elapsed:.1f}s > '
                      f'{max_advance_time:.1f}s) — DWELL anyway')
            self._enter_dwell()
            return

        # Drive forward (slow crawl)
        twist = Twist()
        twist.linear.x = self._dig_advance_speed
        self._cmd_vel_pub.publish(twist)

    # ------------------------------------------------------------------
    # Phase: DODGE  (stop belt → turn → forward → turn-back → resume belt)
    # ------------------------------------------------------------------

    def _enter_dodge(self):
        self._dodge_attempt += 1
        if self._dodge_attempt > self._dodge_max_attempts:
            self._log(f'Dodge attempts exhausted ({self._dodge_max_attempts}) '
                      f'— retracting')
            self._enter_retract()
            return
        self._excavation_phase = 'DODGE'
        self._set_state('EXCAVATE_DODGE')
        self._dodge_step = 'BELT_OFF'
        self._dodge_step_start = time.time()
        # Alternate dodge direction
        self._dodge_direction *= -1.0
        side = 'LEFT (CCW)' if self._dodge_direction > 0 else 'RIGHT (CW)'
        self._log(f'DODGE #{self._dodge_attempt} → {side}: stopping belt first')
        # Fire belt_pause; ignore result (best-effort, will retry if needed)
        if self._belt_pause_client.wait_for_service(timeout_sec=0.5):
            self._belt_pause_client.call_async(Trigger.Request())
        # Make sure we're not moving
        self._cmd_vel_pub.publish(Twist())

    def _phase_dodge(self):
        step = self._dodge_step
        step_elapsed = time.time() - self._dodge_step_start

        if step == 'BELT_OFF':
            # Wait for belt to actually stop spinning
            if step_elapsed >= self._dodge_belt_settle:
                self._log(f'  belt settled ({step_elapsed:.1f}s) — TURN')
                self._dodge_step = 'TURN'
                self._dodge_step_start = time.time()

        elif step == 'TURN':
            turn_duration = abs(self._dodge_turn_rad) / self._dodge_turn_speed
            if step_elapsed >= turn_duration:
                self._cmd_vel_pub.publish(Twist())
                self._log(f'  turn done ({step_elapsed:.1f}s) — FORWARD past rock')
                self._dodge_step = 'FORWARD'
                self._dodge_step_start = time.time()
            else:
                twist = Twist()
                twist.angular.z = self._dodge_turn_speed * self._dodge_direction
                self._cmd_vel_pub.publish(twist)

        elif step == 'FORWARD':
            fwd_duration = self._dodge_forward_m / self._dodge_forward_speed
            # Bail if forward path becomes blocked again
            if self._path_blocked_ahead() and step_elapsed > 1.0:
                self._cmd_vel_pub.publish(Twist())
                self._log('  forward path STILL blocked — aborting dodge, retract')
                self._enter_retract()
                return
            if step_elapsed >= fwd_duration:
                self._cmd_vel_pub.publish(Twist())
                self._log(f'  forward done ({step_elapsed:.1f}s) — TURN_BACK')
                self._dodge_step = 'TURN_BACK'
                self._dodge_step_start = time.time()
            else:
                twist = Twist()
                twist.linear.x = self._dodge_forward_speed
                self._cmd_vel_pub.publish(twist)

        elif step == 'TURN_BACK':
            turn_duration = abs(self._dodge_turn_rad) / self._dodge_turn_speed
            if step_elapsed >= turn_duration:
                self._cmd_vel_pub.publish(Twist())
                self._log(f'  turn_back done ({step_elapsed:.1f}s) — resuming belt')
                self._dodge_step = 'BELT_ON'
                self._dodge_step_start = time.time()
                if self._belt_resume_client.wait_for_service(timeout_sec=0.5):
                    self._belt_resume_client.call_async(Trigger.Request())
            else:
                twist = Twist()
                # Reverse direction (back to original heading)
                twist.angular.z = -self._dodge_turn_speed * self._dodge_direction
                self._cmd_vel_pub.publish(twist)

        elif step == 'BELT_ON':
            # Give belt 1s to spin back up before resuming dwell
            if step_elapsed >= 1.0:
                self._log('  DODGE complete — back to DWELL')
                self._weight_history.clear()
                self._enter_dwell()

    # ------------------------------------------------------------------
    # Phase: RETRACT  (stow hardware, hand off to nav-to-berm)
    # ------------------------------------------------------------------

    def _enter_retract(self, emergency=False):
        if self._excavation_phase == 'RETRACT':
            return  # already retracting
        self._excavation_phase = 'RETRACT'
        self._set_state('EXCAVATE_RETRACT')
        self._phase_start = time.time()
        self._cmd_vel_pub.publish(Twist())
        # Cancel the substate tick — the stow service is one-shot now
        if self._excavation_timer is not None:
            self._excavation_timer.cancel()
            self._excavation_timer = None
        if emergency:
            self._log('EMERGENCY retract — sending stow best-effort')
        # Stow: stops belt, raises actuator, retracts servo
        self._call_service(self._stow_client, self._on_stow_after_dig)

    def _stop_excavation(self):
        """Used by /mission/stop and /mission/reset to bail cleanly."""
        if self._excavation_timer is not None:
            self._excavation_timer.cancel()
            self._excavation_timer = None
        self._cmd_vel_pub.publish(Twist())
        self._excavation_phase = None
        # Emergency stow: best-effort, don't wait for response
        if self._stow_client.wait_for_service(timeout_sec=1.0):
            self._stow_client.call_async(Trigger.Request())
            self._log('Emergency stow sent')

    # ------------------------------------------------------------------
    # Post-excavation flow
    # ------------------------------------------------------------------

    def _on_stow_after_dig(self, _success):
        self._nav_retry_count = 0
        # Cycle 1: forward Nav2 to berm (builds costmap, includes 180° flip).
        # Cycle 2+: open-loop straight reverse (no flip — rear already toward
        # berm), with rear-cone obstacle check; Nav2 picks up the final Y shift.
        if self._cycle >= 2:
            self._log('Cycle 2+ — open-loop reverse to berm (rear toward berm)')
            self._begin_reverse_to_berm()
        else:
            self._log('Cycle 1 — Nav2 forward to berm (builds map)')
            self._navigate_to('berm', self._on_reached_berm)

    # ------------------------------------------------------------------
    # Cycle 2+ open-loop reverse-to-berm
    # ------------------------------------------------------------------

    def _begin_reverse_to_berm(self):
        """Cycle 2+ optimization: drive straight backward at slow speed with
        rear-cone obstacle detection. Stop when arena X drops below target,
        then hand off to Nav2 for final Y-shift + alignment."""
        self._set_state('REVERSE_TO_BERM')
        self._rev_berm_start = time.time()
        self._rev_berm_obs_pause_until = 0.0
        self._rev_berm_last_log = 0.0
        self._cmd_vel_pub.publish(Twist())  # ensure zero before starting
        if self._rev_berm_timer is not None:
            self._rev_berm_timer.cancel()
        # 10 Hz tick for responsive obstacle reaction
        self._rev_berm_timer = self.create_timer(0.1, self._reverse_to_berm_tick)

    def _reverse_to_berm_tick(self):
        # Safety / e-stop
        ok, reason = self._safety_ok()
        if not ok:
            self._finish_reverse_to_berm(f'safety failed: {reason}', fallback_nav=True)
            return

        elapsed = time.time() - self._rev_berm_start

        # Time-out safety
        if elapsed > self._rev_berm_max_sec:
            self._finish_reverse_to_berm(
                f'timeout after {elapsed:.0f}s', fallback_nav=True)
            return

        # Position via TF
        pose = self._get_robot_arena_pose()
        if pose is None:
            # No TF — abort to Nav2 fallback (one-shot, don't keep spinning)
            self._finish_reverse_to_berm(
                'TF lookup failed', fallback_nav=True)
            return
        ax, ay, _ayaw = pose

        # Reached target X (close enough to berm gap mouth)?
        if ax <= self._rev_berm_target_x:
            self._finish_reverse_to_berm(
                f'arena X={ax:.2f} ≤ target {self._rev_berm_target_x:.2f}',
                fallback_nav=False)
            return

        # Rear obstacle check — reuses the same rear-cone parser we use for
        # berm-pair detection. The rear cone is in lidar frame, X<0 = behind.
        sample = self._compute_rear_cone_dist()
        if sample is not None:
            dist, n_pts = sample
            now = time.time()
            if now - self._rev_berm_last_log > 1.0:
                self._rev_berm_last_log = now
                self._log(f'  reverse-to-berm: X={ax:.2f}, rear obstacle '
                          f'{dist:.2f}m ({n_pts} pts), {elapsed:.1f}s elapsed')
            if (n_pts >= self._rev_berm_min_pts
                    and dist < self._rev_berm_rear_clr):
                # Rear obstacle too close — bail to Nav2 (it can plan around)
                self._cmd_vel_pub.publish(Twist())
                self._log(f'Rear obstacle at {dist:.2f}m '
                          f'(< {self._rev_berm_rear_clr:.2f}m clearance) '
                          f'— handing off to Nav2')
                self._finish_reverse_to_berm(
                    'rear obstacle', fallback_nav=True)
                return

        # Drive backward
        twist = Twist()
        twist.linear.x = self._rev_berm_speed  # negative = backward
        self._cmd_vel_pub.publish(twist)

    def _finish_reverse_to_berm(self, reason, fallback_nav):
        """Stop the open-loop reverse, then either hand to Nav2 (which will
        do the small Y-shift to berm waypoint and run the lateral check) or
        skip Nav2 if we ended up close enough."""
        if self._rev_berm_timer is not None:
            self._rev_berm_timer.cancel()
            self._rev_berm_timer = None
        self._cmd_vel_pub.publish(Twist())
        elapsed = time.time() - self._rev_berm_start
        self._log(f'Reverse-to-berm done: {reason} (t={elapsed:.1f}s)')
        # Always go through Nav2 for the final approach — it handles the Y
        # shift to ±1.3m and the lateral-alignment check before reverse.
        # _on_reached_berm runs the same beam-pair logic as cycle 1.
        self._navigate_to('berm', self._on_reached_berm)

    def _on_reached_berm(self, success):
        if not success:
            self._nav_retry_count += 1
            if self._nav_retry_count <= 1:
                self._log('Nav to berm failed — retrying')
                self._navigate_to('berm', self._on_reached_berm)
                return
            self._log('Retry failed — attempting deposit at current position')

        # ---- Lateral alignment pre-check ----
        # The deposition door drops material straight down at the rear, so
        # we never enter the berm square — we only need to align the rear
        # bumper with the front face of the square. But the side beams are
        # at lateral ±0.50m and the robot is 0.82m wide, leaving only ~9cm
        # clearance per side. Nav2's xy_goal_tolerance is 0.15m, so we MUST
        # confirm we're laterally centered before reversing or we'll hit a
        # side beam.
        align_status, lateral_offset, n_pts = self._check_lateral_alignment()
        if align_status == 'off':
            # Try a beam-corrected re-approach FIRST: use the detected beam
            # centroid as a fiducial to fix Point-LIO / setpose drift. Only
            # attempt if the offset is within bounded recoverable range, and
            # only once per cycle (the flag is reset in _begin_cycle).
            if (abs(lateral_offset) <= self._max_beam_correction
                    and not self._beam_correction_attempted):
                self._beam_correction_attempted = True
                bx, by, byaw = self._arena_waypoints['berm']
                corrected_y = by - lateral_offset  # see _check_lateral_alignment
                self._arena_waypoints['berm_corrected'] = (bx, corrected_y, byaw)
                self._log(f'BEAM-CORRECTED RE-APPROACH: offset={lateral_offset:+.3f}m '
                          f'(beams seen, {n_pts} pts). Re-targeting arena Y: '
                          f'{by:.2f} → {corrected_y:.2f}')
                self._navigate_to('berm_corrected', self._on_reached_berm)
                return

            self._log(f'⚠️  LATERAL OFFSET TOO LARGE: {lateral_offset:+.3f}m '
                      f'(tolerance ±{self._lateral_tolerance:.2f}m, max correctable '
                      f'±{self._max_beam_correction:.2f}m, {n_pts} cluster pts seen) — '
                      f'SKIPPING DEPOSIT for cycle {self._cycle}, exiting back to excavation')
            # Skip the reverse entirely; go straight to next-cycle setup.
            # Material stays in the deposition box for the next attempt —
            # better than risking a beam collision. We never tilted the
            # dump door, so no stow needed; jump to the post-dump path
            # which handles the forward exit + Nav2 handoff.
            self._set_state('DEPOSIT_SKIPPED')
            self._on_stow_after_dump(True)
            return
        elif align_status == 'ok':
            self._log(f'Lateral alignment OK: offset={lateral_offset:+.3f}m '
                      f'({n_pts} beam-cluster pts)')
        else:  # 'no_data'
            self._log(f'⚠️  Lateral check: insufficient beam data '
                      f'({n_pts} cluster pts) — proceeding with caution')

        # ---- Beam-pair-driven reverse: ALL cycles use edge alignment ----
        # Robot reverses, front beam pair enters rear cone, distance shrinks
        # toward 0 as the beams pass beside the rear bumper. Stop when the
        # closest distance reaches berm_cycle2plus_stop_distance, OR when
        # the pair "releases" (jumps in distance — beams now beside us).
        # Either condition means rear is at the berm square's front face,
        # which is where the deposition door drops material.
        rev_dur = self._berm_reverse_duration
        stop_dist = self._cycle2plus_stop_dist

        self._current_berm_reverse_duration = rev_dur
        self._reverse_pairs_passed = 0
        self._reverse_obs_state = 'waiting'
        self._reverse_min_in_pair = float('inf')
        self._reverse_last_log_time = 0.0
        self._berm_reverse_start = time.time()

        self._log(f'Berm reverse — EDGE alignment (cycle {self._cycle})')
        self._log(f'  speed={self._berm_reverse_speed:.2f} m/s, '
                  f'max time={rev_dur:.0f}s, stop@{stop_dist:.2f}m '
                  f'(emergency stop@{self._emergency_stop_dist:.2f}m)')

        self._berm_reverse_timer = self.create_timer(
            0.1, self._berm_reverse_tick)

    def _berm_reverse_finish(self, reason):
        """Stop the reverse, log the reason, advance to DEPOSIT."""
        self._cmd_vel_pub.publish(Twist())
        if self._berm_reverse_timer is not None:
            self._berm_reverse_timer.cancel()
            self._berm_reverse_timer = None
        elapsed = time.time() - self._berm_reverse_start
        traveled_cm = abs(self._berm_reverse_speed) * elapsed * 100.0
        self._log(f'Reverse complete: {reason} '
                  f'(t={elapsed:.1f}s, ~{traveled_cm:.0f}cm, '
                  f'pairs_passed={self._reverse_pairs_passed})')
        self._set_state('DEPOSIT')
        self._call_service(self._dump_client, self._on_dump_done)

    def _berm_reverse_tick(self):
        """All cycles: stop when front beam pair is at the rear bumper edge.

        Stop happens on whichever fires first:
          (a) closest dist <= stop_dist (clean stop right at threshold)
          (b) pair release (dist jumps up — beams have moved beside us, we're
              at the edge or just past it; stop immediately to avoid overshoot)
          (c) sample becomes None during 'observing' (cone empty after seeing
              beams close — fallback for very fast pair sweeps)
          (d) emergency: obstacle within emergency_stop_distance
          (e) e-stop / encoder stall / max-time timeout
        """
        elapsed = time.time() - self._berm_reverse_start
        stop_dist = self._cycle2plus_stop_dist

        # ---- (e) E-stop: highest priority ----
        if self._estop_active:
            self._berm_reverse_finish('E-STOP active')
            return

        # ---- (e) Encoder stall: hardware says wheels aren't turning ----
        if self._is_stalled:
            self._berm_reverse_finish('encoder stall detected')
            return

        # ---- (e) Time-based fallback: if beams never seen, give up ----
        if elapsed >= self._current_berm_reverse_duration:
            self._berm_reverse_finish(
                f'timeout ({self._current_berm_reverse_duration:.0f}s reached, '
                f'no front pair detected — depositing here)')
            return

        # ---- LiDAR-driven pair detection ----
        sample = self._compute_rear_cone_dist()

        if sample is None:
            # No obstacle in rear cone. If we WERE observing the pair, it's
            # released — stop now (beams just slipped beside us).
            if self._reverse_obs_state == 'observing':
                if self._reverse_min_in_pair < self._pair_release_thresh:
                    self._reverse_pairs_passed = 1
                    self._berm_reverse_finish(
                        f'pair released (cone emptied; '
                        f'min was {self._reverse_min_in_pair:.2f}m) '
                        f'— rear at berm edge')
                    return
                # Pair was observed but never got close — false alarm, reset
                self._reverse_obs_state = 'waiting'
                self._reverse_min_in_pair = float('inf')
        else:
            dist, n_pts = sample

            # Periodic debug log (every 0.5s)
            now = time.time()
            if now - self._reverse_last_log_time >= 0.5:
                self._reverse_last_log_time = now
                self._log(f'  rear-cone: {dist:.2f}m ({n_pts} pts), '
                          f'state={self._reverse_obs_state}, '
                          f'min_in_pair={self._reverse_min_in_pair:.2f}m')

            # ---- (d) Emergency collision stop ----
            if dist < self._emergency_stop_dist:
                self._berm_reverse_finish(
                    f'EMERGENCY: obstacle {dist:.2f}m < '
                    f'{self._emergency_stop_dist:.2f}m behind rear')
                return

            # ---- State transitions ----
            if self._reverse_obs_state == 'waiting':
                if dist < self._pair_detect_thresh:
                    self._reverse_obs_state = 'observing'
                    self._reverse_min_in_pair = dist
                    self._log(f'  → front pair entering rear cone at {dist:.2f}m')
            else:  # 'observing'
                # Track minimum
                if dist < self._reverse_min_in_pair:
                    self._reverse_min_in_pair = dist

                # ---- (b) Release-by-jump: distance jumped above min ----
                # The pair has moved beside the robot — stop right here.
                if (self._reverse_min_in_pair < self._pair_release_thresh
                        and dist > self._reverse_min_in_pair + 0.40):
                    self._reverse_pairs_passed = 1
                    self._berm_reverse_finish(
                        f'pair released (dist jumped '
                        f'{self._reverse_min_in_pair:.2f}m → {dist:.2f}m) '
                        f'— rear at berm edge')
                    return

                # ---- (a) Threshold stop ----
                if dist <= stop_dist:
                    self._berm_reverse_finish(
                        f'front pair reached threshold: '
                        f'dist={dist:.2f}m <= {stop_dist:.2f}m')
                    return

        # ---- Otherwise: keep reversing ----
        twist = Twist()
        twist.linear.x = self._berm_reverse_speed  # negative = backward
        self._cmd_vel_pub.publish(twist)

    def _on_dump_done(self, success):
        if not success:
            self._log('Dump service failed')

        # Weight-based dump completion. The deposit servo dropped the door
        # open and material is falling out. Monitor the weight: when it
        # returns to within `dump_baseline_threshold` of the dig baseline,
        # the box is empty → stow immediately. Else stow after
        # `dump_max_wait_sec` (covers cases where material sticks, sensor
        # is noisy, or no sensor data is available).
        self._dump_start_time = time.time()
        self._dump_start_weight = self._current_weight
        self._dump_last_log_time = 0.0

        target_weight = self._baseline_weight + self._dump_baseline_threshold
        weight_to_drop = max(0.0, self._current_weight - self._baseline_weight)

        if self._has_weight_data():
            self._log(f'DUMPING — monitoring weight (baseline={self._baseline_weight:.2f}kg, '
                      f'current={self._current_weight:.2f}kg, '
                      f'target<={target_weight:.2f}kg, '
                      f'~{weight_to_drop:.1f}kg to drop, '
                      f'max wait={self._dump_max_wait:.0f}s)')
        else:
            self._log(f'DUMPING — no weight sensor, using fallback wait '
                      f'{self._dump_wait:.0f}s')

        self._dump_monitor_timer = self.create_timer(
            0.5, self._dump_monitor_tick
        )

    def _dump_monitor_tick(self):
        """Check dump completion by weight return-to-baseline OR timeout."""
        elapsed = time.time() - self._dump_start_time

        if self._estop_active:
            self._finish_dump('e-stop active', elapsed)
            return

        # If no fresh weight data, fall back to legacy fixed wait
        if not self._has_weight_data():
            if elapsed >= self._dump_wait:
                self._finish_dump(
                    f'no sensor data, fallback wait {self._dump_wait:.0f}s done',
                    elapsed)
            return

        gain_above_baseline = self._current_weight - self._baseline_weight

        # Periodic log (every 2s)
        now = time.time()
        if now - self._dump_last_log_time >= 2.0:
            self._dump_last_log_time = now
            dropped = self._dump_start_weight - self._current_weight
            self._log(f'  dump: weight={self._current_weight:.2f}kg, '
                      f'gain_above_baseline={gain_above_baseline:+.2f}kg, '
                      f'dropped={dropped:+.2f}kg ({elapsed:.0f}/'
                      f'{self._dump_max_wait:.0f}s)')

        # Stow when weight is back near baseline
        if gain_above_baseline <= self._dump_baseline_threshold:
            self._finish_dump(
                f'weight at baseline (gain={gain_above_baseline:+.2f}kg '
                f'<= {self._dump_baseline_threshold:.2f}kg threshold)',
                elapsed)
            return

        # Hard timeout
        if elapsed >= self._dump_max_wait:
            self._finish_dump(
                f'max wait {self._dump_max_wait:.0f}s reached '
                f'(material may be stuck — gain still '
                f'{gain_above_baseline:+.2f}kg)',
                elapsed)
            return

    def _finish_dump(self, reason, elapsed):
        """Cancel monitor, log reason, call stow."""
        if self._dump_monitor_timer is not None:
            self._dump_monitor_timer.cancel()
            self._dump_monitor_timer = None
        self._log(f'Dump complete: {reason} (t={elapsed:.1f}s) — stowing')
        self._call_service(self._stow_client, self._on_stow_after_dump)

    def _on_stow_after_dump(self, _success):
        remaining = self._time_remaining()
        self._log(f'Cycle {self._cycle} COMPLETE — {remaining:.0f}s remaining')

        if self._cycle >= self._num_cycles:
            self._log('Max cycles reached')
            self._set_state('DONE')
            return

        if remaining < self._return_reserve:
            self._log('Not enough time for another cycle — DONE')
            self._set_state('DONE')
            return

        # Forward-exit before handing back to Nav2.
        # After deposit at the berm edge, robot center is at X ≈ -1.78m.
        # Front beams at X = -2.32m with Nav2 inflation_radius 0.70m extend
        # an inflated zone to X = -1.62m — robot center at -1.78 is INSIDE
        # the inflated obstacle space, so Nav2 will refuse to plan from here.
        # Open-loop drive forward by exit_forward_duration to clear the zone,
        # THEN call _begin_cycle so Nav2 has free space to plan from.
        self._begin_berm_exit()

    def _begin_berm_exit(self):
        """Open-loop forward drive to clear the front-beam inflation zone."""
        self._set_state('EXIT_BERM')
        self._exit_start_time = time.time()
        distance_cm = self._exit_forward_speed * self._exit_forward_duration * 100
        self._log(f'Berm exit — driving forward '
                  f'{self._exit_forward_duration:.1f}s at '
                  f'{self._exit_forward_speed:.2f} m/s (~{distance_cm:.0f}cm) '
                  f'to clear inflation zone')
        self._exit_timer = self.create_timer(0.1, self._exit_tick)

    def _exit_tick(self):
        elapsed = time.time() - self._exit_start_time
        if (elapsed >= self._exit_forward_duration
                or self._estop_active
                or self._is_stalled):
            self._cmd_vel_pub.publish(Twist())
            if self._exit_timer is not None:
                self._exit_timer.cancel()
                self._exit_timer = None
            reason = ('e-stop' if self._estop_active
                      else 'encoder stall' if self._is_stalled
                      else f'duration {self._exit_forward_duration:.1f}s reached')
            self._log(f'Berm exit complete ({reason}, '
                      f't={elapsed:.1f}s) — handing to Nav2')
            self._begin_cycle()
            return
        twist = Twist()
        twist.linear.x = self._exit_forward_speed  # positive = forward
        self._cmd_vel_pub.publish(twist)

    # ------------------------------------------------------------------
    # Navigation
    # ------------------------------------------------------------------

    def _navigate_to(self, waypoint_name, done_callback):
        ok, reason = self._safety_ok()
        if not ok:
            self._log(f'Safety failed: {reason}')
            done_callback(False)
            return

        self._set_state(f'NAVIGATE_TO_{waypoint_name.upper()}')
        pose = self._make_pose(waypoint_name)
        self._log(
            f'NAV → {waypoint_name} '
            f'(map: {pose.pose.position.x:.1f}, {pose.pose.position.y:.1f})'
        )

        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self._log('Nav2 server not available')
            done_callback(False)
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose

        # Navigation timeout — cancel goal if Nav2 is stuck
        self._nav_done_callback = done_callback
        self._nav_timeout_timer = self.create_timer(
            self._nav_timeout, self._nav_timeout_triggered)

        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda f: self._nav_goal_response(f, done_callback)
        )

    def _nav_timeout_triggered(self):
        """Called when navigation takes too long — cancel and fail."""
        if self._nav_timeout_timer:
            self._nav_timeout_timer.cancel()
            self._nav_timeout_timer = None
        self._log(f'Navigation TIMEOUT ({self._nav_timeout:.0f}s) — cancelling goal')
        self._cancel_nav()
        if self._nav_done_callback:
            cb = self._nav_done_callback
            self._nav_done_callback = None
            cb(False)

    def _nav_goal_response(self, future, done_callback):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self._log('Nav2 rejected goal')
            done_callback(False)
            return

        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._nav_result(f, done_callback)
        )

    def _nav_result(self, future, done_callback):
        self._goal_handle = None
        # Cancel timeout timer — we got a result
        if hasattr(self, '_nav_timeout_timer') and self._nav_timeout_timer:
            self._nav_timeout_timer.cancel()
            self._nav_timeout_timer = None
        self._nav_done_callback = None

        result = future.result()
        if result and result.status == 4:
            self._log('Navigation succeeded')
            done_callback(True)
        else:
            status = result.status if result else 'None'
            self._log(f'Navigation failed (status={status})')
            done_callback(False)

    def _cancel_nav(self):
        if hasattr(self, '_nav_timeout_timer') and self._nav_timeout_timer:
            self._nav_timeout_timer.cancel()
            self._nav_timeout_timer = None
        self._nav_done_callback = None
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

    # ------------------------------------------------------------------
    # Service call helper
    # ------------------------------------------------------------------

    def _call_service(self, client, done_callback):
        ok, reason = self._safety_ok()
        if not ok:
            self._log(f'Safety failed: {reason}')
            done_callback(False)
            return

        if not client.wait_for_service(timeout_sec=5.0):
            self._log(f'Service {client.srv_name} not available')
            done_callback(False)
            return

        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f: self._service_result(f, done_callback)
        )

    def _service_result(self, future, done_callback):
        try:
            result = future.result()
            self._log(f'Service: {result.message}')
            done_callback(result.success)
        except Exception as e:
            self._log(f'Service error: {e}')
            done_callback(False)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _set_state(self, state):
        self._state = state

    def _publish_state(self):
        state_msg = String()
        state_msg.data = self._state
        self._state_pub.publish(state_msg)

        remaining = self._time_remaining()
        weight_str = f'{self._current_weight:.2f}kg' if self._weight_available else 'N/A'
        # Include step-and-dig telemetry when in an EXCAVATE_* state
        dig_info = ''
        if self._state.startswith('EXCAVATE'):
            dig_info = (f' dwell#{self._dwell_count} '
                        f'actu={self._dig_actuator_pct:.0f}% '
                        f'belt={self._dig_belt_pwm}')
        status_msg = String()
        status_msg.data = (
            f'[{self._state}] cycle={self._cycle} '
            f'time={remaining:.0f}s zone={self._current_zone} '
            f'weight={weight_str}{dig_info}'
        )
        self._status_pub.publish(status_msg)

    def _log(self, msg):
        self.get_logger().info(msg)
        out = String()
        out.data = msg
        self._status_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
