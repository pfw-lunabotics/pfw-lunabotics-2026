#!/usr/bin/env python3
"""
PFW Lunabotics 2026 — Mission Control GUI (runs on operator VM).

Architecture:
  - Persistent bringup runs on the Jetson (launched/killed from this GUI).
    The bringup brings up pico_bridge, servo driver, sensors, EKF, perception,
    Nav2, and the mission_controller (idle, waiting for /mission/start).
  - Autonomous mission = call /mission/start (service)
  - Manual teleop = GUI publishes Twist to /cmd_vel directly from the VM.
    Pico bridge and servo driver stay up the whole time.
  - E-stop = publish std_msgs/Bool to /estop AND /safety/estop.

All ROS2 IO is done with std_msgs / geometry_msgs / std_srvs, which ship with
ros-humble-ros-base — no luna_msgs build needed on the VM.

The Jetson connection is plain SSH (key-based). The GUI streams the bringup's
stdout/stderr into a log pane.
"""

import os
import sys
import shlex
import subprocess
import threading
import time
from collections import deque
from datetime import datetime

from PyQt5 import QtCore, QtGui, QtWidgets

import math

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Bool, String, Int32, Float32
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
import tf2_ros


# ------------------------------------------------------------------ #
# Connection defaults — override via the top bar at runtime
# ------------------------------------------------------------------ #
JETSON_USER = os.environ.get("JETSON_USER", "lunabotics")
JETSON_HOST = os.environ.get("JETSON_HOST", "192.168.0.200")
WORKSPACE   = os.environ.get("JETSON_WS",   "/home/lunabotics/pfw-lunabotics")

# Bringup command — mirrors scripts/scenario2_full_mission.sh but without tmux.
# Everything between bash -lc '...' runs in a login shell on the Jetson.
BRINGUP_CMD = (
    f"cd {WORKSPACE} && "
    "source /opt/ros/humble/setup.bash && "
    "source install/setup.bash && "
    f"[ -f {WORKSPACE}/fastrtps_no_shm.xml ] && "
    f"export FASTRTPS_DEFAULT_PROFILES_FILE={WORKSPACE}/fastrtps_no_shm.xml; "
    "ros2 launch lunabotics_navigation bringup.launch.py "
    "use_point_lio:=true use_autonomy:=true simulate:=false "
    "use_localization:=true arena_layout:=A debug:=true"
)

PID_FILE = "/tmp/luna_bringup.pid"


# ------------------------------------------------------------------ #
# ROS2 worker — runs in its own thread with a SingleThreadedExecutor
# ------------------------------------------------------------------ #
class RosWorker(QtCore.QObject):
    """Publishers, subscribers, service clients. Lives in a Qt thread."""

    pico_status_changed     = QtCore.pyqtSignal(str)
    system_health_changed   = QtCore.pyqtSignal(str)
    mission_status_changed  = QtCore.pyqtSignal(str)
    mission_state_changed   = QtCore.pyqtSignal(str)
    loc_quality_changed     = QtCore.pyqtSignal(str)
    zone_changed            = QtCore.pyqtSignal(str)
    control_status_changed  = QtCore.pyqtSignal(str)
    estop_state_changed     = QtCore.pyqtSignal(bool)
    log_line                = QtCore.pyqtSignal(str)
    # Live data-flow indicator — fires when ANY subscribed topic delivers a
    # message. Used by the discovery light in the top bar to confirm transport
    # is actually flowing (not just discovery).
    data_received           = QtCore.pyqtSignal()
    # Live cmd_vel readout (subscribed on VM so user can see Nav2 output)
    cmd_vel_received        = QtCore.pyqtSignal(float, float)
    # Deposition weight (HX711 on Pico) — kg of sand in the box
    weight_received         = QtCore.pyqtSignal(float)
    # Deposition servo state ("CONNECTED", "MOVING:52.0deg,4500ms", etc.)
    servo_status_changed    = QtCore.pyqtSignal(str)
    # Servo raw position (3283 stowed → 3870 open)
    servo_position_received = QtCore.pyqtSignal(int)
    # Latest actuator command echoed back (-100..+100)
    actuator_cmd_received   = QtCore.pyqtSignal(int)
    # Step-and-dig telemetry from /excavation/actuator_pct + /excavation/belt_pwm
    actuator_pct_received   = QtCore.pyqtSignal(float)
    belt_pwm_received       = QtCore.pyqtSignal(int)
    # Live robot pose in the arena frame (x, y, yaw_deg). Emitted at ~5 Hz
    # when arena→base_footprint TF is available.
    pose_in_arena_changed   = QtCore.pyqtSignal(float, float, float)
    # Emitted when the arena TF is unavailable (mission not started yet).
    pose_unavailable        = QtCore.pyqtSignal(str)

    # Trigger services we pre-create on start() — keeps round-trip latency low
    # and avoids the on-demand discovery race that previously made buttons
    # "silently do nothing" for the first 2s after launch.
    SERVICES = (
        '/mission/start', '/mission/stop', '/mission/reset',
        '/excavation/dig', '/excavation/dump', '/excavation/stow',
        '/excavation/belt_pause', '/excavation/belt_resume',
        '/excavation/dig_deeper',
    )

    # Mission-controller parameters we know how to set from the GUI.
    # Mapping name -> (ParameterType, casting fn).
    PARAM_TYPES = {
        'arena_layout':    (ParameterType.PARAMETER_STRING,  str),
        'start_x':         (ParameterType.PARAMETER_DOUBLE,  float),
        'start_y':         (ParameterType.PARAMETER_DOUBLE,  float),
        'start_yaw_deg':   (ParameterType.PARAMETER_DOUBLE,  float),
    }

    def __init__(self):
        super().__init__()
        self.node: Node = None
        self.executor: SingleThreadedExecutor = None
        self._thread = None
        self._running = False

        self.pub_cmd_vel = None
        self.pub_cmd_vel_safe = None
        self.pub_estop = None
        self.pub_safety_estop = None
        self.pub_excavation = None
        self.pub_actuator = None

        # Pre-created service clients (filled in start())
        self._trigger_clients = {}
        self._mc_param_client = None

    # -- lifecycle -------------------------------------------------- #
    def start(self):
        rclpy.init(args=None)
        self.node = rclpy.create_node("mission_control_gui")

        # Publishers
        # Twist goes to BOTH /cmd_vel (through cmd_vel_relay) and /cmd_vel_safe
        # (direct to pico_bridge) — same dual-publish pattern as teleop_keyboard
        # so a MANUAL override fully dominates any autonomous Nav2 output.
        self.pub_cmd_vel       = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_cmd_vel_safe  = self.node.create_publisher(Twist, "/cmd_vel_safe", 10)
        self.pub_estop         = self.node.create_publisher(Bool,  "/estop", 10)
        self.pub_safety_estop  = self.node.create_publisher(Bool,  "/safety/estop", 10)
        self.pub_excavation    = self.node.create_publisher(Int32, "/excavation/motor", 10)
        self.pub_actuator      = self.node.create_publisher(Int32, "/actuator/command", 10)

        # Subscribers — every callback also fires data_received so the
        # discovery light in the top bar lights up only if real messages
        # are flowing (not just topic-list discovery).
        sub = self.node.create_subscription
        def _wrap_str(signal):
            def _cb(m):
                signal.emit(m.data)
                self.data_received.emit()
            return _cb
        sub(String, "/pico/status",                       _wrap_str(self.pico_status_changed), 10)
        sub(String, "/system/health",                     _wrap_str(self.system_health_changed), 10)
        sub(String, "/mission/status",                    _wrap_str(self.mission_status_changed), 10)
        sub(String, "/mission/state",                     _wrap_str(self.mission_state_changed), 10)
        sub(String, "/perception/localization_quality",   _wrap_str(self.loc_quality_changed), 10)
        sub(String, "/perception/current_zone",           _wrap_str(self.zone_changed), 10)
        sub(String, "/control/status",                    _wrap_str(self.control_status_changed), 10)
        sub(Bool,   "/estop",                             lambda m: (self.estop_state_changed.emit(m.data), self.data_received.emit()), 10)
        sub(Twist,  "/cmd_vel",                           lambda m: (self.cmd_vel_received.emit(m.linear.x, m.angular.z), self.data_received.emit()), 10)
        # Excavation hardware telemetry
        sub(Float32,"/deposition/weight",                 lambda m: (self.weight_received.emit(m.data), self.data_received.emit()), 10)
        sub(String, "/servo/status",                      _wrap_str(self.servo_status_changed), 10)
        sub(Int32,  "/servo/position",                    lambda m: (self.servo_position_received.emit(m.data), self.data_received.emit()), 10)
        sub(Int32,  "/actuator/command",                  lambda m: (self.actuator_cmd_received.emit(m.data), self.data_received.emit()), 10)
        # Step-and-dig telemetry — actuator depth pct + current belt PWM
        sub(Float32,"/excavation/actuator_pct",           lambda m: (self.actuator_pct_received.emit(m.data), self.data_received.emit()), 10)
        sub(Int32,  "/excavation/belt_pwm",               lambda m: (self.belt_pwm_received.emit(m.data), self.data_received.emit()), 10)

        # Pre-created service clients — created once, reused. Avoids the
        # 2-second discovery wait + race-on-first-call that made buttons
        # appear unresponsive after bringup restart.
        for srv_name in self.SERVICES:
            self._trigger_clients[srv_name] = self.node.create_client(Trigger, srv_name)

        # Pre-created mission_controller parameter client — replaces the
        # SSH-based 'ros2 param set' round-trip (3+ s) with a native DDS
        # service call (sub-100 ms typical).
        self._mc_param_client = self.node.create_client(
            SetParameters, '/mission_controller/set_parameters'
        )

        # TF listener for live arena→base_footprint readout. Mission
        # controller publishes arena→map at /mission/start; chained with
        # map→odom→base_footprint from Point-LIO + EKF this gives the robot
        # pose in arena coords.
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self.node)
        self.node.create_timer(0.2, self._poll_pose)  # 5 Hz

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self._running = True
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()
        self.log_line.emit("[ros2] node up — ROS_DOMAIN_ID={}".format(os.environ.get("ROS_DOMAIN_ID", "0")))

    def _spin(self):
        while self._running and rclpy.ok():
            self.executor.spin_once(timeout_sec=0.1)

    def _poll_pose(self):
        """Look up arena→base_footprint and emit a live pose signal."""
        try:
            tf = self._tf_buffer.lookup_transform(
                'arena', 'base_footprint', rclpy.time.Time())
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            # arena frame not published yet (mission not started) — try
            # base_link as a fallback in case base_footprint isn't in the
            # tree on this run.
            try:
                tf = self._tf_buffer.lookup_transform(
                    'arena', 'base_link', rclpy.time.Time())
            except Exception:
                self.pose_unavailable.emit(
                    "arena TF not available — press START MISSION to anchor"
                )
                return
        t = tf.transform.translation
        q = tf.transform.rotation
        # Yaw from quaternion (z-axis rotation only — robot is planar).
        yaw_rad = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.pose_in_arena_changed.emit(t.x, t.y, math.degrees(yaw_rad))

    def shutdown(self):
        self._running = False
        try:
            if self._thread:
                self._thread.join(timeout=1.0)
            if self.node:
                self.node.destroy_node()
                self.node = None
            rclpy.shutdown()
        except Exception:
            pass

    # -- commands --------------------------------------------------- #
    def publish_cmd_vel(self, lin: float, ang: float):
        if not self._running or self.node is None:
            return
        msg = Twist()
        msg.linear.x = float(lin)
        msg.angular.z = float(ang)
        try:
            self.pub_cmd_vel.publish(msg)
            self.pub_cmd_vel_safe.publish(msg)
        except Exception:
            pass

    def publish_estop(self, value: bool):
        if not self._running or self.node is None:
            return
        msg = Bool(); msg.data = bool(value)
        try:
            self.pub_estop.publish(msg)
            self.pub_safety_estop.publish(msg)
        except Exception:
            pass

    def publish_excavation(self, pwm: int):
        if not self._running or self.node is None:
            return
        msg = Int32(); msg.data = int(pwm)
        try:
            self.pub_excavation.publish(msg)
        except Exception:
            pass

    def publish_actuator(self, value: int):
        if not self._running or self.node is None:
            return
        msg = Int32(); msg.data = int(value)
        try:
            self.pub_actuator.publish(msg)
        except Exception:
            pass

    def call_trigger(self, service_name: str, on_done):
        client = self._trigger_clients.get(service_name)
        if client is None:
            # Service wasn't pre-registered (e.g., dynamic service); make one
            # ad-hoc. This path is slower but keeps backward compatibility.
            client = self.node.create_client(Trigger, service_name)
            self._trigger_clients[service_name] = client
        if not client.service_is_ready():
            # Don't block GUI thread; fail fast and let the user retry once
            # bringup is actually up.
            on_done(False, f"service {service_name} not ready (bringup running?)")
            return
        fut = client.call_async(Trigger.Request())
        def _cb(f):
            try:
                res = f.result()
                on_done(res.success, res.message)
            except Exception as e:
                on_done(False, str(e))
        fut.add_done_callback(_cb)

    def set_mc_params(self, updates: dict, on_done):
        """Set multiple mission_controller parameters in ONE service call.

        updates is {param_name: value}. Native DDS call — replaces the
        SSH-based 'ros2 param set' round-trip used previously.
        on_done is called as (ok: bool, msg: str).
        """
        if self._mc_param_client is None or not self._mc_param_client.service_is_ready():
            on_done(False, "/mission_controller/set_parameters not ready (bringup running?)")
            return
        req = SetParameters.Request()
        for name, value in updates.items():
            if name not in self.PARAM_TYPES:
                on_done(False, f"unsupported param: {name}")
                return
            ptype, cast = self.PARAM_TYPES[name]
            pv = ParameterValue()
            pv.type = ptype
            casted = cast(value)
            if ptype == ParameterType.PARAMETER_STRING:
                pv.string_value = casted
            elif ptype == ParameterType.PARAMETER_DOUBLE:
                pv.double_value = casted
            elif ptype == ParameterType.PARAMETER_INTEGER:
                pv.integer_value = casted
            elif ptype == ParameterType.PARAMETER_BOOL:
                pv.bool_value = casted
            req.parameters.append(Parameter(name=name, value=pv))
        fut = self._mc_param_client.call_async(req)
        def _cb(f):
            try:
                res = f.result()
                successes = [r.successful for r in res.results]
                if all(successes):
                    on_done(True, f"set {list(updates.keys())}")
                else:
                    reasons = ', '.join(r.reason for r in res.results if not r.successful)
                    on_done(False, f"partial set; reasons: {reasons}")
            except Exception as e:
                on_done(False, str(e))
        fut.add_done_callback(_cb)


# ------------------------------------------------------------------ #
# SSH helper — long-running QProcess (for bringup) + one-shot fire-and-forget
# ------------------------------------------------------------------ #
def ssh_args(user: str, host: str, remote_cmd: str, tty: bool = True):
    args = [
        "ssh",
        "-o", "StrictHostKeyChecking=accept-new",
        "-o", "ServerAliveInterval=10",
        "-o", "ServerAliveCountMax=3",
    ]
    if tty:
        args += ["-tt"]
    # OpenSSH concatenates argv-after-host with spaces WITHOUT re-quoting,
    # so passing ["bash", "-lc", "source ...; ros2 ..."] arrives at the
    # remote shell as `bash -lc source ...; ros2 ...` — bash -lc then sees
    # only "source" as its command (filename arg becomes $0). Wrap the
    # inner command in a single shell-quoted arg so the remote sees one
    # `bash -lc '...'` invocation instead.
    args += [f"{user}@{host}", f"bash -lc {shlex.quote(remote_cmd)}"]
    return args


def ssh_oneshot(user: str, host: str, remote_cmd: str, timeout: float = 10.0):
    """Run a quick SSH command, return (returncode, stdout+stderr)."""
    try:
        p = subprocess.run(
            ssh_args(user, host, remote_cmd, tty=False),
            capture_output=True, text=True, timeout=timeout,
        )
        return p.returncode, (p.stdout or "") + (p.stderr or "")
    except subprocess.TimeoutExpired:
        return -1, "(ssh timeout)"
    except Exception as e:
        return -1, str(e)


# ------------------------------------------------------------------ #
# Manual teleop pane — captures W/A/S/D on key events
# ------------------------------------------------------------------ #
class TeleopPane(QtWidgets.QWidget):
    LINEAR_MAX  = 0.40
    ANGULAR_MAX = 0.80
    PUBLISH_HZ  = 20.0

    # Belt PWM control (matches teleop_keyboard)
    BELT_PWM_MIN  = 6000
    BELT_PWM_MAX  = 34500
    BELT_PWM_STEP = 1500
    BELT_PWM_DEFAULT = 11000

    def __init__(self, ros: RosWorker, parent=None):
        super().__init__(parent)
        self.ros = ros
        self._held = set()
        self._manual = False
        # Belt state — toggled by 'r', adjusted by 'f'/'v'
        self._belt_on = False
        self._belt_speed = self.BELT_PWM_DEFAULT

        root = QtWidgets.QVBoxLayout(self)
        title = QtWidgets.QLabel("<b>Manual Teleop</b>")
        root.addWidget(title)
        help_html = (
            "<small style='color:#aaa;'>"
            "<b>Click pane → toggle MANUAL → use keys</b><br>"
            "<b>Drive:</b> W/A/S/D  &nbsp; <b>Stop:</b> Space<br>"
            "<b>Belt (raw PWM, use OUTSIDE a dig):</b> R toggle, F/V faster/slower<br>"
            "<b>Belt (DURING a dig):</b> P pause &nbsp; O resume (services — survive bridge heartbeat)<br>"
            "<b>Actuator:</b> U (up)  &nbsp; J (down)  &nbsp; H (stop)<br>"
            "<b>Servo:</b> T (dump)  &nbsp; Y (stow)<br>"
            "<b>E-STOP:</b> ESC (anywhere)"
            "</small>"
        )
        help_label = QtWidgets.QLabel(help_html)
        help_label.setStyleSheet("padding: 2px 4px;")
        help_label.setWordWrap(True)
        root.addWidget(help_label)

        # Mode toggle (the 'm' equivalent)
        toggle_row = QtWidgets.QHBoxLayout()
        self.mode_btn = QtWidgets.QPushButton("OBSERVER — click to enter MANUAL")
        self.mode_btn.setCheckable(True)
        self.mode_btn.setStyleSheet("padding: 8px; font-weight: bold;")
        self.mode_btn.toggled.connect(self._toggle_manual)
        toggle_row.addWidget(self.mode_btn)
        self.mode_label = QtWidgets.QLabel("MODE: OBSERVER")
        self.mode_label.setStyleSheet("color: #888; padding: 0 8px;")
        toggle_row.addWidget(self.mode_label)
        toggle_row.addStretch(1)
        root.addLayout(toggle_row)

        # Speed sliders
        sliders = QtWidgets.QGridLayout()
        sliders.addWidget(QtWidgets.QLabel("Linear max (m/s):"), 0, 0)
        self.lin_slider = QtWidgets.QDoubleSpinBox()
        self.lin_slider.setRange(0.05, 0.40); self.lin_slider.setSingleStep(0.05); self.lin_slider.setValue(0.20)
        sliders.addWidget(self.lin_slider, 0, 1)
        sliders.addWidget(QtWidgets.QLabel("Angular max (rad/s):"), 1, 0)
        self.ang_slider = QtWidgets.QDoubleSpinBox()
        self.ang_slider.setRange(0.10, 0.80); self.ang_slider.setSingleStep(0.10); self.ang_slider.setValue(0.50)
        sliders.addWidget(self.ang_slider, 1, 1)
        root.addLayout(sliders)

        # Excavation + actuator + servo
        # These are gated on MANUAL mode so an accidental click during an
        # autonomous cycle can't fire /excavation/dig, /actuator/command, etc.
        actions = QtWidgets.QGridLayout()
        self.btn_dig  = QtWidgets.QPushButton("Dig (full sequence — actuator first, belt engages ~60% depth)")
        self.btn_dump = QtWidgets.QPushButton("Dump (open servo)")
        self.btn_stow = QtWidgets.QPushButton("Stow (close servo)")
        self.btn_dig.clicked.connect(lambda: self._call("/excavation/dig"))
        self.btn_dump.clicked.connect(lambda: self._call("/excavation/dump"))
        self.btn_stow.clicked.connect(lambda: self._call("/excavation/stow"))
        actions.addWidget(self.btn_dig,  0, 0, 1, 3)
        actions.addWidget(self.btn_dump, 1, 0)
        actions.addWidget(self.btn_stow, 1, 1)

        # Belt control during a dig: the bridge runs a 10 Hz heartbeat that
        # republishes the target PWM, so a raw "publish 0 to /excavation/motor"
        # gets immediately overwritten. The proper way to halt the belt mid-dig
        # is /excavation/belt_pause (sets _belt_paused=True so heartbeat sends
        # 0); /excavation/belt_resume restarts at the last ramped PWM.
        self.btn_belt_pause  = QtWidgets.QPushButton("Belt PAUSE (stop belt, hold actuator)")
        self.btn_belt_resume = QtWidgets.QPushButton("Belt RESUME")
        self.btn_dig_deeper  = QtWidgets.QPushButton("Dig DEEPER (extend cap 85%→92%)")
        self.btn_belt_pause.clicked.connect(lambda: self._call("/excavation/belt_pause"))
        self.btn_belt_resume.clicked.connect(lambda: self._call("/excavation/belt_resume"))
        self.btn_dig_deeper.clicked.connect(lambda: self._call("/excavation/dig_deeper"))
        actions.addWidget(self.btn_belt_pause,  2, 0)
        actions.addWidget(self.btn_belt_resume, 2, 1)
        actions.addWidget(self.btn_dig_deeper,  2, 2)

        # Raw belt motor=0: only effective when NOT in a dig sequence (no
        # heartbeat overriding). Useful for bench testing / clearing after a
        # manual R-key toggle. During a dig, use Belt PAUSE above.
        self.btn_belt_off = QtWidgets.QPushButton("Belt MOTOR=0 (raw — use OUTSIDE a dig)")
        self.btn_belt_off.clicked.connect(lambda: self.ros.publish_excavation(0))
        actions.addWidget(self.btn_belt_off, 3, 0, 1, 3)

        act_row = QtWidgets.QHBoxLayout()
        self.btn_act_up   = QtWidgets.QPushButton("Actuator UP (hold)")
        self.btn_act_dn   = QtWidgets.QPushButton("Actuator DOWN (hold)")
        self.btn_act_stop = QtWidgets.QPushButton("Actuator STOP")
        self.btn_act_up.pressed.connect(lambda: self._actuator_hold(+100))
        self.btn_act_up.released.connect(lambda: self.ros.publish_actuator(0))
        self.btn_act_dn.pressed.connect(lambda: self._actuator_hold(-100))
        self.btn_act_dn.released.connect(lambda: self.ros.publish_actuator(0))
        self.btn_act_stop.clicked.connect(lambda: self.ros.publish_actuator(0))
        act_row.addWidget(self.btn_act_up)
        act_row.addWidget(self.btn_act_dn)
        act_row.addWidget(self.btn_act_stop)
        root.addLayout(actions)
        root.addLayout(act_row)

        # Live readout
        self.readout = QtWidgets.QLabel("cmd_vel: (0.00, 0.00)")
        self.readout.setStyleSheet("font-family: monospace; padding: 4px;")
        root.addWidget(self.readout)
        root.addStretch(1)

        # Publish timer
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(int(1000 / self.PUBLISH_HZ))
        self.timer.timeout.connect(self._tick)

        # Capture keys
        self.setFocusPolicy(QtCore.Qt.StrongFocus)

        # Collect every action button so we can gate them on MANUAL mode
        self._action_buttons = [
            self.btn_dig, self.btn_dump, self.btn_stow,
            self.btn_belt_pause, self.btn_belt_resume, self.btn_dig_deeper,
            self.btn_belt_off,
            self.btn_act_up, self.btn_act_dn, self.btn_act_stop,
        ]
        self._set_actions_enabled(False)

    def _set_actions_enabled(self, enabled: bool):
        for b in self._action_buttons:
            b.setEnabled(enabled)

    def _toggle_manual(self, checked: bool):
        self._manual = checked
        if checked:
            self.mode_btn.setText("MANUAL — click to release to OBSERVER")
            self.mode_btn.setStyleSheet("padding: 8px; font-weight: bold; background: #d68a00; color: white;")
            self.mode_label.setText("MODE: MANUAL  (W=fwd  S=back  A=left  D=right)")
            self.mode_label.setStyleSheet("color: #d68a00; padding: 0 8px; font-weight: bold;")
            self.timer.start()
            self._set_actions_enabled(True)
            self.setFocus()
        else:
            self.mode_btn.setText("OBSERVER — click to enter MANUAL")
            self.mode_btn.setStyleSheet("padding: 8px; font-weight: bold;")
            self.mode_label.setText("MODE: OBSERVER")
            self.mode_label.setStyleSheet("color: #888; padding: 0 8px;")
            self.timer.stop()
            self._held.clear()
            # In OBSERVER we publish nothing — autonomous Nav2 owns /cmd_vel.
            # We intentionally do NOT push a final zero Twist, because that
            # would race the autonomous publisher and could brake the robot.
            self._set_actions_enabled(False)
            self.readout.setText("cmd_vel: (idle — autonomous owns /cmd_vel)")

    def keyPressEvent(self, e: QtGui.QKeyEvent):
        if e.isAutoRepeat() or not self._manual:
            return super().keyPressEvent(e)
        k = e.text().lower()
        # Drive keys (held)
        if k in ("w", "a", "s", "d"):
            self._held.add(k); e.accept(); return
        # Space = full stop
        if e.key() == QtCore.Qt.Key_Space:
            self._held.clear()
            self.ros.publish_cmd_vel(0.0, 0.0)
            e.accept(); return
        # Excavation belt
        if k == "r":
            self._belt_on = not self._belt_on
            pwm = self._belt_speed if self._belt_on else 0
            self.ros.publish_excavation(pwm)
            self.readout.setText(self._readout_str())
            self.window().log(f"[teleop] BELT {'ON' if self._belt_on else 'OFF'} (pwm={pwm})")
            e.accept(); return
        if k == "f":
            self._belt_speed = min(self.BELT_PWM_MAX, self._belt_speed + self.BELT_PWM_STEP)
            if self._belt_on:
                self.ros.publish_excavation(self._belt_speed)
            self.window().log(f"[teleop] belt speed = {self._belt_speed}")
            self.readout.setText(self._readout_str())
            e.accept(); return
        if k == "v":
            self._belt_speed = max(self.BELT_PWM_MIN, self._belt_speed - self.BELT_PWM_STEP)
            if self._belt_on:
                self.ros.publish_excavation(self._belt_speed)
            self.window().log(f"[teleop] belt speed = {self._belt_speed}")
            self.readout.setText(self._readout_str())
            e.accept(); return
        # Actuator
        if k == "u":
            self.ros.publish_actuator(+100)
            self.window().log("[teleop] actuator UP (+100)")
            e.accept(); return
        if k == "j":
            self.ros.publish_actuator(-100)
            self.window().log("[teleop] actuator DOWN (-100)")
            e.accept(); return
        if k == "h":
            self.ros.publish_actuator(0)
            self.window().log("[teleop] actuator STOP")
            e.accept(); return
        # Belt pause/resume (service calls — only way to stop the belt while
        # a dig sequence is running, since the bridge's heartbeat overrides
        # any direct /excavation/motor=0 message)
        if k == "p":
            self._call("/excavation/belt_pause")
            self.window().log("[teleop] belt PAUSE (service)")
            e.accept(); return
        if k == "o":
            self._call("/excavation/belt_resume")
            self.window().log("[teleop] belt RESUME (service)")
            e.accept(); return
        # Deposition servo
        if k == "t":
            self._call("/excavation/dump")
            e.accept(); return
        if k == "y":
            self._call("/excavation/stow")
            e.accept(); return
        super().keyPressEvent(e)

    def _readout_str(self) -> str:
        belt_state = f"belt={'ON' if self._belt_on else 'off'} pwm={self._belt_speed}"
        return f"cmd_vel: held={sorted(self._held)}  |  {belt_state}"

    def keyReleaseEvent(self, e: QtGui.QKeyEvent):
        if e.isAutoRepeat() or not self._manual:
            return super().keyReleaseEvent(e)
        k = e.text().lower()
        if k in self._held:
            self._held.discard(k); e.accept(); return
        super().keyReleaseEvent(e)

    def _tick(self):
        lin_max = float(self.lin_slider.value())
        ang_max = float(self.ang_slider.value())
        lin = 0.0
        ang = 0.0
        if "w" in self._held: lin += lin_max
        if "s" in self._held: lin -= lin_max
        if "a" in self._held: ang += ang_max
        if "d" in self._held: ang -= ang_max
        self.ros.publish_cmd_vel(lin, ang)
        self.readout.setText(f"cmd_vel: ({lin:+.2f}, {ang:+.2f})   held={sorted(self._held)}")

    def _actuator_hold(self, value: int):
        self.ros.publish_actuator(value)

    def _call(self, service: str):
        def done(ok, msg):
            self.window().log(f"[svc] {service}: {'OK' if ok else 'FAIL'} — {msg}")
        self.ros.call_trigger(service, done)

    def stop_motion(self):
        self._held.clear()
        self.ros.publish_cmd_vel(0.0, 0.0)


# ------------------------------------------------------------------ #
# Autonomous pane — bringup over SSH, mission service calls
# ------------------------------------------------------------------ #
class AutonomyPane(QtWidgets.QWidget):
    def __init__(self, ros: RosWorker, get_conn, parent=None):
        super().__init__(parent)
        self.ros = ros
        self.get_conn = get_conn  # returns (user, host)
        self.bringup_proc: QtCore.QProcess = None

        root = QtWidgets.QVBoxLayout(self)
        root.addWidget(QtWidgets.QLabel("<b>Autonomous Mission</b>"))
        root.addWidget(QtWidgets.QLabel(
            "<small>Convention: A=TOP arena (berm at +Y north), "
            "B=BOTTOM arena (berm at -Y south). "
            "+X=east (fiducial wall). +Y=north (compass-anchored).</small>"
        ))

        # Arena layout (A=TOP, B=BOTTOM)
        layout_row = QtWidgets.QHBoxLayout()
        layout_row.addWidget(QtWidgets.QLabel("Arena:"))
        self.layout_combo = QtWidgets.QComboBox()
        self.layout_combo.addItems(["A (TOP)", "B (BOTTOM)"])
        layout_row.addWidget(self.layout_combo)
        self.btn_setlayout = QtWidgets.QPushButton("setarena")
        self.btn_setlayout.clicked.connect(self._setlayout)
        layout_row.addWidget(self.btn_setlayout)
        layout_row.addStretch(1)
        root.addLayout(layout_row)

        # Facing row (N/S/E/W -> start_yaw_deg)
        facing_row = QtWidgets.QHBoxLayout()
        facing_row.addWidget(QtWidgets.QLabel("Facing:"))
        self.facing_combo = QtWidgets.QComboBox()
        self.facing_combo.addItems([
            "W (180 - toward berm)",
            "E (0 - toward fiducial)",
            "N (90 - image up)",
            "S (270 - image down)",
        ])
        facing_row.addWidget(self.facing_combo)
        self.btn_setfacing = QtWidgets.QPushButton("setfacing")
        self.btn_setfacing.clicked.connect(self._setfacing)
        facing_row.addWidget(self.btn_setfacing)
        facing_row.addStretch(1)
        root.addLayout(facing_row)

        # Position row (OPTIONAL — beams self-correct up to 0.5m)
        pose_row = QtWidgets.QHBoxLayout()
        pose_row.addWidget(QtWidgets.QLabel("Position (optional):"))
        self.x_in   = QtWidgets.QDoubleSpinBox(); self.x_in.setRange(-5.0, 5.0); self.x_in.setValue(3.0); self.x_in.setSingleStep(0.1)
        self.y_in   = QtWidgets.QDoubleSpinBox(); self.y_in.setRange(-3.0, 3.0); self.y_in.setValue(0.0); self.y_in.setSingleStep(0.1)
        pose_row.addWidget(QtWidgets.QLabel("X")); pose_row.addWidget(self.x_in)
        pose_row.addWidget(QtWidgets.QLabel("Y")); pose_row.addWidget(self.y_in)
        self.btn_setpos = QtWidgets.QPushButton("setpos")
        self.btn_setpos.clicked.connect(self._setpos)
        pose_row.addWidget(self.btn_setpos)
        # Advanced: yaw spinbox (rarely needed if you use Facing dropdown)
        self.yaw_in = QtWidgets.QDoubleSpinBox(); self.yaw_in.setRange(0.0, 360.0); self.yaw_in.setValue(180.0); self.yaw_in.setSingleStep(5.0)
        pose_row.addWidget(QtWidgets.QLabel("Yaw°")); pose_row.addWidget(self.yaw_in)
        self.btn_setpose = QtWidgets.QPushButton("setpose(adv)")
        self.btn_setpose.setToolTip("Advanced: sets X, Y, AND yaw at once. Prefer setarena+setfacing+setpos.")
        self.btn_setpose.clicked.connect(self._setpose)
        pose_row.addWidget(self.btn_setpose)
        pose_row.addStretch(1)
        root.addLayout(pose_row)

        # Live arena-frame pose readout (subscribes to TF). Read-only.
        live_row = QtWidgets.QHBoxLayout()
        live_row.addWidget(QtWidgets.QLabel("Live pose:"))
        self.live_pose_label = QtWidgets.QLabel("— (mission not started — arena TF unavailable)")
        self.live_pose_label.setStyleSheet(
            "font-family: monospace; padding: 2px 8px; color: #888;"
        )
        live_row.addWidget(self.live_pose_label, 1)
        root.addLayout(live_row)
        self.ros.pose_in_arena_changed.connect(self._on_live_pose)
        self.ros.pose_unavailable.connect(self._on_pose_unavailable)

        # Buttons row
        btns = QtWidgets.QHBoxLayout()
        self.btn_bringup_start = QtWidgets.QPushButton("Start Bringup")
        self.btn_bringup_kill  = QtWidgets.QPushButton("Kill Bringup")
        self.btn_mission_start = QtWidgets.QPushButton("START MISSION")
        self.btn_mission_stop  = QtWidgets.QPushButton("Stop Mission")
        self.btn_mission_reset = QtWidgets.QPushButton("Reset")
        self.btn_bringup_start.setStyleSheet("background: #2e7d32; color: white; padding: 6px; font-weight: bold;")
        self.btn_bringup_kill.setStyleSheet("background: #6a1b1b; color: white; padding: 6px;")
        self.btn_mission_start.setStyleSheet("background: #1b5e20; color: white; padding: 6px; font-weight: bold;")
        self.btn_mission_stop.setStyleSheet("background: #b26a00; color: white; padding: 6px;")
        self.btn_bringup_start.clicked.connect(self._start_bringup)
        self.btn_bringup_kill.clicked.connect(self._kill_bringup)
        self.btn_mission_start.clicked.connect(lambda: self._call("/mission/start"))
        self.btn_mission_stop.clicked.connect(lambda: self._call("/mission/stop"))
        self.btn_mission_reset.clicked.connect(lambda: self._call("/mission/reset"))
        btns.addWidget(self.btn_bringup_start)
        btns.addWidget(self.btn_bringup_kill)
        btns.addWidget(self.btn_mission_start)
        btns.addWidget(self.btn_mission_stop)
        btns.addWidget(self.btn_mission_reset)
        root.addLayout(btns)

        # Output log
        self.out = QtWidgets.QPlainTextEdit()
        self.out.setReadOnly(True)
        self.out.setMaximumBlockCount(5000)
        self.out.setStyleSheet("font-family: monospace; background: #111; color: #ddd;")
        root.addWidget(self.out, 1)

        # Mission state badge
        self.state_label = QtWidgets.QLabel("state: —")
        self.state_label.setStyleSheet("font-family: monospace; padding: 4px;")
        root.addWidget(self.state_label)

        self.ros.mission_state_changed.connect(lambda s: self.state_label.setText(f"state: {s}"))

    def _append(self, text: str):
        ts = datetime.now().strftime("%H:%M:%S")
        for line in text.rstrip("\n").splitlines():
            self.out.appendPlainText(f"{ts}  {line}")

    def _start_bringup(self):
        if self.bringup_proc and self.bringup_proc.state() != QtCore.QProcess.NotRunning:
            self._append("[bringup] already running")
            return
        user, host = self.get_conn()
        layout = self.layout_combo.currentText().split()[0]  # "A (TOP)" -> "A"
        cmd = BRINGUP_CMD.replace("arena_layout:=A", f"arena_layout:={layout}")
        # PID marker so we can kill from a separate ssh later
        # ssh_args wraps the inner command as `bash -lc '...'` already, so
        # we just need a compound that records PID then runs the launch.
        # No `exec cd` needed — the launch is the longest-lived process and
        # the kill path uses both the PID file and pkill on bringup.launch.py.
        wrapped = f"echo $$ > {PID_FILE}; {cmd}"

        self.bringup_proc = QtCore.QProcess(self)
        self.bringup_proc.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        self.bringup_proc.readyReadStandardOutput.connect(self._on_bringup_stdout)
        self.bringup_proc.finished.connect(self._on_bringup_finished)
        args = ssh_args(user, host, wrapped, tty=True)
        self._append(f"[bringup] starting on {user}@{host} ...")
        self.bringup_proc.start(args[0], args[1:])

    def _on_bringup_stdout(self):
        data = bytes(self.bringup_proc.readAllStandardOutput()).decode("utf-8", errors="replace")
        self._append(data)

    def _on_bringup_finished(self, code, status):
        self._append(f"[bringup] exited code={code}")

    def _kill_bringup(self):
        user, host = self.get_conn()
        # Kill by pid file + pkill fallback. SIGINT first, then SIGKILL.
        kill_cmd = (
            f"if [ -f {PID_FILE} ]; then "
            f"  kill -INT $(cat {PID_FILE}) 2>/dev/null; "
            f"  sleep 2; kill -9 $(cat {PID_FILE}) 2>/dev/null; "
            f"  rm -f {PID_FILE}; "
            f"fi; "
            "pkill -INT -f bringup.launch.py 2>/dev/null; sleep 1; "
            "pkill -9   -f bringup.launch.py 2>/dev/null; true"
        )
        rc, out = ssh_oneshot(user, host, kill_cmd, timeout=10.0)
        self._append(f"[bringup] kill rc={rc} {out.strip()}")
        if self.bringup_proc and self.bringup_proc.state() != QtCore.QProcess.NotRunning:
            self.bringup_proc.terminate()
            self.bringup_proc.waitForFinished(3000)
            if self.bringup_proc.state() != QtCore.QProcess.NotRunning:
                self.bringup_proc.kill()

    def _call(self, service: str):
        def done(ok, msg):
            self._append(f"[svc] {service}: {'OK' if ok else 'FAIL'} — {msg}")
        self.ros.call_trigger(service, done)

    def _setpose(self):
        x, y, yaw = self.x_in.value(), self.y_in.value(), self.yaw_in.value()
        self._append(f"[setpose] x={x} y={y} yaw={yaw} ...")
        self.ros.set_mc_params(
            {'start_x': x, 'start_y': y, 'start_yaw_deg': yaw},
            lambda ok, msg: self._append(f"[setpose] {'OK' if ok else 'FAIL'} - {msg}")
        )

    def _setpos(self):
        x, y = self.x_in.value(), self.y_in.value()
        self._append(f"[setpos] x={x} y={y} ...")
        self.ros.set_mc_params(
            {'start_x': x, 'start_y': y},
            lambda ok, msg: self._append(f"[setpos] {'OK' if ok else 'FAIL'} - {msg}")
        )

    def _setfacing(self):
        sel = self.facing_combo.currentText().strip()
        letter = sel[0] if sel else "W"
        yaw_map = {"E": 0.0, "N": 90.0, "W": 180.0, "S": 270.0}
        yaw = yaw_map.get(letter, 180.0)
        self.yaw_in.setValue(yaw)
        self._append(f"[setfacing] {letter} -> yaw={yaw} ...")
        self.ros.set_mc_params(
            {'start_yaw_deg': yaw},
            lambda ok, msg: self._append(f"[setfacing] {'OK' if ok else 'FAIL'} - {msg}")
        )

    def _setlayout(self):
        layout = self.layout_combo.currentText().split()[0]  # "A (TOP)" -> "A"
        self._append(f"[setarena] {layout} ...")
        self.ros.set_mc_params(
            {'arena_layout': layout},
            lambda ok, msg: self._append(f"[setarena] {'OK' if ok else 'FAIL'} - {msg}")
        )

    # -- Live pose readout (from arena→base_footprint TF) ---------- #
    def _on_live_pose(self, x: float, y: float, yaw_deg: float):
        # Normalize yaw to [0, 360) for display consistency with setfacing.
        yaw_deg = yaw_deg % 360.0
        self.live_pose_label.setText(
            f"x={x:+.2f}  y={y:+.2f}  yaw={yaw_deg:6.1f}°  (arena frame)"
        )
        self.live_pose_label.setStyleSheet(
            "font-family: monospace; padding: 2px 8px; color: #6abf69; font-weight: bold;"
        )

    def _on_pose_unavailable(self, reason: str):
        self.live_pose_label.setText(f"— {reason}")
        self.live_pose_label.setStyleSheet(
            "font-family: monospace; padding: 2px 8px; color: #888;"
        )


# ------------------------------------------------------------------ #
# Health pane — live ROS2 subscriptions
# ------------------------------------------------------------------ #
class HealthPane(QtWidgets.QWidget):
    def __init__(self, ros: RosWorker, get_conn, parent=None):
        super().__init__(parent)
        self.ros = ros
        self.get_conn = get_conn

        root = QtWidgets.QVBoxLayout(self)
        root.addWidget(QtWidgets.QLabel("<b>Robot Health & Status</b>"))

        self.fields = {}
        grid = QtWidgets.QGridLayout()
        rows = [
            ("Jetson",        "jetson"),
            ("Bringup nodes", "nodes"),
            ("Pico",          "pico"),
            ("System health", "health"),
            ("Mission state", "state"),
            ("Mission status","status"),
            ("Current zone",  "zone"),
            ("Loc quality",   "loc"),
            ("Control relay", "control"),
            ("cmd_vel",       "cmdvel"),    # live readout of /cmd_vel (Nav2 + manual)
            ("Weight (kg)",   "weight"),    # /deposition/weight  (HX711)
            ("Deposition",    "servo"),     # /servo/status + /servo/position
            ("Actuator",      "actuator"),  # /actuator/command  (-100..+100)
            ("Dig phase",     "digphase"),  # actuator depth %% + belt PWM (step-and-dig)
            ("E-STOP",        "estop"),
        ]
        for i, (label, key) in enumerate(rows):
            grid.addWidget(QtWidgets.QLabel(label + ":"), i, 0)
            v = QtWidgets.QLabel("—")
            v.setStyleSheet("font-family: monospace; padding: 2px 6px;")
            v.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            grid.addWidget(v, i, 1)
            self.fields[key] = v
        root.addLayout(grid)
        root.addStretch(1)

        # /cmd_vel — track last-seen for freshness rendering, plus a rolling
        # count to compute publish rate.
        self._cmdvel_last = (0.0, 0.0)
        self._cmdvel_last_t = 0.0
        self._cmdvel_count = 0
        ros.cmd_vel_received.connect(self._on_cmd_vel)
        self._cmdvel_timer = QtCore.QTimer(self)
        self._cmdvel_timer.setInterval(500)  # render at 2 Hz
        self._cmdvel_timer.timeout.connect(self._refresh_cmdvel)
        self._cmdvel_timer.start()

        # Weight sensor — HX711 on Pico publishes /deposition/weight (kg).
        # Higher number = more sand picked up. Display as "X.XX kg".
        ros.weight_received.connect(self._on_weight)
        # Deposition servo state — combine /servo/status + /servo/position
        # into a single human-readable row. Stowed=3283, Open=3870.
        self._servo_pos = None
        self._servo_state = "—"
        ros.servo_status_changed.connect(self._on_servo_status)
        ros.servo_position_received.connect(self._on_servo_position)
        # Actuator — show last commanded value as percent of full travel.
        ros.actuator_cmd_received.connect(self._on_actuator)
        # Step-and-dig telemetry: actuator depth %% + current belt PWM
        # Combined into a single "Dig phase" row that's only meaningful while
        # mission state is one of the EXCAVATE_* substates.
        self._dig_actuator_pct = 0.0
        self._dig_belt_pwm = 0
        ros.actuator_pct_received.connect(self._on_actuator_pct)
        ros.belt_pwm_received.connect(self._on_belt_pwm)

        # Wire signals
        ros.pico_status_changed.connect(lambda s: self._set("pico", s, ok=("OK" in s)))
        ros.system_health_changed.connect(lambda s: self._set("health", s, ok=("OK" in s or "READY" in s)))
        ros.mission_status_changed.connect(lambda s: self._set("status", s, neutral=True))
        ros.mission_state_changed.connect(lambda s: self._set("state", s, neutral=True))
        ros.loc_quality_changed.connect(lambda s: self._set("loc", s, ok=(s.lower() == "good")))
        ros.zone_changed.connect(lambda s: self._set("zone", s, neutral=True))
        ros.control_status_changed.connect(lambda s: self._set("control", s, ok=("RUNNING" in s)))
        ros.estop_state_changed.connect(self._set_estop)

        # Ping timer for Jetson reachability + node count
        self.ping_timer = QtCore.QTimer(self)
        self.ping_timer.timeout.connect(self._poll_jetson)
        self.ping_timer.start(5000)
        QtCore.QTimer.singleShot(500, self._poll_jetson)

    def _on_cmd_vel(self, lin: float, ang: float):
        self._cmdvel_last = (lin, ang)
        self._cmdvel_last_t = time.monotonic()
        self._cmdvel_count += 1

    def _on_weight(self, kg: float):
        # 0–10 kg expected typical range; flag green if there's anything in
        # the box, red if sensor is wildly off (e.g., negative below tare).
        if kg < -0.5:
            self._set("weight", f"{kg:+.2f} kg (sensor drift?)", ok=False)
        elif kg < 0.2:
            self._set("weight", f"{kg:+.2f} kg (empty)", neutral=True)
        else:
            self._set("weight", f"{kg:+.2f} kg", ok=True)

    def _on_servo_status(self, status: str):
        self._servo_state = status
        self._render_servo()

    def _on_servo_position(self, pos: int):
        self._servo_pos = pos
        self._render_servo()

    def _render_servo(self):
        # Map raw position to a human label: 3283 = stowed (closed),
        # 3870 = open. Anything outside that range is reported as raw.
        STOWED, OPEN = 3283, 3870
        label_parts = []
        if self._servo_pos is not None:
            if abs(self._servo_pos - STOWED) < 30:
                label_parts.append("STOWED")
            elif abs(self._servo_pos - OPEN) < 30:
                label_parts.append("OPEN")
            else:
                # Percent open between stowed and fully open
                pct = (self._servo_pos - STOWED) * 100.0 / (OPEN - STOWED)
                pct = max(0.0, min(100.0, pct))
                label_parts.append(f"{pct:.0f}% open ({self._servo_pos})")
        if self._servo_state and self._servo_state != "—":
            label_parts.append(f"[{self._servo_state}]")
        text = "  ".join(label_parts) if label_parts else "—"
        # ok=green when stowed or moving toward stowed; we treat anything
        # reachable as healthy. The disconnected state is the only failure.
        bad = "DISCONNECTED" in self._servo_state or "NO_RESPONSE" in self._servo_state
        self._set("servo", text, ok=not bad if label_parts else None,
                   neutral=not label_parts)

    def _on_actuator(self, cmd: int):
        # cmd ∈ [-100, +100]. -100 = fully extended (down), +100 = retracted (up).
        cmd = max(-100, min(100, int(cmd)))
        if cmd == 0:
            self._set("actuator", "0 (hold)", neutral=True)
        elif cmd > 0:
            self._set("actuator", f"+{cmd}  (raising)", ok=True)
        else:
            self._set("actuator", f"{cmd}  (lowering)", ok=True)

    def _on_actuator_pct(self, pct: float):
        self._dig_actuator_pct = float(pct)
        self._refresh_digphase()

    def _on_belt_pwm(self, pwm: int):
        self._dig_belt_pwm = int(pwm)
        self._refresh_digphase()

    def _refresh_digphase(self):
        """Render the 'Dig phase' row: actuator depth pct + belt PWM.

        Color: green when belt is spinning (i.e., dig is actively cutting),
        neutral when idle or paused. Useful at-a-glance check that the
        step-and-dig flow is alive and not stuck.
        """
        pct = self._dig_actuator_pct
        pwm = self._dig_belt_pwm
        text = f"actuator {pct:.0f}%  belt PWM {pwm}"
        if pwm > 0 and pct >= 30.0:
            self._set("digphase", text, ok=True)
        elif pwm == 0 and pct < 5.0:
            self._set("digphase", text + "  (idle)", neutral=True)
        else:
            self._set("digphase", text, neutral=True)

    def _refresh_cmdvel(self):
        if self._cmdvel_last_t == 0.0:
            self._set("cmdvel", "idle (no publisher)", neutral=True)
            return
        age = time.monotonic() - self._cmdvel_last_t
        lin, ang = self._cmdvel_last
        if age < 1.0:
            # Approximate rate: count msgs in the last second window
            rate = self._cmdvel_count / max(age, 0.001)
            # Reset window
            self._cmdvel_count = 0
            self._cmdvel_last_t = time.monotonic()  # reuse as window-start; ok for display
            txt = f"lin={lin:+.2f}  ang={ang:+.2f}  ~{rate:.0f} Hz"
            moving = abs(lin) > 0.01 or abs(ang) > 0.01
            self._set("cmdvel", txt, ok=moving, neutral=not moving)
        elif age < 5.0:
            self._set("cmdvel", f"stale ({int(age)}s ago) lin={lin:+.2f} ang={ang:+.2f}",
                       neutral=True)
        else:
            self._set("cmdvel", "no traffic", neutral=True)

    def _set(self, key, value, ok=None, neutral=False):
        w = self.fields[key]
        w.setText(value if value else "—")
        if neutral:
            w.setStyleSheet("font-family: monospace; padding: 2px 6px; color: #ddd;")
        elif ok is True:
            w.setStyleSheet("font-family: monospace; padding: 2px 6px; color: #6abf69;")
        elif ok is False:
            w.setStyleSheet("font-family: monospace; padding: 2px 6px; color: #e57373;")

    def _set_estop(self, val: bool):
        if val:
            self.fields["estop"].setText("ENGAGED")
            self.fields["estop"].setStyleSheet("font-family: monospace; padding: 2px 6px; color: white; background: #c62828; font-weight: bold;")
        else:
            self.fields["estop"].setText("clear")
            self.fields["estop"].setStyleSheet("font-family: monospace; padding: 2px 6px; color: #6abf69;")

    def _poll_jetson(self):
        user, host = self.get_conn()
        # Ping
        def worker():
            rc = subprocess.run(["ping", "-c", "1", "-W", "1", host], capture_output=True).returncode
            # Guard every cross-thread invoke: when SSH stalls on a flaky
            # WiFi link and the user closes the GUI, the worker returns
            # after HealthPane is destroyed → invokeMethod on a dead Qt
            # object raises RuntimeError. Swallow it.
            try:
                QtCore.QMetaObject.invokeMethod(
                    self, "_apply_jetson", QtCore.Qt.QueuedConnection,
                    QtCore.Q_ARG(bool, rc == 0),
                )
            except RuntimeError:
                return
            # node count
            rc2, out = ssh_oneshot(user, host,
                "source /opt/ros/humble/setup.bash && ros2 node list 2>/dev/null | wc -l",
                timeout=4.0)
            count = (out.strip().splitlines() or ["?"])[-1] if rc2 == 0 else "?"
            try:
                QtCore.QMetaObject.invokeMethod(
                    self, "_apply_nodes", QtCore.Qt.QueuedConnection,
                    QtCore.Q_ARG(str, count),
                )
            except RuntimeError:
                return
        threading.Thread(target=worker, daemon=True).start()

    @QtCore.pyqtSlot(bool)
    def _apply_jetson(self, reachable: bool):
        self._set("jetson", "reachable" if reachable else "UNREACHABLE", ok=reachable)

    @QtCore.pyqtSlot(str)
    def _apply_nodes(self, count: str):
        try:
            n = int(count)
            self._set("nodes", f"{n} nodes", ok=(n > 0))
        except Exception:
            self._set("nodes", count, neutral=True)


# ------------------------------------------------------------------ #
# Debug terminal pane — ad-hoc remote commands
# ------------------------------------------------------------------ #
class TerminalPane(QtWidgets.QWidget):
    def __init__(self, get_conn, parent=None):
        super().__init__(parent)
        self.get_conn = get_conn
        self.proc: QtCore.QProcess = None
        self.history = deque(maxlen=100)
        self.hist_idx = -1

        root = QtWidgets.QVBoxLayout(self)
        root.addWidget(QtWidgets.QLabel("<b>Debug Terminal</b>  &nbsp; (runs over SSH on the Jetson — auto-sources ROS2)"))

        self.out = QtWidgets.QPlainTextEdit()
        self.out.setReadOnly(True)
        self.out.setMaximumBlockCount(20000)
        self.out.setStyleSheet("font-family: monospace; background: #0a0a0a; color: #ccc;")
        root.addWidget(self.out, 1)

        row = QtWidgets.QHBoxLayout()
        self.prompt = QtWidgets.QLabel("$")
        self.prompt.setStyleSheet("font-family: monospace; color: #6abf69; padding: 0 4px;")
        row.addWidget(self.prompt)
        self.input = QtWidgets.QLineEdit()
        self.input.setStyleSheet("font-family: monospace;")
        self.input.returnPressed.connect(self._run)
        self.input.installEventFilter(self)
        row.addWidget(self.input, 1)
        self.cancel_btn = QtWidgets.QPushButton("Cancel")
        self.cancel_btn.clicked.connect(self._cancel)
        row.addWidget(self.cancel_btn)
        root.addLayout(row)

        # Quick buttons
        quick = QtWidgets.QHBoxLayout()
        for label, cmd in [
            ("topic list",  "ros2 topic list"),
            ("node list",   "ros2 node list"),
            ("pico status", "timeout 1 ros2 topic echo --once /pico/status"),
            ("mission",     "timeout 1 ros2 topic echo --once /mission/status"),
            ("hz lidar",    "timeout 3 ros2 topic hz /unilidar/cloud"),
            ("clear estop", "ros2 topic pub --once /estop std_msgs/msg/Bool '{data: false}' && ros2 topic pub --once /safety/estop std_msgs/msg/Bool '{data: false}'"),
        ]:
            b = QtWidgets.QPushButton(label)
            b.clicked.connect(lambda _, c=cmd: self._quick(c))
            quick.addWidget(b)
        root.addLayout(quick)

    def eventFilter(self, obj, event):
        if obj is self.input and event.type() == QtCore.QEvent.KeyPress:
            if event.key() == QtCore.Qt.Key_Up:
                if self.history and self.hist_idx > 0:
                    self.hist_idx -= 1
                    self.input.setText(self.history[self.hist_idx])
                return True
            if event.key() == QtCore.Qt.Key_Down:
                if self.history and self.hist_idx < len(self.history) - 1:
                    self.hist_idx += 1
                    self.input.setText(self.history[self.hist_idx])
                elif self.history:
                    self.hist_idx = len(self.history)
                    self.input.clear()
                return True
        return super().eventFilter(obj, event)

    def _quick(self, cmd: str):
        self.input.setText(cmd)
        self._run()

    def _run(self):
        cmd = self.input.text().strip()
        if not cmd:
            return
        self.history.append(cmd); self.hist_idx = len(self.history)
        self.input.clear()
        self._append(f"$ {cmd}", color="#6abf69")
        if self.proc and self.proc.state() != QtCore.QProcess.NotRunning:
            self._append("(another command is running — click Cancel first)", color="#e57373")
            return
        user, host = self.get_conn()
        # Source ROS2 in the same shell, then run the user's command
        remote = (
            "source /opt/ros/humble/setup.bash 2>/dev/null; "
            f"[ -f {WORKSPACE}/install/setup.bash ] && source {WORKSPACE}/install/setup.bash 2>/dev/null; "
            f"cd {WORKSPACE} 2>/dev/null; "
            f"{cmd}"
        )
        self.proc = QtCore.QProcess(self)
        self.proc.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        self.proc.readyReadStandardOutput.connect(self._on_out)
        self.proc.finished.connect(self._on_done)
        args = ssh_args(user, host, remote, tty=False)
        self.proc.start(args[0], args[1:])

    def _on_out(self):
        data = bytes(self.proc.readAllStandardOutput()).decode("utf-8", errors="replace")
        self._append(data.rstrip("\n"))

    def _on_done(self, code, status):
        self._append(f"(exit {code})", color="#888")

    def _cancel(self):
        if self.proc and self.proc.state() != QtCore.QProcess.NotRunning:
            self.proc.terminate()
            if not self.proc.waitForFinished(1500):
                self.proc.kill()
            self._append("(cancelled)", color="#e57373")

    def _append(self, text: str, color: str = None):
        if color:
            self.out.appendHtml(f'<pre style="color:{color};margin:0;">{QtGui.QGuiApplication.instance().translate("", text)}</pre>')
        else:
            self.out.appendPlainText(text)
        self.out.verticalScrollBar().setValue(self.out.verticalScrollBar().maximum())


# ------------------------------------------------------------------ #
# Main window
# ------------------------------------------------------------------ #
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PFW Lunabotics — Mission Control")
        # Reasonable initial size — gets overridden by showMaximized() in main()
        self.resize(1500, 950)
        # Allow content to scroll if a pane gets resized smaller than its
        # natural minimum (prevents widgets from being clipped off-screen).
        self.setMinimumSize(900, 600)

        # IMPORTANT: do NOT call self.ros.start() here — start() emits a
        # log_line signal which routes through _log_to_autonomy and would
        # touch self.autonomy before AutonomyPane has been constructed
        # (causes AttributeError on launch). We instantiate the worker now
        # but only start it after every pane is wired up.
        self.ros = RosWorker()
        self.autonomy = None  # sentinel so early log emits don't crash
        self.ros.log_line.connect(self._log_to_autonomy)

        # Central layout: top E-STOP bar + 2x2 grid of resizable panes
        central = QtWidgets.QWidget(); self.setCentralWidget(central)
        v = QtWidgets.QVBoxLayout(central); v.setContentsMargins(6, 6, 6, 6)

        # Top bar
        topbar = QtWidgets.QHBoxLayout()
        self.estop_btn = QtWidgets.QPushButton("⏹  E-STOP")
        self.estop_btn.setStyleSheet(
            "background: #b71c1c; color: white; font-size: 22px; font-weight: bold; "
            "padding: 14px 30px; border: 3px solid #5a0000; border-radius: 6px;"
        )
        self.estop_btn.setCheckable(True)
        self.estop_btn.toggled.connect(self._toggle_estop)
        topbar.addWidget(self.estop_btn)

        self.estop_state = QtWidgets.QLabel("not engaged")
        self.estop_state.setStyleSheet("padding: 0 12px; font-weight: bold; color: #6abf69;")
        topbar.addWidget(self.estop_state)

        # DDS data-flow indicator. Goes green when ANY subscribed topic
        # actually delivers a message — confirms transport (not just
        # discovery) is working VM <-> Jetson.
        self.dds_light = QtWidgets.QLabel("DDS: waiting")
        self.dds_light.setStyleSheet("padding: 0 12px; font-weight: bold; color: #888;")
        topbar.addWidget(self.dds_light)
        self._dds_last_rx = 0.0
        self._dds_msg_count = 0

        # Mission state badge (was buried in the AutonomyPane; put it up here
        # so the operator always sees the current state during a run).
        self.state_badge = QtWidgets.QLabel("state: —")
        self.state_badge.setStyleSheet(
            "font-family: monospace; padding: 4px 10px; "
            "background: #222; color: #ddd; border-radius: 3px;"
        )
        topbar.addWidget(self.state_badge)

        topbar.addStretch(1)

        topbar.addWidget(QtWidgets.QLabel("Jetson:"))
        self.user_in = QtWidgets.QLineEdit(JETSON_USER); self.user_in.setMaximumWidth(120)
        self.host_in = QtWidgets.QLineEdit(JETSON_HOST); self.host_in.setMaximumWidth(150)
        topbar.addWidget(self.user_in); topbar.addWidget(QtWidgets.QLabel("@"))
        topbar.addWidget(self.host_in)

        v.addLayout(topbar)

        # 2x2 splitter grid
        outer = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        top_row = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        bot_row = QtWidgets.QSplitter(QtCore.Qt.Horizontal)

        self.autonomy = AutonomyPane(self.ros, self._get_conn)
        self.teleop   = TeleopPane(self.ros)
        self.health   = HealthPane(self.ros, self._get_conn)
        self.terminal = TerminalPane(self._get_conn)

        # Wrap each pane in a group box + scroll area. The scroll area means
        # that if the user shrinks a pane below its natural minimum, the
        # widgets stay reachable via scrollbars instead of being clipped off.
        def wrap(name, widget):
            box = QtWidgets.QGroupBox(name)
            inner = QtWidgets.QVBoxLayout(box)
            inner.setContentsMargins(6, 6, 6, 6)
            scroll = QtWidgets.QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
            scroll.setWidget(widget)
            inner.addWidget(scroll)
            return box

        top_row.addWidget(wrap("1 · Autonomous", self.autonomy))
        top_row.addWidget(wrap("2 · Manual Teleop", self.teleop))
        bot_row.addWidget(wrap("3 · Robot Health", self.health))
        bot_row.addWidget(wrap("4 · Debug Terminal", self.terminal))
        outer.addWidget(top_row); outer.addWidget(bot_row)
        outer.setSizes([550, 380])
        top_row.setSizes([800, 700])
        bot_row.setSizes([550, 950])

        v.addWidget(outer, 1)

        # E-stop signal
        self.ros.estop_state_changed.connect(self._sync_estop_button)

        # Top-bar mission state badge
        self.ros.mission_state_changed.connect(
            lambda s: self.state_badge.setText(f"state: {s}")
        )

        # DDS data-flow indicator — every received message bumps a counter.
        # A 1Hz timer checks freshness and flips colour.
        self.ros.data_received.connect(self._on_data_received)
        self._dds_timer = QtCore.QTimer(self)
        self._dds_timer.setInterval(1000)
        self._dds_timer.timeout.connect(self._refresh_dds_light)
        self._dds_timer.start()

        # All panes exist now — safe to start the ROS worker (it emits a
        # log_line signal on init that lands in the Autonomous pane).
        self.ros.start()

    def _get_conn(self):
        return self.user_in.text().strip() or JETSON_USER, self.host_in.text().strip() or JETSON_HOST

    def log(self, text: str):
        self._log_to_autonomy(text)

    def _log_to_autonomy(self, text: str):
        # Defensive: log signals can fire before AutonomyPane is constructed
        # (e.g. RosWorker.start() emits before __init__ finishes wiring panes).
        if getattr(self, "autonomy", None) is None:
            print(f"[pre-init] {text}")
            return
        self.autonomy._append(text)

    # -- E-STOP ----------------------------------------------------- #
    def _toggle_estop(self, checked: bool):
        if checked:
            self.estop_btn.setText("⏹  E-STOP  (ENGAGED — click to clear)")
            self.estop_state.setText("ENGAGED"); self.estop_state.setStyleSheet("padding: 0 12px; color: #e57373; font-weight: bold;")
            self.teleop.stop_motion()
            self.ros.publish_estop(True)
            self.log("[E-STOP] published true to /estop AND /safety/estop")
        else:
            self.estop_btn.setText("⏹  E-STOP")
            self.estop_state.setText("not engaged"); self.estop_state.setStyleSheet("padding: 0 12px; color: #6abf69; font-weight: bold;")
            self.ros.publish_estop(False)
            self.log("[E-STOP] cleared on /estop AND /safety/estop")

    # -- DDS data-flow light --------------------------------------- #
    def _on_data_received(self):
        self._dds_last_rx = time.monotonic()
        self._dds_msg_count += 1

    def _refresh_dds_light(self):
        if self._dds_last_rx == 0.0:
            self.dds_light.setText("DDS: waiting for data")
            self.dds_light.setStyleSheet("padding: 0 12px; font-weight: bold; color: #888;")
            return
        age = time.monotonic() - self._dds_last_rx
        if age < 3.0:
            self.dds_light.setText(f"DDS: OK ({self._dds_msg_count} msgs)")
            self.dds_light.setStyleSheet("padding: 0 12px; font-weight: bold; color: #6abf69;")
        elif age < 10.0:
            self.dds_light.setText(f"DDS: stale ({int(age)}s)")
            self.dds_light.setStyleSheet("padding: 0 12px; font-weight: bold; color: #d68a00;")
        else:
            self.dds_light.setText(f"DDS: LOST ({int(age)}s)")
            self.dds_light.setStyleSheet("padding: 0 12px; font-weight: bold; color: #e57373;")

    def _sync_estop_button(self, val: bool):
        # Reflect external e-stop changes without re-firing the publish
        if val != self.estop_btn.isChecked():
            self.estop_btn.blockSignals(True)
            self.estop_btn.setChecked(val)
            if val:
                self.estop_btn.setText("⏹  E-STOP  (ENGAGED — click to clear)")
                self.estop_state.setText("ENGAGED"); self.estop_state.setStyleSheet("padding: 0 12px; color: #e57373; font-weight: bold;")
            else:
                self.estop_btn.setText("⏹  E-STOP")
                self.estop_state.setText("not engaged"); self.estop_state.setStyleSheet("padding: 0 12px; color: #6abf69; font-weight: bold;")
            self.estop_btn.blockSignals(False)

    def keyPressEvent(self, e: QtGui.QKeyEvent):
        # Global ESC = E-STOP toggle
        if e.key() == QtCore.Qt.Key_Escape:
            self.estop_btn.toggle()
            e.accept(); return
        super().keyPressEvent(e)

    def closeEvent(self, e):
        # Order matters: stop Qt timers FIRST so they don't fire publish
        # calls into an rclpy node that's about to be destroyed (which
        # raised "InvalidHandle: cannot use Destroyable" every shutdown).
        try:
            self.teleop.timer.stop()
        except Exception:
            pass
        try:
            self.health.ping_timer.stop()
            self.health._cmdvel_timer.stop()
        except Exception:
            pass
        # Kill any in-flight SSH QProcess in the terminal pane so we don't
        # leak the child process on exit ("QProcess: Destroyed while process
        # ssh is still running").
        try:
            tp = getattr(self, "terminal", None)
            if tp and tp.proc and tp.proc.state() != QtCore.QProcess.NotRunning:
                tp.proc.terminate()
                if not tp.proc.waitForFinished(800):
                    tp.proc.kill()
        except Exception:
            pass
        try:
            self.teleop.stop_motion()
        except Exception:
            pass
        try:
            self.ros.shutdown()
        except Exception:
            pass
        super().closeEvent(e)


def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    win = MainWindow()
    # Start maximized so the four panes fill the visible screen on launch.
    # If you want true fullscreen (no window chrome), set FULLSCREEN=1 env.
    if os.environ.get("FULLSCREEN", "0") == "1":
        win.showFullScreen()
    else:
        win.showMaximized()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
