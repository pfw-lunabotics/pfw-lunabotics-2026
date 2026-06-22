#!/usr/bin/env python3
"""
waypoint_navigator.py — Lunabotics 2026 Competition Waypoint Sequencer
=======================================================================
Sends NavigateToPose goals to Nav2 in competition order:
  Start (excavation zone) → obstacle zone → construction zone → berm target

Services:
  /mission/start  (std_srvs/Trigger) — begin waypoint sequence
  /mission/stop   (std_srvs/Trigger) — abort current mission
  /mission/reset  (std_srvs/Trigger) — reset to waypoint 0

Topics published:
  /mission/status         (std_msgs/String)       — human-readable progress updates
  /mission/waypoints_viz  (visualization_msgs/MarkerArray) — waypoint spheres + labels for RViz

Parameters:
  start_position (str, default 'lower') — 'lower' or 'upper' start box
  auto_start     (bool, default False)  — begin mission 5 s after node starts
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, ColorRGBA
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


# ---------------------------------------------------------------------------
# Competition waypoints (UCF arena, single-team half)
# Arena: X ∈ [-4.07, 4.07]  Y ∈ [-2.285, 2.285]
#   Construction zone:  X ∈ [-4.07, -2.07]
#   Obstacle zone:      X ∈ [-2.07,  0.07]
#   Excavation zone:    X ∈ [ 0.07,  4.07]
#
# Each waypoint: (x, y, yaw_degrees)
# Nav2 handles local obstacle avoidance — waypoints just need to be in the
# right zone; the DWB controller will slalom around boulders/craters.
# ---------------------------------------------------------------------------

# Lower start (Y < 0 half of arena)
WAYPOINTS_LOWER = [
    # (x,    y,     yaw_deg,  label)
    ( 2.00, -0.50,  180.0, "mid-excavation"),
    (-0.50,  0.50,  180.0, "enter obstacle zone"),
    (-1.80, -0.50,  180.0, "cross obstacle zone"),
    (-3.00, -0.50,  180.0, "enter construction zone"),
    (-3.07, -1.30,   90.0, "lower berm target"),
]

# Upper start (Y > 0 half of arena)
WAYPOINTS_UPPER = [
    ( 2.00,  0.50,  180.0, "mid-excavation"),
    (-0.50, -0.50,  180.0, "enter obstacle zone"),
    (-1.80,  0.50,  180.0, "cross obstacle zone"),
    (-3.00,  0.50,  180.0, "enter construction zone"),
    (-3.07,  1.30,  -90.0, "upper berm target"),
]


def _make_pose(x, y, yaw_deg):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    yaw = math.radians(yaw_deg)
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


class WaypointNavigator(Node):

    def __init__(self):
        super().__init__('waypoint_navigator')

        self.declare_parameter('start_position', 'lower')
        self.declare_parameter('auto_start', False)

        start_pos  = self.get_parameter('start_position').value
        auto_start = self.get_parameter('auto_start').value

        raw_wps = WAYPOINTS_LOWER if start_pos == 'lower' else WAYPOINTS_UPPER
        self._waypoints = [(_make_pose(x, y, yaw), label)
                           for x, y, yaw, label in raw_wps]

        self._current_wp  = 0
        self._running     = False
        self._goal_handle = None

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self._status_pub   = self.create_publisher(String,      '/mission/status',        10)
        self._markers_pub  = self.create_publisher(MarkerArray, '/mission/waypoints_viz', 10)

        # Publish markers at 1 Hz so RViz picks them up even before mission starts
        self.create_timer(1.0, self._publish_markers)

        self.create_service(Trigger, '/mission/start', self._start_cb)
        self.create_service(Trigger, '/mission/stop',  self._stop_cb)
        self.create_service(Trigger, '/mission/reset', self._reset_cb)

        if auto_start:
            self._auto_timer = self.create_timer(5.0, self._auto_start_cb)

        self.get_logger().info(
            f'WaypointNavigator ready — {len(self._waypoints)} waypoints '
            f'({start_pos} start). Call /mission/start to begin.'
        )

    # ------------------------------------------------------------------
    # Service callbacks
    # ------------------------------------------------------------------

    def _start_cb(self, _req, res):
        if self._running:
            res.success = False
            res.message = 'Mission already running'
            return res
        self._running = True
        self._publish(f'Mission started ({len(self._waypoints)} waypoints)')
        self._navigate_next()
        res.success = True
        res.message = f'Mission started — {len(self._waypoints)} waypoints'
        return res

    def _stop_cb(self, _req, res):
        self._running = False
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        self._publish('Mission stopped')
        res.success = True
        res.message = 'Mission stopped'
        return res

    def _reset_cb(self, _req, res):
        self._running    = False
        self._current_wp = 0
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        self._publish('Mission reset to waypoint 1')
        res.success = True
        res.message = 'Mission reset'
        return res

    # ------------------------------------------------------------------
    # Navigation
    # ------------------------------------------------------------------

    def _navigate_next(self):
        if not self._running:
            return

        if self._current_wp >= len(self._waypoints):
            self._publish('Mission complete — all waypoints reached')
            self._running = False
            return

        pose, label = self._waypoints[self._current_wp]
        pose.header.stamp = self.get_clock().now().to_msg()

        self._publish(
            f'WP {self._current_wp + 1}/{len(self._waypoints)}: '
            f'{label}  ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})'
        )

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self._publish('Nav2 not ready — retrying in 2 s')
            self.create_timer(2.0, self._retry_navigate)
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose
        send_future = self._nav_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_cb,
        )
        send_future.add_done_callback(self._goal_accepted_cb)

    def _retry_navigate(self):
        # One-shot retry — destroy timer by not storing it; just call directly
        self._navigate_next()

    def _goal_accepted_cb(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle or not self._goal_handle.accepted:
            self._publish(
                f'WP {self._current_wp + 1} rejected by Nav2 — mission aborted'
            )
            self._running = False
            return
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        if not self._running:
            return
        self._goal_handle = None
        self._current_wp += 1
        self._navigate_next()

    def _feedback_cb(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        if dist > 0.1:
            self.get_logger().debug(f'  distance remaining: {dist:.2f} m')

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _publish_markers(self):
        """Publish MarkerArray showing waypoint status — grey/yellow/green."""
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()

        for i, (pose, label) in enumerate(self._waypoints):
            # Colour: green=done, yellow=current, grey=pending
            if i < self._current_wp:
                color = ColorRGBA(r=0.1, g=0.8, b=0.1, a=0.8)   # green — done
            elif i == self._current_wp and self._running:
                color = ColorRGBA(r=1.0, g=0.8, b=0.0, a=1.0)   # yellow — active
            else:
                color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.6)   # grey — pending

            # Sphere at waypoint position
            sphere = Marker()
            sphere.header.frame_id = 'map'
            sphere.header.stamp    = now
            sphere.ns     = 'waypoints'
            sphere.id     = i
            sphere.type   = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose   = pose.pose
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.30
            sphere.color  = color
            arr.markers.append(sphere)

            # Text label above sphere
            text = Marker()
            text.header.frame_id = 'map'
            text.header.stamp    = now
            text.ns     = 'waypoint_labels'
            text.id     = i
            text.type   = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose   = pose.pose
            text.pose.position.z += 0.45
            text.scale.z = 0.18
            text.color   = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text    = f'{i + 1}: {label}'
            arr.markers.append(text)

        self._markers_pub.publish(arr)

    def _publish(self, msg: str):
        self.get_logger().info(msg)
        out = String()
        out.data = msg
        self._status_pub.publish(out)

    def _auto_start_cb(self):
        self._auto_timer.cancel()
        self._publish('Auto-start triggered')
        req = Trigger.Request()
        res = Trigger.Response()
        self._start_cb(req, res)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
