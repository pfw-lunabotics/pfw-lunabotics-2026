#!/usr/bin/env python3
"""
wander_test.py — Simple obstacle avoidance test node
======================================================
Sends the robot on random nearby goals for a set duration.
Nav2 handles path planning and obstacle avoidance via the costmap.
If a goal fails (blocked), it picks a new random goal.

Usage:
  ros2 run lunabotics_navigation wander_test --ros-args -p duration:=60.0 -p radius:=1.5
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import random
import math
import time


class WanderTest(Node):
    def __init__(self):
        super().__init__('wander_test')

        self.declare_parameter('duration', 60.0)
        self.declare_parameter('radius', 1.5)
        self.declare_parameter('min_radius', 0.5)

        self.duration = self.get_parameter('duration').value
        self.radius = self.get_parameter('radius').value
        self.min_radius = self.get_parameter('min_radius').value

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info(f'Wander test: duration={self.duration}s, radius={self.radius}m')
        self.get_logger().info('Waiting for Nav2 action server...')

        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 ready. Starting wander.')

        self.start_time = time.time()
        self.goal_count = 0
        self.send_random_goal()

    def send_random_goal(self):
        elapsed = time.time() - self.start_time
        if elapsed >= self.duration:
            self.get_logger().info(
                f'Wander complete. {self.goal_count} goals sent in {elapsed:.0f}s.'
            )
            raise SystemExit(0)

        # Random goal in robot's odom frame (relative to current position origin)
        angle = random.uniform(0, 2 * math.pi)
        dist = random.uniform(self.min_radius, self.radius)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'odom'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = dist * math.cos(angle)
        goal_msg.pose.pose.position.y = dist * math.sin(angle)
        goal_msg.pose.pose.position.z = 0.0

        # Random orientation at goal
        yaw = random.uniform(-math.pi, math.pi)
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.goal_count += 1
        self.get_logger().info(
            f'Goal #{self.goal_count}: ({goal_msg.pose.pose.position.x:.2f}, '
            f'{goal_msg.pose.pose.position.y:.2f}) [{elapsed:.0f}s/{self.duration:.0f}s]'
        )

        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected, trying another...')
            self.send_random_goal()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result()
        status = result.status
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Goal reached! Sending next...')
        else:
            self.get_logger().warn(f'Goal ended with status {status}, trying new goal...')

        # Small pause then next goal
        self.create_timer(1.0, self._next_goal_once)

    def _next_goal_once(self):
        # Cancel the timer after one shot
        self.send_random_goal()


def main(args=None):
    rclpy.init(args=args)
    node = WanderTest()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
