#!/usr/bin/env python3

# Copyright 2025 Intelligent Robotics Lab
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

"""Patrolling node for Nav2."""

from enum import Enum

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from tf_transformations import quaternion_from_euler


def orientation_around_z_axis(angle):
    """Convert yaw angle to quaternion."""
    q = quaternion_from_euler(0, 0, angle)
    quaternion = Quaternion()
    quaternion.x = q[0]
    quaternion.y = q[1]
    quaternion.z = q[2]
    quaternion.w = q[3]
    return quaternion


class PatrolState(Enum):
    """Enum for patrol states."""

    IDLE = 0
    SENDING_GOAL = 1
    NAVIGATING = 2
    FINISHED = 3
    ERROR = 4


class PatrollingNode(Node):
    """Patrolling node for Nav2."""

    def __init__(self):
        """Initialize the PatrollingNode."""
        super().__init__('patrolling_node')

        self.state = PatrolState.IDLE
        self.initialized = False
        self.current_waypoint_index = 0
        self.frame_id = ''
        self.waypoints = []
        self.nav_client = None
        self.current_goal_handle = None
        self.send_goal_future = None

        self.timer = self.create_timer(0.1, self.cycle)

    def initialize(self):
        """Initialize waypoints from parameters."""
        # Declare and get waypoint names
        self.declare_parameter('waypoints', [''])
        waypoint_names = self.get_parameter('waypoints').value

        self.declare_parameter('frame_id', 'map')
        self.frame_id = self.get_parameter('frame_id').value

        if not waypoint_names:
            self.get_logger().error('No waypoints provided in "waypoints" parameter')
            return

        self.get_logger().info(f'Loading {len(waypoint_names)} waypoints')

        # Load each waypoint's coordinates
        for wp_name in waypoint_names:
            self.declare_parameter(wp_name, [0.0])
            wp_coord = self.get_parameter(wp_name).value

            if len(wp_coord) != 3:
                self.get_logger().error(
                    f"Waypoint '{wp_name}' has wrong size {len(wp_coord)} "
                    '(expected 3: x, y, yaw)')
                continue

            waypoint = PoseStamped()
            waypoint.header.frame_id = self.frame_id
            waypoint.header.stamp = self.get_clock().now().to_msg()
            waypoint.pose.position.x = float(wp_coord[0])
            waypoint.pose.position.y = float(wp_coord[1])
            waypoint.pose.position.z = 0.0
            waypoint.pose.orientation = orientation_around_z_axis(float(wp_coord[2]))

            self.waypoints.append(waypoint)
            self.get_logger().info(
                f"Loaded waypoint '{wp_name}': "
                f'[{wp_coord[0]:.2f}, {wp_coord[1]:.2f}, {wp_coord[2]:.2f} rad]')

        if not self.waypoints:
            self.get_logger().error('No valid waypoints loaded. Cannot patrol.')
        else:
            self.get_logger().info(
                f'Successfully loaded {len(self.waypoints)} valid waypoints')

    def cycle(self):
        """Run main state machine cycle."""
        if self.state == PatrolState.IDLE:
            if not self.initialized:
                self.get_logger().info('Initializing patrolling node')

                # Create Nav2 action client
                self.nav_client = ActionClient(
                    self, NavigateToPose, 'navigate_to_pose')

                self.initialize()
                self.initialized = True

                if not self.waypoints:
                    self.get_logger().error('Cannot start patrol with no waypoints')
                    self.state = PatrolState.ERROR
                    return

            # Wait for action server
            if not self.nav_client.server_is_ready():
                self.get_logger().info(
                    'Waiting for navigate_to_pose action server...',
                    throttle_duration_sec=5.0)
                return

            self.get_logger().info(f'Starting patrol with {len(self.waypoints)} waypoints')
            self.current_waypoint_index = 0
            self.state = PatrolState.SENDING_GOAL

        elif self.state == PatrolState.SENDING_GOAL:

            # Send new goal
            if self.current_waypoint_index >= len(self.waypoints):
                self.get_logger().info('All waypoints visited')
                self.state = PatrolState.FINISHED
                return

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = self.waypoints[self.current_waypoint_index]
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

            self.get_logger().info(
                f'Sending goal {self.current_waypoint_index + 1}/{len(self.waypoints)}: '
                f'[{goal_msg.pose.pose.position.x:.2f}, '
                f'{goal_msg.pose.pose.position.y:.2f}]')

            self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
            self.state = PatrolState.NAVIGATING

        elif self.state == PatrolState.NAVIGATING:
            # Get the current goal handle
            if not self.send_goal_future:
                self.get_logger().error('No goal handle available')
                self.state = PatrolState.ERROR
                return

            self.current_goal_handle = self.send_goal_future.result()
            status = self.current_goal_handle.status

            # Handle different goal statuses
            if status == GoalStatus.STATUS_ACCEPTED:
                # Goal has been accepted but not yet executing
                self.get_logger().info(
                    'Goal accepted, waiting to start execution...',
                    throttle_duration_sec=2.0)

            elif status == GoalStatus.STATUS_EXECUTING:
                self.get_logger().info(
                    f'Navigating to waypoint {self.current_waypoint_index + 1}/'
                    f'{len(self.waypoints)}...',
                    throttle_duration_sec=2.0)

            elif status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(
                    f'Successfully reached waypoint {self.current_waypoint_index + 1}/'
                    f'{len(self.waypoints)}')
                self.current_waypoint_index += 1
                self.current_goal_handle = None
                self.state = PatrolState.SENDING_GOAL
            else:
                self.get_logger().error(f'Unexpected goal status: {status}')
                self.state = PatrolState.ERROR

        elif self.state == PatrolState.FINISHED:
            self.get_logger().info('Patrol cycle completed. Restarting from first waypoint.')
            self.current_waypoint_index = 0
            self.state = PatrolState.SENDING_GOAL

        elif self.state == PatrolState.ERROR:
            # Stay in error state (could add recovery logic here)
            pass


def main(args=None):
    rclpy.init(args=args)
    node = PatrollingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
