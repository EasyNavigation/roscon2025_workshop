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

"""Waypoint client node for Nav2."""

from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import FollowWaypoints
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


class WaypointClientNode(Node):
    """Waypoint client node for following waypoints."""

    def __init__(self):
        """Initialize the waypoint client node."""
        super().__init__('waypoint_client_node')

        # Declare parameters
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('waypoints', [''])

        self.waypoints = []
        self.restart_timer = None

        # Load waypoints
        self.load_waypoints_from_params()

        # Create action client
        self.action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        if not self.waypoints:
            self.get_logger().warn('No waypoints loaded from parameters.')
            return
        self.send_goal()

    def load_waypoints_from_params(self):
        """Load waypoints from ROS parameters."""
        self.waypoints.clear()

        waypoint_names = self.get_parameter('waypoints').value
        frame_id = self.get_parameter('frame_id').value

        if not waypoint_names:
            self.get_logger().error('No waypoints provided in "waypoints" parameter')
            return

        self.get_logger().info(f'Loading {len(waypoint_names)} waypoints')

        for wp_name in waypoint_names:
            # Declare and get waypoint parameter
            self.declare_parameter(wp_name, [0.0])
            wp_coord = self.get_parameter(wp_name).value

            if len(wp_coord) != 3:
                self.get_logger().error(
                    f"Waypoint '{wp_name}' has wrong size {len(wp_coord)} (expected 3: x, y, yaw)")
                continue

            # Create PoseStamped message
            waypoint = PoseStamped()
            waypoint.header.frame_id = frame_id
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
            self.get_logger().error('No valid waypoints loaded.')
        else:
            self.get_logger().info(f'Successfully loaded {len(self.waypoints)} valid waypoints')

    def send_goal(self):
        """Send waypoints to follow_waypoints action server."""
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('FollowWaypoints action server not available')
            return

        # Create goal message
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = self.waypoints

        # Send goal with callbacks
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal response from action server."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by server')
            return

        self.get_logger().info('Goal accepted, waiting for result')

        # Get result
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle feedback from action server."""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Current waypoint index: {feedback.current_waypoint}',
            throttle_duration_sec=2.0)

    def result_callback(self, future):
        """Handle result from action server."""
        result = future.result()
        status = result.status

        if status == 4:  # SUCCEEDED
            if not result.result.missed_waypoints:
                self.get_logger().info('Successfully visited all waypoints')
            else:
                self.get_logger().warn(
                    f'Missed {len(result.result.missed_waypoints)} waypoints')
        elif status == 5:  # ABORTED
            self.get_logger().error('Goal aborted')
        elif status == 6:  # CANCELED
            self.get_logger().warn('Goal canceled')
        else:
            self.get_logger().error('Unknown result code')

    def restart_callback(self):
        """Timer callback to restart sending goals."""
        if self.restart_timer:
            self.restart_timer.cancel()
            self.restart_timer = None
        self.send_goal()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointClientNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
