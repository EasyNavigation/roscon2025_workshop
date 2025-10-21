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

"""Test for patrolling launch pipeline."""

import os
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import rclpy
from rclpy.node import Node


# Global test configuration - can be set by different test description generators
_test_config = {
    'use_python': False,
    'use_waypoint': False
}


def generate_test_description():
    """
    Generate launch description for patrolling test.

    This will launch:
    1. playground_kobuki.launch.py (simulation with Kobuki robot)
    2. navigation_launch.py (Nav2 stack)
    3. patrol_launch.py (Patrolling node - launched after a delay for navigation to be ready)
    """
    nav2_playground_dir = get_package_share_directory('nav2_playground')

    # Get test parameters from global config
    use_python = _test_config['use_python']
    use_waypoint = _test_config['use_waypoint']

    # Launch playground (Gazebo + Kobuki)
    playground_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_playground_dir, 'launch', 'playground_kobuki.launch.py')
        ),
        launch_arguments={'gui': 'false'}.items()  # Headless for faster testing
    )

    # Launch navigation
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_playground_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={'use_rviz': 'false'}.items()
    )

    # Launch patrolling - delayed to give navigation time to initialize
    # The test itself will verify navigation is ready before checking patrol functionality
    patrol_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_playground_dir, 'launch', 'patrolling_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': os.path.join(nav2_playground_dir, 'config', 'patrolling_params.yaml'),
            'use_python': 'true' if use_python else 'false',
            'use_waypoint': 'true' if use_waypoint else 'false'
        }.items()
    )

    # Delay patrol launch by 20 seconds to allow navigation stack to fully initialize
    patrol_launch_delayed = TimerAction(
        period=20.0,
        actions=[patrol_launch]
    )

    return LaunchDescription([
        playground_launch,
        navigation_launch,
        patrol_launch_delayed,
        launch_testing.actions.ReadyToTest()
    ])


class TestPatrolLaunch(unittest.TestCase):
    """Test that patrolling node launches and starts navigation."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS context for all tests."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS context after all tests."""
        rclpy.shutdown()

    def setUp(self):
        """Set up test node."""
        self.node = Node('test_patrol_node')

    def tearDown(self):
        """Clean up test node."""
        self.node.destroy_node()

    def test_patrol_node_running(self, proc_output):
        """
        Test that patrolling node is running.

        This verifies that:
        1. Playground (Gazebo) is running
        2. Navigation stack is running
        3. Patrolling node is running
        """
        # Wait for node to appear
        timeout_sec = 60.0
        start_time = self.node.get_clock().now()

        node_found = False
        expected_node_name = 'patrolling_node'

        while not node_found:
            # Get list of nodes
            node_names = self.node.get_node_names()

            if expected_node_name in node_names:
                node_found = True
                break

            rclpy.spin_once(self.node, timeout_sec=0.1)

            # Check timeout
            elapsed = (self.node.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > timeout_sec:
                self.fail(
                    f'Timeout waiting for {expected_node_name} node after {timeout_sec} seconds. '
                    f'Available nodes: {node_names}')

        self.assertTrue(node_found, f'Patrolling node {expected_node_name} not found')

    def test_navigation_action_available(self, proc_output):
        """
        Test that navigation action server is available.

        This verifies that Nav2 is ready to accept goals.
        """
        import time
        from rclpy.action import ActionClient
        from nav2_msgs.action import NavigateToPose

        # Create action client
        action_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

        # Wait for action server
        timeout_sec = 60.0
        start_time = time.time()

        server_available = False
        while not server_available:
            server_available = action_client.wait_for_server(timeout_sec=1.0)

            if server_available:
                break

            elapsed = time.time() - start_time
            if elapsed > timeout_sec:
                self.fail(
                    f'Timeout waiting for navigate_to_pose action server '
                    f'after {timeout_sec} seconds')

        self.assertTrue(server_available, 'Navigate to pose action server not available')

        # Clean up
        action_client.destroy()

    def test_robot_moving(self, proc_output):
        """
        Test that robot is receiving velocity commands.

        This verifies that:
        1. The /cmd_vel topic is being published
        2. The robot is actually moving (non-zero velocities)
        """
        # Flag to track if we received cmd_vel
        cmd_vel_received = False
        movement_detected = False

        def cmd_vel_callback(msg):
            nonlocal cmd_vel_received, movement_detected
            cmd_vel_received = True

            # Check if there's actual movement (non-zero velocities)
            if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
                movement_detected = True

        # Subscribe to /cmd_vel topic
        subscription = self.node.create_subscription(
            Twist,
            '/cmd_vel',
            cmd_vel_callback,
            10
        )

        # Wait for cmd_vel to be published with movement (with timeout)
        timeout_sec = 30  # Navigation may take time to start moving
        start_time = self.node.get_clock().now()

        while not movement_detected:
            rclpy.spin_once(self.node, timeout_sec=0.1)

            # Check timeout
            elapsed = (self.node.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > timeout_sec:
                if cmd_vel_received:
                    self.fail(
                        f'Received cmd_vel but no movement detected after {timeout_sec} seconds')
                else:
                    self.fail(
                        f'Timeout waiting for /cmd_vel topic after {timeout_sec} seconds')

        # Clean up
        self.node.destroy_subscription(subscription)

        # Assert that we received movement commands
        self.assertTrue(cmd_vel_received, 'cmd_vel topic was not published')
        self.assertTrue(movement_detected, 'No movement detected in cmd_vel messages')


# Generate test cases for different configurations
def generate_test_description_python_navigate_to_pose():
    """Test with Python implementation and NavigateToPose."""
    _test_config['use_python'] = False
    _test_config['use_waypoint'] = False
    return generate_test_description()


def generate_test_description_python_navigate_through_poses():
    """Test with Python implementation and NavigateThroughPoses."""
    _test_config['use_python'] = False
    _test_config['use_waypoint'] = True
    return generate_test_description()


def generate_test_description_cpp_navigate_to_pose():
    """Test with C++ implementation and NavigateToPose."""
    _test_config['use_python'] = True
    _test_config['use_waypoint'] = False
    return generate_test_description()


def generate_test_description_cpp_navigate_through_poses():
    """Test with C++ implementation and NavigateThroughPoses."""
    _test_config['use_python'] = True
    _test_config['use_waypoint'] = True
    return generate_test_description()
