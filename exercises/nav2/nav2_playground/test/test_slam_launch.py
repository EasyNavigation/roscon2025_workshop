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

"""Test for SLAM launch pipeline."""

import os
import unittest

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import launch_testing
import launch_testing.actions
import launch_testing.markers

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node


@launch_testing.markers.keep_alive
def generate_test_description():
    """
    Generate launch description for SLAM test.

    This will launch:
    1. playground_kobuki.launch.py (simulation with Kobuki robot)
    2. slam_launch.py (SLAM toolbox)
    """
    nav2_playground_dir = get_package_share_directory('nav2_playground')
    kobuki_playground_dir = get_package_share_directory('easynav_playground_kobuki')

    # Launch playground (Gazebo + Kobuki)
    playground_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kobuki_playground_dir, 'launch', 'playground_kobuki.launch.py')
        ),
        launch_arguments={'gui': 'false'}.items()  # Headless for faster testing
    )

    # Launch SLAM
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_playground_dir, 'launch', 'slam_launch.py')
        ),
        launch_arguments={'use_rviz': 'false'}.items()  # No RViz for faster testing
    )

    return LaunchDescription([
        playground_launch,
        slam_launch,
        launch_testing.actions.ReadyToTest()
    ])


class TestSlamLaunch(unittest.TestCase):
    """Test that SLAM launches and publishes map data."""

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
        self.node = Node('test_slam_node')

    def tearDown(self):
        """Clean up test node."""
        self.node.destroy_node()

    def test_map_topic_published(self, proc_output):
        """
        Test that the /map topic is being published.

        This verifies that:
        1. SLAM toolbox is running
        2. Map data is being generated
        """
        # Flag to track if we received a map message
        map_received = False

        def map_callback(msg):
            nonlocal map_received
            map_received = True
            # Verify message is valid OccupancyGrid
            self.assertIsInstance(msg, OccupancyGrid)
            self.assertGreater(msg.info.width, 0)
            self.assertGreater(msg.info.height, 0)
            self.assertGreater(len(msg.data), 0)

        # Subscribe to /map topic
        subscription = self.node.create_subscription(
            OccupancyGrid,
            '/map',
            map_callback,
            10
        )

        # Wait for map to be published (with timeout)
        timeout_sec = 90.0  # SLAM needs time to initialize
        start_time = self.node.get_clock().now()

        while not map_received:
            rclpy.spin_once(self.node, timeout_sec=0.1)

            # Check timeout
            elapsed = (self.node.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > timeout_sec:
                self.fail(f'Timeout waiting for /map topic after {timeout_sec} seconds')

        # Clean up
        self.node.destroy_subscription(subscription)

        # Assert that we received the map
        self.assertTrue(map_received, 'Map topic was not published')
