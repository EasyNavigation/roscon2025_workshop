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

from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    online_async_launch_path = join(slam_toolbox_dir, 'launch', 'online_async_launch.py')

    tutorial_pkg_dir = get_package_share_directory('nav2_playground')
    default_slam_params = join(tutorial_pkg_dir, 'config', 'slam_params.yaml')

    rviz_config_file = join(tutorial_pkg_dir, 'config', 'slam_view.rviz')

    slam_params_file = LaunchConfiguration('slam_params_file')
    ld.add_action(DeclareLaunchArgument(
        'slam_params_file',
        default_value=default_slam_params,
        description='Full path to the slam_toolbox parameters file'
    ))

    use_rviz = LaunchConfiguration('use_rviz')
    ld.add_action(DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz'
    ))

    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(online_async_launch_path),
        launch_arguments={'slam_params_file': slam_params_file}.items()
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(use_rviz)
    )

    ld.add_action(slam_cmd)
    ld.add_action(rviz_cmd)

    return ld
