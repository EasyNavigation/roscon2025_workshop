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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_dir = get_package_share_directory('nav2_playground')
    nav2_dir = get_package_share_directory('nav2_bringup')

    # Configuration Variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    use_rviz = LaunchConfiguration('use_rviz')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam', default_value='False')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz')

    declare_map_cmd = DeclareLaunchArgument(
        'map', default_value=os.path.join(
            package_dir,
            'maps',
            'aws_house.yaml')
    )

    declare_nav_params_cmd = DeclareLaunchArgument(
        'params_file', default_value=os.path.join(
            package_dir,
            'config',
            'nav2_kobuki_params.yaml')
    )

    # Actions
    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam': slam,
            'map': map_file,
            'params_file': params_file,
            'slam_params_file': params_file,
            'use_composition': 'False'
        }.items()
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'rviz_launch.py')
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_nav_params_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(navigation_cmd)
    ld.add_action(rviz_cmd)

    return ld
