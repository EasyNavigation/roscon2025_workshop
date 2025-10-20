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
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('nav2_playground')

    # Configuration Variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    use_waypoint = LaunchConfiguration('use_waypoint')
    use_python = LaunchConfiguration('use_python')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            package_dir,
            'config',
            'patrolling_params.yaml'),
        description='Path to the patrolling node parameters file')

    declare_use_waypoint_cmd = DeclareLaunchArgument(
        'use_waypoint',
        default_value='false',
        description='If true, use nav2 waypoint follower approach; '
                    'if false, use patrolling state machine (default: false)')

    declare_use_python_cmd = DeclareLaunchArgument(
        'use_python',
        default_value='false',
        description='If true, use Python version of the executable; '
                    'if false, use C++ version (default: false)')

    patrolling_node_cpp = Node(
        package='nav2_playground',
        executable='patrolling_node',
        name='patrolling_node',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(
            # Compare substituted values as strings to avoid NameError when they are 'true'/'false'
            PythonExpression([
                "not '", use_waypoint, "' in ('true','True') and not '", use_python,
                "' in ('true','True')"
            ])
        )
    )

    patrolling_node_py = Node(
        package='nav2_playground',
        executable='patrolling_node.py',
        name='patrolling_node',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(
            PythonExpression([
                "not '", use_waypoint, "' in ('true','True') and '", use_python,
                "' in ('true','True')"
            ])
        )
    )

    waypoint_client_node_cpp = Node(
        package='nav2_playground',
        executable='waypoint_client_node',
        name='waypoint_client_node',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(
            PythonExpression([
                "'", use_waypoint, "' in ('true','True') and not '", use_python,
                "' in ('true','True')"
            ])
        )
    )

    waypoint_client_node_py = Node(
        package='nav2_playground',
        executable='waypoint_client_node.py',
        name='waypoint_client_node',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(
            PythonExpression([
                "'", use_waypoint, "' in ('true','True') and '", use_python,
                "' in ('true','True')"
            ])
        )
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_use_waypoint_cmd,
        declare_use_python_cmd,
        patrolling_node_cpp,
        patrolling_node_py,
        waypoint_client_node_cpp,
        waypoint_client_node_py
    ])
