from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('easynav_patrolling_behavior_py')
    params = os.path.join(pkg_share, 'config', 'patrolling_params.yaml')

    return LaunchDescription([
        Node(
            package='easynav_patrolling_behavior_py',
            executable='patrolling_node',
            name='patrolling_node',
            output='screen',
            parameters=[params],
        )
    ])
