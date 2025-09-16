from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
    rviz_config_path = os.path.join(get_package_share_directory('robot_patrol'), 'rviz', 'patrol_config.rviz')

    return LaunchDescription([
        Node(
            package='robot_patrol',
            executable='patrol_node',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ])