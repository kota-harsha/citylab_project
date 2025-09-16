from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
# import time

from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
    rviz_config_path = os.path.join(get_package_share_directory('robot_patrol'), 'rviz', 'patrol_config.rviz')

    return LaunchDescription([
        # Launch the direction service first
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        ),
        Node(
            package='robot_patrol',
            executable='direction_service',
            output='screen',
            name='direction_service_node'
        ),
        ExecuteProcess(
            cmd=['sleep', '2'],
            output='screen'
        ),
        Node(
            package='robot_patrol',
            executable='patrol_with_service',
            output='screen',
            name='patrol_with_service_node'),
    ])