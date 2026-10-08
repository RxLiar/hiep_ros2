import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('hiep_robot2')
    return LaunchDescription([
        Node(package='hiep_robot2', executable='safety_monitor_node', name='safety_monitor',
             output='screen', parameters=[os.path.join(pkg, 'config', 'safety_monitor.yaml')]),
    ])
