import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('hiep_robot2')
    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='sim', description='sim | plc'),
        Node(package='hiep_robot2', executable='forklift_node', name='forklift_node', output='screen',
             parameters=[os.path.join(pkg, 'config', 'forklift.yaml'), {'backend': LaunchConfiguration('backend')}]),
    ])
