from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('hiep_robot2')
    params = os.path.join(pkg, 'config', 'keya_hardware.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel',
                              description='Velocity command topic the bridge listens to'),
        Node(
            package='hiep_robot2',
            executable='keya_driver_bridge_node',
            name='keya_driver_bridge',
            output='screen',
            parameters=[params, {'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic')}],
        )
    ])
