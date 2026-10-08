from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('hiep_robot2')
    backend = LaunchConfiguration('backend')

    return LaunchDescription([
        DeclareLaunchArgument(
            'backend',
            default_value='esp32',
            description='Hardware backend: esp32 or keya',
        ),
        Node(
            package='hiep_robot2',
            executable='esp32_bridge_node',
            name='hardware_bridge',
            output='screen',
            parameters=[os.path.join(pkg, 'config', 'esp32_hardware.yaml')],
            condition=IfCondition(PythonExpression(["'", backend, "' == 'esp32'"])),
        ),
        Node(
            package='hiep_robot2',
            executable='keya_driver_bridge_node',
            name='keya_driver_bridge',
            output='screen',
            parameters=[os.path.join(pkg, 'config', 'keya_hardware.yaml')],
            condition=IfCondition(PythonExpression(["'", backend, "' == 'keya'"])),
        ),
    ])
