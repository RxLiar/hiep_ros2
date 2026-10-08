from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory("hiep_robot2")
    params = os.path.join(package_share, "config", "mega2560_plc.yaml")

    return LaunchDescription([
        Node(
            package="hiep_robot2",
            executable="mega2560_plc_bridge_node",
            name="mega2560_plc_bridge",
            output="screen",
            parameters=[params],
        )
    ])
