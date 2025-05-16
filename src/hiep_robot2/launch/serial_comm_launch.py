from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hiep_robot2',
            executable='serial_comm_node',  # << phải giống TÊN entry_point, không có .py
            name='serial_comm_node',
            output='screen'
        )
    ])
