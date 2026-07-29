from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("warning_timeout_sec", default_value="1.5"),
        DeclareLaunchArgument("offline_timeout_sec", default_value="3.0"),
        Node(
            package="agv_fleet",
            executable="fleet_server",
            name="fleet_server",
            output="screen",
            parameters=[{
                "warning_timeout_sec": LaunchConfiguration("warning_timeout_sec"),
                "offline_timeout_sec": LaunchConfiguration("offline_timeout_sec"),
                "snapshot_hz": 2.0,
            }],
        ),
    ])
