from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_id", default_value="AGV001"),
        DeclareLaunchArgument("robot_name", default_value="Busan AGV 01"),
        # Optional only for backward compatibility. Route execution updates the
        # active map automatically through /fleet/map_context.
        DeclareLaunchArgument("map_id", default_value=""),
        DeclareLaunchArgument("map_version", default_value="0"),
        DeclareLaunchArgument("source_namespace", default_value=""),
        DeclareLaunchArgument("publish_hz", default_value="5.0"),
        Node(
            package="agv_fleet",
            executable="fleet_agent",
            name="fleet_agent",
            output="screen",
            parameters=[{
                "robot_id": LaunchConfiguration("robot_id"),
                "robot_name": LaunchConfiguration("robot_name"),
                "map_id": LaunchConfiguration("map_id"),
                "map_version": ParameterValue(
                    LaunchConfiguration("map_version"), value_type=int
                ),
                "source_namespace": LaunchConfiguration("source_namespace"),
                "publish_hz": ParameterValue(
                    LaunchConfiguration("publish_hz"), value_type=float
                ),
            }],
        ),
    ])
