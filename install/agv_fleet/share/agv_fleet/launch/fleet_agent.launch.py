from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_id", default_value="AGV001"),
        DeclareLaunchArgument("robot_name", default_value="Busan AGV 01"),
        DeclareLaunchArgument("map_id", default_value="M02101"),
        DeclareLaunchArgument("map_version", default_value="1"),
        DeclareLaunchArgument("source_namespace", default_value=""),
        Node(
            package="agv_fleet",
            executable="fleet_agent",
            name="fleet_agent",
            output="screen",
            parameters=[{
                "robot_id": LaunchConfiguration("robot_id"),
                "robot_name": LaunchConfiguration("robot_name"),
                "map_id": LaunchConfiguration("map_id"),
                "map_version": ParameterValue(LaunchConfiguration("map_version"), value_type=int),
                "source_namespace": LaunchConfiguration("source_namespace"),
                "publish_hz": 2.0,
            }],
        ),
    ])
