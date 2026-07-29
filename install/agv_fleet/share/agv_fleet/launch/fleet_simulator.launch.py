from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_id", default_value="AGV001"),
        DeclareLaunchArgument("robot_name", default_value="Simulated AGV 01"),
        DeclareLaunchArgument("map_id", default_value="M02101"),
        DeclareLaunchArgument("map_version", default_value="1"),
        DeclareLaunchArgument("center_x", default_value="0.0"),
        DeclareLaunchArgument("center_y", default_value="0.0"),
        DeclareLaunchArgument("radius", default_value="2.0"),
        Node(
            package="agv_fleet",
            executable="fleet_simulator",
            name=["fleet_simulator_", LaunchConfiguration("robot_id")],
            output="screen",
            parameters=[{
                "robot_id": LaunchConfiguration("robot_id"),
                "robot_name": LaunchConfiguration("robot_name"),
                "map_id": LaunchConfiguration("map_id"),
                "map_version": ParameterValue(LaunchConfiguration("map_version"), value_type=int),
                "center_x": ParameterValue(LaunchConfiguration("center_x"), value_type=float),
                "center_y": ParameterValue(LaunchConfiguration("center_y"), value_type=float),
                "radius": ParameterValue(LaunchConfiguration("radius"), value_type=float),
            }],
        ),
    ])
