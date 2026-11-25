from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    snapshot_hz = LaunchConfiguration("snapshot_hz")
    warning_timeout = LaunchConfiguration("warning_timeout_sec")
    offline_timeout = LaunchConfiguration("offline_timeout_sec")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "fleet_domain",
                default_value="20",
                description="ROS_DOMAIN_ID of the Fleet network.",
            ),
            DeclareLaunchArgument(
                "snapshot_hz",
                default_value="5.0",
            ),
            DeclareLaunchArgument(
                "warning_timeout_sec",
                default_value="1.5",
            ),
            DeclareLaunchArgument(
                "offline_timeout_sec",
                default_value="3.0",
            ),

            SetEnvironmentVariable("ROS_DOMAIN_ID", LaunchConfiguration("fleet_domain")),
            SetEnvironmentVariable(
                "ROS_AUTOMATIC_DISCOVERY_RANGE",
                "SUBNET",
            ),
            SetEnvironmentVariable(
                "RMW_IMPLEMENTATION",
                "rmw_fastrtps_cpp",
            ),

            Node(
                package="agv_fleet",
                executable="fleet_server",
                name="fleet_server",
                output="screen",
                parameters=[
                    {
                        "snapshot_hz": ParameterValue(
                            snapshot_hz,
                            value_type=float,
                        ),
                        "warning_timeout_sec": ParameterValue(
                            warning_timeout,
                            value_type=float,
                        ),
                        "offline_timeout_sec": ParameterValue(
                            offline_timeout,
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
