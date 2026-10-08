from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PACKAGE_NAME = "agv_fleet_domains"


def _bridge_config(filename: str) -> str:
    return os.path.join(
        get_package_share_directory(PACKAGE_NAME),
        "config",
        filename,
    )


def robot_fleet_side(
    *,
    robot_id: str,
    robot_name: str,
    robot_domain: int,
    bridge_config: str,
) -> LaunchDescription:
    source_namespace = LaunchConfiguration("source_namespace")
    publish_hz = LaunchConfiguration("publish_hz")
    persist_map_context = LaunchConfiguration("persist_map_context")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "source_namespace",
                default_value="",
                description=(
                    "Namespace containing the local robot topics. "
                    "Leave empty for the current root-topic navigation stack."
                ),
            ),
            DeclareLaunchArgument(
                "publish_hz",
                default_value="5.0",
                description="Fleet robot state publication frequency.",
            ),
            DeclareLaunchArgument(
                "persist_map_context",
                default_value="true",
                description="Restore the last active map after Agent restart.",
            ),

            # Every local process launched from this file belongs to the
            # robot's private ROS graph.
            SetEnvironmentVariable(
                "ROS_DOMAIN_ID",
                str(robot_domain),
            ),
            SetEnvironmentVariable(
                "AGV_ROBOT_ID",
                robot_id,
            ),
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
                executable="fleet_agent",
                name=f"fleet_agent_{robot_id.lower()}",
                output="screen",
                parameters=[
                    {
                        "robot_id": robot_id,
                        "robot_name": robot_name,
                        "map_id": "",
                        "map_version": 0,
                        "source_namespace": source_namespace,
                        "publish_hz": ParameterValue(
                            publish_hz,
                            value_type=float,
                        ),
                        "persist_map_context": ParameterValue(
                            persist_map_context,
                            value_type=bool,
                        ),
                    }
                ],
            ),

            # domain_bridge creates one DDS participant in the robot
            # domain and one in Fleet domain 20. The YAML only forwards
            # /fleet/* topics, never navigation/TF/velocity topics.
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "domain_bridge",
                    "domain_bridge",
                    _bridge_config(bridge_config),
                ],
                output="screen",
                emulate_tty=True,
            ),
        ]
    )


def bridge_only(
    *,
    robot_domain: int,
    bridge_config: str,
) -> LaunchDescription:
    return LaunchDescription(
        [
            SetEnvironmentVariable(
                "ROS_DOMAIN_ID",
                str(robot_domain),
            ),
            SetEnvironmentVariable(
                "ROS_AUTOMATIC_DISCOVERY_RANGE",
                "SUBNET",
            ),
            SetEnvironmentVariable(
                "RMW_IMPLEMENTATION",
                "rmw_fastrtps_cpp",
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "domain_bridge",
                    "domain_bridge",
                    _bridge_config(bridge_config),
                ],
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
