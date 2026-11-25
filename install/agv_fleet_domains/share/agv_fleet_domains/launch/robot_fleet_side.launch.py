"""Connect ANY robot to the fleet: Fleet Agent + domain_bridge, no per-robot launch file needed.

    ros2 launch agv_fleet_domains robot_fleet_side.launch.py robot_id:=AGV003 robot_name:="Busan AGV 03" robot_domain:=23

The HMI (Fleet page -> "Fleet control") runs exactly this launch file. The bridge YAML for the robot is generated
into ~/.agv_hmi/fleet/<robot_id>_fleet_bridge.yaml, so adding a robot never needs a new file in this package.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from agv_fleet_domains.bridge_config import validate, write_bridge_config


def _setup(context, *args, **kwargs):
    value = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    robot_id, robot_domain, fleet_domain = validate(value("robot_id"), value("robot_domain"), value("fleet_domain"))
    bridge_yaml = write_bridge_config(robot_id, robot_domain, fleet_domain)
    to_bool = lambda s: str(s).strip().lower() in ("1", "true", "yes", "on")  # noqa: E731

    return [
        # Everything launched below belongs to the robot's private ROS graph.
        SetEnvironmentVariable("ROS_DOMAIN_ID", str(robot_domain)),
        SetEnvironmentVariable("AGV_ROBOT_ID", robot_id),
        SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", "SUBNET"),
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
        Node(
            package="agv_fleet",
            executable="fleet_agent",
            name=f"fleet_agent_{robot_id.lower()}",
            output="screen",
            parameters=[{
                "robot_id": robot_id,
                "robot_name": value("robot_name") or robot_id,
                "map_id": "",
                "map_version": 0,
                "source_namespace": value("source_namespace"),
                "publish_hz": float(value("publish_hz")),
                "persist_map_context": to_bool(value("persist_map_context")),
            }],
        ),
        ExecuteProcess(
            cmd=["ros2", "run", "domain_bridge", "domain_bridge", bridge_yaml],
            output="screen",
            emulate_tty=True,
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_id", default_value="AGV001"),
        DeclareLaunchArgument("robot_name", default_value=""),
        DeclareLaunchArgument("robot_domain", default_value="21",
                              description="ROS_DOMAIN_ID of the robot's own navigation stack."),
        DeclareLaunchArgument("fleet_domain", default_value="20",
                              description="ROS_DOMAIN_ID where Fleet Server runs."),
        DeclareLaunchArgument("source_namespace", default_value=""),
        DeclareLaunchArgument("publish_hz", default_value="5.0"),
        DeclareLaunchArgument("persist_map_context", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
