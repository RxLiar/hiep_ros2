"""One entry point for both hardware setups.

  ros2 launch hiep_robot2 agv_hardware.launch.py profile:=model
      ESP32 + L298N test model (traction only; PLC off unless plc:=true)

  ros2 launch hiep_robot2 agv_hardware.launch.py profile:=real
      Real AGV: KEYA traction + Mega2560 PLC (conveyors / bumpers / EMG / lights)

  plc:=auto|true|false      (default auto: off for model, on for real)
  forklift:=false|true|plc  start the fork-lift node (true = built-in simulator, plc = real fork via the Mega2560)
  safety:=auto|true|false   lidar slow/stop gate (default auto: off for model, on for real).
                            When on, Nav2/HMI commands go /cmd_vel -> safety_monitor -> /cmd_vel_safe -> bridge.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

_TRACTION = {
    "model": "esp32_hardware.launch.py",
    "real": "keya_hardware.launch.py",
}
_TRUE = ("1", "true", "yes", "on")


def _setup(context, *args, **kwargs):
    share = get_package_share_directory("hiep_robot2")
    profile = LaunchConfiguration("profile").perform(context).strip().lower()
    plc = LaunchConfiguration("plc").perform(context).strip().lower()

    if profile not in _TRACTION:
        raise RuntimeError(f"profile must be 'model' or 'real', got '{profile}'")
    if plc == "auto":
        plc = "true" if profile == "real" else "false"
    use_plc = plc in _TRUE
    safety = LaunchConfiguration("safety").perform(context).strip().lower()
    if safety == "auto":
        safety = "true" if profile == "real" else "false"
    use_safety = safety in _TRUE
    forklift = LaunchConfiguration("forklift").perform(context).strip().lower()

    def include(name, **launch_arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", name)),
            launch_arguments=launch_arguments.items(),
        )

    actions = [
        LogInfo(msg=f"[hiep_robot2] profile={profile}: traction={_TRACTION[profile]}, plc={use_plc}, safety={use_safety}"),
        include(_TRACTION[profile], cmd_vel_topic="/cmd_vel_safe" if use_safety else "/cmd_vel"),
    ]
    if use_safety:
        actions.append(include("safety_monitor.launch.py"))
    if forklift in _TRUE or forklift == "plc":
        actions.append(include("forklift.launch.py", backend="plc" if forklift == "plc" else "sim"))
    if use_plc:
        actions.append(include("mega2560_plc.launch.py"))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("profile", default_value="model",
                              description="model = ESP32 test model, real = KEYA + Mega2560 PLC"),
        DeclareLaunchArgument("forklift", default_value="false",
                              description="false | true (simulated fork) | plc (real fork on the Mega2560)"),
        DeclareLaunchArgument("safety", default_value="auto",
                              description="auto | true | false: lidar slow/stop gate (safety_monitor)"),
        DeclareLaunchArgument("plc", default_value="auto",
                              description="auto | true | false: start the Mega2560 PLC bridge"),
        OpaqueFunction(function=_setup),
    ])
