from agv_fleet_domains.launch_helpers import robot_fleet_side


def generate_launch_description():
    return robot_fleet_side(
        robot_id="AGV002",
        robot_name="Busan AGV 02",
        robot_domain=22,
        bridge_config="agv002_fleet_bridge.yaml",
    )
