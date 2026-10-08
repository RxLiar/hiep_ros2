from agv_fleet_domains.launch_helpers import robot_fleet_side


def generate_launch_description():
    return robot_fleet_side(
        robot_id="AGV001",
        robot_name="Busan AGV 01",
        robot_domain=21,
        bridge_config="agv001_fleet_bridge.yaml",
    )
