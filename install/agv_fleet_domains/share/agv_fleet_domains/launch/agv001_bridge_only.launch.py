from agv_fleet_domains.launch_helpers import bridge_only


def generate_launch_description():
    return bridge_only(
        robot_domain=21,
        bridge_config="agv001_fleet_bridge.yaml",
    )
