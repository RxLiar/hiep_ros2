from glob import glob
from setuptools import find_packages, setup

package_name = "agv_fleet"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Busan AGV Team",
    maintainer_email="devnull@example.com",
    description="Fleet Agent, Fleet Server and two-computer simulation tools for AGV HMI.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "fleet_agent = agv_fleet.fleet_agent:main",
            "fleet_server = agv_fleet.fleet_server:main",
            "fleet_simulator = agv_fleet.fleet_simulator:main",
            "fleet_map_id = agv_fleet.map_registry:main",
        ],
    },
)
