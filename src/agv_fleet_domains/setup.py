import os
from glob import glob

from setuptools import find_packages, setup


package_name = "agv_fleet_domains"


setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            os.path.join("share", package_name),
            ["package.xml"],
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "env"),
            glob("env/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hiep0247",
    maintainer_email="gaphan247@gmail.com",
    description=(
        "ROS domain isolation and selective bridges for the AGV fleet"
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
