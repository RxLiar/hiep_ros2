from setuptools import setup
from glob import glob
import os

package_name = 'hiep_robot2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/serial_comm_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hiep0247',
    maintainer_email='gaphan247@gmail.com',
    description='ROS2 ESP32 Serial Communication Package',

    # license='MIT',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_comm_node = hiep_robot2.serial_comm_node:main',
        ],
    },
)
