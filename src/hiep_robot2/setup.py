from setuptools import setup
from glob import glob
import os

package_name = 'hiep_robot2'

setup(
    name=package_name,
    version='0.5.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/firmware/agv_esp32', glob('firmware/agv_esp32/*')),
        ('share/' + package_name + '/firmware/mega2560_plc', glob('firmware/mega2560_plc/*')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='hiep0247',
    maintainer_email='gaphan247@gmail.com',
    description='ROS 2 AGV hardware bridges for ESP32/KEYA traction and Mega2560 PLC conveyor/IO control',
    license='MIT',
    entry_points={
        'console_scripts': [
            'esp32_bridge_node = hiep_robot2.esp32_bridge_node:main',
            'keya_driver_bridge_node = hiep_robot2.keya_driver_bridge_node:main',
            'mega2560_plc_bridge_node = hiep_robot2.mega2560_plc_bridge_node:main',
            'safety_monitor_node = hiep_robot2.safety_monitor_node:main',
        ],
    },
)
