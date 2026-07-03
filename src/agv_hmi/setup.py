from setuptools import find_packages, setup
import os
import glob

package_name = 'agv_hmi'

# Thu thập tất cả file .qss trong ui/
qss_files = glob.glob('agv_hmi/ui/*.qss')

setup(
    name=package_name,
    version='4.2.0',
    # find_packages tự động tìm agv_hmi, agv_hmi.ui, agv_hmi.ros
    packages=find_packages(exclude=['test']),
    package_data={
        'agv_hmi.ui': ['*.qss'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hiep0247',
    maintainer_email='gaphan247@gmail.com',
    description='AGV Busan Autonomous Robot HMI v4.2.0',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hmi = agv_hmi.main:main',
        ],
    },
)