# hiep_ros2/src/hiep_robot2/launch/lidar_launch.py
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Lấy đường dẫn tới sllidar_ros2
    pkg_sllidar = get_package_share_directory('sllidar_ros2')

    # Include launch file xem LiDAR
    lidar_view = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sllidar, 'launch', 'view_sllidar_a1_launch.py')
        ),
        # nếu view_sllidar_a1_launch có arg, bạn có thể ném launch_arguments vào đây
        launch_arguments={
            # nếu launch LiDAR cần tham số, ví dụ:
            'serial_port': '/dev/ttyUSB0',
            'frame_id': 'laser'
        }.items()
    )

    return LaunchDescription([
        lidar_view
    ])
