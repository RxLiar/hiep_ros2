import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_hiep = get_package_share_directory('hiep_robot2')
    pkg_sllidar = get_package_share_directory('sllidar_ros2')
    pkg_slam    = get_package_share_directory('slam_toolbox')
    #pkg_mec_mobile_navigation = get_package_share_directory('mec_mobile_navigation')

    # --- Launch arguments ---
    rviz_arg       = DeclareLaunchArgument('rviz',       default_value='true',  description='Open RViz')
    rviz_config    = DeclareLaunchArgument('rviz_config', default_value='mapping.rviz', description='RViz config file')
    sim_time       = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use sim time')

    # --- Node giao tiếp Serial với ESP32 (encoder + cmd_vel) ---
    serial_node = Node(
        package='hiep_robot2',
        executable='serial_comm_node',
        name='serial_comm_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # --- Include launch của LiDAR (thay bằng file bạn cần) ---
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sllidar, 'launch', 'sllidar_a1_launch.py')
        ),
        launch_arguments={
            # nếu launch LiDAR cần tham số, ví dụ:
            'serial_port': '/dev/ttyUSB0',
            'frame_id': 'laser'
        }.items()
    )

    # --- Include SLAM Toolbox mapping ---
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': os.path.join(pkg_hiep, 'config', 'slam_toolbox_mapping.yaml'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    # --- RViz ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_hiep, 'rviz', LaunchConfiguration('rviz_config')])],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        rviz_arg,
        rviz_config,
        sim_time,
        serial_node,
        lidar_launch,
        slam_launch,
        rviz_node,
    ])
