"""
navigation.launch.py — mec_mobile_navigation v4.2.6

Fix:
  - Thêm launch argument 'map'.
  - Nếu HMI app truyền map:=/path/to/map.yaml thì Nav2 map_server/AMCL
    dùng đúng map app đang chọn trong Navigation/Routes.
  - Nếu không truyền map:=... thì dùng map mặc định maps/my_map.yaml.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_mec_mobile_navigation = get_package_share_directory(
        'mec_mobile_navigation'
    )

    default_map_path = os.path.join(
        pkg_mec_mobile_navigation,
        'maps',
        'my_map.yaml'
    )

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Open RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='navigation_amcl.rviz',
        description='RViz config file'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to map yaml file for Nav2 map_server/AMCL'
    )

    nav2_localization_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'localization_launch.py'
    )

    nav2_navigation_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )

    ekf_config_path = os.path.join(
        pkg_mec_mobile_navigation,
        'config',
        'ekf.yaml'
    )

    localization_params_path = os.path.join(
        pkg_mec_mobile_navigation,
        'config',
        'amcl_localization_ekf.yaml'
    )

    navigation_params_path = os.path.join(
        pkg_mec_mobile_navigation,
        'config',
        'navigation_ekf.yaml'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_path,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    laser_merger = Node(
        package='mec_mobile_navigation',
        executable='laser_merger_node.py',
        name='laser_merger',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            PathJoinSubstitution([
                pkg_mec_mobile_navigation,
                'rviz',
                LaunchConfiguration('rviz_config'),
            ]),
        ],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_localization_launch_path),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': localization_params_path,
            'map': LaunchConfiguration('map'),
        }.items()
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch_path),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': navigation_params_path,
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(rviz_launch_arg)
    ld.add_action(rviz_config_arg)
    ld.add_action(sim_time_arg)
    ld.add_action(map_arg)
    ld.add_action(laser_merger)
    ld.add_action(ekf_node)
    ld.add_action(rviz_node)
    ld.add_action(localization_launch)
    ld.add_action(navigation_launch)
    return ld
