import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_mec_mobile_navigation = get_package_share_directory('mec_mobile_navigation')

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='mapping.rviz',
        description='RViz config file'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    # Path to the Slam Toolbox launch file
    slam_toolbox_launch_path = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    slam_toolbox_params_path = os.path.join(
        get_package_share_directory('mec_mobile_navigation'),
        'config',
        'slam_toolbox_mapping.yaml'
    )
    ekf_config_path = os.path.join(
    get_package_share_directory('mec_mobile_navigation'),
    'config',
    'ekf.yaml'
    )
    # Launch rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_mec_mobile_navigation, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )
    
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'slam_params_file': slam_toolbox_params_path,
        }.items()
    )

    # Them EKF node de fuse odom + IMU
    ekf_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[
        ekf_config_path,
        {'use_sim_time': LaunchConfiguration('use_sim_time')}
    ]
    )
    # ========================
    # LASER MERGER NODE
    # ========================
    laser_merger_node = Node(
        package='mec_mobile_navigation',
        executable='laser_merger_node.py',
        name='laser_merger',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    #launchDescriptionObject.add_action(ekf_node)
    
    launchDescriptionObject = LaunchDescription()
    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(rviz_config_arg)
    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(ekf_node)
    launchDescriptionObject.add_action(laser_merger_node)
    launchDescriptionObject.add_action(slam_toolbox_launch)
    launchDescriptionObject.add_action(rviz_node)

    return launchDescriptionObject
