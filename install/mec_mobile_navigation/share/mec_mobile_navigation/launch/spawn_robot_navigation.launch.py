#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    #TimerAction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_description = get_package_share_directory('mec_mobile_description')
    pkg_gazebo      = get_package_share_directory('mec_mobile_gazebo')
    pkg_navigation  = get_package_share_directory('mec_mobile_navigation')

    gazebo_resource_path = os.path.dirname(os.path.dirname(pkg_description))
    os.environ["GZ_SIM_RESOURCE_PATH"] = (
        os.environ.get("GZ_SIM_RESOURCE_PATH", "")
        + os.pathsep + gazebo_resource_path
    )

    rviz_arg  = DeclareLaunchArgument('rviz',  default_value='true')
    world_arg = DeclareLaunchArgument('world', default_value='world.sdf')

    urdf_file = os.path.join(
        pkg_description, 'urdf', 'robots', 'robot_3d_agv.urdf.xacro'
    )

    bridge_config = os.path.join(
        pkg_navigation, 'config', 'gz_bridge.yaml'
    )

    # ==================================================
    # WORLD - khoi dong dau tien
    # ==================================================
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'world.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # ==================================================
    # BRIDGE - khoi dong sau world de co /clock
    # ==================================================
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args', '-p',
            f'config_file:={bridge_config}'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # ==================================================
    # ROBOT STATE PUBLISHER - doi /clock san sang
    # ==================================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_file]),
                value_type=str
            ),
            'use_sim_time': True,
            'publish_frequency': 50.0,
        }]
    )

    # robot_state_publisher_delayed = TimerAction(
    #     period=2.0,    # doi 2s cho /clock tu Gazebo san sang
    #     actions=[robot_state_publisher]
    # )

    # ==================================================
    # SPAWN ROBOT
    # ==================================================
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_robot',
            '-topic', 'robot_description',
            '-x', '-3.0',
            '-y', '3.0',
            '-z', '0.05',
            '-Y', '0.0'
        ],
        output='screen'
    )

    # spawn_robot_delayed = TimerAction(
    #     period=4.0,    # doi 4s cho robot_state_publisher publish robot_description
    #     actions=[spawn_robot]
    # )

    # ==================================================
    # LASER MERGER
    # ==================================================
    laser_merger = Node(
        package='mec_mobile_navigation',
        executable='laser_merger_node.py',
        name='laser_merger',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )



    # ==================================================
    # RVIZ
    # ==================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_navigation, 'rviz', 'mapping.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        rviz_arg,
        world_arg,

        world_launch,               

        bridge,                     

        robot_state_publisher,  

        spawn_robot,            

        # laser_merger,          

        #rviz_node,                 
    ])