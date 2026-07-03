from launch import LaunchDescription

from launch_ros.actions import Node

from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    pkg_path = get_package_share_directory(
        'mec_mobile_description'
    )

    xacro_file = os.path.join(
        pkg_path,
        'urdf','robots',
        'robot_3d_agv.urdf.xacro'
    )

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    return LaunchDescription([

        # ROBOT STATE PUBLISHER
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',

            parameters=[{
                'robot_description': robot_description
            }]
        ),

        # JOINT STATE PUBLISHER GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),

        # RVIZ
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        )

    ])