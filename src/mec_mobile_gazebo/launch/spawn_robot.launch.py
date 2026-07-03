import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # =========================
    # PACKAGE PATHS
    # =========================

    pkg_description = get_package_share_directory(
        'mec_mobile_description'
    )

    pkg_gazebo = get_package_share_directory(
        'mec_mobile_gazebo'
    )

    # VERY IMPORTANT
    # Gazebo must see the parent folder of package

    gazebo_resource_path = os.path.dirname(
        os.path.dirname(pkg_description)
    )

    os.environ["GZ_SIM_RESOURCE_PATH"] = (
        os.environ.get("GZ_SIM_RESOURCE_PATH", "")
        + os.pathsep +
        gazebo_resource_path
    )

    # =========================
    # ARGUMENTS
    # =========================

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='world.sdf'
    )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value='robot_3d_agv.urdf.xacro'
    )

    # =========================
    # ROBOT FILE
    # =========================

    urdf_file = os.path.join(
        pkg_description,
        'urdf',
        'robots',
        'robot_3d_agv.urdf.xacro'
    )

    # =========================
    # GAZEBO WORLD
    # =========================

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_gazebo,
                'launch',
                'world.launch.py'
            )
        ),
        launch_arguments={
            'world': LaunchConfiguration('world')
        }.items()
    )

    # =========================
    # ROBOT STATE PUBLISHER
    # =========================

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',

        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_file]),
                value_type=str
            ),
            'use_sim_time': True
        }]
    )

    # =========================
    # SPAWN ROBOT
    # =========================

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',

        arguments=[
            '-name', 'my_robot',
            '-topic', 'robot_description',

            '-x', '-3.0',
            '-y', '3.0',
            '-z', '0.0',
            '-y', '3.14159'  # xoay 180° để đầu xe khớp với RViz
        ],

        output='screen'
    )

    # =========================
    # RVIZ
    # =========================

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',

        arguments=[
            '-d',
            os.path.join(
                pkg_description,
                'rviz',
                'rviz.rviz'
            )
        ],

        parameters=[
            {'use_sim_time': True}
        ],

        condition=IfCondition(
            LaunchConfiguration('rviz')
        )
    )

    # =========================
    # BRIDGE
    # =========================

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',

        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',

            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',

            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',

            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',

            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',

            # GZ -> ROS, 1 chieu ([)
            #'/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/front_lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/rear_lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],

        output='screen'
    )

    # =========================
    # LAUNCH
    # =========================

    return LaunchDescription([

        rviz_arg,
        world_arg,
        model_arg,

        world_launch,

        robot_state_publisher,

        spawn_robot,

        rviz_node,

        bridge
    ])