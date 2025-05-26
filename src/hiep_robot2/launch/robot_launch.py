import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_hiep_robot2 = get_package_share_directory('hiep_robot2')
    pkg_mec_navigation = get_package_share_directory('mec_mobile_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # === Các tham số cấu hình ===
    model_arg = DeclareLaunchArgument(
        'model', default_value='robot_3d.urdf.xacro',
        description='URDF model file'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time (false for real robot)'
    )

    # === Đường dẫn đến URDF file ===
    urdf_file_path = PathJoinSubstitution([
        pkg_hiep_robot2, 'urdf', 'robots', LaunchConfiguration('model')
    ])

    # === Node robot_state_publisher ===
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file_path]),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # === Node đọc encoder/odometry từ ESP32 ===
    serial_comm_node = Node(
        package='hiep_robot2',
        executable='serial_comm_node',
        name='serial_comm_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # === EKF từ robot_localization ===
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_mec_navigation, 'config', 'ekf.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # === RViz (tùy chọn) ===
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(pkg_mec_navigation, 'rviz', 'navigation.rviz')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # === Navigation2 (gọi lại file navigation.launch.py đã có) ===
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mec_navigation, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'rviz': LaunchConfiguration('rviz'),
            'rviz_config': 'navigation.rviz'
        }.items()
    )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        sim_time_arg,
        robot_state_publisher_node,
        serial_comm_node,
        ekf_node,
        rviz_node,
        navigation_launch
    ])
