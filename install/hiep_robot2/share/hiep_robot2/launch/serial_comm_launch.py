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
    pkg_sllidar = get_package_share_directory('sllidar_ros2')

    # === Launch Arguments ===
    model_arg = DeclareLaunchArgument(
        'model', default_value='robot_3d.urdf.xacro',
        description='URDF model file (relative to urdf/robots)'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time'
    )

    # === URDF path ===
    urdf_file = PathJoinSubstitution([
        pkg_hiep_robot2, 'urdf', 'robots', LaunchConfiguration('model')
    ])

    # === robot_state_publisher ===
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # === ESP32 node ===
    serial_comm_node = Node(
        package='hiep_robot2',
        executable='serial_comm_node',
        name='serial_comm_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # === Lidar node (sllidar_ros2) ===
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sllidar, 'launch', 'sllidar_a1_launch.py')
        ),
        launch_arguments={
            'serial_port': '/dev/ttyUSB1',  # hoặc thay bằng thiết bị cố định nếu biết
            'frame_id': 'laser'
        }.items()
    )

    # === RViz (tùy chọn) ===
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        sim_time_arg,
        robot_state_publisher_node,
        serial_comm_node,
        lidar_launch,
        rviz_node
    ])
