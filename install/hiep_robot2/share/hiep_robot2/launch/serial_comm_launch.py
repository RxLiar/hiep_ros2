
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_hiep = get_package_share_directory('hiep_robot2')
    pkg_lidar = get_package_share_directory('sllidar_ros2')

    # === Launch arguments ===
    model_arg = DeclareLaunchArgument('model', default_value='robot_3d.urdf.xacro')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    # === URDF and robot_state_publisher ===
    urdf_file = PathJoinSubstitution([pkg_hiep, 'urdf', 'robots', LaunchConfiguration('model')])
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )

    # === Serial Node for ESP32 ===
    esp32_node = Node(
        package='hiep_robot2',
        executable='serial_comm_node',
        name='serial_comm_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # === LiDAR driver node ===
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_lidar, 'launch', 'sllidar_a1_launch.py')
        ),
        # launch_arguments={
        #     'serial_port': '/dev/ttyUSB1',  # hoặc tự dò nếu cần
        #     'frame_id': 'laser'
        # }.items()
    )

    # === RViz ===
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_hiep, 'rviz', 'mapping.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        sim_time_arg,
        robot_state_publisher_node,
        esp32_node,
        lidar_launch,
        rviz_node
    ])
