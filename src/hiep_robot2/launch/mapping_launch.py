import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
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

        # === Các tham số cấu hình ===
    model_arg = DeclareLaunchArgument(
        'model', default_value='robot_3d.urdf.xacro',
        description='URDF model file'
    )
        # === Đường dẫn đến URDF file ===
    urdf_file_path = PathJoinSubstitution([
        pkg_hiep, 'urdf', 'robots', LaunchConfiguration('model')
    ])
    # === Node robot_state_publisher ===    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': Command(['xacro', ' ', urdf_file_path]),
             'use_sim_time': True},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )    

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
            # 'serial_port': '/dev/serial/by-id/usb-SLLIDAR_LiDAR_A1_XXXXXX-if00',
            'baud_rate': '115200',

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
        # name='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_hiep, 'rviz', LaunchConfiguration('rviz_config')])],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    # --- Static TF từ base_link tới laser ---
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
        output='screen'
    )
    # --- Node odometry_publisher ---
    odometry_publisher_node = Node(
        package='hiep_robot2',
        executable='odometry_publisher',
        name='odometry_publisher',
        output='screen'
    )
    
    return LaunchDescription([
        rviz_arg,
        rviz_config,
        sim_time,
        model_arg,
        robot_state_publisher_node,
        serial_node,
        lidar_launch,
        slam_launch,
        rviz_node,
        static_tf_laser,
        odometry_publisher_node
    ])
