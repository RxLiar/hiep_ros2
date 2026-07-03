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
    pkg_slam = get_package_share_directory('slam_toolbox')

    # === Launch arguments ===
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true', description='Open RViz')
    rviz_config = DeclareLaunchArgument('rviz_config', default_value='mapping.rviz', description='RViz config file')
    sim_time = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use sim time')
    model_arg = DeclareLaunchArgument('model', default_value='robot_3d.urdf.xacro', description='URDF model file')

    # === URDF file path ===
    urdf_file_path = PathJoinSubstitution([
        pkg_hiep, 'urdf', 'robots', LaunchConfiguration('model')
    ])

    # === Robot State Publisher ===
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro', ' ', urdf_file_path]),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_frequency': 30.0,
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # === Joint State Publisher ===
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'rate': 30.0,
        }]
    )

    # === Serial Communication Node (với odometry tích hợp) ===
    serial_node = Node(
        package='hiep_robot2',
        executable='serial_comm_node',
        name='serial_comm_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'pulses_per_rev': 2970.0,
            'wheel_radius': 0.024,  # 24mm
            'wheel_base': 0.20,     # 20cm
            'max_wheel_speed': 0.5, # 0.5 m/s
        }],
        remappings=[
            # Đảm bảo odometry được publish đúng topic
            ('odom', '/odom'),
        ]
    )

    # === Static TF Publishers ===
    # Base footprint -> Base link
    static_tf_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_footprint',
        arguments=['0', '0', '0.05', '0', '0', '0', 'base_footprint', 'base_link'],
        output='screen'
    )

    # Base link -> Laser
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        arguments=['0.1', '0', '0.15', '0', '0', '0', 'base_link', 'laser'],
        output='screen'
    )

    # === LiDAR Launch ===
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sllidar, 'launch', 'sllidar_a1_launch.py')
        ),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0',
            'baud_rate': '115200',
            'frame_id': 'laser',
            'angle_compensate': 'true',
            'scan_mode': 'Standard',
        }.items()
    )

    # === SLAM Toolbox ===
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': os.path.join(pkg_hiep, 'config', 'slam_toolbox_mapping.yaml'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    # === RViz ===
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_hiep, 'rviz', LaunchConfiguration('rviz_config')])],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # === Teleop Keyboard (để test robot) ===
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',  # Chạy trong terminal riêng
        remappings=[('cmd_vel', '/cmd_vel')]
    )

    # === Robot Localization (EKF) - Tùy chọn để fusion odometry ===
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_hiep, 'config', 'ekf.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('odometry/filtered', '/odometry/filtered'),
            ('/set_pose', '/initialpose')
        ]
    )

    return LaunchDescription([
        # Arguments
        rviz_arg,
        rviz_config,
        sim_time,
        model_arg,
        
        # Core robot nodes
        robot_state_publisher_node,
        joint_state_publisher_node,
        serial_node,
        
        # Static transforms
        static_tf_base_footprint,
        static_tf_laser,
        
        # Sensors
        lidar_launch,
        
        # SLAM
        slam_launch,
        
        # Visualization and control
        rviz_node,
        teleop_node,
        
        # Optional: EKF for sensor fusion
        # ekf_node,  # Uncomment nếu muốn dùng EKF
    ])