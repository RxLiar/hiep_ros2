import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # === Get package directories ===
    pkg_hiep = get_package_share_directory('hiep_robot2')
    pkg_sllidar = get_package_share_directory('sllidar_ros2')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    # === Launch arguments ===
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true', description='Launch RViz')

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use sim time')

    model_arg = DeclareLaunchArgument(
        'model', default_value='robot_3d.urdf.xacro', description='URDF model file')

    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_hiep, 'maps', 'map.yaml'),
        description='Full path to map yaml file to load')

    # === URDF path ===
    urdf_path = PathJoinSubstitution([
        pkg_hiep, 'urdf', 'robots', LaunchConfiguration('model')
    ])

    # === Nodes ===
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro', ' ', urdf_path]),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_frequency': 30.0,
        }]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    serial_node = Node(
        package='hiep_robot2',
        executable='serial_comm_node',
        name='serial_comm_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'pulses_per_rev': 2970.0,
            'wheel_radius': 0.024,
            'wheel_base': 0.20,
            'max_wheel_speed': 0.5,
        }],
        remappings=[('odom', '/odom')]
    )

    static_tf_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base',
        arguments=['0', '0', '0.05', '0', '0', '0', 'base_footprint', 'base_link'],
    )

    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        arguments=['0.1', '0', '0.15', '0', '0', '0', 'base_link', 'laser'],
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sllidar, 'launch', 'sllidar_a1_launch.py')
        ),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0',
            'baud_rate': '115200',
            'frame_id': 'laser',
            'angle_compensate': 'true',
        }.items()
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'true',
            'map': LaunchConfiguration('map'),
            'params_file': os.path.join(pkg_hiep, 'config', 'nav2_params.yaml')
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(pkg_hiep, 'rviz', 'nav2.rviz')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    return LaunchDescription([
        rviz_arg,
        sim_time_arg,
        model_arg,
        map_file_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        serial_node,
        static_tf_base,
        static_tf_laser,
        lidar_launch,
        nav2_launch,
        rviz_node,
    ])
