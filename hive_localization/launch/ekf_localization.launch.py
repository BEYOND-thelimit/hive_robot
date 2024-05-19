from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    share_dir = get_package_share_directory('hive_localization')
    config_ekf = os.path.join(share_dir, 'config', 'ekf_config.yaml')

    robot_name = 'robot1'

    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[config_ekf,
                    {'use_sim_time': True},
                    ],
        remappings=[
            ('/cmd_vel', f'/{robot_name}/cmd_vel'),
            ('/odometry/filtered', f'/{robot_name}/ekf_odom')
        ]
    )
    start_lidar_odom = Node(
        package='hive_localization',
        executable='laser_scan_matcher_node',
        name='laser_scan_matcher_node',
        parameters=[
            {'base_frame': f'{robot_name}_base_link'},
            {'odom_frame': f'{robot_name}_odom'},
            {'laser_frame': f'{robot_name}_lidar_link'},
            {'publish_odom': f'{robot_name}/laser_odom'},
        ],
        remappings=[
            ('/converted_scan', f'/{robot_name}/converted_scan'),
        ]
    )
    start_lidar_converter = Node(
        package='hive_lidar',
        executable='scan_converter',
        name='scan_converter',
        remappings=[
            ('/converted_scan', f'/{robot_name}/converted_scan'),
        ]
    )
    return LaunchDescription([
        start_lidar_converter,
        start_lidar_odom,
        start_robot_localization_cmd
    ])
