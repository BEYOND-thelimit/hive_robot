from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    share_dir = get_package_share_directory('hive_localization')
    config_ekf = os.path.join(share_dir, 'config', 'ekf_config.yaml')

    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[config_ekf,
                    {'use_sim_time': True},
                    ],
        remappings=[
            ('/cmd_vel', '/robot1/cmd_vel'),
        ]
    )

    return LaunchDescription([
        start_robot_localization_cmd
    ])
