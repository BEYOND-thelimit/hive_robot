from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    config_ekf = LaunchConfiguration('config_ekf', default='$(find robot_localization)/config/ekf_config.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_ekf',
            default_value=config_ekf,
            description='Path to the configuration file for the EKF node.'
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_localization',
            output='screen',
            parameters=[config_ekf],
            remappings=[
                # Add any necessary topic remappings here
            ],
        ),
    ])
