#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 로봇 번호를 launch 매개변수로 선언
    robot_num_arg = DeclareLaunchArgument(
        'robot_num',
        default_value='2',
        description='Number of the robot'
    )

    button_publisher_node = Node(
        package='hive_tact',
        executable='hive_tact',
        parameters=[{'robot_num': LaunchConfiguration('robot_num')}],
        output='screen',
    )

    return LaunchDescription([
        robot_num_arg,
        button_publisher_node,
    ])

