#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('bocbot_hunter')
    world = os.path.join(pkg, 'worlds', 'arena.world')
    urdf = os.path.join(pkg, 'urdf', 'bocbot.urdf.xacro')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', urdf]), 'use_sim_time': True}],
            output='screen'
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'bocbot', '-topic', 'robot_description', '-x', '0.0', '-y', '0.0', '-z', '0.1'],
            output='screen'
        ),

        Node(
            package='bocbot_hunter',
            executable='wheel_test',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
    ])
