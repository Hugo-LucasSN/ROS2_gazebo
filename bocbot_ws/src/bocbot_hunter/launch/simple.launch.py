#!/usr/bin/env python3
"""
Simple Launch - Gazebo + robot + simple_hunter (1 node unique)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('bocbot_hunter')
    world = os.path.join(pkg, 'worlds', 'arena.world')
    urdf = os.path.join(pkg, 'urdf', 'bocbot.urdf.xacro')

    return LaunchDescription([
        # Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', urdf]), 'use_sim_time': True}],
            output='screen'
        ),

        # Spawn Robot DANS SA CAGE (bleu, gauche)
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'bocbot', '-topic', 'robot_description',
                      '-x', '-5.0', '-y', '0.0', '-z', '0.15', '-Y', '0.0'],
            output='screen'
        ),

        # FAST HUNTER - Node unique rapide!
        Node(
            package='bocbot_hunter',
            executable='fast_hunter',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
    ])
