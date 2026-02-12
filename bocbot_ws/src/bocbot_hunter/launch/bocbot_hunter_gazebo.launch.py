#!/usr/bin/env python3
"""
Bocbot Hunter - Lancement Gazebo avec arene
Version LIDAR-ONLY (pas de camera pour la detection)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_bocbot = get_package_share_directory('bocbot_hunter')

    # Fichiers
    world_file = os.path.join(pkg_bocbot, 'worlds', 'arena.world')
    urdf_file = os.path.join(pkg_bocbot, 'urdf', 'bocbot.urdf.xacro')

    # Robot description
    robot_description = Command(['xacro ', urdf_file])

    return LaunchDescription([
        # Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
            output='screen'
        ),

        # Spawn robot au centre de l'arene
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'bocbot', '-topic', 'robot_description',
                       '-x', '0.0', '-y', '0.0', '-z', '0.15'],
            output='screen'
        ),

        # Fast Hunter - LIDAR ONLY (pas de camera pour la detection)
        Node(
            package='bocbot_hunter',
            executable='fast_hunter',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
    ])
