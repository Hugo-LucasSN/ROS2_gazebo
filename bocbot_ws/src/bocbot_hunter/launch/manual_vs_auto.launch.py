#!/usr/bin/env python3
"""
Launch: Robot BLEU + Robot ORANGE + Gazebo + GameManager
Le controle clavier est lance separement par launch.sh (game_viewer)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('bocbot_hunter')
    world = os.path.join(pkg, 'worlds', 'arena.world')
    urdf = os.path.join(pkg, 'urdf', 'bocbot.urdf.xacro')

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world,
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    rsp_blue = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', namespace='bocbot_blue',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', urdf, ' robot_name:=bocbot_blue color:=Blue']),
                value_type=str
            ),
            'use_sim_time': True, 'frame_prefix': 'bocbot_blue/'
        }], output='screen')

    rsp_orange = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', namespace='bocbot_orange',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', urdf, ' robot_name:=bocbot_orange color:=Orange']),
                value_type=str
            ),
            'use_sim_time': True, 'frame_prefix': 'bocbot_orange/'
        }], output='screen')

    spawn_blue = TimerAction(period=2.0, actions=[
        Node(package='gazebo_ros', executable='spawn_entity.py', name='spawn_blue',
             arguments=['-entity', 'bocbot_blue',
                        '-topic', '/bocbot_blue/robot_description',
                        '-robot_namespace', '/bocbot_blue',
                        '-x', '-4.0', '-y', '0.0', '-z', '0.16', '-Y', '0.0'],
             output='screen')])

    spawn_orange = TimerAction(period=3.0, actions=[
        Node(package='gazebo_ros', executable='spawn_entity.py', name='spawn_orange',
             arguments=['-entity', 'bocbot_orange',
                        '-topic', '/bocbot_orange/robot_description',
                        '-robot_namespace', '/bocbot_orange',
                        '-x', '4.0', '-y', '0.0', '-z', '0.16', '-Y', '3.14159'],
             output='screen')])

    # GameManager pour le score et reset automatique
    game_manager = TimerAction(period=5.0, actions=[
        Node(package='bocbot_hunter', executable='game_manager',
             name='game_manager',
             output='screen')])

    # Uprighter: garde les robots au sol et auto-flip si renverses
    uprighter = TimerAction(period=5.5, actions=[
        Node(package='bocbot_hunter', executable='robot_uprighter',
             output='screen')])

    return LaunchDescription([
        gazebo, rsp_blue, rsp_orange,
        spawn_blue, spawn_orange, game_manager, uprighter,
    ])
