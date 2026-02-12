#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Fichier de lancement pour le système complet Bocbot Hunter.
    Lance tous les nodes nécessaires:
    1. Ball Detector - Détecte la boule blanche avec OpenCV
    2. Obstacle Avoidance - Analyse le laser pour éviter les obstacles
    3. Hunter Controller - Combine les informations et contrôle le robot
    """
    
    # Déclaration des arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Utiliser le temps de simulation'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Node Ball Detector
    ball_detector_node = Node(
        package='bocbot_hunter',
        executable='ball_detector',
        name='ball_detector',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    # Node Obstacle Avoidance
    obstacle_avoidance_node = Node(
        package='bocbot_hunter',
        executable='obstacle_avoidance',
        name='obstacle_avoidance',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    # Node Hunter Controller
    hunter_controller_node = Node(
        package='bocbot_hunter',
        executable='hunter_controller',
        name='hunter_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    # Rviz pour la visualisation (optionnel)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('bocbot_hunter'),
            'rviz',
            'bocbot_hunter.rviz'
        )] if os.path.exists(os.path.join(
            get_package_share_directory('bocbot_hunter'),
            'rviz',
            'bocbot_hunter.rviz'
        )) else [],
        condition=None,  # Toujours lancer Rviz
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', 'warn']
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        ball_detector_node,
        obstacle_avoidance_node,
        hunter_controller_node,
        # rviz_node,  # Commenté pour éviter les erreurs si Rviz n'est pas configuré
    ])