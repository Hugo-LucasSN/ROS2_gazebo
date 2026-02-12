#!/usr/bin/env python3
"""
Soccer Match - Match de foot: Bleu vs Orange avec respawn
Nettoie tous les processus Python au démarrage et à l'arrêt
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('bocbot_hunter')
    world = os.path.join(pkg, 'worlds', 'arena.world')
    urdf = os.path.join(pkg, 'urdf', 'bocbot.urdf.xacro')

    # Cleanup process - tue tous les Python/Gazebo zombies au démarrage
    cleanup_process = ExecuteProcess(
        cmd=['bash', '-c', 'pkill -9 -f "dual_pov_viewer" 2>/dev/null; pkill -9 -f "run_pov_viewer" 2>/dev/null; echo "✓ Nettoyage processus zombies terminé"'],
        output='screen'
    )

    # Gazebo avec terrain de foot
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world,
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Cleanup shutdown - tue tous les processus quand Gazebo s'arrête
    cleanup_shutdown = ExecuteProcess(
        cmd=['bash', '-c', 'sleep 2; killall -9 python3 python gzserver gzclient 2>/dev/null; echo "✓ Nettoyage final terminé"'],
        output='screen'
    )

    return LaunchDescription([
        # 1. NETTOYAGE AU DÉMARRAGE
        cleanup_process,

        # 2. Gazebo avec terrain de foot
        gazebo,

        # 3. Robot State Publisher BLEU
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='bocbot_blue',
            parameters=[{
                'robot_description': Command(['xacro ', urdf, ' robot_name:=bocbot_blue color:=Blue']),
                'use_sim_time': True,
                'frame_prefix': 'bocbot_blue/'
            }],
            output='screen'
        ),

        # 4. Robot State Publisher ORANGE
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='bocbot_orange',
            parameters=[{
                'robot_description': Command(['xacro ', urdf, ' robot_name:=bocbot_orange color:=Orange']),
                'use_sim_time': True,
                'frame_prefix': 'bocbot_orange/'
            }],
            output='screen'
        ),

        # 5. Spawn robot BLEU (gauche)
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_blue',
            arguments=[
                '-entity', 'bocbot_blue',
                '-topic', '/bocbot_blue/robot_description',
                '-robot_namespace', '/bocbot_blue',
                '-x', '-5.0', '-y', '0.0', '-z', '0.16', '-Y', '0.0'
            ],
            output='screen'
        ),

        # 6. Spawn robot ORANGE (droite)
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_orange',
            arguments=[
                '-entity', 'bocbot_orange',
                '-topic', '/bocbot_orange/robot_description',
                '-robot_namespace', '/bocbot_orange',
                '-x', '5.0', '-y', '0.0', '-z', '0.16', '-Y', '0.0'
            ],
            output='screen'
        ),

        # 7. Color Ring Detector BLEU
        Node(
            package='bocbot_hunter',
            executable='color_ring_detector',
            name='color_ring_detector',
            namespace='bocbot_blue',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        # 8. Color Ring Detector ORANGE
        Node(
            package='bocbot_hunter',
            executable='color_ring_detector',
            name='color_ring_detector',
            namespace='bocbot_orange',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        # 9. Contrôleur BLEU (LIDAR + Color fusion)
        Node(
            package='bocbot_hunter',
            executable='fast_hunter',
            name='fast_hunter',
            namespace='bocbot_blue',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        # 10. Contrôleur ORANGE (LIDAR + Color fusion)
        Node(
            package='bocbot_hunter',
            executable='fast_hunter',
            name='fast_hunter',
            namespace='bocbot_orange',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        # 11. Score Tracker - Détecte les buts et affiche le score
        Node(
            package='bocbot_hunter',
            executable='score_tracker',
            name='score_tracker',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        # 12. Dual POV Viewer - Affiche les caméras des deux robots (avec auto-restart)
        ExecuteProcess(
            cmd=['bash', '/home/seatech/Bureau/ROS2_SEATECH/projet/bocbot_ws/src/bocbot_hunter/scripts/run_pov_viewer.sh'],
            output='screen'
        ),

        # 13. NETTOYAGE À L'ARRÊT (quand Gazebo s'arrête)
        RegisterEventHandler(
            OnProcessExit(
                target_action=gazebo,
                on_exit=[cleanup_shutdown]
            )
        ),
    ])
