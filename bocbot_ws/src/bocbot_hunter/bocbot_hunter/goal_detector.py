#!/usr/bin/env python3
"""
Goal Detector - Détecte les buts et respawn robots + balle
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose, Twist
import time
import numpy as np


class GoalDetector(Node):
    def __init__(self):
        super().__init__('goal_detector')

        # Scores
        self.score_blue = 0
        self.score_orange = 0

        # État du jeu
        self.ball_in_goal = False
        self.respawn_timer = 0
        self.tick = 0

        # Client pour téléporter modèles
        self.set_state_client = self.create_client(
            SetEntityState, '/gazebo/set_entity_state')

        # Subscriber pour positions
        self.create_subscription(
            ModelStates, '/gazebo/model_states', self.model_states_cb, 10)

        # Timer pour vérifier respawn
        self.create_timer(0.1, self.check_respawn)

        # Timer pour afficher score périodiquement
        self.create_timer(15.0, self.display_score)

        self.get_logger().info('')
        self.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
        self.get_logger().info('⚽ GOAL DETECTOR READY - MATCH DE FOOT! ⚽')
        self.get_logger().info('📊 SCORE INITIAL: 🔵 BLEU 0 - 0 ORANGE 🟠')
        self.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
        self.get_logger().info('')

    def model_states_cb(self, msg):
        """Surveille position de la balle"""
        try:
            ball_idx = msg.name.index('ball')
            ball_pos = msg.pose[ball_idx].position

            # Vérifier si but marqué
            if not self.ball_in_goal:
                if ball_pos.x < -7.2:  # BUT POUR ORANGE (balle dans cage bleue)
                    self.score_orange += 1
                    self.ball_in_goal = True
                    self.respawn_timer = time.time()
                    self.get_logger().info('')
                    self.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
                    self.get_logger().info('⚽⚽⚽ BUT POUR ORANGE! 🟠 ⚽⚽⚽')
                    self.get_logger().info(f'📊 SCORE: 🔵 BLEU {self.score_blue} - {self.score_orange} ORANGE 🟠')
                    self.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
                    self.get_logger().info('')

                elif ball_pos.x > 7.2:  # BUT POUR BLEU (balle dans cage orange)
                    self.score_blue += 1
                    self.ball_in_goal = True
                    self.respawn_timer = time.time()
                    self.get_logger().info('')
                    self.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
                    self.get_logger().info('⚽⚽⚽ BUT POUR BLEU! 🔵 ⚽⚽⚽')
                    self.get_logger().info(f'📊 SCORE: 🔵 BLEU {self.score_blue} - {self.score_orange} ORANGE 🟠')
                    self.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
                    self.get_logger().info('')

        except (ValueError, IndexError):
            pass

    def display_score(self):
        """Affiche le score périodiquement"""
        if self.score_blue > 0 or self.score_orange > 0:
            self.get_logger().info(f'📊 SCORE ACTUEL: 🔵 BLEU {self.score_blue} - {self.score_orange} ORANGE 🟠')

    def check_respawn(self):
        """Respawn après 2 secondes quand but marqué"""
        self.tick += 1
        if self.ball_in_goal and (time.time() - self.respawn_timer) > 2.0:
            self.get_logger().info('🔄 RESPAWN - Nouvelle manche!')
            self.respawn_all()
            self.ball_in_goal = False

    def respawn_all(self):
        """Téléporte balle et robots aux positions de départ"""
        # Attendre que le service soit disponible
        while not self.set_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service set_entity_state pas disponible...')

        # Respawn BALLE au centre
        self.teleport_model('ball', 0.0, 0.0, 0.15, 0.0)

        # Respawn ROBOT BLEU dans cage bleue
        self.teleport_model('bocbot_blue', -5.0, 0.0, 0.15, 0.0)

        # Respawn ROBOT ORANGE dans cage orange
        self.teleport_model('bocbot_orange', 5.0, 0.0, 0.15, 3.14159)

        self.get_logger().info('✅ Respawn terminé!')

    def teleport_model(self, name, x, y, z, yaw):
        """Téléporte un modèle à une position"""
        from gazebo_msgs.srv import SetEntityState
        from gazebo_msgs.msg import EntityState

        request = SetEntityState.Request()
        state = EntityState()
        state.name = name
        state.pose = Pose()
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z
        # Quaternion pour yaw
        state.pose.orientation.z = np.sin(yaw / 2.0)
        state.pose.orientation.w = np.cos(yaw / 2.0)
        state.twist = Twist()  # Vitesse nulle

        request.state = state

        future = self.set_state_client.call_async(request)
        # Ne pas attendre la réponse pour ne pas bloquer


def main(args=None):
    rclpy.init(args=args)
    node = GoalDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
