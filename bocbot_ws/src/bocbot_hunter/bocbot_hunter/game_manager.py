#!/usr/bin/env python3
"""
Game Manager - Gestion du score et reset automatique
Utilise /gazebo/model_states (topic) et /gazebo/set_entity_state (service)
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
import time
import math
import threading


class GameManager(Node):
    def __init__(self):
        super().__init__('game_manager')

        # Scores
        self.score_blue = 0
        self.score_orange = 0

        # Position de la balle (pour détection but)
        self.ball_x = 0.0
        self.ball_y = 0.0
        self.ball_z = 0.0
        self.prev_ball_x = 0.0

        # Position des robots pour reset
        self.robot_positions = {}

        # Positions initiales
        self.blue_start = {'x': -4.0, 'y': 0.0, 'z': 0.16, 'yaw': 0.0}
        self.orange_start = {'x': 4.0, 'y': 0.0, 'z': 0.16, 'yaw': 3.14159}
        self.ball_start = {'x': 0.0, 'y': 0.0, 'z': 4.0}  # Plus haut pour tomber bien

        # Limites du terrain - cages a x = +/-9.0, largeur 5m (y = -2.5 a 2.5)
        self.goal_x = 9.0  # Position des cages
        self.goal_width = 2.6  # Un peu plus que 2.5 pour marge
        self.goal_max_z = 1.0

        # Reset state
        self.resetting = False
        self.reset_start_time = 0.0
        self.RESET_DELAY = 2.0  # secondes

        # Service client pour set (async avec callback)
        self.set_state_cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')

        # Subscription model_states (fourni par le plugin world)
        self.create_subscription(ModelStates, '/gazebo/model_states', self.model_states_cb, 10)

        # Publisher du score
        self.score_pub = self.create_publisher(String, '/game/score', 10)

        self.get_logger().info('🏆 GAME MANAGER - Démarrage!')
        self.get_logger().info('   BLEU (WASD) vs ORANGE (Fleches)')
        self.get_logger().info('   Attente /gazebo/model_states...')

        # Flag pour savoir si on a reçu des données
        self.states_received = False
        self._publish_score()

    def model_states_cb(self, msg):
        """Callback pour /gazebo/model_states - mise à jour positions"""
        if not self.states_received:
            self.states_received = True
            self.get_logger().info('   /gazebo/model_states reçu!')

        if self.resetting:
            # Pendant reset, on ignore les buts
            return

        try:
            # Trouver la balle
            if 'ball' in msg.name:
                ball_idx = msg.name.index('ball')
                self.prev_ball_x = self.ball_x
                self.ball_x = msg.pose[ball_idx].position.x
                self.ball_y = msg.pose[ball_idx].position.y
                self.ball_z = msg.pose[ball_idx].position.z

                # Sauvegarder positions robots pour reset
                if 'bocbot_blue' in msg.name:
                    blue_idx = msg.name.index('bocbot_blue')
                    self.robot_positions['blue'] = msg.pose[blue_idx]
                if 'bocbot_orange' in msg.name:
                    orange_idx = msg.name.index('bocbot_orange')
                    self.robot_positions['orange'] = msg.pose[orange_idx]

                # Détecter les buts
                self.check_goals()

        except (ValueError, IndexError):
            pass

    def check_goals(self):
        """Vérifie si un but a été marqué"""

        # Détecter les buts
        # BUT ORANGE: balle traverse x=7.5 (cage orange à droite) = point pour BLEU
        if self.prev_ball_x < self.goal_x and self.ball_x >= self.goal_x:
            if abs(self.ball_y) < self.goal_width and self.ball_z < self.goal_max_z:
                self._goal_blue()
                return

        # BUT BLEU: balle traverse x=-7.5 (cage bleue à gauche) = point pour ORANGE
        if self.prev_ball_x > -self.goal_x and self.ball_x <= -self.goal_x:
            if abs(self.ball_y) < self.goal_width and self.ball_z < self.goal_max_z:
                self._goal_orange()
                return

    def _goal_blue(self):
        """But marqué par BLEU (dans cage orange)"""
        self.score_blue += 1
        self.get_logger().info(f'🔵 BUT BLEU! Score: {self.score_blue} - {self.score_orange}')
        self._publish_score()
        self._start_reset()

    def _goal_orange(self):
        """But marqué par ORANGE (dans cage bleue)"""
        self.score_orange += 1
        self.get_logger().info(f'🟠 BUT ORANGE! Score: {self.score_blue} - {self.score_orange}')
        self._publish_score()
        self._start_reset()

    def _publish_score(self):
        """Publie le score sur le topic"""
        msg = String()
        msg.data = f'{self.score_blue}:{self.score_orange}'
        self.score_pub.publish(msg)

    def _start_reset(self):
        """Démarre le countdown de reset"""
        self.resetting = True
        self.reset_start_time = time.time()
        self.get_logger().info(f'⏱️  Reset dans {self.RESET_DELAY}s...')

        # Lancer timer pour reset effectif
        threading.Timer(self.RESET_DELAY, self._do_reset).start()

    def _create_pose(self, x, y, z, yaw=0.0):
        """Crée un Pose Gazebo"""
        pose = Pose()
        pose.position = Point(x=x, y=y, z=z)
        pose.orientation = Quaternion(
            x=0.0, y=0.0,
            z=math.sin(yaw / 2.0),
            w=math.cos(yaw / 2.0)
        )
        return pose

    def _do_reset(self):
        """Effectue le reset des positions via service Gazebo"""
        if not rclpy.ok():
            return

        # Reset balle (en l'air au centre)
        self._call_set_state('ball',
                           self.ball_start['x'],
                           self.ball_start['y'],
                           self.ball_start['z'])

        # Reset robots
        self._call_set_state('bocbot_blue',
                           self.blue_start['x'],
                           self.blue_start['y'],
                           self.blue_start['z'],
                           self.blue_start['yaw'])

        self._call_set_state('bocbot_orange',
                           self.orange_start['x'],
                           self.orange_start['y'],
                           self.orange_start['z'],
                           self.orange_start['yaw'])

        self.resetting = False
        self.get_logger().info('🔄 Positions réinitialisées!')

    def _call_set_state(self, entity_name, x, y, z, yaw=0.0):
        """Appelle le service SetEntityState de manière bloquante"""
        if not self.set_state_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'Service /gazebo/set_entity_state pas disponible pour {entity_name}')
            return

        request = SetEntityState.Request()
        request.state.name = entity_name
        request.state.pose = self._create_pose(x, y, z, yaw)
        request.state.reference_frame = ''

        # Appel synchrone pour attendre la réponse
        future = self.set_state_cli.call_async(request)
        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = GameManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'SCORE FINAL: BLEU {node.score_blue} - {node.score_orange} ORANGE')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
