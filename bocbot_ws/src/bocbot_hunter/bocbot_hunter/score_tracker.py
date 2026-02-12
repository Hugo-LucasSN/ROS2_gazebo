#!/usr/bin/env python3
"""
Score Tracker - Détecte les buts et affiche le score sur le scoreboard
Suit la position de la balle et détecte quand elle entre dans un but
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker
from std_msgs.msg import String
import math


class ScoreTracker(Node):
    def __init__(self):
        super().__init__('score_tracker')

        # Scores
        self.score_blue = 0
        self.score_orange = 0

        # Position de la balle
        self.ball_x = 0.0
        self.ball_y = 0.0
        self.ball_z = 0.0

        # Position précédente pour détecter l'entrée dans le but
        self.prev_ball_x = 0.0

        # États des buts
        self.blue_goal_x = -7.0  # But bleu à gauche
        self.orange_goal_x = 7.0  # But orange à droite
        self.goal_depth = 1.0  # Profondeur du but

        # Temps de reset après but
        self.reset_timer = 0
        self.RESET_TIME = 100  # ticks à 500Hz = 0.2 sec

        # Publisher pour afficher les scores
        self.marker_pub_blue = self.create_publisher(Marker, '/score_marker_blue', 10)
        self.marker_pub_orange = self.create_publisher(Marker, '/score_marker_orange', 10)

        # Subscriber pour la position de la balle
        self.create_subscription(ModelStates, '/model_states', self.model_states_cb, 10)

        # Timer pour mettre à jour l'affichage (100Hz)
        self.create_timer(0.01, self.update_display)
        self.tick = 0

        self.get_logger().info('🏆 SCORE TRACKER - Démarrage!')

    def model_states_cb(self, msg):
        """Récupère la position de la balle depuis Gazebo"""
        try:
            ball_idx = msg.name.index('ball')
            self.prev_ball_x = self.ball_x
            self.ball_x = msg.pose[ball_idx].position.x
            self.ball_y = msg.pose[ball_idx].position.y
            self.ball_z = msg.pose[ball_idx].position.z

            # Détecter les buts
            self.detect_goals()
        except ValueError:
            pass

    def detect_goals(self):
        """Détecte si la balle est entrée dans un but"""
        # Vérifier si la balle est dans la zone de but (en profondeur)
        ball_in_goal_area = (self.ball_z < 0.5)  # Balle au sol/près du sol

        if not ball_in_goal_area:
            return

        # DÉTECTION BUT ORANGE (balle dans le but de droite = point pour BLEU)
        # La balle doit traverser x=7 vers la droite
        if self.prev_ball_x < self.orange_goal_x and self.ball_x >= self.orange_goal_x:
            # Vérifier que la balle est dans les limites y du but (-2 à +2)
            if abs(self.ball_y) < 2.0:
                self.score_blue += 1
                self.get_logger().info(f'🔵 BUT BLEU! Score: {self.score_blue} - {self.score_orange}')
                self.reset_ball()

        # DÉTECTION BUT BLEU (balle dans le but de gauche = point pour ORANGE)
        # La balle doit traverser x=-7 vers la gauche
        elif self.prev_ball_x > self.blue_goal_x and self.ball_x <= self.blue_goal_x:
            # Vérifier que la balle est dans les limites y du but (-2 à +2)
            if abs(self.ball_y) < 2.0:
                self.score_orange += 1
                self.get_logger().info(f'🟠 BUT ORANGE! Score: {self.score_blue} - {self.score_orange}')
                self.reset_ball()

    def reset_ball(self):
        """Réinitialise la balle au centre en l'air"""
        # Note: Dans une vraie implémentation, on utiliserait un service Gazebo
        # Pour l'instant, on loggue simplement
        self.get_logger().info('🍅 Réinitialisation balle...')
        self.reset_timer = self.RESET_TIME

    def update_display(self):
        """Met à jour l'affichage du score sur le scoreboard"""
        self.tick += 1

        # Afficher score BLEU
        self.create_score_marker(self.score_blue, 'blue')

        # Afficher score ORANGE
        self.create_score_marker(self.score_orange, 'orange')

    def create_score_marker(self, score, team):
        """Crée un marker pour afficher le score"""
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f"score_{team}"
        marker.id = int(score)  # Change l'ID pour forcer la mise à jour
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        # Position sur le scoreboard
        if team == 'blue':
            marker.pose.position.x = -1.5
            marker.pose.position.y = 0.0
            marker.pose.position.z = 6.3
            marker.color.r = 0.3
            marker.color.g = 0.3
            marker.color.b = 1.0
        else:  # orange
            marker.pose.position.x = 1.5
            marker.pose.position.y = 0.0
            marker.pose.position.z = 6.3
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.0

        marker.pose.position.z = 6.3
        marker.pose.orientation.w = 1.0

        # Taille du texte
        marker.scale.z = 1.5  # Hauteur du texte

        # Texte du score
        marker.text = str(score)

        marker.color.a = 1.0

        # Publier le marker
        if team == 'blue':
            self.marker_pub_blue.publish(marker)
        else:
            self.marker_pub_orange.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = ScoreTracker()

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
