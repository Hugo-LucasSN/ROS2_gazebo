#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from bocbot_interfaces.msg import BallDetection
import math
import time


class FakeBallPublisher(Node):
    """
    Node de test qui publie de fausses détections de balle.
    Utile pour tester le système sans vraie caméra.
    Simule une balle qui bouge en cercle autour du robot.
    """
    
    def __init__(self):
        super().__init__('fake_ball_publisher')
        
        # Publisher pour les détections de balle
        self.ball_pub = self.create_publisher(
            BallDetection,
            '/bocbot/ball_detection',
            10
        )
        
        # Timer pour publier les détections (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_detection)
        
        # Paramètres de simulation
        self.start_time = time.time()
        self.ball_radius = 2.0  # Rayon du cercle
        self.ball_speed = 0.5  # Vitesse angulaire
        self.ball_distance_base = 1.5  # Distance moyenne
        
        self.get_logger().info('Fake Ball Publisher Node démarré')
        self.get_logger().info('Simule une balle qui tourne autour du robot')
        
    def publish_detection(self):
        """
        Publie une fausse détection de balle qui tourne autour du robot.
        """
        # Calculer la position de la balle en fonction du temps
        elapsed = time.time() - self.start_time
        angle = self.ball_speed * elapsed
        
        # Position de la balle en coordonnées polaires
        ball_x = self.ball_radius * math.cos(angle)
        ball_y = self.ball_radius * math.sin(angle)
        
        # Calculer l'angle vers la balle depuis le robot (à l'origine)
        ball_angle = math.atan2(ball_y, ball_x)
        
        # Distance de la balle (avec légère variation pour le réalisme)
        distance = self.ball_distance_base + 0.2 * math.sin(3 * angle)
        
        # Créer le message de détection
        detection = BallDetection()
        detection.angle = float(ball_angle)
        detection.distance = float(distance)
        detection.radius_pixels = float(50.0 + 10.0 * math.sin(2 * angle))  # Taille apparente
        detection.confidence = float(0.8 + 0.2 * math.cos(angle))  # Variation de confiance
        detection.center_x = int(400 + 300 * math.sin(angle))  # Position dans l'image
        detection.center_y = int(400 + 200 * math.cos(angle))
        
        # Publier la détection
        self.ball_pub.publish(detection)
        
        # Logger périodiquement
        if int(elapsed * 10) % 20 == 0:  # Toutes les 2 secondes
            self.get_logger().info(
                f'🎯 Balle simulée - Angle: {math.degrees(ball_angle):.1f}°, '
                f'Distance: {distance:.2f} m, '
                f'Confiance: {detection.confidence:.2f}'
            )


def main(args=None):
    rclpy.init(args=args)
    fake_ball_publisher = FakeBallPublisher()
    
    try:
        rclpy.spin(fake_ball_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        fake_ball_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()