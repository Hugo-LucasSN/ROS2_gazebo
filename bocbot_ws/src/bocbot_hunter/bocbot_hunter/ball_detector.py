#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from bocbot_interfaces.msg import BallDetection


class BallDetectorNode(Node):
    """
    Node ROS2 pour détecter une boule blanche dans les images de la caméra.
    Utilise OpenCV pour le traitement d'image.
    """
    
    def __init__(self):
        super().__init__('ball_detector')
        
        # CvBridge pour convertir les messages ROS en images OpenCV
        self.bridge = CvBridge()
        
        # Parametres de detection (optimise pour 160x120)
        self.min_radius = 2  # Plus petit pour detecter de loin
        self.max_radius = 80
        self.image_width = 160
        self.image_height = 120
        self.camera_fov = 1.4
        self.ball_real_radius = 0.15
        
        # Publisher pour les détections
        self.detection_pub = self.create_publisher(
            BallDetection,
            '/bocbot/ball_detection',
            10
        )
        
        # Subscriber pour les images de la caméra
        self.image_sub = self.create_subscription(
            Image,
            '/bocbot/camera/image',
            self.image_callback,
            10
        )
        
        self.get_logger().info('Ball Detector Node démarré')
        
    def image_callback(self, msg):
        """
        Callback appelé à chaque nouvelle image de la caméra.
        Traite l'image pour détecter la boule blanche.
        """
        try:
            # Convertir l'image ROS en image OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Détecter la boule blanche
            detection = self.detect_white_ball(cv_image)
            
            # Publier la détection
            self.detection_pub.publish(detection)
            
        except Exception as e:
            self.get_logger().error(f'Erreur lors du traitement de l\'image: {e}')
    
    def detect_white_ball(self, image):
        """
        Détecte une boule blanche dans l'image.
        
        Args:
            image: Image OpenCV (BGR)
            
        Returns:
            BallDetection: Message contenant les informations de détection
        """
        # Créer un message de détection par défaut (pas de détection)
        detection = BallDetection()
        detection.confidence = 0.0
        detection.angle = 0.0
        detection.distance = 0.0
        detection.radius_pixels = 0.0
        detection.center_x = 0
        detection.center_y = 0
        
        # Convertir en espace HSV pour une meilleure détection de couleur
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Définir la plage de couleur blanche en HSV
        # Plus permissif pour l'éclairage Gazebo
        # Blanc/gris clair: faible saturation, haute valeur
        lower_white = np.array([0, 0, 160])
        upper_white = np.array([180, 50, 255])
        
        # Créer un masque pour les pixels blancs
        mask = cv2.inRange(hsv, lower_white, upper_white)
        
        # Appliquer des opérations morphologiques pour nettoyer le masque
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)
        
        # Trouver les contours dans le masque
        contours, _ = cv2.findContours(
            mask, 
            cv2.RETR_EXTERNAL, 
            cv2.CHAIN_APPROX_SIMPLE
        )
        
        if contours:
            # Trouver le plus grand contour (présumé être la boule)
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            # Calculer les propriétés du contour
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)
            
            # Vérifier que le rayon est dans une plage raisonnable
            if self.min_radius <= radius <= self.max_radius:
                # Calculer le centre de l'image
                image_center_x = self.image_width // 2
                
                # Calculer l'angle par rapport au centre
                # angle = atan2((x - center_x), focal_length)
                # On simplifie en utilisant le champ de vision
                angle = ((x - image_center_x) / (self.image_width / 2)) * (self.camera_fov / 2)
                
                # Estimer la distance basée sur le rayon en pixels
                # distance = (rayon_réel * focal_length) / rayon_pixels
                # On utilise une approximation simple
                focal_length_pixels = self.image_width / (2 * np.tan(self.camera_fov / 2))
                estimated_distance = (self.ball_real_radius * focal_length_pixels) / radius
                
                # Calculer la confiance basée sur la circularité et la taille
                circularity = 4 * np.pi * area / (2 * np.pi * radius) ** 2
                confidence = min(1.0, circularity * 0.7 + (radius / 100) * 0.3)
                
                # Remplir le message de détection
                detection.angle = float(angle)
                detection.distance = float(estimated_distance)
                detection.radius_pixels = float(radius)
                detection.confidence = float(confidence)
                detection.center_x = int(x)
                detection.center_y = int(y)
                
                # Dessiner pour le débogage (optionnel)
                cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 0), 2)
                cv2.circle(image, (int(x), int(y)), 5, (0, 0, 255), -1)
                
                self.get_logger().info(
                    f'Balle détectée - Angle: {angle:.3f} rad, '
                    f'Distance: {estimated_distance:.2f} m, '
                    f'Confiance: {confidence:.2f}'
                )
            else:
                self.get_logger().debug(
                    f'Contour trouvé mais rayon hors plage: {radius:.1f} pixels'
                )
        
        return detection


def main(args=None):
    rclpy.init(args=args)
    ball_detector = BallDetectorNode()
    
    try:
        rclpy.spin(ball_detector)
    except KeyboardInterrupt:
        pass
    finally:
        ball_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()