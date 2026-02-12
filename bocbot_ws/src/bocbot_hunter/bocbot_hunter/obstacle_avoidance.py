#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np


class ObstacleAvoidanceNode(Node):
    """
    Node ROS2 pour analyser les données du scanner laser
    et identifier les zones libres et les obstacles.
    """
    
    def __init__(self):
        super().__init__('obstacle_avoidance')
        
        # Paramètres de sécurité
        self.safety_distance = 0.8  # Distance de sécurité en mètres
        self.critical_distance = 0.4  # Distance critique en mètres
        self.front_angle_range = np.pi / 2  # ±90° devant le robot
        
        # Diviser le scan en secteurs pour l'analyse
        self.num_sectors = 5  # Gauche, Avant-Gauche, Avant, Avant-Droite, Droite
        self.sector_angles = np.linspace(
            -np.pi, 
            np.pi, 
            self.num_sectors + 1
        )[:-1]  # Centre de chaque secteur
        
        # Publisher pour les directions libres
        self.free_directions_pub = self.create_publisher(
            LaserScan,
            '/bocbot/free_directions',
            10
        )
        
        # Subscriber pour le scan laser
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/bocbot/scan',
            self.scan_callback,
            10
        )
        
        self.get_logger().info('Obstacle Avoidance Node démarré')
        
    def scan_callback(self, msg):
        """
        Callback appelé à chaque nouveau scan laser.
        Analyse les données pour identifier les obstacles.
        
        Args:
            msg: Message LaserScan contenant les données du scanner
        """
        # Créer un nouveau scan avec les directions libres
        free_scan = self.analyze_scan(msg)
        
        # Publier les directions libres
        self.free_directions_pub.publish(free_scan)
        
        # Logger les informations importantes
        self.log_obstacle_info(free_scan)
    
    def analyze_scan(self, scan_msg):
        """
        Analyse le scan laser pour déterminer les directions libres.
        
        Args:
            scan_msg: Message LaserScan original
            
        Returns:
            LaserScan: Scan modifié avec les directions libres marquées
        """
        # Créer une copie du scan original
        free_scan = LaserScan()
        free_scan.header = scan_msg.header
        free_scan.angle_min = scan_msg.angle_min
        free_scan.angle_max = scan_msg.angle_max
        free_scan.angle_increment = scan_msg.angle_increment
        free_scan.time_increment = scan_msg.time_increment
        free_scan.scan_time = scan_msg.scan_time
        free_scan.range_min = scan_msg.range_min
        free_scan.range_max = scan_msg.range_max
        free_scan.ranges = list(scan_msg.ranges)
        free_scan.intensities = list(scan_msg.intensities) if scan_msg.intensities else []
        
        # Analyser chaque secteur
        sector_status = {
            'front_left': True,
            'front': True,
            'front_right': True,
            'left': True,
            'right': True
        }
        
        num_ranges = len(scan_msg.ranges)
        
        # Analyser les distances dans chaque secteur
        for i in range(num_ranges):
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            distance = scan_msg.ranges[i]
            
            # Ignorer les valeurs invalides
            if not scan_msg.range_min <= distance <= scan_msg.range_max:
                continue
            
            # Déterminer le secteur
            sector = self.get_sector(angle)
            
            # Vérifier s'il y a un obstacle dans ce secteur
            if distance < self.safety_distance:
                if sector in sector_status:
                    sector_status[sector] = False
                
                # Marquer les points critiques
                if distance < self.critical_distance:
                    # Modifier la distance pour indiquer un obstacle critique
                    # (optionnel: on pourrait mettre une valeur spéciale)
                    pass
        
        # Logger le statut des secteurs
        self.get_logger().debug(f'Secteurs: {sector_status}')
        
        # Stocker le statut pour le contrôleur
        self.sector_status = sector_status
        
        return free_scan
    
    def get_sector(self, angle):
        """
        Détermine à quel secteur appartient un angle donné.
        
        Args:
            angle: Angle en radians (-π à π)
            
        Returns:
            str: Nom du secteur
        """
        # Normaliser l'angle entre -π et π
        angle = (angle + np.pi) % (2 * np.pi) - np.pi
        
        # Avant
        if -self.front_angle_range / 2 <= angle <= self.front_angle_range / 2:
            return 'front'
        # Avant-gauche
        elif -np.pi <= angle < -self.front_angle_range / 2:
            return 'front_left'
        # Avant-droite
        elif self.front_angle_range / 2 < angle <= np.pi:
            return 'front_right'
        else:
            # Cas par défaut
            if angle < 0:
                return 'left'
            else:
                return 'right'
    
    def log_obstacle_info(self, scan_msg):
        """
        Log des informations importantes sur les obstacles.
        
        Args:
            scan_msg: Message LaserScan
        """
        if not hasattr(self, 'sector_status'):
            return
        
        # Trouver l'obstacle le plus proche
        ranges = [r for r in scan_msg.ranges 
                  if scan_msg.range_min <= r <= scan_msg.range_max]
        
        if ranges:
            min_distance = min(ranges)
            
            if min_distance < self.critical_distance:
                self.get_logger().warn(
                    f'⚠️  OBSTACLE CRITIQUE à {min_distance:.2f} m!'
                )
            elif min_distance < self.safety_distance:
                self.get_logger().info(
                    f'Obstacle proche à {min_distance:.2f} m - '
                    f'Libre devant: {self.sector_status["front"]}'
                )
            else:
                self.get_logger().debug(
                    f'Zone dégagée - Distance min: {min_distance:.2f} m'
                )
    
    def is_direction_clear(self, direction):
        """
        Vérifie si une direction est libre d'obstacles.
        
        Args:
            direction: 'front', 'left', 'right', 'front_left', 'front_right'
            
        Returns:
            bool: True si la direction est libre
        """
        if not hasattr(self, 'sector_status'):
            return True
        
        return self.sector_status.get(direction, True)


def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance = ObstacleAvoidanceNode()
    
    try:
        rclpy.spin(obstacle_avoidance)
    except KeyboardInterrupt:
        pass
    finally:
        obstacle_avoidance.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()