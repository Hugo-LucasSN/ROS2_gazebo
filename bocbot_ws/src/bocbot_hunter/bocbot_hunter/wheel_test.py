#!/usr/bin/env python3
"""
Wheel Test - Test simple pour voir les roues tourner
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class WheelTest(Node):
    def __init__(self):
        super().__init__('wheel_test')

        self.cmd_pub = self.create_publisher(Twist, '/bocbot/cmd_vel', 10)

        self.create_subscription(Odometry, '/bocbot/odom', self.odom_cb, 10)

        self.last_x = 0.0
        self.rotations = 0

        # Timer pour envoyer des commandes et vérifier l'odom
        self.create_timer(0.1, self.test_loop)

        self.get_logger().info('WHEEL TEST - Envoi commandes et vérification odom')

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        moved = abs(x - self.last_x)
        self.last_x = x

        if moved > 0.001:
            self.rotations += 1
            if self.rotations % 10 == 0:
                self.get_logger().info(f'Robot BOUGE! x={x:.3f} moved={moved:.4f}')

    def test_loop(self):
        # Envoyer commande simple: tout droit
        twist = Twist()
        twist.linear.x = 0.5  # Vitesse réaliste pour le robot
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = WheelTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
