#!/usr/bin/env python3
"""
Basic Mover - Node minimal qui fait bouger le robot
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np


class BasicMover(Node):
    def __init__(self):
        super().__init__('basic_mover')

        # Etat
        self.min_dist = 10.0
        self.tick = 0
        self.forward_speed = 1.0
        self.turn_speed = 0.0

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/bocbot/cmd_vel', 10)

        # Subscriber
        self.create_subscription(LaserScan, '/bocbot/scan', self.scan_cb, 10)

        # Timer 20Hz
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info('BASIC MOVER - Robot en mouvement!')

    def scan_cb(self, msg):
        ranges = np.array(msg.ranges)
        valid = ranges[(ranges > 0.1) & (ranges < 10)]
        if len(valid) > 0:
            self.min_dist = float(np.min(valid))

    def control_loop(self):
        twist = Twist()
        self.tick += 1

        # Changer direction toutes les 2-3 secondes (40-60 ticks)
        if self.tick > 50:
            self.tick = 0
            self.forward_speed = np.random.uniform(0.8, 1.5)
            self.turn_speed = np.random.uniform(-2.5, 2.5)
            self.get_logger().info(f'New: forward={self.forward_speed:.2f} turn={self.turn_speed:.2f}')

        # Logique simple
        if self.min_dist < 0.6:
            # Obstacle - tourner sur place
            twist.linear.x = 0.0
            twist.angular.z = 4.0 if np.random.random() > 0.5 else -4.0
        else:
            # Avancer avec un peu de rotation
            twist.linear.x = self.forward_speed
            twist.angular.z = self.turn_speed

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = BasicMover()
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
