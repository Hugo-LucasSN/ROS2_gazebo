#!/usr/bin/env python3
"""
Hunter Controller - Robot qui explore et chasse la balle
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from bocbot_interfaces.msg import BallDetection
from sensor_msgs.msg import LaserScan
import numpy as np
import random


class HunterControllerNode(Node):
    def __init__(self):
        super().__init__('hunter_controller')

        # Etat
        self.ball_detected = False
        self.ball_angle = 0.0
        self.ball_distance = 0.0
        self.min_front_distance = 5.0
        self.scan_received = False

        # Vitesses réalistes pour le robot
        self.max_linear = 0.8
        self.max_angular = 2.0

        # Distances
        self.touch_distance = 0.45
        self.safety_distance = 0.5

        # Exploration
        self.explore_linear = 0.25
        self.explore_angular = 0.0
        self.explore_timer = 0
        self.change_direction()

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/bocbot/cmd_vel', 10)

        # Subscribers
        self.create_subscription(
            BallDetection, '/bocbot/ball_detection', self.ball_cb, 10)
        self.create_subscription(
            LaserScan, '/bocbot/scan', self.scan_cb, 10)

        # Timer 10Hz - TOUJOURS publier
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Hunter Controller READY')

    def change_direction(self):
        self.explore_linear = random.uniform(0.3, 0.6)
        self.explore_angular = random.uniform(-1.5, 1.5)
        self.explore_timer = 0
        self.get_logger().info(f'New direction: lin={self.explore_linear:.2f} ang={self.explore_angular:.2f}')

    def ball_cb(self, msg):
        self.ball_detected = msg.confidence > 0.2
        self.ball_angle = msg.angle
        self.ball_distance = msg.distance

    def scan_cb(self, msg):
        self.scan_received = True
        ranges = np.array(msg.ranges)
        n = len(ranges)

        # Zone frontale centrale
        front = ranges[n//3:2*n//3]
        valid = front[(front > 0.1) & (front < 30.0)]

        if len(valid) > 0:
            self.min_front_distance = float(np.min(valid))

    def control_loop(self):
        twist = Twist()
        self.explore_timer += 1

        # Changer direction toutes les 3-5 sec
        if self.explore_timer > random.randint(30, 50):
            self.change_direction()

        # Logique de controle
        if self.min_front_distance < self.safety_distance:
            # OBSTACLE - tourner
            twist.linear.x = 0.0
            twist.angular.z = 1.5 if random.random() > 0.5 else -1.5
            self.get_logger().info(f'AVOID {self.min_front_distance:.2f}m')

        elif self.ball_detected:
            if self.ball_distance < self.touch_distance:
                # POUSSER la balle
                twist.linear.x = self.max_linear
                twist.angular.z = -2.0 * self.ball_angle
                self.get_logger().info('PUSH ball!')
            else:
                # CHASSER la balle
                twist.linear.x = 0.6
                twist.angular.z = -2.0 * self.ball_angle
                self.get_logger().info(f'CHASE d={self.ball_distance:.2f}')
        else:
            # EXPLORER
            twist.linear.x = self.explore_linear
            twist.angular.z = self.explore_angular

            # Eviter les murs proches
            if self.min_front_distance < 1.5:
                twist.angular.z = 1.5 if twist.angular.z >= 0 else -1.5

        # Toujours publier
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = HunterControllerNode()
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
