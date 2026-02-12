#!/usr/bin/env python3
"""
Extreme Test - Vitesses MAXIMALES pour tester si le robot bouge
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ExtremeTest(Node):
    def __init__(self):
        super().__init__('extreme_test')

        self.cmd_pub = self.create_publisher(Twist, '/bocbot/cmd_vel', 10)
        self.tick = 0

        # Timer 20Hz
        self.create_timer(0.05, self.move)

        self.get_logger().info('EXTREME TEST - MAXIMUM SPEED!')

    def move(self):
        twist = Twist()
        self.tick += 1

        # Vitesse maximale raisonnable - tester si le robot bouge
        cycle = self.tick % 40
        if cycle < 30:
            twist.linear.x = 1.0  # Vitesse maximale raisonnable
            twist.angular.z = 0.0
        else:
            twist.linear.x = 0.0
            twist.angular.z = 2.0  # Rotation maximale raisonnable

        self.cmd_pub.publish(twist)

        if self.tick % 10 == 0:
            self.get_logger().info(f'Tick {self.tick}: lin={twist.linear.x} ang={twist.angular.z}')


def main(args=None):
    rclpy.init(args=args)
    node = ExtremeTest()
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
