#!/usr/bin/env python3
"""
Test Mover - Version ultra simple pour tester
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TestMover(Node):
    def __init__(self):
        super().__init__('test_mover')

        self.cmd_pub = self.create_publisher(Twist, '/bocbot/cmd_vel', 10)
        self.tick = 0

        # Timer 10Hz
        self.create_timer(0.1, self.move)

        self.get_logger().info('TEST MOVER - Publishing velocity!')

    def move(self):
        twist = Twist()
        self.tick += 1

        # Mouvement simple: avant 2 sec, tourne 1 sec, repeat
        cycle = self.tick % 30
        if cycle < 20:
            twist.linear.x = 0.5  # Avancer à vitesse raisonnable
            twist.angular.z = 0.0
        else:
            twist.linear.x = 0.0  # Tourner
            twist.angular.z = 1.0  # Rotation raisonnable

        self.cmd_pub.publish(twist)

        if self.tick % 30 == 0:
            self.get_logger().info(f'Tick {self.tick}: lin={twist.linear.x} ang={twist.angular.z}')


def main(args=None):
    rclpy.init(args=args)
    node = TestMover()
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
