#!/usr/bin/env python3
"""
Simple Hunter - Node unique: detection + controle + affichage
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np


class SimpleHunter(Node):
    def __init__(self):
        super().__init__('simple_hunter')

        self.bridge = CvBridge()
        self.latest_image = None

        # Etat robot
        self.ball_detected = False
        self.ball_angle = 0.0
        self.ball_dist = 0.0
        self.min_dist = 10.0

        # Timer pour changer de direction
        self.direction_timer = 0
        self.explore_lin = 0.8
        self.explore_ang = 0.0

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/bocbot/cmd_vel', 10)

        # Subscribers
        self.create_subscription(Image, '/bocbot/camera/image', self.cam_cb, 10)
        self.create_subscription(LaserScan, '/bocbot/scan', self.lidar_cb, 10)

        # Timer controle 20Hz
        self.create_timer(0.05, self.control_loop)
        self.timer_display = self.create_timer(0.1, self.display_loop)

        self.get_logger().info('SIMPLE HUNTER READY - Robot rapide!')

    def cam_cb(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.detect_ball()
        except:
            pass

    def lidar_cb(self, msg):
        ranges = np.array(msg.ranges)
        valid = ranges[(ranges > 0.1) & (ranges < 10)]
        if len(valid) > 0:
            self.min_dist = float(np.min(valid))

    def detect_ball(self):
        if self.latest_image is None:
            return

        hsv = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2HSV)

        # Blanc/gris - seuils larges
        lower = np.array([0, 0, 140])
        upper = np.array([180, 60, 255])

        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.dilate(mask, np.ones((4,4), np.uint8), iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        self.ball_detected = False
        if contours:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area > 50:
                (x, y), radius = cv2.minEnclosingCircle(c)
                if 5 <= radius <= 80:
                    self.ball_detected = True
                    # Angle: centre de l'image = 0
                    cx = 80  # 160/2
                    self.ball_angle = (x - cx) / cx * 0.7  # max ±0.7 rad
                    # Distance approx
                    self.ball_dist = 0.5 * (20 / (radius + 1))

    def control_loop(self):
        twist = Twist()
        self.direction_timer += 1

        # Changer direction toutes les 2-3 secondes (40-60 ticks)
        if self.direction_timer > 50:
            self.direction_timer = 0
            self.explore_lin = np.random.uniform(0.3, 0.6)
            self.explore_ang = np.random.uniform(-1.5, 1.5)
            self.get_logger().info(f'New dir: lin={self.explore_lin:.2f}')

        # Logique
        if self.min_dist < 0.4:
            # Obstacle proche - tourner
            twist.linear.x = 0.0
            twist.angular.z = 1.5 if np.random.random() > 0.5 else -1.5

        elif self.ball_detected:
            # Chasser la balle!
            twist.linear.x = 0.6
            twist.angular.z = -2.0 * self.ball_angle

            if self.ball_dist < 0.3:
                twist.linear.x = 0.8  # Pousser!
        else:
            # Explorer
            twist.linear.x = self.explore_lin
            twist.angular.z = self.explore_ang

        self.cmd_pub.publish(twist)

    def display_loop(self):
        if self.latest_image is None:
            return

        display = self.latest_image.copy()

        if self.ball_detected:
            cx, cy = 80, 60  # centre image 160x120
            # Cercle approximatif
            cv2.circle(display, (int(cx + self.ball_angle * 100), cy), 20, (0, 255, 0), 2)
            cv2.putText(display, f"BALL! d={self.ball_dist:.1f}", (5, 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        else:
            cv2.putText(display, "Searching...", (5, 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

        cv2.putText(display, f"Obs: {self.min_dist:.1f}m", (5, 105),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 0), 1)

        cv2.imshow('Bocbot POV', display)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleHunter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
