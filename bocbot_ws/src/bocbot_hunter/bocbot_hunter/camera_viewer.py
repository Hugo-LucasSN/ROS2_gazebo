#!/usr/bin/env python3
"""
Camera Viewer - Affiche le POV du robot avec la detection de balle
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from bocbot_interfaces.msg import BallDetection


class CameraViewerNode(Node):
    def __init__(self):
        super().__init__('camera_viewer')

        self.bridge = CvBridge()
        self.latest_image = None
        self.ball_info = None

        # Subscriber camera
        self.create_subscription(
            Image, '/bocbot/camera/image', self.image_cb, 10)

        # Subscriber detection
        self.create_subscription(
            BallDetection, '/bocbot/ball_detection', self.ball_cb, 10)

        # Timer pour afficher (30 FPS max)
        self.timer = self.create_timer(0.033, self.display_loop)

        self.get_logger().info('Camera Viewer started - Press Q to quit')

    def image_cb(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

    def ball_cb(self, msg):
        self.ball_info = msg

    def display_loop(self):
        if self.latest_image is None:
            return

        # Copier l'image
        display = self.latest_image.copy()

        # Dessiner la detection de balle
        if self.ball_info and self.ball_info.confidence > 0.1:
            cx = self.ball_info.center_x
            cy = self.ball_info.center_y
            r = int(self.ball_info.radius_pixels)

            # Cercle vert autour de la balle
            cv2.circle(display, (cx, cy), r, (0, 255, 0), 2)
            cv2.circle(display, (cx, cy), 3, (0, 0, 255), -1)

            # Texte info
            info = f"D:{self.ball_info.distance:.2f}m A:{self.ball_info.angle:.2f}rad"
            cv2.putText(display, info, (10, 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(display, f"Conf:{self.ball_info.confidence:.2f}", (10, 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        else:
            cv2.putText(display, "No ball detected", (10, 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # Afficher
        cv2.imshow('BocBot POV', display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info('Quit requested')
            raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
