#!/usr/bin/env python3
"""
Manual Control - Controle clavier du robot BLEU
Fleches/ZQSD dans le terminal. Avancer + tourner en meme temps.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import sys
import signal
import termios
import tty
import select
import time


class ManualControl(Node):
    def __init__(self):
        super().__init__('manual_control')
        signal.signal(signal.SIGINT, self._sig)
        signal.signal(signal.SIGTERM, self._sig)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Image, 'camera/image_raw', self.img_cb, 10)

        self.bridge = CvBridge()
        self.image = None
        self.lock = threading.Lock()
        self.running = True

        # Vitesses
        self.speed = 5.0
        self.turn = 6.0

        # Etat des axes (independants pour combiner avancer+tourner)
        self.cur_linear = 0.0
        self.cur_angular = 0.0
        self.last_key_time = time.time()

        # Publish commandes a 50Hz
        self.create_timer(0.02, self.publish_cmd)

        # Camera en thread daemon
        self.cam_thread = threading.Thread(target=self.camera_loop, daemon=True)
        self.cam_thread.start()

    def _sig(self, *_):
        self.running = False

    def img_cb(self, msg):
        try:
            with self.lock:
                self.image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            pass

    def camera_loop(self):
        cv2.namedWindow('Camera BLEU', cv2.WINDOW_AUTOSIZE)
        while self.running:
            with self.lock:
                img = self.image.copy() if self.image is not None else None
            if img is None:
                img = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(img, 'Attente camera...', (200, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow('Camera BLEU', img)
            cv2.waitKey(30)
        cv2.destroyAllWindows()

    def publish_cmd(self):
        # Auto-stop si aucune touche depuis 0.15s
        if time.time() - self.last_key_time > 0.15:
            self.cur_linear = 0.0
            self.cur_angular = 0.0

        twist = Twist()
        twist.linear.x = self.cur_linear
        twist.angular.z = self.cur_angular
        self.cmd_pub.publish(twist)

    def read_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.03)
        if rlist:
            ch = sys.stdin.read(1)
            if ch == '\x1b':
                ch2 = sys.stdin.read(2)
                if ch2 == '[A':
                    return 'UP'
                if ch2 == '[B':
                    return 'DOWN'
                if ch2 == '[C':
                    return 'RIGHT'
                if ch2 == '[D':
                    return 'LEFT'
                return 'ESC'
            if ch == '\x03':
                return 'QUIT'
            return ch
        return None

    def run(self):
        old_settings = termios.tcgetattr(sys.stdin)

        print('')
        print('========================================')
        print('   CONTROLE ROBOT BLEU')
        print('========================================')
        print('  Fleches / ZQSD = bouger')
        print('  Avancer + tourner en meme temps !')
        print('  ESPACE = stop  |  Ctrl+C = quitter')
        print('========================================')
        print('')

        try:
            while self.running and rclpy.ok():
                key = self.read_key()

                if key is None:
                    continue

                self.last_key_time = time.time()

                # Avancer/reculer (ne touche pas angular)
                if key in ('UP', 'z', 'Z', 'w', 'W'):
                    self.cur_linear = self.speed
                elif key in ('DOWN', 's', 'S'):
                    self.cur_linear = -self.speed
                # Tourner (ne touche pas linear)
                elif key in ('LEFT', 'q', 'Q', 'a', 'A'):
                    self.cur_angular = self.turn
                elif key in ('RIGHT', 'd', 'D'):
                    self.cur_angular = -self.turn
                # Stop
                elif key == ' ':
                    self.cur_linear = 0.0
                    self.cur_angular = 0.0
                elif key in ('ESC', 'QUIT'):
                    break

                sys.stdout.write(
                    f'\r  v={self.cur_linear:+.1f}  rot={self.cur_angular:+.1f}    '
                )
                sys.stdout.flush()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            self.running = False
            self.cur_linear = 0.0
            self.cur_angular = 0.0
            print('\nArret.')


def main(args=None):
    rclpy.init(args=args)
    node = ManualControl()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    node.run()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
