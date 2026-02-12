#!/usr/bin/env python3
"""
Dual POV Viewer - Affiche les images DEBUG annotées des deux robots
Chaque fast_hunter publie /bocbot_*/camera/debug avec l'overlay lock/état/balle
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import sys
import signal
import traceback


class DualPOVViewer(Node):
    def __init__(self):
        super().__init__('dual_pov_viewer')

        self.bridge = CvBridge()
        self.blue_image   = None
        self.orange_image = None
        self.running      = True
        self.frame_count  = 0
        self.image_lock   = threading.Lock()

        self.img_h = 480
        self.img_w = 640

        # S'abonner aux images DEBUG annotées (publiées par fast_hunter)
        self.create_subscription(Image, '/bocbot_blue/camera/debug',
                                 self.blue_cb,   10)
        self.create_subscription(Image, '/bocbot_orange/camera/debug',
                                 self.orange_cb, 10)

        signal.signal(signal.SIGINT,  self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

        self.get_logger().info('📷 DUAL POV VIEWER — BLEU | ORANGE (images debug annotées)')
        self.get_logger().info('   Q ou Echap pour quitter')

    def signal_handler(self, signum, frame):
        self.running = False

    def blue_cb(self, msg):
        try:
            with self.image_lock:
                self.blue_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            pass

    def orange_cb(self, msg):
        try:
            with self.image_lock:
                self.orange_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            pass

    # ──────────────────────────────────────────────────────────
    def run_display(self):
        self.get_logger().info('🖥️  Thread affichage démarré')

        # Placeholder "en attente" pour chaque robot
        def placeholder(text, color):
            ph = np.zeros((self.img_h, self.img_w, 3), dtype=np.uint8)
            cv2.putText(ph, text, (self.img_w//2 - 100, self.img_h//2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            return ph

        ph_blue   = placeholder('BLEU: en attente...', (255, 120, 0))
        ph_orange = placeholder('ORANGE: en attente...', (0, 140, 255))

        while self.running and rclpy.ok():
            try:
                with self.image_lock:
                    bl = self.blue_image.copy()   if self.blue_image   is not None else None
                    or_ = self.orange_image.copy() if self.orange_image is not None else None

                # Préparer chaque moitié
                if bl is not None:
                    try:
                        left = cv2.resize(bl, (self.img_w, self.img_h))
                    except Exception:
                        left = ph_blue
                else:
                    left = ph_blue

                if or_ is not None:
                    try:
                        right = cv2.resize(or_, (self.img_w, self.img_h))
                    except Exception:
                        right = ph_orange
                else:
                    right = ph_orange

                # Bandeau titre en haut de chaque moitié
                self._add_title(left,  'ROBOT BLEU',   (255, 100, 0))
                self._add_title(right, 'ROBOT ORANGE', (0, 100, 255))

                # Assemblage côte à côte
                display = np.concatenate([left, right], axis=1)

                # Séparateur vertical central
                mid = self.img_w
                cv2.line(display, (mid, 0), (mid, self.img_h), (200, 200, 200), 2)

                self.frame_count += 1
                if self.frame_count % 100 == 0:
                    has_b = bl   is not None
                    has_o = or_  is not None
                    self.get_logger().info(
                        f'📺 Frame {self.frame_count} — '
                        f'BLEU: {"OK" if has_b else "NO"} | '
                        f'ORANGE: {"OK" if has_o else "NO"}'
                    )

                cv2.imshow('POV DEBUG : BLEU | ORANGE  (Q = quitter)', display)
                key = cv2.waitKey(30) & 0xFF
                if key in (27, ord('q'), ord('Q')):
                    self.get_logger().info('Fermeture demandée')
                    self.running = False
                    break

            except KeyboardInterrupt:
                self.running = False
                break
            except Exception as e:
                self.get_logger().error(f'Erreur affichage: {e}')

        cv2.destroyAllWindows()
        self.get_logger().info('🖥️  Thread affichage terminé')

    # ──────────────────────────────────────────────────────────
    @staticmethod
    def _add_title(img, text, color):
        """Bandeau semi-transparent en haut de l'image."""
        overlay = img.copy()
        cv2.rectangle(overlay, (0, 0), (img.shape[1], 28), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.5, img, 0.5, 0, img)
        cv2.putText(img, text, (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2, cv2.LINE_AA)


# ──────────────────────────────────────────────────────────────
def main(args=None):
    try:
        rclpy.init(args=args)
        node = DualPOVViewer()

        display_thread = threading.Thread(target=node.run_display, daemon=False)
        display_thread.start()

        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.running = False
            display_thread.join(timeout=2.0)
            node.destroy_node()
            rclpy.shutdown()

    except Exception as e:
        print(f"ERREUR FATALE: {e}")
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
