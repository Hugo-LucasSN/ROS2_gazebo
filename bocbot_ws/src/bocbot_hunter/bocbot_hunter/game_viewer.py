#!/usr/bin/env python3
"""
Game Viewer - POV vertical + Camera terrain + Controle
Layout: BLEU (haut), ORANGE (bas), Terrain (droite = vraie cam Gazebo)
Controle: WASD=BLEU, Fleches=ORANGE
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import sys
import signal
import time
try:
    from pynput import keyboard as pyn_keyboard
except Exception:
    pyn_keyboard = None

cv2.setUseOptimized(True)


class GameViewer(Node):
    def __init__(self):
        super().__init__('game_viewer')
        signal.signal(signal.SIGINT, self._sig)
        signal.signal(signal.SIGTERM, self._sig)

        # Publishers cmd_vel
        self.cmd_pub_blue = self.create_publisher(Twist, '/bocbot_blue/cmd_vel', 10)
        self.cmd_pub_orange = self.create_publisher(Twist, '/bocbot_orange/cmd_vel', 10)

        # Subscriptions cameras robots
        self.create_subscription(Image, '/bocbot_blue/camera/image_raw', self.blue_img_cb, 10)
        self.create_subscription(Image, '/bocbot_orange/camera/image_raw', self.orange_img_cb, 10)

        # Subscription score
        self.create_subscription(String, '/game/score', self.score_cb, 10)

        # Subscription camera terrain (vraie cam Gazebo)
        self.create_subscription(Image, '/game/top_down/image_raw', self.topdown_img_cb, 10)

        self.bridge = CvBridge()
        self.blue_image = None
        self.orange_image = None
        self.topdown_image = None
        self.lock = threading.Lock()
        self.running = True

        # Score
        self.score_blue = 0
        self.score_orange = 0

        # Vitesses - plus reactif
        self.speed = 4.2
        self.turn = 6.0

        # Etat des touches (BLEU et ORANGE independants)
        self.blue_keys = {'w': False, 's': False, 'a': False, 'd': False}
        self.orange_keys = {'up': False, 'down': False, 'left': False, 'right': False}

        # Etat des axes (BLEU et ORANGE independants)
        self.blue_linear = 0.0
        self.blue_angular = 0.0
        self.orange_linear = 0.0
        self.orange_angular = 0.0

        self.blue_key_time = {'w': 0.0, 's': 0.0, 'a': 0.0, 'd': 0.0}
        self.orange_key_time = {'up': 0.0, 'down': 0.0, 'left': 0.0, 'right': 0.0}
        self.has_focus = False

        # Relachement par touche (court): si on lache, ca s'arrete vite
        self.key_release_delay = 0.08

        # Publish commandes a 200Hz ET mise a jour des vitesses
        self.create_timer(0.004, self.update_and_publish)

        # Camera en thread daemon
        self.cam_thread = threading.Thread(target=self.camera_loop, daemon=True)
        self.cam_thread.start()

        # Controle clavier robuste: key-down / key-up reels (combo fiable)
        self.use_pynput = pyn_keyboard is not None
        self.key_listener = None
        if self.use_pynput:
            self.key_listener = pyn_keyboard.Listener(
                on_press=self._on_key_press,
                on_release=self._on_key_release
            )
            self.key_listener.daemon = True
            self.key_listener.start()
        else:
            # Fallback si pynput indisponible
            self.key_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
            self.key_thread.start()

    def _sig(self, *_):
        self.running = False
        if self.key_listener is not None:
            try:
                self.key_listener.stop()
            except Exception:
                pass

    def blue_img_cb(self, msg):
        try:
            with self.lock:
                self.blue_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            pass

    def orange_img_cb(self, msg):
        try:
            with self.lock:
                self.orange_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            pass

    def score_cb(self, msg):
        try:
            parts = msg.data.split(':')
            self.score_blue = int(parts[0])
            self.score_orange = int(parts[1])
        except Exception:
            pass

    def topdown_img_cb(self, msg):
        try:
            with self.lock:
                self.topdown_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            pass

    def camera_loop(self):
        cv2.namedWindow('MATCH: BLEU (WASD) | ORANGE (Fleches)', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('MATCH: BLEU (WASD) | ORANGE (Fleches)', 1280, 750)

        while self.running:
            with self.lock:
                blue = self.blue_image.copy() if self.blue_image is not None else None
                orange = self.orange_image.copy() if self.orange_image is not None else None
                topdown = self.topdown_image.copy() if self.topdown_image is not None else None

            # Tailles: legerement reduites pour monter le FPS d'affichage
            pov_h, pov_w = 288, 384  # ratio 4:3

            # Placeholder si pas d'image
            if blue is None:
                blue = np.zeros((pov_h, pov_w, 3), dtype=np.uint8)
                cv2.putText(blue, 'BLEU: Attente...', (120, 160),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 100, 0), 2)

            if orange is None:
                orange = np.zeros((pov_h, pov_w, 3), dtype=np.uint8)
                cv2.putText(orange, 'ORANGE: Attente...', (110, 160),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 100, 255), 2)

            # Resize en format 4:3
            blue = cv2.resize(blue, (pov_w, pov_h))
            orange = cv2.resize(orange, (pov_w, pov_h))

            # Ajouter titres
            self._add_title(blue, 'BLEU (WASD)', (255, 100, 0))
            self._add_title(orange, 'ORANGE (Fleches)', (0, 100, 255))

            # Vue terrain - meme hauteur que les 2 POV ensemble
            field_h = 576
            field_w = 384
            if topdown is None:
                topdown = np.zeros((field_h, field_w, 3), dtype=np.uint8)
                cv2.putText(topdown, 'TERRAIN: Attente...', (110, field_h // 2),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 2)
            else:
                src_h, src_w = topdown.shape[:2]
                scale = min(field_w / src_w, field_h / src_h)
                resized_w = max(1, int(src_w * scale))
                resized_h = max(1, int(src_h * scale))
                topdown_resized = cv2.resize(topdown, (resized_w, resized_h))
                canvas = np.zeros((field_h, field_w, 3), dtype=np.uint8)
                off_x = (field_w - resized_w) // 2
                off_y = (field_h - resized_h) // 2
                canvas[off_y:off_y + resized_h, off_x:off_x + resized_w] = topdown_resized
                topdown = canvas

            # Assemblage: BLEU haut gauche, ORANGE bas gauche, TERRAIN droite
            left_side = np.concatenate([blue, orange], axis=0)  # 640 haut x 426 large
            display = np.concatenate([left_side, topdown], axis=1)  # 640 haut x (426+426)=852 large

            # Ligne separatrice verticale
            mid_v = pov_w
            cv2.line(display, (mid_v, 0), (mid_v, display.shape[0]), (200, 200, 200), 2)

            # Bandes noires haut/bas autour de la vue top-down pour afficher gros score
            band_h = 58
            cv2.rectangle(display, (mid_v + 2, 0), (display.shape[1], band_h), (0, 0, 0), -1)
            cv2.rectangle(display, (mid_v + 2, display.shape[0] - band_h), (display.shape[1], display.shape[0]), (0, 0, 0), -1)

            # Score BLEU (bande haute)
            cv2.putText(display, f'BLEU  {self.score_blue}',
                       (mid_v + 16, 42),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.15, (255, 120, 0), 3, cv2.LINE_AA)

            # Score ORANGE (bande basse)
            cv2.putText(display, f'ORANGE  {self.score_orange}',
                       (mid_v + 16, display.shape[0] - 16),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.05, (0, 100, 255), 3, cv2.LINE_AA)

            # Focus indicator et controles
            focus_color = (0, 255, 0) if self.has_focus else (100, 100, 100)
            focus_text = 'CLICK TO ACTIVATE' if not self.has_focus else 'ACTIVE'
            cv2.putText(display, focus_text, (mid_v + 250, display.shape[0] - 34),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.3, focus_color, 1, cv2.LINE_AA)

            hint = 'WASD=BLEU | Arrows=ORANGE | SPACE=Stop'
            cv2.putText(display, hint, (mid_v + 170, display.shape[0] - 4),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.25, (180, 180, 180), 1, cv2.LINE_AA)

            cv2.setWindowProperty('MATCH: BLEU (WASD) | ORANGE (Fleches)',
                               cv2.WND_PROP_TOPMOST, 1)

            cv2.imshow('MATCH: BLEU (WASD) | ORANGE (Fleches)', display)

            # Mouse callback pour detecter le clic/focus
            cv2.setMouseCallback('MATCH: BLEU (WASD) | ORANGE (Fleches)',
                               self.on_mouse_event)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                self.running = False
                break
            # Fallback: OpenCV key polling uniquement si pynput indisponible
            if not self.use_pynput:
                self.process_opencv_key(key)

        cv2.destroyAllWindows()

    def on_mouse_event(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.has_focus = True
        elif event == cv2.EVENT_MOUSEMOVE:
            self.has_focus = True

    def _on_key_press(self, key):
        self.has_focus = True
        now = time.time()
        try:
            ch = key.char.lower() if key.char is not None else None
        except Exception:
            ch = None

        with self.lock:
            if ch == 'w':
                self.blue_keys['w'] = True
                self.blue_keys['s'] = False
                self.blue_key_time['w'] = now
                self.blue_key_time['s'] = 0.0
                return
            if ch == 's':
                self.blue_keys['s'] = True
                self.blue_keys['w'] = False
                self.blue_key_time['s'] = now
                self.blue_key_time['w'] = 0.0
                return
            if ch == 'a':
                self.blue_keys['a'] = True
                self.blue_keys['d'] = False
                self.blue_key_time['a'] = now
                self.blue_key_time['d'] = 0.0
                return
            if ch == 'd':
                self.blue_keys['d'] = True
                self.blue_keys['a'] = False
                self.blue_key_time['d'] = now
                self.blue_key_time['a'] = 0.0
                return

            if key == pyn_keyboard.Key.up:
                self.orange_keys['up'] = True
                self.orange_keys['down'] = False
                self.orange_key_time['up'] = now
                self.orange_key_time['down'] = 0.0
            elif key == pyn_keyboard.Key.down:
                self.orange_keys['down'] = True
                self.orange_keys['up'] = False
                self.orange_key_time['down'] = now
                self.orange_key_time['up'] = 0.0
            elif key == pyn_keyboard.Key.left:
                self.orange_keys['left'] = True
                self.orange_keys['right'] = False
                self.orange_key_time['left'] = now
                self.orange_key_time['right'] = 0.0
            elif key == pyn_keyboard.Key.right:
                self.orange_keys['right'] = True
                self.orange_keys['left'] = False
                self.orange_key_time['right'] = now
                self.orange_key_time['left'] = 0.0
            elif key == pyn_keyboard.Key.space:
                self.blue_keys = {'w': False, 's': False, 'a': False, 'd': False}
                self.orange_keys = {'up': False, 'down': False, 'left': False, 'right': False}
                self.blue_key_time = {'w': 0.0, 's': 0.0, 'a': 0.0, 'd': 0.0}
                self.orange_key_time = {'up': 0.0, 'down': 0.0, 'left': 0.0, 'right': 0.0}
            elif key == pyn_keyboard.Key.esc:
                self.running = False

    def _on_key_release(self, key):
        now = 0.0
        try:
            ch = key.char.lower() if key.char is not None else None
        except Exception:
            ch = None

        with self.lock:
            if ch == 'w':
                self.blue_keys['w'] = False
                self.blue_key_time['w'] = now
                return
            if ch == 's':
                self.blue_keys['s'] = False
                self.blue_key_time['s'] = now
                return
            if ch == 'a':
                self.blue_keys['a'] = False
                self.blue_key_time['a'] = now
                return
            if ch == 'd':
                self.blue_keys['d'] = False
                self.blue_key_time['d'] = now
                return

            if key == pyn_keyboard.Key.up:
                self.orange_keys['up'] = False
                self.orange_key_time['up'] = now
            elif key == pyn_keyboard.Key.down:
                self.orange_keys['down'] = False
                self.orange_key_time['down'] = now
            elif key == pyn_keyboard.Key.left:
                self.orange_keys['left'] = False
                self.orange_key_time['left'] = now
            elif key == pyn_keyboard.Key.right:
                self.orange_keys['right'] = False
                self.orange_key_time['right'] = now

    def process_opencv_key(self, key):
        if key == 0 or key == 255:
            return

        self.has_focus = True

        # Touche pressee - enregistre la touche active
        # Codes fleches (ORANGE) d'abord pour eviter conflit avec ASCII majuscules
        if key in (82, 65362):  # UP arrow (0x52, GTK)
            self.orange_keys['up'] = True
            self.orange_keys['down'] = False
            now = time.time()
            self.orange_key_time['up'] = now
            self.orange_key_time['down'] = 0.0
        elif key in (84, 65364):  # DOWN arrow (0x54, GTK)
            self.orange_keys['down'] = True
            self.orange_keys['up'] = False
            now = time.time()
            self.orange_key_time['down'] = now
            self.orange_key_time['up'] = 0.0
        elif key in (81, 65361):  # LEFT arrow (0x51, GTK)
            self.orange_keys['left'] = True
            self.orange_keys['right'] = False
            now = time.time()
            self.orange_key_time['left'] = now
            self.orange_key_time['right'] = 0.0
        elif key in (83, 65363):  # RIGHT arrow (0x53, GTK)
            self.orange_keys['right'] = True
            self.orange_keys['left'] = False
            now = time.time()
            self.orange_key_time['right'] = now
            self.orange_key_time['left'] = 0.0
        # Codes WASD (BLEU)
        elif key in (ord('w'), ord('W')):
            self.blue_keys['w'] = True
            self.blue_keys['s'] = False
            now = time.time()
            self.blue_key_time['w'] = now
            self.blue_key_time['s'] = 0.0
        elif key == ord('s'):
            self.blue_keys['s'] = True
            self.blue_keys['w'] = False
            now = time.time()
            self.blue_key_time['s'] = now
            self.blue_key_time['w'] = 0.0
        elif key in (ord('a'), ord('A')):
            self.blue_keys['a'] = True
            self.blue_keys['d'] = False
            now = time.time()
            self.blue_key_time['a'] = now
            self.blue_key_time['d'] = 0.0
        elif key in (ord('d'), ord('D')):
            self.blue_keys['d'] = True
            self.blue_keys['a'] = False
            now = time.time()
            self.blue_key_time['d'] = now
            self.blue_key_time['a'] = 0.0
        # SPACE = stop tout
        elif key == 32:
            self.blue_keys = {'w': False, 's': False, 'a': False, 'd': False}
            self.orange_keys = {'up': False, 'down': False, 'left': False, 'right': False}
            self.blue_key_time = {'w': 0.0, 's': 0.0, 'a': 0.0, 'd': 0.0}
            self.orange_key_time = {'up': 0.0, 'down': 0.0, 'left': 0.0, 'right': 0.0}

    def keyboard_loop(self):
        try:
            import termios
            import tty
            import select
        except ImportError:
            return

        if not sys.stdin.isatty():
            return

        old_settings = None
        try:
            old_settings = termios.tcgetattr(sys.stdin)
        except termios.error:
            return

        while self.running and rclpy.ok():
            try:
                tty.setraw(sys.stdin.fileno())
                rlist, _, _ = select.select([sys.stdin], [], [], 0.001)
                if rlist:
                    ch = sys.stdin.read(1)
                    if ch == '\x1b':
                        ch2 = sys.stdin.read(2)
                        if ch2 == '[A':
                            self.handle_key('UP')
                        elif ch2 == '[B':
                            self.handle_key('DOWN')
                        elif ch2 == '[C':
                            self.handle_key('RIGHT')
                        elif ch2 == '[D':
                            self.handle_key('LEFT')
                        elif ch2 == '':
                            self.handle_key('ESC')
                    elif ch == '\x03':
                        self.handle_key('QUIT')
                    else:
                        self.handle_key(ch)
            except Exception:
                pass
            finally:
                if old_settings:
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def handle_key(self, key):
        self.has_focus = True

        # Mise a jour des touches actives - garde les touches presse
        # Robot BLEU: WASD
        if key in ('w', 'W'):
            self.blue_keys['w'] = True
            self.blue_keys['s'] = False
            now = time.time()
            self.blue_key_time['w'] = now
            self.blue_key_time['s'] = 0.0
        elif key in ('s', 'S'):
            self.blue_keys['s'] = True
            self.blue_keys['w'] = False
            now = time.time()
            self.blue_key_time['s'] = now
            self.blue_key_time['w'] = 0.0
        elif key in ('a', 'A'):
            self.blue_keys['a'] = True
            self.blue_keys['d'] = False
            now = time.time()
            self.blue_key_time['a'] = now
            self.blue_key_time['d'] = 0.0
        elif key in ('d', 'D'):
            self.blue_keys['d'] = True
            self.blue_keys['a'] = False
            now = time.time()
            self.blue_key_time['d'] = now
            self.blue_key_time['a'] = 0.0

        # Robot ORANGE: Fleches directionnelles
        elif key == 'UP':
            self.orange_keys['up'] = True
            self.orange_keys['down'] = False
            now = time.time()
            self.orange_key_time['up'] = now
            self.orange_key_time['down'] = 0.0
        elif key == 'DOWN':
            self.orange_keys['down'] = True
            self.orange_keys['up'] = False
            now = time.time()
            self.orange_key_time['down'] = now
            self.orange_key_time['up'] = 0.0
        elif key == 'LEFT':
            self.orange_keys['left'] = True
            self.orange_keys['right'] = False
            now = time.time()
            self.orange_key_time['left'] = now
            self.orange_key_time['right'] = 0.0
        elif key == 'RIGHT':
            self.orange_keys['right'] = True
            self.orange_keys['left'] = False
            now = time.time()
            self.orange_key_time['right'] = now
            self.orange_key_time['left'] = 0.0

        # SPACE reset tout
        elif key == ' ':
            self.blue_keys = {'w': False, 's': False, 'a': False, 'd': False}
            self.orange_keys = {'up': False, 'down': False, 'left': False, 'right': False}
            self.blue_key_time = {'w': 0.0, 's': 0.0, 'a': 0.0, 'd': 0.0}
            self.orange_key_time = {'up': 0.0, 'down': 0.0, 'left': 0.0, 'right': 0.0}
        elif key in ('ESC', 'QUIT'):
            self.running = False

    def update_and_publish(self):
        """Met a jour les vitesses selon les touches pressées et publie"""
        if not self.running or not rclpy.ok():
            return

        now = time.time()
        with self.lock:
            blue_keys = self.blue_keys.copy()
            orange_keys = self.orange_keys.copy()
            blue_time = self.blue_key_time.copy()
            orange_time = self.orange_key_time.copy()

        if self.use_pynput:
            # Mode fiable key-down / key-up: combos parfaits
            blue_w = blue_keys['w']
            blue_s = blue_keys['s']
            blue_a = blue_keys['a']
            blue_d = blue_keys['d']
            orange_up = orange_keys['up']
            orange_down = orange_keys['down']
            orange_left = orange_keys['left']
            orange_right = orange_keys['right']
        else:
            # Fallback polling: base sur timeout court
            blue_w = (now - blue_time['w']) < self.key_release_delay
            blue_s = (now - blue_time['s']) < self.key_release_delay
            blue_a = (now - blue_time['a']) < self.key_release_delay
            blue_d = (now - blue_time['d']) < self.key_release_delay
            orange_up = (now - orange_time['up']) < self.key_release_delay
            orange_down = (now - orange_time['down']) < self.key_release_delay
            orange_left = (now - orange_time['left']) < self.key_release_delay
            orange_right = (now - orange_time['right']) < self.key_release_delay

        # Calcul vitesse BLEU (WASD) - touche active si True
        blue_lin = 0.0
        blue_ang = 0.0
        if blue_w:
            blue_lin += self.speed
        if blue_s:
            blue_lin -= self.speed
        if blue_a:
            blue_ang += self.turn
        if blue_d:
            blue_ang -= self.turn

        # Calcul vitesse ORANGE (Fleches)
        orange_lin = 0.0
        orange_ang = 0.0
        if orange_up:
            orange_lin += self.speed
        if orange_down:
            orange_lin -= self.speed
        if orange_left:
            orange_ang += self.turn
        if orange_right:
            orange_ang -= self.turn

        self.blue_linear = blue_lin
        self.blue_angular = blue_ang
        self.orange_linear = orange_lin
        self.orange_angular = orange_ang

        # Publication
        twist_blue = Twist()
        twist_blue.linear.x = self.blue_linear
        twist_blue.angular.z = self.blue_angular
        self.cmd_pub_blue.publish(twist_blue)

        twist_orange = Twist()
        twist_orange.linear.x = self.orange_linear
        twist_orange.angular.z = self.orange_angular
        self.cmd_pub_orange.publish(twist_orange)

    def _add_title(self, img, text, color):
        overlay = img.copy()
        cv2.rectangle(overlay, (0, 0), (img.shape[1], 28), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.5, img, 0.5, 0, img)
        cv2.putText(img, text, (8, 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)

    def publish_cmds(self):
        twist_blue = Twist()
        twist_blue.linear.x = self.blue_linear
        twist_blue.angular.z = self.blue_angular
        self.cmd_pub_blue.publish(twist_blue)

        twist_orange = Twist()
        twist_orange.linear.x = self.orange_linear
        twist_orange.angular.z = self.orange_angular
        self.cmd_pub_orange.publish(twist_orange)


def main(args=None):
    rclpy.init(args=args)
    node = GameViewer()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        while node.running and rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass

    node.running = False
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
