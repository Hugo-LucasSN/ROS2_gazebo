#!/usr/bin/env python3
"""
Color Ring Detector - Assemble 4 mini-cameras en anneau 360 degres de couleurs.
Publie un ColorRing message avec la couleur dominante par degre.
Couleurs: 0=UNKNOWN, 1=RED (balle), 2=BLUE (cage), 3=ORANGE (cage), 4=WHITE (mur), 5=GREEN (pelouse)
Optimise numpy vectorise — pas de boucle Python par colonne.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from bocbot_interfaces.msg import ColorRing
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import threading


# Couleur IDs
COLOR_UNKNOWN = 0
COLOR_RED     = 1   # Balle ROUGE
COLOR_BLUE    = 2
COLOR_ORANGE  = 3
COLOR_WHITE   = 4   # Mur
COLOR_BLACK   = 5   # Murs NOIRS (utilise BROWN pour compat)

# FOV de chaque camera en degres (~100 degres = 1.75 rad)
CAM_FOV_DEG = 100.0

# Cameras: nom -> angle central en degres (0=avant robot, sens LIDAR)
CAMERAS = {
    'cam360_front': 0.0,
    'cam360_right': 270.0,
    'cam360_back':  180.0,
    'cam360_left':  90.0,
}

# Bande centrale pour extraction (pourcentage de la hauteur)
# UNE SEULE LIGNE au centre de l'image, à la hauteur du LIDAR
STRIP_CENTER_RATIO = 0.5  # Centre exact de l'image
STRIP_HALF_WIDTH   = 2    # ±2 pixels autour du centre (ligne très fine)


def classify_columns_vectorized(bgr_strip, hsv_strip):
    """Classifie toutes les colonnes d'une bande en une seule passe numpy.
    Compte les pixels de chaque couleur par colonne et prend la majoritaire.
    Retourne un array (w_img,) de couleur IDs."""
    h, w, _ = hsv_strip.shape

    result = np.zeros(w, dtype=np.uint8)

    # Pour chaque colonne, classifier chaque pixel puis prendre la majorite
    # Ceci permet de detecter la balle rouge meme si elle n'occupe pas toute la colonne

    # Masks HSV (shape: h x w)
    H = hsv_strip[:,:,0]
    S = hsv_strip[:,:,1]
    V = hsv_strip[:,:,2]
    B = bgr_strip[:,:,0]
    G = bgr_strip[:,:,1]
    R = bgr_strip[:,:,2]

    # Masks par couleur (tous les pixels) - SEULEMENT 5 COULEURS !

    # BLACK = murs NOIRS (V faible, tous canaux bas)
    mask_black = (V < 60) & (R < 80) & (G < 80) & (B < 80)

    # RED = balle ROUGE (H=0-10 ou 170-180, S élevé, V moyen/élevé)
    mask_red = (((H >= 0) & (H <= 10)) | ((H >= 170) & (H <= 180))) & (S > 80) & (V > 100)

    # WHITE = murs (V élevé, S faible, tous canaux élevés)
    mask_white = (V > 200) & (S < 30) & (R > 180) & (G > 180) & (B > 180)

    # BLUE = cage bleue (H=100-125, S moyen/élevé, V moyen/élevé)
    mask_blue = (H > 100) & (H < 125) & (S > 100) & (V > 120)

    # ORANGE = cage orange (H=5-40, S moyen/élevé, V moyen/élevé)
    mask_orange = (H >= 5) & (H <= 40) & (S > 90) & (V > 120)

    # Compter les pixels de chaque couleur par colonne
    black_count = np.sum(mask_black, axis=0)
    red_count = np.sum(mask_red, axis=0)
    white_count = np.sum(mask_white, axis=0)
    blue_count = np.sum(mask_blue, axis=0)
    orange_count = np.sum(mask_orange, axis=0)

    # Pour chaque colonne, trouver la couleur majoritaire
    for col in range(w):
        counts = [black_count[col], red_count[col], white_count[col], blue_count[col], orange_count[col]]
        max_count = max(counts)

        # SEUIL STRICT : minimum 1 pixel (une seule ligne = très peu de pixels)
        if max_count >= 1:
            # ROUGE en priorité = balle
            if red_count[col] == max_count:
                result[col] = COLOR_RED     # 1 = balle ROUGE
            elif white_count[col] == max_count:
                result[col] = COLOR_WHITE   # 4 = mur
            elif blue_count[col] == max_count:
                result[col] = COLOR_BLUE    # 2 = cage bleue
            elif orange_count[col] == max_count:
                result[col] = COLOR_ORANGE  # 3 = cage orange
            elif black_count[col] == max_count:
                result[col] = COLOR_BLACK   # 5 = murs NOIRS

    return result


class ColorRingDetector(Node):
    def __init__(self):
        super().__init__('color_ring_detector')
        self.bridge = CvBridge()
        self.lock = threading.Lock()

        # Stockage des dernieres images par camera
        self.cam_images = {}
        for name in CAMERAS:
            self.cam_images[name] = None

        # Pre-calculer les tables de mapping colonne -> degre pour chaque camera
        # On suppose 160px de large (mis a jour au premier frame si different)
        self.col_to_deg = {}
        self._build_col_tables(160)

        # Subscribers pour les 4 cameras
        for name in CAMERAS:
            topic = f'{name}/image_raw'
            self.create_subscription(
                Image, topic,
                lambda msg, n=name: self._cam_cb(n, msg),
                10
            )

        # Publisher ColorRing
        self.ring_pub = self.create_publisher(ColorRing, 'color_ring', 10)

        # Timer a ~30Hz pour plus de reactivite
        self.create_timer(0.033, self._process_ring)

        self.get_logger().info('ColorRingDetector demarre - 4 cameras, numpy vectorise')

    def _build_col_tables(self, w):
        """Pre-calcule la table colonne->index_ring pour chaque camera."""
        self._table_width = w
        fracs = np.arange(w) / max(w - 1, 1)
        offsets = (fracs - 0.5) * CAM_FOV_DEG  # offset en degres
        for cam_name, center_deg in CAMERAS.items():
            global_degs = (center_deg + offsets) % 360.0
            self.col_to_deg[cam_name] = np.round(global_degs).astype(int) % 360

    def _cam_cb(self, cam_name, msg):
        """Stocke la derniere image de chaque camera."""
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self.lock:
                self.cam_images[cam_name] = img
        except Exception as e:
            self.get_logger().warning(f'Erreur image {cam_name}: {e}')

    def _process_ring(self):
        """Assemble les 4 images en anneau 360 degres — numpy vectorise."""
        ring = np.zeros(360, dtype=np.uint8)
        cam_count = 0

        with self.lock:
            images = {k: v.copy() if v is not None else None
                      for k, v in self.cam_images.items()}

        for cam_name, center_deg in CAMERAS.items():
            img = images.get(cam_name)
            if img is None:
                continue
            cam_count += 1

            h_img, w_img = img.shape[:2]

            # Rebuild table si resolution changee
            if w_img != self._table_width:
                self._build_col_tables(w_img)

            # Extraire UNE SEULE LIGNE horizontale au centre de l'image
            # C'est la hauteur exacte du LIDAR (z=0.30)
            y_center = int(h_img * STRIP_CENTER_RATIO)
            y_top = max(0, y_center - STRIP_HALF_WIDTH)
            y_bot = min(h_img, y_center + STRIP_HALF_WIDTH + 1)
            strip = img[y_top:y_bot, :]

            # Si la bande est vide (image corrompue), continuer
            if strip.size == 0:
                continue

            # Moyenner sur la verticale pour obtenir une seule ligne par colonne
            # strip.shape = (hauteur_fine, largeur)
            strip_line = np.mean(strip, axis=0, keepdims=True).astype(np.uint8)
            # strip_line.shape = (1, largeur)

            # HSV (pour les autres couleurs)
            hsv = cv2.cvtColor(strip_line, cv2.COLOR_BGR2HSV)

            # Classification vectorisee de toutes les colonnes (BGR + HSV)
            # strip_line est (1, w, 3), hsv est (1, w, 3)
            col_colors = classify_columns_vectorized(strip_line, hsv)

            # Mapper dans l'anneau
            deg_indices = self.col_to_deg[cam_name]
            ring[deg_indices] = col_colors

        # Message
        msg = ColorRing()
        msg.color_ring = ring.tolist()

        msg.red_count = int(np.sum(ring == COLOR_RED))  # Compte ROUGE pour balle
        msg.blue_count = int(np.sum(ring == COLOR_BLUE))
        msg.orange_count = int(np.sum(ring == COLOR_ORANGE))
        msg.white_count = int(np.sum(ring == COLOR_WHITE))
        msg.green_count = int(np.sum(ring == COLOR_BLACK))  # Utilise green_count pour BLACK/murs

        # Angle de la balle (ROUGE) - SEUIL BAISSE pour meilleure détection
        # Avec une seule ligne, on a moins de pixels, donc seuil très bas
        if msg.red_count >= 2:  # Seuil réduit de 5 à 2 degrés
            msg.ball_angle = self._find_color_angle(ring, COLOR_RED)
        else:
            msg.ball_angle = float('nan')  # Pas assez de rouge = pas de balle

        msg.blue_goal_angle = self._find_color_angle(ring, COLOR_BLUE)
        msg.orange_goal_angle = self._find_color_angle(ring, COLOR_ORANGE)

        msg.confidence = cam_count / 4.0

        self.ring_pub.publish(msg)

    def _find_color_angle(self, ring, color_id):
        """Trouve l'angle central d'une couleur (radians). NaN si absente."""
        indices = np.where(ring == color_id)[0]
        if len(indices) == 0:
            return float('nan')

        # Centre de masse circulaire
        angles_rad = indices * (2.0 * math.pi / 360.0)
        sin_sum = float(np.sum(np.sin(angles_rad)))
        cos_sum = float(np.sum(np.cos(angles_rad)))
        center_rad = math.atan2(sin_sum, cos_sum)

        if center_rad > math.pi:
            center_rad -= 2.0 * math.pi
        return center_rad


def main(args=None):
    rclpy.init(args=args)
    node = ColorRingDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
