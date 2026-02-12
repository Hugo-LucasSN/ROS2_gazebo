#!/usr/bin/env python3
"""
Fast Hunter — LIDAR 360° + Fusion ColorRing (cameras 360°)
Détection balle par profil circulaire LIDAR + confirmation couleur WHITE.
Cages identifiees par LIDAR + BLUE/ORANGE.
Strategie: pousser la balle vers la cage adverse.
Publie une vue radar top-down sur /bocbot/camera/debug.
Timing réel (time.monotonic).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import math
import time
import signal
import sys

# ── Paramètre physique balle ────────────────────────────────────
BALL_RADIUS = 0.50   # rayon balle en mètres (Gazebo)

# ── Couleurs BGR pour l'overlay ────────────────────────────────
C_BG     = ( 20,  20,  20)
C_GRID   = ( 45,  45,  45)
C_LIDAR  = (130, 130, 130)
C_BALL   = (  0, 220,   0)   # vert
C_LOCK   = (  0, 255,   0)   # vert vif
C_AVOID  = (  0,   0, 220)   # rouge
C_WARN   = (  0, 140, 255)   # orange
C_WHITE  = (255, 255, 255)
C_BLACK  = (  0,   0,   0)
C_YELLOW = (  0, 215, 255)
C_CYAN   = (255, 220,   0)
C_ROBOT  = ( 80, 160, 255)   # bleu clair pour trace robot

STATE_COL = {
    "IDLE":           (80,  80,  80),
    "CHASE":          C_YELLOW,
    "AVOID":          C_AVOID,
    "SEARCH":         C_CYAN,
    "FLIP":           (255, 0,   255),
}


def badge(img, x, y, text, bg, fg=C_WHITE):
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
    p = 5
    cv2.rectangle(img, (x, y), (x+tw+2*p, y+th+2*p), bg, -1)
    cv2.putText(img, text, (x+p, y+th+p),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, fg, 2, cv2.LINE_AA)


class FastHunter(Node):
    def __init__(self):
        super().__init__('fast_hunter')
        signal.signal(signal.SIGINT,  self._sig)
        signal.signal(signal.SIGTERM, self._sig)

        self.bridge = CvBridge()

        # Identité
        self.robot_name = self.get_namespace().replace('/', '')
        self.is_blue = 'blue' in self.robot_name
        self.opponent_goal_x = 7.0 if self.is_blue else -7.0

        # Odométrie
        self.robot_x   = 0.0
        self.robot_y   = 0.0
        self.robot_yaw = 0.0
        self.robot_roll  = 0.0
        self.robot_pitch = 0.0

        # ── LIDAR brut ─────────────────────────────────────────
        self.lidar_ranges = np.array([])
        self.lidar_angles = np.array([])
        self.lidar_lock   = threading.Lock()

        # ── Détection balle (LIDAR) + POSITION MONDE ─────────────
        self.ball_detected         = False
        self.ball_angle            = 0.0    # angle dans ref robot (0=avant, rad)
        self.ball_distance_estimate= 30.0   # distance centre balle (m)
        self.ball_seg_indices      = []     # indices LIDAR appartenant à la balle
        self.last_ball_seen_time   = 0.0

        # ── POSITION MONDE DE LA BALLE (COORDONNÉES TERRAIN) ─────
        self.ball_world_x   = float('nan')  # Position x de la balle dans le monde
        self.ball_world_y   = float('nan')  # Position y de la balle dans le monde
        self.ball_position_valid = False    # Est-ce qu'on a une position monde valide ?

        # ── LOCK ───────────────────────────────────────────────
        self.has_lock          = False
        self.lock_angle        = 0.0
        self.lock_confidence   = 0
        self.lock_history      = []
        self.LOCK_PERSIST_TIME = 120.0  # LOCK gardé 120 secondes !!! TRES LONG TEMPS
        self.LOCK_HISTORY_SIZE = 50     # Moyenne sur 50 détections pour stabilité MAX avec résolution 7200

        # ── Limitation réalignement direction ────────────────────
        self.last_direction_update = 0.0
        self.DIRECTION_UPDATE_INTERVAL = 0.5  # Max 1 réalignement par 0.5s
        self.current_target_angle = 0.0  # Angle cible mémorisé

        # ── Obstacles ──────────────────────────────────────────
        # Calculés APRÈS exclusion des indices balle
        self.min_dist          = 30.0
        self.front_left_dist   = 30.0
        self.front_right_dist  = 30.0
        self.avoid_turn_dir    = 1.0
        self.avoid_immune_until= 0.0

        # ── Machine à états ────────────────────────────────────
        self.state            = "IDLE"
        self.tick             = 0
        self.now              = time.monotonic()
        self.running          = True

        # ── Déblocage ──────────────────────────────────────────
        self.stuck_ref_x    = 0.0
        self.stuck_ref_y    = 0.0
        self.stuck_ref_time = 0.0
        self.is_stuck        = False
        self.unstuck_phase   = 0
        self.unstuck_time    = 0.0
        self.STUCK_DIST     = 0.15
        self.STUCK_TIME     = 1.0

        # ── Robot retourné ─────────────────────────────────────
        self.is_upside_down  = False
        self.flip_phase      = 0
        self.flip_start_time = 0.0
        self.UPSIDE_THR      = 1.2

        # ── Vitesses ──────────────────────────────────────────
        self.SPD_CHARGE  = 4.0  # Plus rapide
        self.SPD_GOTO    = 2.5
        self.SPD_EXPLORE = 2.0
        self.TURN_FAST   = 6.0
        self.TURN_EXPLORE= 3.0
        self.TURN_ALIGN  = 5.0  # Rotation plus rapide
        self.TURN_AVOID  = 5.0

        # ── Seuils ────────────────────────────────────────────
        self.ALIGN_TOL    = 0.10
        self.CHARGE_TRIG  = 0.30
        self.OBS_CRIT     = 0.35
        self.OBS_NEAR     = 0.55
        self.OBS_SAFE     = 0.60
        self.MAX_ACCEL    = 8.0   # Accélération plus rapide
        self.MAX_DECEL    = 12.0  # Freinage plus rapide

        # ── Commandes ─────────────────────────────────────────
        self.cur_lin = 0.0
        self.cur_ang = 0.0

        # ── Publishers / Subscribers ──────────────────────────
        self.cmd_pub   = self.create_publisher(Twist, 'cmd_vel',      10)
        self.debug_pub = self.create_publisher(Image, 'camera/debug', 10)

        self.create_subscription(LaserScan, 'scan',  self.lidar_cb, 10)
        self.create_subscription(Odometry,  'odom',  self.odom_cb,  10)

        self.create_timer(0.005, self.control_loop)  # 200Hz au lieu de 500Hz - plus stable
        self.get_logger().info(f'⚡ FAST HUNTER [{self.robot_name}] LIDAR+COLOR démarré')

    # ──────────────────────────────────────────────────────────
    def _sig(self, *_):
        self.running = False

    # ──────────────────────────────────────────────────────────
    def odom_cb(self, msg):
        p = msg.pose.pose
        self.robot_x = p.position.x
        self.robot_y = p.position.y
        q = p.orientation
        self.robot_yaw = math.atan2(
            2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self.robot_roll = math.atan2(
            2*(q.w*q.x + q.y*q.z), 1 - 2*(q.x*q.x + q.y*q.y))
        sp = 2*(q.w*q.y - q.z*q.x)
        self.robot_pitch = (math.copysign(math.pi/2, sp)
                            if abs(sp) >= 1 else math.asin(sp))

    # ──────────────────────────────────────────────────────────
    def lidar_cb(self, msg):
        n = len(msg.ranges)
        if n < 10:
            return
        r = np.array(msg.ranges, dtype=np.float32)
        a = np.linspace(msg.angle_min, msg.angle_max, n)

        with self.lidar_lock:
            self.lidar_ranges = r
            self.lidar_angles = a

        self._process_lidar(r, a, n)

    # ──────────────────────────────────────────────────────────
    def _process_lidar(self, ranges, angles, n):
        """
        Détecte la balle, calcule sa position MONDE, et les distances d'obstacle.
        """
        # ── 1. Détecter la balle ───────────────────────────────
        ball_found, ball_angle, ball_dist, ball_indices = \
            self._find_ball(ranges, angles, n)

        t = time.monotonic()

        if ball_found:
            # Balle détectée ! Calculer sa position MONDE
            self.ball_detected = True
            self.ball_angle    = ball_angle
            self.ball_distance_estimate = ball_dist
            self.ball_seg_indices = ball_indices
            self.last_ball_seen_time = t

            # CALCULER LA POSITION MONDE DE LA BALLE
            # Position balle dans repère robot
            ball_rel_x = ball_dist * math.sin(ball_angle)
            ball_rel_y = ball_dist * math.cos(ball_angle)

            # Rotation vers repère monde
            cos_yaw = math.cos(self.robot_yaw)
            sin_yaw = math.sin(self.robot_yaw)

            # Position monde de la balle
            self.ball_world_x = self.robot_x + ball_rel_x * cos_yaw + ball_rel_y * sin_yaw
            self.ball_world_y = self.robot_y + ball_rel_x * sin_yaw + ball_rel_y * cos_yaw
            self.ball_position_valid = True

            self.get_logger().info(
                f'⚽ BALLE monde: ({self.ball_world_x:.2f}, {self.ball_world_y:.2f}) '
                f'robot: ({self.robot_x:.2f}, {self.robot_y:.2f}) '
                f'dist={ball_dist:.2f}m'
            )
        else:
            self.ball_detected = False
            self.ball_seg_indices = []

            # Position monde invalide après 60 secondes sans détection
            if t - self.last_ball_seen_time > 60.0:
                self.ball_position_valid = False
                self.ball_world_x = float('nan')
                self.ball_world_y = float('nan')

        # ── 2. Obstacles (exclusion points balle) ──────────────
        ball_set = set(ball_indices)

        # Masque : valide ET pas balle
        valid = np.array([(i not in ball_set) and
                          np.isfinite(ranges[i]) and
                          0.1 < ranges[i] < 30.0
                          for i in range(n)], dtype=bool)

        # Cone avant ±45° (indices autour de n//2)
        i_c   = n // 2
        i_45  = n // 8
        fi    = np.arange(i_c - i_45, i_c + i_45)
        fi    = fi[(fi >= 0) & (fi < n)]
        front_r = ranges[fi][valid[fi]]
        self.min_dist = float(np.min(front_r)) if len(front_r) > 0 else 30.0

        # Gauche / droite pour choisir direction AVOID
        li = np.arange(int(n*0.60), int(n*0.85))
        ri = np.arange(int(n*0.15), int(n*0.40))
        li = li[(li >= 0) & (li < n)]
        ri = ri[(ri >= 0) & (ri < n)]
        lr = ranges[li][valid[li]]
        rr = ranges[ri][valid[ri]]
        self.front_left_dist  = float(np.min(lr)) if len(lr) > 0 else 30.0
        self.front_right_dist = float(np.min(rr)) if len(rr) > 0 else 30.0

        # ── 3. Vue radar debug ─────────────────────────────────
        self._publish_radar(ranges, angles, n, ball_indices,
                            ball_found, ball_angle, ball_dist)

    # ──────────────────────────────────────────────────────────
    def _find_ball(self, ranges, angles, n):
        """
        Cherche UNIQUEMENT un segment circulaire (demi-cercle) correspondant à la balle.
        PLUS de filtrage par couleur BLANC - QUE le profil circulaire LIDAR !
        Retourne (found, angle, distance, indices_list).
        """
        # Marquer les points valides - Portée étendue à 150m
        valid = (ranges > 0.01) & (ranges < 150.0) & np.isfinite(ranges)

        # ── Segmentation : groupes de points proches ───────────
        MAX_JUMP = 0.5  # Plus tolérant pour les segments plus fins
        MIN_SEG  = 1    # Un seul point peut suffire avec résolution 7200 !

        segments = []
        i = 0
        while i < n:
            if not valid[i]:
                i += 1
                continue
            j = i + 1
            while j < n and valid[j] and abs(ranges[j] - ranges[j-1]) < MAX_JUMP:
                j += 1
            if j - i >= MIN_SEG:
                segments.append((i, j))
            i = j

        # Wrap-around
        if (len(segments) >= 2 and segments[0][0] == 0 and
                segments[-1][1] == n and
                abs(ranges[0] - ranges[n-1]) < MAX_JUMP):
            sL_start, sL_end = segments[-1]
            s0_start, s0_end = segments[0]
            segments = [(sL_start, s0_end)]

        best_found = False
        best_angle = 0.0
        best_dist  = 150.0
        best_idxs  = []
        best_score = 0.0

        for (s, e) in segments:
            if e <= s:
                continue
            seg_r = ranges[s:e]
            seg_a = angles[s:e]
            L = e - s
            if L == 0:
                continue

            d_min   = float(np.min(seg_r))
            d_ball  = d_min + BALL_RADIUS

            if d_ball > 150.0:  # Portée maximale du LIDAR
                continue

            # CALCULER LE CENTRE DU SEGMENT
            a_center = float(np.mean(seg_a))

            # FILTRE : le segment doit être "face au robot" - PLUS TOLERANT
            min_idx_local = int(np.argmin(seg_r))
            a_min = float(seg_a[min_idx_local])

            # L'angle du point le plus proche doit être proche du centre du segment
            center_diff = np.abs(np.arctan2(np.sin(a_min - a_center),
                                            np.cos(a_min - a_center)))
            if center_diff > math.pi / 3:  # 60° au lieu de 30° - PLUS TOLERANT
                continue

            # Vérification du profil circulaire (TRES TOLERANT)
            if L > 1:
                ang_span = float(seg_a[-1] - seg_a[0])
            else:
                ang_span = 0.0
            if ang_span < 0:
                ang_span += 2*math.pi

            sin_val = min(BALL_RADIUS / d_ball, 0.9999)
            expected = 2.0 * math.asin(sin_val)
            ratio = ang_span / expected if expected > 0 else 999.0

            # Accepter TOUT profil circulaire, même très petit arc (ratio entre 0.05 et 10.0)
            # TRES TOLERANT - accepte même 1/10 de cercle
            if ratio < 0.05 or ratio > 10.0:
                continue

            # Plus de filtre de profondeur - trop restrictif
            # On accepte le segment dès que le profil angulaire est bon

            # Score : UNIQUEMENT la proximité (pas de bonus couleur)
            score = 1.0 / (d_ball + 0.5)

            if score > best_score:
                best_score = score
                best_found = True
                best_angle = a_center  # Centre du segment = angle de la balle
                best_dist = d_ball
                best_idxs = list(range(s, e))

        return best_found, best_angle, best_dist, best_idxs

    # ──────────────────────────────────────────────────────────
    def _publish_radar(self, ranges, angles, n, ball_idxs,
                       ball_found, ball_angle, ball_dist):
        """Génère et publie une image radar top-down 500×500."""
        SZ    = 500
        SCALE = 16       # pixels / mètre  (30m → 480px → tout le terrain visible)
        CX, CY = SZ//2, SZ//2

        img = np.full((SZ, SZ, 3), C_BG, dtype=np.uint8)

        # Grille concentrique
        for r_m in [2, 5, 10, 15, 20, 25]:
            r_px = int(r_m * SCALE)
            if r_px < SZ//2:
                cv2.circle(img, (CX, CY), r_px, C_GRID, 1)
                cv2.putText(img, f'{r_m}m', (CX + r_px + 2, CY),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.3, C_GRID, 1)

        # Axes
        cv2.line(img, (CX, 0),    (CX, SZ),    C_GRID, 1)
        cv2.line(img, (0,  CY),   (SZ, CY),    C_GRID, 1)

        # Points LIDAR - BALLE en vert, le reste en gris
        ball_set = set(ball_idxs)
        for i in range(n):
            r = ranges[i]
            if not (0.1 < r < 100.0) or not np.isfinite(r):
                continue
            a = angles[i]
            px = int(CX + r * SCALE * math.sin(a))
            py = int(CY - r * SCALE * math.cos(a))
            px = int(np.clip(px, 0, SZ-1))
            py = int(np.clip(py, 0, SZ-1))

            if i in ball_set:
                col, sz = C_BALL, 3
            else:
                col, sz = C_LIDAR, 1
            cv2.circle(img, (px, py), sz, col, -1)

        # Cercle balle estimé
        if ball_found:
            bpx = int(CX + ball_dist * SCALE * math.sin(ball_angle))
            bpy = int(CY - ball_dist * SCALE * math.cos(ball_angle))
            brad_px = int(BALL_RADIUS * SCALE)
            col = C_LOCK if self.has_lock else C_BALL
            cv2.circle(img, (bpx, bpy), brad_px, col, 2)
            cv2.circle(img, (bpx, bpy), 3, col, -1)
            lbl = f'{ball_dist:.1f}m'
            cv2.putText(img, lbl,
                        (bpx + brad_px + 3, bpy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1, cv2.LINE_AA)

        # Robot (triangle pointant vers l'avant)
        robot_pts = np.array([[0, -12], [-8, 8], [8, 8]], np.int32) + [CX, CY]
        cv2.polylines(img, [robot_pts], True, C_ROBOT, 2)
        cv2.arrowedLine(img, (CX, CY), (CX, CY-18), C_ROBOT, 2, tipLength=0.4)

        # Direction LOCK
        if self.has_lock:
            lx = int(CX + 60 * math.sin(self.lock_angle))
            ly = int(CY - 60 * math.cos(self.lock_angle))
            cv2.arrowedLine(img, (CX, CY), (lx, ly), C_LOCK, 2, tipLength=0.25)

        # Position monde de la balle (indicateur)
        if self.ball_position_valid and not math.isnan(self.ball_world_x):
            # Afficher position monde de la balle en bas à gauche
            cv2.putText(img, f'BALLE MONDE:', (5, SZ-50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0), 1, cv2.LINE_AA)
            cv2.putText(img, f'X:{self.ball_world_x:.2f} Y:{self.ball_world_y:.2f}', (5, SZ-30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0), 1, cv2.LINE_AA)
            # Afficher position robot
            cv2.putText(img, f'ROBOT:', (5, SZ-15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, C_ROBOT, 1, cv2.LINE_AA)
            cv2.putText(img, f'X:{self.robot_x:.2f} Y:{self.robot_y:.2f}', (120, SZ-15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.3, C_ROBOT, 1, cv2.LINE_AA)

        # Badges état
        state_col = STATE_COL.get(self.state, C_WHITE)
        badge(img, 5, 5,  self.state, state_col,
              C_BLACK if self.state != "AVOID" else C_WHITE)

        # Indicateur POSITION BALLE
        if self.ball_position_valid:
            badge(img, SZ//2-30, 5, 'POS BALLE OK', (0, 255, 0), C_BLACK)
        elif ball_found:
            badge(img, SZ//2-35, 5, 'DETECT', C_YELLOW, C_BLACK)
        else:
            badge(img, SZ//2-38, 5, 'NO BALL', (60, 60, 60))

        # Obstacle
        if self.min_dist < self.OBS_SAFE:
            col_o = C_AVOID if self.min_dist < self.OBS_CRIT else C_WARN
            cv2.putText(img, f'OBS {self.min_dist:.2f}m',
                        (5, SZ-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, col_o, 1, cv2.LINE_AA)

        # Distance obstacle dessinée
        if self.min_dist < 9.0:
            r_obs_px = int(self.min_dist * SCALE)
            col_o    = C_AVOID if self.min_dist < self.OBS_CRIT else C_WARN
            # Arc avant ±45°
            cv2.ellipse(img, (CX, CY), (r_obs_px, r_obs_px),
                        -90, -45, 45, col_o, 1)

        try:
            msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
            self.debug_pub.publish(msg)
        except Exception:
            pass

    # ──────────────────────────────────────────────────────────
    def _smooth(self, tlin, tang):
        dt = 0.002
        dl = tlin - self.cur_lin
        if abs(dl) > 0:
            mc = (self.MAX_DECEL if
                  (self.cur_lin > 0 and dl < 0) or
                  (self.cur_lin < 0 and dl > 0)
                  else self.MAX_ACCEL) * dt
            if abs(dl) > mc:
                dl = math.copysign(mc, dl)
        self.cur_lin += dl
        self.cur_ang  = tang
        return self.cur_lin, self.cur_ang

    # ──────────────────────────────────────────────────────────
    def control_loop(self):
        """Logique ULTRA-SIMPLE : aller vers lock_angle (demi-cercle LIDAR fiable)."""
        self.tick += 1
        self.now = time.monotonic()
        if not self.running or not rclpy.ok():
            return

        tl, ta = 0.0, 0.0

        # ── PRIORITÉ ABSOLUE : Robot retourné sur le côté (roll/pitch) ──
        roll_abs = abs(self.robot_roll)
        pitch_abs = abs(self.robot_pitch)
        if roll_abs > 1.2 or pitch_abs > 1.2:
            self.state = "FLIP"
            tl = 0.0
            ta = 0.0
            self._pub(tl, ta)
            return

        # ── PRIORITÉ 2 : Obstacle critique ──
        if self.min_dist < self.OBS_CRIT:
            self.state = "AVOID"
            tl = -2.0  # RECULER FORT
            ta = (self.TURN_AVOID if self.front_left_dist > self.front_right_dist
                  else -self.TURN_AVOID)
            self._pub(tl, ta)
            return

        # ── DÉTECTION BALLE : UNIQUEMENT lock_angle (demi-cercle LIDAR) ──
        if not self.ball_position_valid:
            # Pas de position balle connue = attendre
            self.state = "SEARCH"
            tl = 0.0
            ta = 0.0
            self._pub(tl, ta)
            if self.tick % 200 == 0:
                self.get_logger().info('🔍 SEARCH - attend position balle')
            return

        # ── ALLER VERS LA BALLE EN COORDONNÉES MONDE ──
        self.state = "CHASE"

        # Différence position robot -> balle (monde)
        dx = self.ball_world_x - self.robot_x
        dy = self.ball_world_y - self.robot_y

        # Distance vers la balle
        dist_to_ball = math.sqrt(dx*dx + dy*dy)

        # Angle vers la balle dans le repère MONDE
        target_yaw_world = math.atan2(dy, dx)

        # Angle vers la balle dans le repère ROBOT
        target_yaw_robot = target_yaw_world - self.robot_yaw
        # Normaliser entre -pi et pi
        while target_yaw_robot > math.pi: target_yaw_robot -= 2*math.pi
        while target_yaw_robot < -math.pi: target_yaw_robot += 2*math.pi

        # Contrôle : tourner vers la balle et avancer
        ta = float(np.clip(-4.0 * target_yaw_robot, -10.0, 10.0))

        # Plus on est aligné, plus on va vite
        align_factor = max(0.2, 1.0 - abs(target_yaw_robot) / 1.0)
        tl = self.SPD_CHARGE * align_factor

        # LOG toutes les 50 ticks
        if self.tick % 50 == 0:
            self.get_logger().info(
                f'🎯 CHASSE! monde_balle=({self.ball_world_x:.2f},{self.ball_world_y:.2f}) '
                f'robot=({self.robot_x:.2f},{self.robot_y:.2f}) '
                f'dist={dist_to_ball:.2f}m angle={math.degrees(target_yaw_robot):.1f}°'
            )

        self._pub(tl, ta)

    def _pub(self, tl, ta):
        al, aa = self._smooth(tl, ta)
        t = Twist(); t.linear.x = al; t.angular.z = aa
        self.cmd_pub.publish(t)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = FastHunter()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.running = False
            node.destroy_node()
            rclpy.shutdown()
    except Exception as e:
        print(f"ERREUR: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
