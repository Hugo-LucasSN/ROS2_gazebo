#!/usr/bin/env python3
"""
Robot Uprighter - Détecte quand robot est renversé et le redresse avec un saut
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Twist
import math
import time


class RobotUprighter(Node):
    def __init__(self, robot_name):
        super().__init__(f'{robot_name}_uprighter')

        self.robot_name = robot_name
        self.last_upright_time = 0
        # Reactif: redressement possible toutes les ~1.5s
        self.upright_cooldown = 1.5
        self.current_pose = None

        # Tracking pour flip rapide
        self.tipped_start_time = None  # Quand le robot s'est renversé
        self.is_currently_tipped = False

        # Client pour téléporter robot
        self.set_state_client = self.create_client(
            SetEntityState, '/gazebo/set_entity_state')

        # Subscriber odométrie
        self.create_subscription(
            Odometry, f'/{robot_name}/odom', self.odom_cb, 10)

        self.get_logger().info(f'🛡️ Uprighter activé pour {robot_name}')

    def odom_cb(self, msg):
        """Vérifie orientation du robot"""
        # Sauvegarder position actuelle
        self.current_pose = msg.pose.pose

        # Quaternion → Roll, Pitch, Yaw
        q = msg.pose.pose.orientation

        # Calcul roll (rotation autour X)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Calcul pitch (rotation autour Y)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Calcul yaw (rotation autour Z)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Détection: robot fortement incliné
        tipped_threshold = math.radians(38.0)
        current_time = time.time()

        if abs(roll) > tipped_threshold or abs(pitch) > tipped_threshold:
            # Robot renversé!
            if not self.is_currently_tipped:
                # Vient de se renverser - commencer à compter
                self.tipped_start_time = current_time
                self.is_currently_tipped = True
                self.get_logger().info(f'⚠️ {self.robot_name} RENVERSE! roll={math.degrees(roll):.1f}° pitch={math.degrees(pitch):.1f}°')

            # Vérifier si renversé depuis un court delai (anti jitter)
            time_tipped = current_time - self.tipped_start_time
            if time_tipped > 0.35:
                # Renversé depuis suffisamment longtemps - FLIP!
                if (current_time - self.last_upright_time) > self.upright_cooldown:
                    self.get_logger().info(f'💥 FLIP après {time_tipped:.1f}s! roll={math.degrees(roll):.1f}° pitch={math.degrees(pitch):.1f}°')
                    self.apply_jump(yaw)
                    self.last_upright_time = current_time
                    self.is_currently_tipped = False
                    self.tipped_start_time = None
        else:
            # Robot droit - réinitialiser tracking
            if self.is_currently_tipped:
                self.get_logger().info(f'✅ {self.robot_name} redressé seul!')
            self.is_currently_tipped = False
            self.tipped_start_time = None

    def apply_jump(self, yaw):
        """Flip robot sur le côté pour le redresser"""
        if self.current_pose is None:
            return

        if not self.set_state_client.wait_for_service(timeout_sec=1.0):
            return

        request = SetEntityState.Request()
        state = EntityState()
        state.name = self.robot_name

        # Position proche du sol pour rester stable
        state.pose = Pose()
        state.pose.position.x = self.current_pose.position.x
        state.pose.position.y = self.current_pose.position.y
        state.pose.position.z = 0.18

        # Orientation redressée (roll=0, pitch=0, yaw conservé)
        state.pose.orientation.x = 0.0
        state.pose.orientation.y = 0.0
        state.pose.orientation.z = math.sin(yaw / 2.0)
        state.pose.orientation.w = math.cos(yaw / 2.0)

        # Petite impulsion de flip pour retomber sur les roues
        state.twist = Twist()
        state.twist.linear.x = 0.0
        state.twist.linear.y = 0.0
        state.twist.linear.z = 0.6
        state.twist.angular.x = 4.0
        state.twist.angular.y = 0.0
        state.twist.angular.z = 0.0

        request.state = state

        # APPEL DIRECT
        try:
            self.set_state_client.call_async(request)
            self.get_logger().info(f'💫 FLIP {self.robot_name}!')
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)

    # Créer uprighters pour les deux robots
    blue_uprighter = RobotUprighter('bocbot_blue')
    orange_uprighter = RobotUprighter('bocbot_orange')

    # Spin both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(blue_uprighter)
    executor.add_node(orange_uprighter)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        blue_uprighter.destroy_node()
        orange_uprighter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
