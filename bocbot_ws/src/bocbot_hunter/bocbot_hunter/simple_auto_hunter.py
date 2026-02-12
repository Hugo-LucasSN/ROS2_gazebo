#!/usr/bin/env python3
"""
Simple Auto Hunter - Robot ORANGE qui suit la balle
Appelle le service /gazebo/get_entity_state pour la position
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetEntityState
import math
import signal


class SimpleAutoHunter(Node):
    def __init__(self):
        super().__init__('simple_auto_hunter')
        signal.signal(signal.SIGINT, self._sig)
        signal.signal(signal.SIGTERM, self._sig)

        self.robot_name = self.get_namespace().strip('/')
        self.ball_x = 0.0
        self.ball_y = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.ball_found = False
        self.running = True
        self.tick = 0
        self.pending = False

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_state = self.create_client(
            GetEntityState, '/gazebo/get_entity_state'
        )

        self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f'Auto hunter [{self.robot_name}] - service /gazebo/get_entity_state'
        )

    def _sig(self, *_):
        self.running = False

    def control_loop(self):
        self.tick += 1
        if not self.running:
            return

        if not self.get_state.service_is_ready():
            if self.tick % 40 == 0:
                self.get_logger().info('Attente service get_entity_state...')
            self._pub(0.0, 0.0)
            return

        # Request ball + robot positions (async, non-blocking)
        if not self.pending:
            self.pending = True

            req_ball = GetEntityState.Request()
            req_ball.name = 'ball'
            f1 = self.get_state.call_async(req_ball)
            f1.add_done_callback(self._on_ball)

            req_robot = GetEntityState.Request()
            req_robot.name = self.robot_name
            f2 = self.get_state.call_async(req_robot)
            f2.add_done_callback(self._on_robot)

        # Navigate with last known positions
        self._navigate()

    def _on_ball(self, future):
        self.pending = False
        try:
            resp = future.result()
            if resp and resp.success:
                self.ball_x = resp.state.pose.position.x
                self.ball_y = resp.state.pose.position.y
                if not self.ball_found:
                    self.get_logger().info(
                        f'Balle trouvee: ({self.ball_x:.1f}, {self.ball_y:.1f})'
                    )
                self.ball_found = True
        except Exception:
            pass

    def _on_robot(self, future):
        try:
            resp = future.result()
            if resp and resp.success:
                p = resp.state.pose.position
                q = resp.state.pose.orientation
                self.robot_x = p.x
                self.robot_y = p.y
                self.robot_yaw = math.atan2(
                    2 * (q.w * q.z + q.x * q.y),
                    1 - 2 * (q.y * q.y + q.z * q.z)
                )
        except Exception:
            pass

    def _navigate(self):
        if not self.ball_found:
            self._pub(0.0, 1.5)
            return

        dx = self.ball_x - self.robot_x
        dy = self.ball_y - self.robot_y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < 0.5:
            self._pub(1.0, 0.0)
            return

        target_yaw = math.atan2(dy, dx)
        err = math.atan2(
            math.sin(target_yaw - self.robot_yaw),
            math.cos(target_yaw - self.robot_yaw)
        )

        angular = max(-12.0, min(12.0, -6.0 * err))
        align = max(0.1, 1.0 - abs(err) / 0.7)
        linear = 6.0 * align

        if self.tick % 40 == 0:
            self.get_logger().info(
                f'Balle ({self.ball_x:.1f},{self.ball_y:.1f}) '
                f'robot ({self.robot_x:.1f},{self.robot_y:.1f}) d={dist:.1f}m'
            )

        self._pub(linear, angular)

    def _pub(self, linear, angular):
        t = Twist()
        t.linear.x = float(linear)
        t.angular.z = float(angular)
        self.cmd_pub.publish(t)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleAutoHunter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
