#!/usr/bin/env python3
"""
Ball Position Publisher - Publie la position de la balle depuis Gazebo
Appelle le service /get_entity_state et publie sur /ball/position a 10Hz
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from gazebo_msgs.srv import GetEntityState
import signal
import time


class BallPositionPublisher(Node):
    def __init__(self):
        super().__init__('ball_position_publisher')
        signal.signal(signal.SIGINT, self._sig)
        signal.signal(signal.SIGTERM, self._sig)

        self.pub = self.create_publisher(PointStamped, '/ball/position', 10)
        self.client = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        self.running = True
        self.ready = False
        self.pub_count = 0

        self.get_logger().info('Ball position publisher demarre - attente service /gazebo/get_entity_state...')

        # Timer wall-clock (pas sim_time) pour poll regulier
        self.create_timer(0.1, self.poll_ball)

    def _sig(self, *_):
        self.running = False

    def poll_ball(self):
        if not self.running:
            return

        if not self.client.service_is_ready():
            return

        if not self.ready:
            self.get_logger().info('Service /gazebo/get_entity_state PRET - debut publication')
            self.ready = True

        req = GetEntityState.Request()
        req.name = 'ball'
        future = self.client.call_async(req)
        future.add_done_callback(self._cb)

    def _cb(self, future):
        try:
            result = future.result()
            if result.success:
                msg = PointStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'world'
                msg.point.x = result.state.pose.position.x
                msg.point.y = result.state.pose.position.y
                msg.point.z = result.state.pose.position.z
                self.pub.publish(msg)
                self.pub_count += 1
                if self.pub_count % 50 == 1:
                    self.get_logger().info(
                        f'Balle: x={msg.point.x:.2f} y={msg.point.y:.2f} z={msg.point.z:.2f} '
                        f'(pub #{self.pub_count})'
                    )
            else:
                if self.pub_count == 0:
                    self.get_logger().warn('Service OK mais entity "ball" non trouvee')
        except Exception as e:
            self.get_logger().error(f'Erreur service: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = BallPositionPublisher()
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
