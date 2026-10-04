#!/usr/bin/env python3
"""Standalone map saving node for exploration completion (ROS 2 port of savemap.py)."""
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class SaveMapNode(Node):
    def __init__(self):
        super().__init__('savemap')
        self.declare_parameter('run_dir', '')
        self.declare_parameter('data_dir', 'data')
        self.declare_parameter('map_topic', '/map')

        self.run_dir = self.get_parameter('run_dir').value
        self.data_dir = self.get_parameter('data_dir').value
        self.map_topic = self.get_parameter('map_topic').value

        self.sub = self.create_subscription(
            Bool,
            'explore/exploration_termination',
            self.on_termination,
            10
        )
        self.get_logger().info('savemap node started, waiting for termination signal...')

    def on_termination(self, msg: Bool):
        if not msg.data:
            return
        self.get_logger().info('Exploration termination received, triggering map save.')

        if self.run_dir:
            target_dir = self.run_dir
        else:
            import datetime
            ts = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
            target_dir = os.path.join(self.data_dir, ts)

        os.makedirs(target_dir, exist_ok=True)
        target_path = os.path.join(target_dir, 'map')
        cmd = f"ros2 run nav2_map_server map_saver_cli -t {self.map_topic} -f {target_path} --occ 0.65 --free 0.25"
        self.get_logger().info(f"Running: {cmd}")
        os.system(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SaveMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
