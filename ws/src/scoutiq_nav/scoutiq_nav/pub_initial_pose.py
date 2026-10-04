#!/usr/bin/env python3
"""Publish initial pose (0, 0, 0) to /initialpose for AMCL localization.

Ported from fetch_navigation/scripts/pub_initial_pose.py for ROS 2 Humble.
"""
import math
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('cov_x', 1.0)
        self.declare_parameter('cov_y', 1.0)
        self.declare_parameter('cov_yaw', float(np.deg2rad(10.0) ** 2))
        self.declare_parameter('delay', 1.0)
        self.declare_parameter('repeat_count', 3)
        self.declare_parameter('rate', 1.0)
        self.declare_parameter('exit_after_publish', True)

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.x = self.get_parameter('x').get_parameter_value().double_value
        self.y = self.get_parameter('y').get_parameter_value().double_value
        self.z = self.get_parameter('z').get_parameter_value().double_value
        self.yaw = self.get_parameter('yaw').get_parameter_value().double_value
        self.cov_x = self.get_parameter('cov_x').get_parameter_value().double_value
        self.cov_y = self.get_parameter('cov_y').get_parameter_value().double_value
        self.cov_yaw = self.get_parameter('cov_yaw').get_parameter_value().double_value
        self.delay = self.get_parameter('delay').get_parameter_value().double_value
        self.repeat_count = self.get_parameter('repeat_count').get_parameter_value().integer_value
        self.rate = self.get_parameter('rate').get_parameter_value().double_value
        self.exit_after_publish = self.get_parameter('exit_after_publish').get_parameter_value().bool_value

        # Contract: Reliable QoS; Transient Local so late-joining AMCL receives the pose
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', qos)
        self.count = 0
        self.start_time = self.get_clock().now()

        timer_period = 1.0 / max(0.1, self.rate)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('initial_pose_publisher initialized, waiting for initial delay...')

    def timer_callback(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        if elapsed < self.delay:
            return

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = self.z

        # Orientation quaternion from yaw
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)

        # Full 6x6 covariance matrix matching original fetch_navigation:
        # cov_x, cov_y, cov_yaw with 1e-9 for non-planar components
        cov = [
            self.cov_x, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, self.cov_y, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e-9, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e-9, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e-9, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, self.cov_yaw
        ]
        msg.pose.covariance = cov

        self.pub.publish(msg)
        self.count += 1
        self.get_logger().info(
            f"Published initial pose with zero position and orientation: "
            f"pos=({self.x:.2f}, {self.y:.2f}, {self.z:.2f}), yaw={self.yaw:.2f} ({self.count}/{self.repeat_count})"
        )

        if self.count >= self.repeat_count:
            self.get_logger().info("Initial pose publishing completed.")
            self.timer.cancel()
            if self.exit_after_publish:
                # Give DDS network a moment to send before exiting
                raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
