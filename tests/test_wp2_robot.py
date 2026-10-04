#!/usr/bin/env python3
"""
Test script for WP2: Robot & Sensors Port.
Verifies all CONTRACT requirements:
- /clock (rosgraph_msgs/Clock)
- /cmd_vel (geometry_msgs/Twist) drives the robot
- /odom (nav_msgs/Odometry) updates with odom -> base_link
- /scan (sensor_msgs/LaserScan, frame laser_link, 10Hz, 360 samples, range 0.05-25m)
- /head_camera/rgb/image_raw (sensor_msgs/Image, rgb8/bgr8, frame head_camera_rgb_optical_frame)
- /head_camera/depth_registered/image_raw (sensor_msgs/Image, 32FC1 metres, frame head_camera_rgb_optical_frame)
- /head_camera/rgb/camera_info (sensor_msgs/CameraInfo, frame head_camera_rgb_optical_frame)
- /joint_states (sensor_msgs/JointState)
- /tf (TF tree: odom -> base_link -> laser_link, head_camera_rgb_optical_frame, etc.)
"""

import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan
from tf2_msgs.msg import TFMessage
import tf2_ros


class WP2Tester(Node):
    def __init__(self):
        super().__init__('wp2_tester')

        self.clock_msgs = []
        self.odom_msgs = []
        self.scan_msgs = []
        self.rgb_msgs = []
        self.depth_msgs = []
        self.cinfo_msgs = []
        self.joint_msgs = []
        self.tf_msgs = []

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(Clock, '/clock', lambda m: self.clock_msgs.append(m), 10)
        self.create_subscription(Odometry, '/odom', lambda m: self.odom_msgs.append(m), reliable_qos)
        self.create_subscription(LaserScan, '/scan', lambda m: self.scan_msgs.append(m), qos_profile_sensor_data)
        self.create_subscription(Image, '/head_camera/rgb/image_raw', lambda m: self.rgb_msgs.append(m), qos_profile_sensor_data)
        self.create_subscription(Image, '/head_camera/depth_registered/image_raw', lambda m: self.depth_msgs.append(m), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/head_camera/rgb/camera_info', lambda m: self.cinfo_msgs.append(m), qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', lambda m: self.joint_msgs.append(m), reliable_qos)
        self.create_subscription(TFMessage, '/tf', lambda m: self.tf_msgs.append(m), reliable_qos)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', reliable_qos)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)


def main():
    print("=" * 60)
    print("WP2 Verification Test")
    print(f"ROS_DOMAIN_ID: {os.environ.get('ROS_DOMAIN_ID', 'unset')}")
    print(f"IGN_PARTITION: {os.environ.get('IGN_PARTITION', 'unset')}")
    print("=" * 60)

    rclpy.init()
    tester = WP2Tester()

    # Phase 1: Wait for sensor topics and clock
    print("[1/3] Waiting for sensor topics and clock (up to 15s)...")
    start = time.time()
    while time.time() - start < 15:
        rclpy.spin_once(tester, timeout_sec=0.1)
        if (len(tester.clock_msgs) >= 5 and
            len(tester.odom_msgs) >= 3 and
            len(tester.scan_msgs) >= 3 and
            len(tester.rgb_msgs) >= 3 and
            len(tester.depth_msgs) >= 3 and
            len(tester.cinfo_msgs) >= 3 and
            len(tester.joint_msgs) >= 3 and
            len(tester.tf_msgs) >= 3):
            break

    # Check received counts
    print(f"  Clock: {len(tester.clock_msgs)}")
    print(f"  Odom: {len(tester.odom_msgs)}")
    print(f"  Scan: {len(tester.scan_msgs)}")
    print(f"  RGB: {len(tester.rgb_msgs)}")
    print(f"  Depth: {len(tester.depth_msgs)}")
    print(f"  CameraInfo: {len(tester.cinfo_msgs)}")
    print(f"  JointStates: {len(tester.joint_msgs)}")
    print(f"  TF: {len(tester.tf_msgs)}")

    assert len(tester.clock_msgs) > 0, "FAIL: No /clock received"
    assert len(tester.odom_msgs) > 0, "FAIL: No /odom received"
    assert len(tester.scan_msgs) > 0, "FAIL: No /scan received"
    assert len(tester.rgb_msgs) > 0, "FAIL: No /head_camera/rgb/image_raw received"
    assert len(tester.depth_msgs) > 0, "FAIL: No /head_camera/depth_registered/image_raw received"
    assert len(tester.cinfo_msgs) > 0, "FAIL: No /head_camera/rgb/camera_info received"
    assert len(tester.joint_msgs) > 0, "FAIL: No /joint_states received"
    assert len(tester.tf_msgs) > 0, "FAIL: No /tf received"

    # Phase 2: Verify CONTRACT specifications
    print("[2/3] Verifying CONTRACT specifications...")

    # Odom
    odom = tester.odom_msgs[-1]
    print(f"  Odom frame_id: '{odom.header.frame_id}', child_frame_id: '{odom.child_frame_id}'")
    assert odom.header.frame_id == 'odom', f"Expected odom frame 'odom', got '{odom.header.frame_id}'"
    assert odom.child_frame_id == 'base_link', f"Expected child frame 'base_link', got '{odom.child_frame_id}'"

    # Scan
    scan = tester.scan_msgs[-1]
    print(f"  Scan frame_id: '{scan.header.frame_id}', samples: {len(scan.ranges)}, range: [{scan.range_min:.2f}, {scan.range_max:.2f}]")
    assert scan.header.frame_id == 'laser_link', f"Expected scan frame 'laser_link', got '{scan.header.frame_id}'"
    assert len(scan.ranges) == 360, f"Expected 360 scan samples, got {len(scan.ranges)}"
    assert abs(scan.range_min - 0.05) < 0.01, f"Expected range_min 0.05, got {scan.range_min}"
    assert abs(scan.range_max - 25.0) < 0.1, f"Expected range_max 25.0, got {scan.range_max}"

    # RGB
    rgb = tester.rgb_msgs[-1]
    print(f"  RGB frame_id: '{rgb.header.frame_id}', encoding: '{rgb.encoding}', size: {rgb.width}x{rgb.height}")
    assert rgb.header.frame_id == 'head_camera_rgb_optical_frame', f"Expected RGB frame 'head_camera_rgb_optical_frame', got '{rgb.header.frame_id}'"
    assert rgb.encoding in ('rgb8', 'bgr8'), f"Expected rgb8/bgr8, got '{rgb.encoding}'"
    assert rgb.width == 640 and rgb.height == 480, f"Expected 640x480, got {rgb.width}x{rgb.height}"

    # Depth
    depth = tester.depth_msgs[-1]
    print(f"  Depth frame_id: '{depth.header.frame_id}', encoding: '{depth.encoding}', size: {depth.width}x{depth.height}")
    assert depth.header.frame_id == 'head_camera_rgb_optical_frame', f"Expected Depth frame 'head_camera_rgb_optical_frame', got '{depth.header.frame_id}'"
    assert depth.encoding == '32FC1', f"Expected 32FC1, got '{depth.encoding}'"
    assert depth.width == 640 and depth.height == 480, f"Expected 640x480, got {depth.width}x{depth.height}"

    # CameraInfo
    cinfo = tester.cinfo_msgs[-1]
    print(f"  CameraInfo frame_id: '{cinfo.header.frame_id}', size: {cinfo.width}x{cinfo.height}")
    assert cinfo.header.frame_id == 'head_camera_rgb_optical_frame', f"Expected cinfo frame 'head_camera_rgb_optical_frame', got '{cinfo.header.frame_id}'"
    assert cinfo.width == 640 and cinfo.height == 480, f"Expected 640x480, got {cinfo.width}x{cinfo.height}"

    # Joint states
    joints = tester.joint_msgs[-1]
    print(f"  JointStates joints count: {len(joints.name)}, sample joints: {joints.name[:4]}")
    assert 'l_wheel_joint' in joints.name, "l_wheel_joint missing from joint_states"
    assert 'r_wheel_joint' in joints.name, "r_wheel_joint missing from joint_states"

    # TF frames and buffer lookup
    t_laser = tester.tf_buffer.lookup_transform('odom', 'laser_link', rclpy.time.Time())
    print(f"  TF lookup odom -> laser_link: [{t_laser.transform.translation.x:.3f}, {t_laser.transform.translation.y:.3f}, {t_laser.transform.translation.z:.3f}]")
    t_cam = tester.tf_buffer.lookup_transform('odom', 'head_camera_rgb_optical_frame', rclpy.time.Time())
    print(f"  TF lookup odom -> head_camera_rgb_optical_frame: [{t_cam.transform.translation.x:.3f}, {t_cam.transform.translation.y:.3f}, {t_cam.transform.translation.z:.3f}]")

    # Phase 3: Test DiffDrive actuation via /cmd_vel
    print("[3/3] Testing robot drive via /cmd_vel...")
    initial_x = tester.odom_msgs[-1].pose.pose.position.x
    initial_y = tester.odom_msgs[-1].pose.pose.position.y
    print(f"  Initial pose: x={initial_x:.3f}, y={initial_y:.3f}")

    cmd = Twist()
    cmd.linear.x = 0.5
    drive_start = time.time()
    while time.time() - drive_start < 3.0:
        tester.cmd_pub.publish(cmd)
        rclpy.spin_once(tester, timeout_sec=0.1)

    # Stop the robot
    cmd.linear.x = 0.0
    for _ in range(5):
        tester.cmd_pub.publish(cmd)
        rclpy.spin_once(tester, timeout_sec=0.05)

    final_x = tester.odom_msgs[-1].pose.pose.position.x
    final_y = tester.odom_msgs[-1].pose.pose.position.y
    dist_moved = math.hypot(final_x - initial_x, final_y - initial_y)
    print(f"  Final pose: x={final_x:.3f}, y={final_y:.3f}, distance moved: {dist_moved:.3f} m")

    assert dist_moved > 0.2, f"FAIL: Robot moved only {dist_moved:.3f}m (< 0.2m required)"

    print("=" * 60)
    print("ALL WP2 VERIFICATION TESTS PASSED SUCCESSFULLY!")
    print("=" * 60)
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
