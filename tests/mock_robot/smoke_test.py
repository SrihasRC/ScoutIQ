#!/usr/bin/env python3
"""Smoke test for mock_robot: topics flow, TF resolves, navigate_to_pose works. Exit 0 on success."""
import sys, time, math
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from tf2_ros import Buffer, TransformListener

rclpy.init()
n = Node('mock_smoke')
got = {}
n.create_subscription(LaserScan, '/scan', lambda m: got.__setitem__('scan', m), qos_profile_sensor_data)
n.create_subscription(Image, '/head_camera/rgb/image_raw', lambda m: got.__setitem__('rgb', m), qos_profile_sensor_data)
n.create_subscription(Image, '/head_camera/depth_registered/image_raw', lambda m: got.__setitem__('depth', m), qos_profile_sensor_data)
n.create_subscription(CameraInfo, '/head_camera/rgb/camera_info', lambda m: got.__setitem__('info', m), qos_profile_sensor_data)
n.create_subscription(OccupancyGrid, '/map', lambda m: got.__setitem__('map', m),
                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
buf = Buffer(); TransformListener(buf, n)
t0 = time.time()
while time.time() - t0 < 6 and len(got) < 5:
    rclpy.spin_once(n, timeout_sec=0.1)
ok = True
def check(name, cond):
    global ok
    print(('[ok]   ' if cond else '[FAIL] ') + name); ok &= bool(cond)
check('all topics received: %s' % sorted(got), len(got) == 5)
if 'scan' in got:
    check('scan has finite ranges', min(got['scan'].ranges) > 0.1)
if 'depth' in got:
    check('depth encoding 32FC1', got['depth'].encoding == '32FC1')

ac = ActionClient(n, NavigateToPose, 'navigate_to_pose')
check('navigate_to_pose server up', ac.wait_for_server(timeout_sec=5))
g = NavigateToPose.Goal()
g.pose.header.frame_id = 'map'; g.pose.pose.position.x = -2.0; g.pose.pose.position.y = -1.5; g.pose.pose.orientation.w = 1.0
fut = ac.send_goal_async(g); rclpy.spin_until_future_complete(n, fut, timeout_sec=5)
res = fut.result().get_result_async(); rclpy.spin_until_future_complete(n, res, timeout_sec=20)
time.sleep(0.3)
for _ in range(20): rclpy.spin_once(n, timeout_sec=0.05)
try:
    tr = buf.lookup_transform('map', 'base_link', rclpy.time.Time()).transform.translation
    check('map->base_link at goal (%.2f,%.2f)' % (tr.x, tr.y), abs(tr.x + 2) < 0.1 and abs(tr.y + 1.5) < 0.1)
    buf.lookup_transform('map', 'head_camera_rgb_optical_frame', rclpy.time.Time()); check('map->camera optical TF', True)
    buf.lookup_transform('base_link', 'laser_link', rclpy.time.Time()); check('base_link->laser_link TF', True)
except Exception as e:
    check('TF lookups: %s' % e, False)
print('RESULT:', 'PASS' if ok else 'FAIL')
sys.exit(0 if ok else 1)
