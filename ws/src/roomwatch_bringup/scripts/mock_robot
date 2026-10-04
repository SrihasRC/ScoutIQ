#!/usr/bin/env python3
"""Mock robot for simulator-free development (owned by the main agent).

Fakes everything in docs/CONTRACT.md that downstream nodes consume:
  TF  map->odom (identity), odom->base_link (integrated), static base_link->laser_link / head camera frames
  /odom, /scan (ray-cast in a synthetic 10x8 m room with an inner pillar), /map (OccupancyGrid of the room),
  /head_camera/rgb/image_raw (synthetic colored blocks), /head_camera/depth_registered/image_raw (32FC1 m),
  /head_camera/rgb/camera_info, /joint_states, /cmd_vel subscriber, and a `navigate_to_pose` action server
  (straight-line "teleport-drive" at 1 m/s; no obstacle avoidance).
Run:  python tests/mock_robot/mock_robot.py   (source /opt/ros/humble/setup.bash first; system or venv python)
"""
import math
import numpy as np
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry, OccupancyGrid
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan, Image, CameraInfo, JointState
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

ROOM = (-5.0, 5.0, -4.0, 4.0)          # xmin, xmax, ymin, ymax
PILLAR = (1.0, 2.0, 0.5, 1.5)          # inner box obstacle
W, H, FX = 320, 240, 277.0


def yaw_to_q(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))


def q_to_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def ray_box(ox, oy, dx, dy, box, inside):
    """Distance along ray to a box; `inside` = ray starts inside box (hit the walls from within)."""
    xmin, xmax, ymin, ymax = box
    best = math.inf
    for (px, nx_, lo, hi, axis) in ((xmin, 1, ymin, ymax, 0), (xmax, 1, ymin, ymax, 0),
                                    (ymin, 1, xmin, xmax, 1), (ymax, 1, xmin, xmax, 1)):
        d = (dx if axis == 0 else dy)
        if abs(d) < 1e-9:
            continue
        o = ox if axis == 0 else oy
        t = (px - o) / d
        if t <= 1e-6:
            continue
        other = (oy + t * dy) if axis == 0 else (ox + t * dx)
        if lo - 1e-9 <= other <= hi + 1e-9:
            best = min(best, t)
    return best


class MockRobot(Node):
    def __init__(self):
        super().__init__('mock_robot')
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.v = self.w = 0.0
        self.tfb = TransformBroadcaster(self)
        self.stf = StaticTransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.rgb_pub = self.create_publisher(Image, '/head_camera/rgb/image_raw', qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(Image, '/head_camera/depth_registered/image_raw', qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, '/head_camera/rgb/camera_info', qos_profile_sensor_data)
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.publish_static()
        self.publish_map()
        self.create_timer(0.05, self.step)          # 20 Hz physics/TF/odom
        self.create_timer(0.1, self.publish_scan)   # 10 Hz
        self.create_timer(0.2, self.publish_camera) # 5 Hz
        ActionServer(self, NavigateToPose, 'navigate_to_pose', self.on_goal)
        self.get_logger().info('mock_robot up (room %s)' % (ROOM,))

    # ---- inputs
    def on_cmd(self, m):
        self.v, self.w = m.linear.x, m.angular.z

    def on_goal(self, gh):
        g = gh.request.pose.pose
        tx, ty = g.position.x, g.position.y
        tyaw = q_to_yaw(g.orientation)
        self.v = self.w = 0.0
        while rclpy.ok():
            dx, dy = tx - self.x, ty - self.y
            d = math.hypot(dx, dy)
            if d < 0.05:
                break
            s = min(1.0 * 0.05, d)
            self.x += s * dx / d
            self.y += s * dy / d
            self.yaw = math.atan2(dy, dx)
            self.sleep(0.05)
        self.yaw = tyaw
        gh.succeed()
        return NavigateToPose.Result()

    def sleep(self, t):
        import time
        time.sleep(t)

    # ---- publishers
    def stamp(self):
        return self.get_clock().now().to_msg()

    def tf(self, parent, child, x=0., y=0., z=0., q=None):
        t = TransformStamped()
        t.header.stamp = self.stamp()
        t.header.frame_id, t.child_frame_id = parent, child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, z
        t.transform.rotation = q or Quaternion(w=1.0)
        return t

    def publish_static(self):
        # optical frame: x right, y down, z forward  (rotation -90 about z then -90 about x)
        opt = Quaternion(x=-0.5, y=0.5, z=-0.5, w=0.5)
        self.stf.sendTransform([
            self.tf('map', 'odom'),
            self.tf('base_link', 'laser_link', 0.235, 0.0, 0.29),
            self.tf('base_link', 'head_camera_link', 0.12, 0.0, 1.1),
            self.tf('head_camera_link', 'head_camera_rgb_optical_frame', q=opt),
            self.tf('head_camera_link', 'head_camera_depth_optical_frame', q=opt),
        ])

    def publish_map(self):
        res = 0.05
        gw, gh = int(10 / res) + 40, int(8 / res) + 40
        ox, oy = ROOM[0] - 1.0, ROOM[2] - 1.0
        grid = np.full((gh, gw), -1, np.int8)
        xs = ox + (np.arange(gw) + 0.5) * res
        ys = oy + (np.arange(gh) + 0.5) * res
        X, Y = np.meshgrid(xs, ys)
        inside = (X > ROOM[0]) & (X < ROOM[1]) & (Y > ROOM[2]) & (Y < ROOM[3])
        grid[inside] = 0
        wall = ((np.abs(X - ROOM[0]) < res) | (np.abs(X - ROOM[1]) < res) | (np.abs(Y - ROOM[2]) < res) | (np.abs(Y - ROOM[3]) < res)) & (X >= ROOM[0] - res) & (X <= ROOM[1] + res) & (Y >= ROOM[2] - res) & (Y <= ROOM[3] + res)
        grid[wall] = 100
        grid[(X > PILLAR[0]) & (X < PILLAR[1]) & (Y > PILLAR[2]) & (Y < PILLAR[3])] = 100
        m = OccupancyGrid()
        m.header.frame_id = 'map'
        m.header.stamp = self.stamp()
        m.info.resolution, m.info.width, m.info.height = res, gw, gh
        m.info.origin.position.x, m.info.origin.position.y = ox, oy
        m.info.origin.orientation.w = 1.0
        m.data = grid.flatten().tolist()
        self.map_pub.publish(m)

    def step(self):
        dt = 0.05
        if self.v or self.w:
            self.yaw += self.w * dt
            self.x += self.v * math.cos(self.yaw) * dt
            self.y += self.v * math.sin(self.yaw) * dt
        self.x = min(max(self.x, ROOM[0] + 0.3), ROOM[1] - 0.3)
        self.y = min(max(self.y, ROOM[2] + 0.3), ROOM[3] - 0.3)
        q = yaw_to_q(self.yaw)
        self.tfb.sendTransform(self.tf('odom', 'base_link', self.x, self.y, 0.0, q))
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = self.stamp(), 'odom', 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation = q
        o.twist.twist.linear.x, o.twist.twist.angular.z = self.v, self.w
        self.odom_pub.publish(o)
        js = JointState()
        js.header.stamp = self.stamp()
        js.name = ['torso_lift_joint', 'head_pan_joint', 'head_tilt_joint']
        js.position = [0.0, 0.0, 0.0]
        self.js_pub.publish(js)

    def cast(self, ang_world):
        """Distance from laser origin along world angle; hits room walls (from inside) and pillar."""
        lx = self.x + 0.235 * math.cos(self.yaw)
        ly = self.y + 0.235 * math.sin(self.yaw)
        dx, dy = math.cos(ang_world), math.sin(ang_world)
        t = ray_box(lx, ly, dx, dy, ROOM, True)
        t = min(t, ray_box(lx, ly, dx, dy, PILLAR, False)) if not (PILLAR[0] < lx < PILLAR[1] and PILLAR[2] < ly < PILLAR[3]) else t
        return t

    def publish_scan(self):
        n = 360
        s = LaserScan()
        s.header.stamp, s.header.frame_id = self.stamp(), 'laser_link'
        s.angle_min, s.angle_max = -math.pi, math.pi
        s.angle_increment = 2 * math.pi / n
        s.range_min, s.range_max = 0.05, 25.0
        s.ranges = [min(self.cast(self.yaw + s.angle_min + i * s.angle_increment), s.range_max) for i in range(n)]
        self.scan_pub.publish(s)

    def publish_camera(self):
        st = self.stamp()
        # synthetic scene: floor-gray image with 3 colored blocks; depth = constant 2 m, blocks 1.5 m
        rgb = np.full((H, W, 3), 120, np.uint8)
        depth = np.full((H, W), 2.0, np.float32)
        for (u0, u1, col) in ((40, 100, (200, 40, 40)), (130, 190, (40, 200, 40)), (220, 280, (40, 40, 200))):
            rgb[90:200, u0:u1] = col
            depth[90:200, u0:u1] = 1.5
        im = Image()
        im.header.stamp, im.header.frame_id = st, 'head_camera_rgb_optical_frame'
        im.height, im.width, im.encoding, im.step = H, W, 'rgb8', W * 3
        im.data = rgb.tobytes()
        self.rgb_pub.publish(im)
        dm = Image()
        dm.header.stamp, dm.header.frame_id = st, 'head_camera_depth_optical_frame'
        dm.height, dm.width, dm.encoding, dm.step = H, W, '32FC1', W * 4
        dm.data = depth.tobytes()
        self.depth_pub.publish(dm)
        ci = CameraInfo()
        ci.header.stamp, ci.header.frame_id = st, 'head_camera_rgb_optical_frame'
        ci.width, ci.height = W, H
        ci.k = [FX, 0., W / 2, 0., FX, H / 2, 0., 0., 1.]
        ci.p = [FX, 0., W / 2, 0., 0., FX, H / 2, 0., 0., 0., 1., 0.]
        ci.distortion_model = 'plumb_bob'
        ci.d = [0.0] * 5
        self.info_pub.publish(ci)


def main():
    rclpy.init()
    n = MockRobot()
    from rclpy.executors import MultiThreadedExecutor
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
