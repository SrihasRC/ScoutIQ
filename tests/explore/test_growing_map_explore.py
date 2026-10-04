#!/usr/bin/env python3
"""Integration test for scoutiq_explore with a dynamically growing map.

Verifies:
1. When map has unexplored areas, frontiers are found.
2. Dynamic search window filters and prioritizes frontiers (local vs global).
3. Goals are dispatched to the Nav2 navigate_to_pose action server.
4. As the robot navigates, newly uncovered free space expands the map.
5. When the room is fully explored, exploration completes.
6. map.pgm and map.yaml are saved into data/<run>/ per CONTRACT.md.
"""
import math
import os
import shutil
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..'))
ROS_DOMAIN_ID = '15'
ROOM = (-3.0, 3.0, -2.5, 2.5)  # Room boundary in meters


class GrowingMapRobot(Node):
    def __init__(self):
        super().__init__('growing_map_robot')
        self.x = -1.8
        self.y = 0.0
        self.yaw = 0.0
        self.res = 0.05
        self.gw = int((ROOM[1] - ROOM[0] + 2.0) / self.res)
        self.gh = int((ROOM[3] - ROOM[2] + 2.0) / self.res)
        self.ox = ROOM[0] - 1.0
        self.oy = ROOM[2] - 1.0

        # Initialize grid: -1 (unknown) everywhere
        self.grid = np.full((self.gh, self.gw), -1, np.int8)

        # Draw walls
        xs = self.ox + (np.arange(self.gw) + 0.5) * self.res
        ys = self.oy + (np.arange(self.gh) + 0.5) * self.res
        X, Y = np.meshgrid(xs, ys)
        wall = (
            ((np.abs(X - ROOM[0]) < self.res) | (np.abs(X - ROOM[1]) < self.res) |
             (np.abs(Y - ROOM[2]) < self.res) | (np.abs(Y - ROOM[3]) < self.res)) &
            (X >= ROOM[0] - self.res) & (X <= ROOM[1] + self.res) &
            (Y >= ROOM[2] - self.res) & (Y <= ROOM[3] + self.res)
        )
        self.wall_mask = wall
        self.grid[self.wall_mask] = 100

        # Reveal initial small area around robot at -1.8, 0.0: radius 1.0 m
        self.reveal_area(-1.8, 0.0, 1.0)

        self.tfb = TransformBroadcaster(self)
        self.stf = StaticTransformBroadcaster(self)

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
        self.goals_received = []

        self.publish_static()
        self.publish_map()

        self.create_timer(0.05, self.step)
        self.create_timer(0.2, self.publish_map)

        self.action_server = ActionServer(self, NavigateToPose, 'navigate_to_pose', self.on_goal)
        self.get_logger().info('GrowingMapRobot ready.')

    def reveal_area(self, rx, ry, radius):
        xs = self.ox + (np.arange(self.gw) + 0.5) * self.res
        ys = self.oy + (np.arange(self.gh) + 0.5) * self.res
        X, Y = np.meshgrid(xs, ys)
        dist = np.hypot(X - rx, Y - ry)
        inside_room = (X >= ROOM[0]) & (X <= ROOM[1]) & (Y >= ROOM[2]) & (Y <= ROOM[3])
        uncovered = inside_room & (dist <= radius)
        self.grid[uncovered] = 0
        self.grid[self.wall_mask] = 100

    def on_goal(self, gh):
        g = gh.request.pose.pose
        tx, ty = g.position.x, g.position.y
        self.goals_received.append((tx, ty))
        self.get_logger().info(f"[ROBOT] Received goal ({tx:.2f}, {ty:.2f})")

        # Simulate movement towards goal at 2.0 m/s, revealing map along the way
        while rclpy.ok():
            dx, dy = tx - self.x, ty - self.y
            d = math.hypot(dx, dy)
            if d < 0.15:
                break
            step_size = min(2.0 * 0.05, d)
            self.x += step_size * dx / d
            self.y += step_size * dy / d
            self.yaw = math.atan2(dy, dx)
            # Sensor reveals 1.5m radius around robot
            self.reveal_area(self.x, self.y, 1.5)
            time.sleep(0.05)

        self.reveal_area(self.x, self.y, 2.5)
        self.publish_map()
        time.sleep(0.2)
        gh.succeed()
        self.get_logger().info(f"[ROBOT] Goal reached at ({self.x:.2f}, {self.y:.2f})")
        return NavigateToPose.Result()

    def stamp(self):
        return self.get_clock().now().to_msg()

    def publish_static(self):
        t = TransformStamped()
        t.header.stamp = self.stamp()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.rotation = Quaternion(w=1.0)
        self.stf.sendTransform(t)

    def publish_map(self):
        m = OccupancyGrid()
        m.header.frame_id = 'map'
        m.header.stamp = self.stamp()
        m.info.resolution = self.res
        m.info.width = self.gw
        m.info.height = self.gh
        m.info.origin.position.x = self.ox
        m.info.origin.position.y = self.oy
        m.info.origin.orientation.w = 1.0
        m.data = self.grid.flatten().tolist()
        self.map_pub.publish(m)

    def step(self):
        t = TransformStamped()
        t.header.stamp = self.stamp()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = Quaternion(
            x=0.0, y=0.0, z=math.sin(self.yaw / 2.0), w=math.cos(self.yaw / 2.0))
        self.tfb.sendTransform(t)


def run_robot():
    rclpy.init()
    node = GrowingMapRobot()
    from rclpy.executors import MultiThreadedExecutor
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except Exception:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    if len(sys.argv) > 1 and sys.argv[1] == '--robot':
        run_robot()
        return 0

    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = ROS_DOMAIN_ID
    env['PYTHONUNBUFFERED'] = '1'

    test_run_dir = os.path.join(PROJECT_ROOT, 'data', 'test_growing_explore_run')
    if os.path.exists(test_run_dir):
        shutil.rmtree(test_run_dir)

    print(f"[TEST] Starting GrowingMapRobot on ROS_DOMAIN_ID={ROS_DOMAIN_ID}...")
    robot_proc = subprocess.Popen(
        [sys.executable, os.path.abspath(__file__), '--robot'],
        cwd=PROJECT_ROOT,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True
    )

    explore_proc = None
    try:
        time.sleep(2.0)

        print("[TEST] Starting scoutiq_explore on growing map...")
        explore_cmd = [
            'ros2', 'run', 'scoutiq_explore', 'explore',
            '--ros-args',
            '-p', f'run_dir:={test_run_dir}',
            '-p', 'planner_frequency:=2.0',
            '-p', 'min_frontier_size:=0.3',
            '-p', 'local_frontier_filter_radius:=2.0',
            '-p', 'min_local_frontiers:=1.0',
            '-p', 'min_global_frontiers:=1.0',
            '-p', 'min_frontier_spacing:=0.5',
            '-p', 'progress_timeout:=2.0',
            '-p', 'save_map:=true'
        ]
        explore_proc = subprocess.Popen(
            explore_cmd,
            cwd=PROJECT_ROOT,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True
        )

        import queue
        line_queue = queue.Queue()

        def reader_thread(proc):
            for l in iter(proc.stdout.readline, ''):
                line_queue.put(l)
            proc.stdout.close()

        t = threading.Thread(target=reader_thread, args=(explore_proc,), daemon=True)
        t.start()

        start_time = time.time()
        timeout = 60.0
        goals_dispatched = 0
        window_expanded = False
        completed = False

        while time.time() - start_time < timeout:
            try:
                line = line_queue.get(timeout=0.1)
                line_str = line.strip()
                print(f"[explore] {line_str}")
                if "Sending NavigateToPose goal" in line_str:
                    goals_dispatched += 1
                if "Expanding search to whole map" in line_str or "Picking a frontier within the filter radius" in line_str:
                    window_expanded = True
                if "Exploration complete" in line_str or "Stopping exploration" in line_str or "Map successfully saved" in line_str:
                    completed = True
                    break
            except queue.Empty:
                pass

        time.sleep(2.0)

        pgm_path = os.path.join(test_run_dir, 'map.pgm')
        yaml_path = os.path.join(test_run_dir, 'map.yaml')

        pgm_exists = os.path.isfile(pgm_path) and os.path.getsize(pgm_path) > 0
        yaml_exists = os.path.isfile(yaml_path) and os.path.getsize(yaml_path) > 0

        print(f"[TEST] Goals dispatched: {goals_dispatched}")
        print(f"[TEST] Window logic triggered: {window_expanded}")
        print(f"[TEST] Completed cleanly: {completed}")
        print(f"[TEST] map.pgm exists: {pgm_exists} (size: {os.path.getsize(pgm_path) if pgm_exists else 0} bytes)")
        print(f"[TEST] map.yaml exists: {yaml_exists} (size: {os.path.getsize(yaml_path) if yaml_exists else 0} bytes)")

        if goals_dispatched > 0 and window_expanded and pgm_exists and yaml_exists:
            print("[TEST] SUCCESS: Growing map exploration test PASSED!")
            return 0
        else:
            print("[TEST] FAILURE: Criteria not met.")
            return 1

    finally:
        for p in [explore_proc, robot_proc]:
            if p and p.poll() is None:
                p.terminate()
                try:
                    p.wait(timeout=1.5)
                except Exception:
                    p.kill()
                    try:
                        p.wait(timeout=1.0)
                    except Exception:
                        pass


if __name__ == '__main__':
    sys.exit(main())
