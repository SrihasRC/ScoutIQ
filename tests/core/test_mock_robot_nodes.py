"""Integration tests running roomwatch_core nodes against mock_robot (ROS_DOMAIN_ID=16)."""

import os
import subprocess
import sys
import time
import numpy as np
import pytest

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from visualization_msgs.msg import MarkerArray

from roomwatch_core.listener import ImageListener
from roomwatch_core.save_data import SaveData
from roomwatch_core.publish_traj import PosePublisher
from roomwatch_core.navigate import Navigate


MOCK_ROBOT_SCRIPT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "mock_robot", "mock_robot.py")
)


@pytest.fixture(scope="module")
def mock_robot_proc():
    """Starts mock_robot in a background process with ROS_DOMAIN_ID=16."""
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = "16"
    proc = subprocess.Popen(
        [sys.executable, MOCK_ROBOT_SCRIPT],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    time.sleep(2.5)  # Allow mock_robot nodes and TF to initialize
    yield proc
    proc.terminate()
    try:
        proc.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        proc.kill()


@pytest.fixture(autouse=True)
def setup_ros_domain():
    os.environ["ROS_DOMAIN_ID"] = "16"


def test_image_listener_with_mock(mock_robot_proc):
    """Verifies that ImageListener receives images, depth, and resolves TF poses from mock_robot."""
    if not rclpy.ok():
        rclpy.init()

    listener = ImageListener(node_name="test_img_listener", start_spin_thread=True)
    t0 = time.time()
    rt_cam, rt_base = None, None

    # Wait up to 8 seconds for synchronized images and TF transforms
    while time.time() - t0 < 8.0:
        rt_cam, rt_base = listener.get_data_to_save()
        if rt_cam is not None and rt_base is not None:
            break
        time.sleep(0.1)

    try:
        assert rt_cam is not None, "Failed to receive RT_camera from TF"
        assert rt_base is not None, "Failed to receive RT_base from TF"
        assert rt_cam.shape == (4, 4)
        assert rt_base.shape == (4, 4)
        assert listener.im is not None
        assert listener.depth is not None
        assert listener.depth.shape == (240, 320)
        assert listener.im.shape == (240, 320, 3)
    finally:
        listener.stop_spinning()
        listener.destroy_node()


def test_save_data_with_mock(mock_robot_proc, tmp_path):
    """Verifies that SaveData saves valid %06d_pose.npz files conforming to contract schema."""
    if not rclpy.ok():
        rclpy.init()

    run_dir = str(tmp_path / "test_run")
    saver = SaveData(time_interval=0.1, output_dir=run_dir, warmup_sec=2.0)

    try:
        t0 = time.time()
        saved_count = 0
        while time.time() - t0 < 6.0 and saved_count < 2:
            if saver.save_step(saved_count):
                saved_count += 1
            time.sleep(0.1)

        assert saved_count >= 2, f"Expected at least 2 saved poses, got {saved_count}"

        pose0 = os.path.join(run_dir, "pose", "000000_pose.npz")
        pose1 = os.path.join(run_dir, "pose", "000001_pose.npz")
        assert os.path.exists(pose0)
        assert os.path.exists(pose1)

        d0 = np.load(pose0)
        assert "RT_camera" in d0
        assert "RT_base" in d0
        assert d0["RT_camera"].shape == (4, 4)
        assert d0["RT_base"].shape == (4, 4)
    finally:
        saver.close()


def test_publish_traj_with_mock(mock_robot_proc):
    """Verifies that PosePublisher publishes MarkerArray on /visualization_marker_array."""
    if not rclpy.ok():
        rclpy.init()

    ref_traj = "/home/srihasrc/Music/AutoX-SemMap-main/scripts/surveillance_traj.npz"
    pub_node = PosePublisher(node_name="test_pub_traj", traj_file=ref_traj, publish_rate_hz=10.0)

    # Test subscriber node
    sub_node = Node("test_traj_sub")
    received_markers = []

    sub = sub_node.create_subscription(
        MarkerArray,
        "/visualization_marker_array",
        lambda m: received_markers.append(m),
        10,
    )

    t0 = time.time()
    try:
        while time.time() - t0 < 4.0 and not received_markers:
            pub_node.publish_poses()
            rclpy.spin_once(sub_node, timeout_sec=0.1)

        assert len(received_markers) > 0, "No MarkerArray received on /visualization_marker_array"
        last_array = received_markers[-1]
        assert len(last_array.markers) == 21
        assert last_array.markers[0].header.frame_id == "map"
        assert last_array.markers[0].ns == "poses"
    finally:
        pub_node.destroy_node()
        sub_node.destroy_node()


def test_navigate_with_mock(mock_robot_proc):
    """Verifies Navigate action client, /yes_no pause subscription, and movement against mock_robot."""
    if not rclpy.ok():
        rclpy.init()

    nav = Navigate(node_name="test_navigate", start_spin_thread=True)

    try:
        assert nav.wait_for_server(timeout_sec=5.0), "mock_robot navigate_to_pose action server not ready"

        # 1. Test /yes_no subscriber
        pause_pub_node = Node("test_pause_pub")
        pub = pause_pub_node.create_publisher(Int32, "/yes_no", 10)
        pub.publish(Int32(data=1))
        t0 = time.time()
        while time.time() - t0 < 2.0 and nav.pause != 1:
            time.sleep(0.05)
        assert nav.pause == 1, "Failed to receive pause state 1 on /yes_no"

        # Resume
        pub.publish(Int32(data=0))
        t0 = time.time()
        while time.time() - t0 < 2.0 and nav.pause != 0:
            time.sleep(0.05)
        assert nav.pause == 0, "Failed to receive pause state 0 on /yes_no"
        pause_pub_node.destroy_node()

        # 2. Test navigate_to
        target = [1.0, 1.0, 0.0]
        res = nav.navigate_to(target)
        assert res is not None

        # Verify base position reached target in mock_robot
        pos = nav.get_base_position(timeout_sec=3.0)
        assert abs(pos[0] - 1.0) < 0.15
        assert abs(pos[1] - 1.0) < 0.15
    finally:
        nav.close()
        nav.destroy_node()
