"""Integration test: test perception construct and update nodes against mock_robot."""

import json
import os
import subprocess
import sys
import time

import networkx as nx
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray

os.environ["ROS_DOMAIN_ID"] = "17"


def test_mock_robot_perception_pipeline(tmp_path):
    graph_construct_path = str(tmp_path / "graph.json")
    graph_update_path = str(tmp_path / "graph_updated.json")

    # 1. Start mock robot
    mock_robot_cmd = [
        sys.executable,
        "tests/mock_robot/mock_robot.py"
    ]
    mock_proc = subprocess.Popen(mock_robot_cmd)
    time.sleep(2.0)  # Wait for mock robot to spin up

    try:
        # 2. Test monitor node to verify /seg_image and /graph_nodes topics
        rclpy.init()
        monitor = Node("pipeline_monitor")
        received = {}

        def on_seg_image(msg):
            received["seg_image"] = msg

        def on_markers(msg):
            received["markers"] = msg

        monitor.create_subscription(Image, "/seg_image", on_seg_image, qos_profile_sensor_data)
        monitor.create_subscription(
            MarkerArray,
            "/graph_nodes",
            on_markers,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        # 3. Run rw-semantic-construct
        construct_cmd = [
            sys.executable,
            "-m", "roomwatch_perception.semantic_map_construction",
            "--output", graph_construct_path,
            "--fake-detector",
            "--rate-limit", "0.5",
            "--max-iterations", "3",
        ]
        construct_proc = subprocess.Popen(construct_cmd)

        # Spin monitor while construct runs
        t0 = time.time()
        while time.time() - t0 < 15.0 and construct_proc.poll() is None:
            rclpy.spin_once(monitor, timeout_sec=0.1)

        construct_proc.wait(timeout=5.0)
        assert construct_proc.returncode == 0, "rw-semantic-construct failed"

        # Assert topics were published
        assert "seg_image" in received, "Did not receive /seg_image"
        assert "markers" in received, "Did not receive /graph_nodes"
        assert received["seg_image"].encoding == "rgb8"
        assert len(received["markers"].markers) > 0

        # Assert graph.json exists and has nodes
        assert os.path.exists(graph_construct_path), f"File {graph_construct_path} was not created"
        with open(graph_construct_path, "r") as f:
            data = json.load(f)
        assert len(data["nodes"]) > 0, "No nodes constructed in graph.json"

        categories = {node["category"] for node in data["nodes"]}
        assert any(cat in categories for cat in ["table", "chair", "door"]), (
            f"Unexpected categories: {categories}"
        )
        for node in data["nodes"]:
            assert "pose" in node
            assert len(node["pose"]) == 3
            assert "robot_pose" in node

        print(f"[ok] rw-semantic-construct created {len(data['nodes'])} nodes: {categories}")

        # 4. Run rw-semantic-update
        received.clear()
        update_cmd = [
            sys.executable,
            "-m", "roomwatch_perception.semantic_map_update",
            "--input", graph_construct_path,
            "--output", graph_update_path,
            "--fake-detector",
            "--rate-limit", "0.5",
            "--max-iterations", "3",
        ]
        update_proc = subprocess.Popen(update_cmd)

        t0 = time.time()
        while time.time() - t0 < 15.0 and update_proc.poll() is None:
            rclpy.spin_once(monitor, timeout_sec=0.1)

        update_proc.wait(timeout=5.0)
        assert update_proc.returncode == 0, "rw-semantic-update failed"

        assert os.path.exists(graph_update_path), f"File {graph_update_path} was not created"
        with open(graph_update_path, "r") as f:
            update_data = json.load(f)
        assert len(update_data["nodes"]) > 0, "No nodes in updated graph"

        monitor.destroy_node()
        rclpy.shutdown()

    finally:
        mock_proc.terminate()
        mock_proc.wait()


if __name__ == "__main__":
    import tempfile
    from pathlib import Path
    with tempfile.TemporaryDirectory() as td:
        test_mock_robot_perception_pipeline(Path(td))
    print("ALL INTEGRATION TESTS PASSED!")
