"""Unit tests for roomwatch_core.utils."""

import os
import numpy as np
import pytest
import networkx as nx

from roomwatch_core.utils import (
    compute_xyz,
    pose_to_map_pixel,
    pose_along_line,
    read_map_image,
    read_map_metadata,
    is_nearby,
    normalize_depth_image,
    denormalize_depth_image,
    pose_in_map_frame,
    is_nearby_in_map,
    save_graph_json,
    read_graph_json,
    pointcloud2_to_xyz_array,
    xyz_array_to_pointcloud2,
    laser_scan_to_xyz_array,
    fx, fy, px, py,
)
from sensor_msgs.msg import LaserScan


def test_map_metadata_reading():
    meta_path = "/home/srihasrc/Music/AutoX-SemMap-main/scripts/map.yaml"
    assert os.path.exists(meta_path)
    meta = read_map_metadata(meta_path)
    assert meta["resolution"] == 0.05
    assert meta["origin"] == [-100.0, -100.0, 0.0]
    assert meta["image"] == "map.pgm"


def test_pose_to_map_pixel():
    metadata = {
        "resolution": 0.05,
        "origin": [-100.0, -100.0, 0.0],
    }
    # At (0, 0), pixel should be (0 - (-100)) / 0.05 = 2000
    pixels = pose_to_map_pixel(metadata, [0.0, 0.0])
    assert pixels == [2000, 2000]

    # At (1.5, -2.5)
    pixels = pose_to_map_pixel(metadata, [1.5, -2.5])
    assert pixels == [int(101.5 / 0.05), int(97.5 / 0.05)]


def test_is_nearby():
    assert is_nearby([0.0, 0.0], [0.2, 0.2], threshold=0.5) is True
    assert is_nearby([0.0, 0.0], [1.0, 1.0], threshold=0.5) is False


def test_is_nearby_in_map():
    poses = [[0.0, 0.0, 0.0], [2.0, 2.0, 0.0]]
    # Close to first
    poses, nearby = is_nearby_in_map(poses, [0.1, 0.1, 0.0], threshold=0.5)
    assert nearby is True
    assert len(poses) == 2

    # Far away -> should append
    poses, nearby = is_nearby_in_map(poses, [5.0, 5.0, 0.0], threshold=0.5)
    assert nearby is False
    assert len(poses) == 3


def test_compute_xyz():
    H, W = 10, 10
    depth = np.ones((H, W), dtype=np.float32) * 2.0
    xyz = compute_xyz(depth, fx, fy, px, py, H, W)
    assert xyz.shape == (H, W, 3)
    # Z should be 2.0 everywhere
    assert np.allclose(xyz[..., 2], 2.0)

    # Test NaN handling
    depth[0, 0] = np.nan
    xyz = compute_xyz(depth, fx, fy, px, py, H, W)
    assert np.all(np.isfinite(xyz))
    assert xyz[0, 0, 2] == 0.0


def test_normalize_denormalize_depth():
    depth = np.array([[1.0, 2.0], [3.0, 4.0]], dtype=np.float32)
    max_d = 5.0
    norm_img = normalize_depth_image(depth, max_d)
    assert norm_img.dtype == np.uint8
    denorm = denormalize_depth_image(norm_img, max_d)
    # Precision within uint8 quant error (5.0 / 255 ~= 0.02)
    assert np.allclose(depth, denorm, atol=0.03)


def test_graph_json_roundtrip(tmp_path):
    g = nx.Graph()
    g.add_node(0, pose=[1.0, 2.0, 3.0], category="door")
    g.add_node(1, pose=[4.0, 5.0, 6.0], category="table")
    g.add_edge(0, 1, weight=5.2)

    target_file = str(tmp_path / "test_graph.json")
    save_graph_json(g, target_file)

    loaded_g = read_graph_json(target_file)
    assert len(loaded_g.nodes) == 2
    assert loaded_g.nodes[0]["category"] == "door"
    assert loaded_g.nodes[0]["pose"] == [1.0, 2.0, 3.0]
    assert loaded_g.nodes[1]["category"] == "table"
    assert loaded_g.has_edge(0, 1)
    assert np.isclose(loaded_g.edges[0, 1]["weight"], 5.2)


def test_pointcloud2_and_laser_conversions():
    pts = np.array([
        [1.0, 2.0, 3.0],
        [4.0, 5.0, 6.0],
        [-1.0, 0.0, 2.5],
    ], dtype=np.float32)

    msg = xyz_array_to_pointcloud2(pts, frame_id="laser_link")
    assert msg.header.frame_id == "laser_link"
    assert msg.width == 3

    recovered_pts = pointcloud2_to_xyz_array(msg)
    assert np.allclose(pts, recovered_pts)

    # Test laser scan conversion
    scan = LaserScan()
    scan.angle_min = -np.pi / 2.0
    scan.angle_max = np.pi / 2.0
    scan.angle_increment = np.pi / 2.0
    scan.range_min = 0.1
    scan.range_max = 10.0
    scan.ranges = [2.0, 3.0, 4.0]

    scan_pts = laser_scan_to_xyz_array(scan)
    assert len(scan_pts) == 3
    # Check angles: -pi/2 (y = -2), 0 (x = 3), +pi/2 (y = 4)
    assert np.isclose(scan_pts[0, 0], 0.0, atol=1e-5)
    assert np.isclose(scan_pts[0, 1], -2.0, atol=1e-5)
    assert np.isclose(scan_pts[1, 0], 3.0, atol=1e-5)
    assert np.isclose(scan_pts[1, 1], 0.0, atol=1e-5)
    assert np.isclose(scan_pts[2, 0], 0.0, atol=1e-5)
    assert np.isclose(scan_pts[2, 1], 4.0, atol=1e-5)
