"""Test perception utils, graph serialization, and geometry functions."""

import os
import networkx as nx
import numpy as np
from roomwatch_perception.utils import (
    compute_xyz,
    get_fov_points_in_map,
    is_nearby_in_map,
    pose_in_map_frame,
    read_graph_json,
    save_graph_json,
)


def test_compute_xyz():
    depth = np.ones((10, 10), dtype=np.float32) * 2.0
    xyz = compute_xyz(depth, fx=100.0, fy=100.0, px=5.0, py=5.0, height=10, width=10)
    assert xyz.shape == (10, 10, 3)
    assert np.allclose(xyz[..., 2], 2.0)


def test_graph_serialization(tmp_path):
    g = nx.Graph()
    g.add_node(
        "table_0_0",
        id="table_0_0",
        pose=[1.5, 2.0, 0.0],
        robot_pose=np.eye(4).tolist(),
        category="table",
    )
    g.add_node(
        "chair_0_0",
        id="chair_0_0",
        pose=[0.5, -1.0, 0.0],
        robot_pose=np.eye(4).tolist(),
        category="chair",
    )

    out_file = str(tmp_path / "test_graph.json")
    save_graph_json(g, out_file)
    assert os.path.exists(out_file)

    g_read = read_graph_json(out_file)
    assert len(g_read.nodes) == 2
    assert "table_0_0" in g_read.nodes
    assert g_read.nodes["table_0_0"]["category"] == "table"
    assert g_read.nodes["table_0_0"]["pose"] == [1.5, 2.0, 0.0]


def test_is_nearby():
    pose_list = [[0.0, 0.0, 0.0], [5.0, 5.0, 0.0]]
    updated_list, nearby = is_nearby_in_map(pose_list, [0.1, 0.1, 0.0], threshold=0.5)
    assert nearby is True

    updated_list, nearby = is_nearby_in_map(pose_list, [10.0, 10.0, 0.0], threshold=0.5)
    assert nearby is False
    assert len(updated_list) == 3


def test_pose_in_map_frame():
    RT_cam = np.eye(4)
    RT_base = np.eye(4)
    depth = np.ones((20, 20), dtype=np.float32) * 2.0
    pose = pose_in_map_frame(RT_cam, RT_base, depth, fx=100.0, fy=100.0, px=10.0, py=10.0)
    assert pose is not None
    assert len(pose) == 3
    assert abs(pose[2] - 2.0) < 1e-4
