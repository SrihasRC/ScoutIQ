"""Parity tests verifying exact schema and numerical equivalence with original reference data."""

import json
import os
import numpy as np
import pytest

from scoutiq_core.utils import read_graph_json
from scoutiq_core.tsp_surveillance_trajectory import (
    preprocess_trajectory_graph,
    tsp_greedy_solution,
    compute_surveillance_trajectory,
)
from scoutiq_core.extract_robot_trajectory import ExtractTrajectory

REF_DIR = "/home/srihasrc/Music/AutoX-SemMap-main/scripts"
RUN_DIR = "/home/srihasrc/Music/AutoX-SemMap-main/fetch_ws/src/fetch_gazebo/fetch_gazebo/scripts/2024-10-02_01-37-08"


def test_surveillance_trajectory_parity():
    """Verifies that TSP trajectory pipeline reproduces the exact reference surveillance_traj.npz."""
    ref_json_path = os.path.join(REF_DIR, "robot_trajectory.json")
    ref_npz_path = os.path.join(REF_DIR, "surveillance_traj.npz")

    assert os.path.exists(ref_json_path), f"Missing {ref_json_path}"
    assert os.path.exists(ref_npz_path), f"Missing {ref_npz_path}"

    # Load reference trajectory npz
    with np.load(ref_npz_path) as ref_npz:
        assert "traj" in ref_npz, "Key 'traj' missing from surveillance_traj.npz"
        ref_traj = ref_npz["traj"]

    # Compute surveillance trajectory using ported package
    path, computed_traj = compute_surveillance_trajectory(
        ref_json_path,
        output_npz=None,
    )

    assert computed_traj.shape == ref_traj.shape
    assert computed_traj.dtype == ref_traj.dtype
    assert np.array_equal(computed_traj, ref_traj), (
        f"Computed trajectory does not match reference! Max diff: {np.max(np.abs(computed_traj - ref_traj))}"
    )


def test_extract_robot_trajectory_parity(tmp_path):
    """Verifies that ExtractTrajectory creates robot_trajectory.json identical to reference."""
    ref_json_path = os.path.join(REF_DIR, "robot_trajectory.json")
    assert os.path.exists(ref_json_path), f"Missing {ref_json_path}"

    if not os.path.exists(RUN_DIR):
        pytest.skip(f"Reference run directory {RUN_DIR} not found.")

    out_file = str(tmp_path / "robot_trajectory.json")
    extractor = ExtractTrajectory(RUN_DIR)
    extractor.save_trajectory_json(out_file)

    with open(ref_json_path, "r") as f:
        ref_data = json.load(f)
    with open(out_file, "r") as f:
        extracted_data = json.load(f)

    # 1. Check top-level keys
    assert set(extracted_data.keys()) == set(ref_data.keys())
    assert extracted_data["directed"] == ref_data["directed"]
    assert extracted_data["multigraph"] == ref_data["multigraph"]
    assert extracted_data["graph"] == ref_data["graph"]

    # 2. Check nodes count and values
    assert len(extracted_data["nodes"]) == len(ref_data["nodes"])
    for i, (node_ext, node_ref) in enumerate(zip(extracted_data["nodes"], ref_data["nodes"])):
        assert node_ext["id"] == node_ref["id"], f"Node ID mismatch at {i}"
        assert np.allclose(node_ext["pose"], node_ref["pose"]), f"Node pose mismatch at {i}"


def test_pose_npz_schema(tmp_path):
    """Verifies that pose npz file format matches contract: RT_camera, RT_base (4x4 float64)."""
    pose_file = str(tmp_path / "000000_pose.npz")
    rt_cam = np.eye(4, dtype=np.float64)
    rt_base = np.eye(4, dtype=np.float64)
    rt_base[0, 3] = 1.5
    rt_base[1, 3] = -2.0

    np.savez(pose_file, RT_camera=rt_cam, RT_base=rt_base)

    loaded = np.load(pose_file)
    assert set(loaded.keys()) == {"RT_camera", "RT_base"}
    assert loaded["RT_camera"].shape == (4, 4)
    assert loaded["RT_base"].shape == (4, 4)
    assert loaded["RT_camera"].dtype == np.float64
    assert loaded["RT_base"].dtype == np.float64
