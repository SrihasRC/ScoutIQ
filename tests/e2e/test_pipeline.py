#!/usr/bin/env python3
"""
End-to-End Pipeline Integration Test.
Runs the complete RoomWatch pipeline in mock mode:
  1. Spawns mock robot (ROS 2 Humble topics & actions)
  2. Autonomous exploration + pose recording -> map.pgm, map.yaml, pose/%06d_pose.npz
  3. Trajectory extraction + TSP ordering -> robot_trajectory.json, surveillance_traj.npz
  4. Traverse & semantic map construction -> graph.json
  5. Traverse & semantic map update -> graph_updated.json
Asserts that all artifacts exist and strictly conform to docs/CONTRACT.md schema.
"""

import json
import os
import shutil
import subprocess
import sys
import tempfile
import time

import numpy as np


def run_pipeline_test():
    test_domain = "18"
    os.environ["ROS_DOMAIN_ID"] = test_domain
    os.environ["PYTHONNOUSERSITE"] = "1"
    print("=" * 65)
    print(f"RoomWatch E2E Pipeline Integration Test (ROS_DOMAIN_ID={test_domain})")
    print("=" * 65)

    test_dir = tempfile.mkdtemp(prefix="rw_e2e_")
    pose_dir = os.path.join(test_dir, "pose")
    os.makedirs(pose_dir, exist_ok=True)

    print(f"Working directory: {test_dir}")

    procs = []

    def cleanup():
        print("Cleaning up background processes...")
        for p in procs:
            try:
                p.terminate()
                p.wait(timeout=2.0)
            except Exception:
                try:
                    p.kill()
                except Exception:
                    pass
        # Kill lingering nodes
        subprocess.run(["pkill", "-f", "mock_robot.py"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-f", "async_slam_toolbox"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-f", "explore"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-f", "save_data"], stderr=subprocess.DEVNULL)

    try:
        # Phase 1: Start Mock Robot
        print("[1/5] Starting mock robot...")
        mock_proc = subprocess.Popen(
            [sys.executable, "tests/mock_robot/mock_robot.py"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        procs.append(mock_proc)
        time.sleep(3.0)
        assert mock_proc.poll() is None, "Mock robot crashed at startup"
        print("  [OK] Mock robot running.")

        # Phase 2: Autonomous Exploration & Pose Recording
        print("[2/5] Running exploration & pose recording...")
        explore_proc = subprocess.Popen(
            [
                "ros2", "run", "roomwatch_explore", "explore", "--ros-args",
                "-p", "costmap_topic:=map",
                "-p", "planner_frequency:=2.0",
                "-p", "progress_timeout:=4.0",
                "-p", "min_local_frontiers:=1.0",
                "-p", "min_global_frontiers:=1.0",
                "-p", "save_map:=true",
                "-p", f"run_dir:={test_dir}",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        procs.append(explore_proc)

        save_proc = subprocess.Popen(
            ["ros2", "run", "roomwatch_core", "save_data", "0.2", test_dir],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        procs.append(save_proc)

        # Wait for pose files and map to be saved
        map_yaml = os.path.join(test_dir, "map.yaml")
        map_pgm = os.path.join(test_dir, "map.pgm")
        start_wait = time.time()
        while time.time() - start_wait < 15.0:
            poses = [f for f in os.listdir(pose_dir) if f.endswith(".npz")]
            if os.path.exists(map_yaml) and len(poses) >= 5:
                break
            time.sleep(0.5)

        # Stop exploration and pose recording
        explore_proc.terminate()
        save_proc.terminate()
        time.sleep(1.0)

        # Verify Stage 1 artifacts
        assert os.path.exists(map_yaml), "map.yaml not generated"
        assert os.path.exists(map_pgm), "map.pgm not generated"
        poses = sorted([f for f in os.listdir(pose_dir) if f.endswith(".npz")])
        assert len(poses) >= 3, f"Expected >=3 poses, got {len(poses)}"

        # Validate pose npz schema
        sample_npz = np.load(os.path.join(pose_dir, poses[0]))
        assert "RT_camera" in sample_npz and "RT_base" in sample_npz, "Pose npz missing keys"
        assert sample_npz["RT_camera"].shape == (4, 4), "RT_camera shape != (4, 4)"
        assert sample_npz["RT_base"].shape == (4, 4), "RT_base shape != (4, 4)"
        print(f"  [OK] Saved {len(poses)} pose files and valid map ({os.path.getsize(map_pgm)} bytes).")

        # Phase 3: Trajectory Post-processing
        print("[3/5] Extracting trajectory & computing surveillance TSP path...")
        traj_json = os.path.join(test_dir, "robot_trajectory.json")
        surv_npz = os.path.join(test_dir, "surveillance_traj.npz")

        res1 = subprocess.run(
            [sys.executable, "-m", "roomwatch_core.extract_robot_trajectory", pose_dir, traj_json],
            capture_output=True, text=True, check=True
        )
        assert os.path.exists(traj_json), "robot_trajectory.json not created"
        with open(traj_json, "r") as f:
            t_data = json.load(f)
        assert len(t_data) > 0, "robot_trajectory.json is empty"

        res2 = subprocess.run(
            [sys.executable, "-m", "roomwatch_core.tsp_surveillance_trajectory", traj_json, surv_npz],
            capture_output=True, text=True, check=True
        )
        assert os.path.exists(surv_npz), "surveillance_traj.npz not created"
        with np.load(surv_npz) as f:
            assert "traj" in f, "surveillance_traj.npz missing 'traj' key"
            surv_pts = f["traj"]
            assert surv_pts.ndim == 2 and surv_pts.shape[1] >= 2, "Invalid traj array shape"
        print(f"  [OK] Extracted {len(t_data)} points; TSP generated {len(surv_pts)} waypoints.")

        # Phase 4: Semantic Map Construction
        print("[4/5] Running semantic map construction (rw-semantic-construct)...")
        graph_json = os.path.join(test_dir, "graph.json")
        construct_res = subprocess.run(
            [
                "rw-semantic-construct",
                "--output", graph_json,
                "--fake-detector",
                "--max-iterations", "4",
                "--rate-limit", "0.2",
            ],
            capture_output=True, text=True, check=True
        )
        assert os.path.exists(graph_json), "graph.json not created"
        with open(graph_json, "r") as f:
            g_data = json.load(f)
        assert "nodes" in g_data, "graph.json missing 'nodes'"
        assert len(g_data["nodes"]) > 0, "graph.json contains 0 nodes"
        print(f"  [OK] Constructed semantic graph with {len(g_data['nodes'])} nodes.")

        # Phase 5: Semantic Map Update
        print("[5/5] Running semantic map update (rw-semantic-update)...")
        graph_updated_json = os.path.join(test_dir, "graph_updated.json")
        update_res = subprocess.run(
            [
                "rw-semantic-update",
                "--input", graph_json,
                "--output", graph_updated_json,
                "--fake-detector",
                "--max-iterations", "4",
                "--rate-limit", "0.2",
            ],
            capture_output=True, text=True, check=True
        )
        assert os.path.exists(graph_updated_json), "graph_updated.json not created"
        with open(graph_updated_json, "r") as f:
            gu_data = json.load(f)
        assert "nodes" in gu_data, "graph_updated.json missing 'nodes'"
        assert len(gu_data["nodes"]) > 0, "graph_updated.json contains 0 nodes"
        print(f"  [OK] Updated semantic graph with {len(gu_data['nodes'])} nodes.")

        # Final Verification of All 7 Artifacts
        print("=" * 65)
        print("Final Artifact Checklist:")
        artifacts = [
            ("map.pgm", map_pgm),
            ("map.yaml", map_yaml),
            ("pose/", pose_dir),
            ("robot_trajectory.json", traj_json),
            ("surveillance_traj.npz", surv_npz),
            ("graph.json", graph_json),
            ("graph_updated.json", graph_updated_json),
        ]
        for name, path in artifacts:
            exists = os.path.exists(path)
            size = os.path.getsize(path) if os.path.isfile(path) else len(os.listdir(path))
            unit = "bytes" if os.path.isfile(path) else "files"
            print(f"  [PASS] {name:<26} ({size} {unit})")
            assert exists, f"Missing artifact: {name}"

        print("=" * 65)
        print("ALL END-TO-END PIPELINE TESTS PASSED SUCCESSFULLY!")
        print("=" * 65)
        return 0

    finally:
        cleanup()
        shutil.rmtree(test_dir, ignore_errors=True)


if __name__ == "__main__":
    sys.exit(run_pipeline_test())
