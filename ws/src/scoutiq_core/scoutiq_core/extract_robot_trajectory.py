"""Extracts robot trajectory from recorded pose NPZ files into a NetworkX graph JSON."""

import os
from os.path import join
import sys
from typing import Optional
import numpy as np
import networkx as nx

try:
    from scoutiq_core.utils import save_graph_json
except ImportError:
    from utils import save_graph_json


class ExtractTrajectory:
    """Reads recorded pose NPZ files and builds a NetworkX trajectory graph."""

    def __init__(self, root_dir: str) -> None:
        self.root_dir = root_dir
        if os.path.basename(os.path.normpath(root_dir)) == "pose" and os.path.isdir(root_dir):
            self.pose_dir = root_dir
        elif os.path.isdir(join(root_dir, "pose")):
            self.pose_dir = join(root_dir, "pose")
        else:
            self.pose_dir = root_dir

        if not os.path.exists(self.pose_dir):
            raise FileNotFoundError(f"Pose directory does not exist: {self.pose_dir}")

        self.pose_files = sorted([f for f in os.listdir(self.pose_dir) if f.endswith(".npz")])
        self.trajectory_graph = nx.Graph()

    def build_graph(self) -> nx.Graph:
        """Populates and returns the trajectory graph."""
        self.trajectory_graph.clear()
        for node_id, pose_data_file in enumerate(self.pose_files):
            file_path = join(self.pose_dir, pose_data_file)
            pose_data = np.load(file_path)
            pose_xyz = pose_data["RT_base"][:3, 3].tolist()
            self.trajectory_graph.add_node(
                node_id,
                pose=pose_xyz,
            )
        return self.trajectory_graph

    def save_trajectory_json(self, file_name: str = "robot_trajectory.json") -> None:
        """Builds graph and saves to node-link JSON."""
        self.build_graph()
        save_graph_json(self.trajectory_graph, file=file_name)


def main():
    if len(sys.argv) < 2:
        print("Usage: extract_robot_trajectory <root_dir> [output_file.json]")
        sys.exit(1)

    root_dir = sys.argv[1]
    out_file = sys.argv[2] if len(sys.argv) > 2 else "robot_trajectory.json"

    extractor = ExtractTrajectory(root_dir)
    extractor.save_trajectory_json(file_name=out_file)
    print(f"Extracted {len(extractor.pose_files)} poses to {out_file}")


if __name__ == "__main__":
    main()
