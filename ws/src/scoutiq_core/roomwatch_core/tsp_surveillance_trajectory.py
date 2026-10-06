"""Greedy TSP solution to compute pruned surveillance patrol trajectory from robot trajectory."""

import os
import sys
from typing import Any, List, Optional, Tuple, Union
import cv2
import numpy as np
from numpy.linalg import norm
import networkx as nx

try:
    from scoutiq_core.utils import (
        read_graph_json,
        read_map_image,
        read_map_metadata,
        pose_to_map_pixel,
        display_map_image,
    )
except ImportError:
    from utils import (
        read_graph_json,
        read_map_image,
        read_map_metadata,
        pose_to_map_pixel,
        display_map_image,
    )

MIN_SEPARATION_TSP = 1.5


def tsp_greedy_solution(G: nx.Graph, start_node: Optional[Any] = None) -> List[Any]:
    """Computes a greedy Traveling Salesperson Problem tour on graph G starting from start_node."""
    nodes = list(G.nodes)
    if not nodes:
        return []
    if len(nodes) == 1:
        return [nodes[0], nodes[0]]

    if start_node is None or start_node not in G:
        start_node = sorted(nodes)[0]

    path = [start_node]
    visited = set(path)

    while len(path) < len(nodes):
        last = path[-1]
        next_node = min(
            (n for n in nodes if n not in visited),
            key=lambda n: G[last][n]["weight"],
        )
        path.append(next_node)
        visited.add(next_node)

    # Safely retrace through traversed waypoints back to start
    # so the robot smoothly returns along the corridor without cutting across walls/sofas:
    if len(path) > 2:
        return path + path[-2::-1]
    elif len(path) == 2:
        return path + [path[0]]
    return path


def preprocess_trajectory_graph(
    trajectory_graph: nx.Graph,
    min_separation: float = MIN_SEPARATION_TSP,
) -> nx.Graph:
    """Prunes nearby nodes closer than min_separation and adds complete edge weights."""
    deleted_nodes = []
    trajectory_graph_ = trajectory_graph.copy()

    for id, (node, data) in enumerate(trajectory_graph.nodes(data=True)):
        if node in deleted_nodes:
            continue
        for id_, (node_, data_) in enumerate(trajectory_graph.nodes(data=True)):
            if node_ in deleted_nodes:
                continue

            if node == node_ or (id_ < id):
                continue
            pose1 = data["pose"]
            pose2 = data_["pose"]
            weight = float(norm((pose1[0] - pose2[0], pose1[1] - pose2[1])))
            if weight < min_separation:
                trajectory_graph_.remove_node(node_)
                deleted_nodes.append(node_)

    # Add edges between remaining nodes
    for id, (node, data) in enumerate(trajectory_graph_.nodes(data=True)):
        for id_, (node_, data_) in enumerate(trajectory_graph_.nodes(data=True)):
            if id < id_:
                pose1 = data["pose"]
                pose2 = data_["pose"]
                weight = float(norm((pose1[0] - pose2[0], pose1[1] - pose2[1])))
                trajectory_graph_.add_edge(node, node_, weight=weight)

    return trajectory_graph_


def filter_poses_by_map_clearance(
    trajectory_graph: nx.Graph,
    map_yaml_path: str,
    min_clearance: float = 0.45,
) -> nx.Graph:
    """Filters out trajectory nodes too close to obstacles or walls."""
    if not os.path.exists(map_yaml_path):
        return trajectory_graph

    try:
        import yaml
        with open(map_yaml_path, "r") as f:
            meta = yaml.safe_load(f)

        map_dir = os.path.dirname(os.path.abspath(map_yaml_path))
        pgm_path = os.path.join(map_dir, meta.get("image", "map.pgm"))
        if not os.path.exists(pgm_path):
            return trajectory_graph

        map_img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
        if map_img is None:
            return trajectory_graph

        res = meta["resolution"]
        origin = meta["origin"]

        # Free space binary mask: >250 is free space in map_saver (254)
        free_mask = (map_img > 250).astype(np.uint8)
        dist_map = cv2.distanceTransform(free_mask, cv2.DIST_L2, 5) * res

        filtered_graph = nx.Graph()
        for node, data in trajectory_graph.nodes(data=True):
            pose = data.get("pose", [0.0, 0.0, 0.0])
            wx, wy = pose[0], pose[1]
            mx = int((wx - origin[0]) / res)
            my = map_img.shape[0] - 1 - int((wy - origin[1]) / res)

            if 0 <= my < map_img.shape[0] and 0 <= mx < map_img.shape[1]:
                clearance = dist_map[my, mx]
                if clearance >= min_clearance:
                    filtered_graph.add_node(node, **data)

        if len(filtered_graph.nodes) >= 3:
            return filtered_graph
    except Exception as e:
        print(f"[tsp] Map clearance filtering note: {e}")

    return trajectory_graph


def compute_surveillance_trajectory(
    trajectory_source: Union[str, nx.Graph],
    output_npz: Optional[str] = "surveillance_traj.npz",
    min_separation: float = MIN_SEPARATION_TSP,
) -> Tuple[List[Any], np.ndarray]:
    """End-to-end pipeline to compute and save surveillance trajectory.

    :param trajectory_source: Path to robot_trajectory.json or loaded NetworkX Graph
    :param output_npz: Output npz filename (or None to skip saving)
    :param min_separation: Minimum distance between patrol waypoints
    :return: (surveillance_path, surveillance_traj_array)
    """
    if isinstance(trajectory_source, str):
        robot_trajectory = read_graph_json(trajectory_source)
        candidate_yaml = os.path.join(os.path.dirname(os.path.abspath(trajectory_source)), "map.yaml")
        if os.path.exists(candidate_yaml):
            robot_trajectory = filter_poses_by_map_clearance(robot_trajectory, candidate_yaml, min_clearance=0.45)
    else:
        robot_trajectory = trajectory_source

    pruned_trajectory = preprocess_trajectory_graph(robot_trajectory, min_separation=min_separation)

    # Determine best start node closest to the robot's final position at the end of exploration
    best_start = None
    if len(robot_trajectory.nodes) > 0 and len(pruned_trajectory.nodes) > 0:
        last_node = max(robot_trajectory.nodes)
        last_pose = robot_trajectory.nodes[last_node]["pose"]
        best_start = min(
            pruned_trajectory.nodes,
            key=lambda n: float(norm(
                (pruned_trajectory.nodes[n]["pose"][0] - last_pose[0],
                 pruned_trajectory.nodes[n]["pose"][1] - last_pose[1])
            ))
        )

    surveillance_path = tsp_greedy_solution(pruned_trajectory, start_node=best_start)

    surveillance_traj = []
    for node in surveillance_path:
        pose = pruned_trajectory.nodes[node]["pose"]
        surveillance_traj.append(pose)

    traj_array = np.asarray(surveillance_traj, dtype=np.float64)

    if output_npz is not None:
        np.savez(output_npz, traj=traj_array)

    return surveillance_path, traj_array


def main():
    if len(sys.argv) < 2:
        print("Usage: tsp_surveillance_trajectory <robot_trajectory.json> [output_npz] [--visualize]")
        sys.exit(1)

    json_path = sys.argv[1]
    out_npz = "surveillance_traj.npz"
    visualize = False

    for arg in sys.argv[2:]:
        if arg == "--visualize":
            visualize = True
        elif not arg.startswith("--"):
            out_npz = arg

    surveillance_path, traj_arr = compute_surveillance_trajectory(
        json_path,
        output_npz=out_npz,
    )
    print(f"Computed surveillance trajectory with {len(traj_arr)} waypoints -> saved to {out_npz}")

    if visualize:
        try:
            map_image = read_map_image("map.pgm")
            map_metadata = read_map_metadata("map.yaml")
            for node_idx, pose in enumerate(traj_arr):
                x, y = pose_to_map_pixel(map_metadata, pose)
                cv2.circle(map_image, (x, y), 5, (0, 0, 255), -1)
            display_map_image(map_image, write=True, save_path="surveillance_path.png")
        except Exception as e:
            print(f"Visualization skipped: {e}")


if __name__ == "__main__":
    main()
