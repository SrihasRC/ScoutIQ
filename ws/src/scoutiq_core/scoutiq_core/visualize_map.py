"""CLI utility to generate visual 2D map, trajectory overlay, and semantic object map."""

import argparse
import json
import os
import sys
from typing import Any, Dict, List, Optional
import cv2
import numpy as np
import yaml


def generate_map_visualizations(run_dir: str) -> None:
    """Reads map and generates map.png, map_trajectory.png, and map_semantic.png."""
    run_dir = os.path.abspath(run_dir)
    map_pgm = os.path.join(run_dir, "map.pgm")
    map_yaml = os.path.join(run_dir, "map.yaml")

    if not os.path.exists(map_pgm) or not os.path.exists(map_yaml):
        print(f"[visualize_map] Warning: {map_pgm} or {map_yaml} not found. Skipping visualization.")
        return

    map_img = cv2.imread(map_pgm)
    if map_img is None:
        print(f"[visualize_map] Error: Failed to read {map_pgm}")
        return

    with open(map_yaml, "r") as f:
        meta = yaml.safe_load(f)

    res = float(meta["resolution"])
    origin = meta["origin"]
    height, width = map_img.shape[:2]

    def world_to_pixel(x: float, y: float):
        px = int((x - origin[0]) / res)
        # ROS 2 map origin is bottom-left, OpenCV row 0 is top
        py = height - 1 - int((y - origin[1]) / res)
        return px, py

    # 1. Clean map.png
    out_map_png = os.path.join(run_dir, "map.png")
    cv2.imwrite(out_map_png, map_img)
    print(f"[visualize_map] Saved clean map image: {out_map_png}")

    # 2. Map with Trajectory
    traj_img = map_img.copy()
    surveillance_npz = os.path.join(run_dir, "surveillance_traj.npz")
    if os.path.exists(surveillance_npz):
        try:
            traj_data = np.load(surveillance_npz)["traj"]
            pts = [world_to_pixel(p[0], p[1]) for p in traj_data]

            # Draw patrol path
            for i in range(len(pts) - 1):
                cv2.line(traj_img, pts[i], pts[i + 1], (230, 100, 30), 2)

            # Draw unique waypoints
            seen_pts = set()
            wp_idx = 0
            for px, py in pts:
                if 0 <= px < width and 0 <= py < height and (px, py) not in seen_pts:
                    seen_pts.add((px, py))
                    cv2.circle(traj_img, (px, py), 4, (0, 0, 230), -1)
                    cv2.putText(
                        traj_img, str(wp_idx), (px + 4, py - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (30, 140, 30), 1
                    )
                    wp_idx += 1
            out_traj_png = os.path.join(run_dir, "map_trajectory.png")
            cv2.imwrite(out_traj_png, traj_img)
            print(f"[visualize_map] Saved map trajectory image: {out_traj_png}")
        except Exception as e:
            print(f"[visualize_map] Trajectory overlay skipped: {e}")

    # 3. Map with Semantic Object Nodes
    semantic_img = traj_img.copy()
    graph_path = os.path.join(run_dir, "graph_updated.json")
    if not os.path.exists(graph_path):
        graph_path = os.path.join(run_dir, "graph.json")

    if os.path.exists(graph_path):
        try:
            with open(graph_path, "r") as f:
                graph_data = json.load(f)

            color_map = {
                "table": (220, 50, 50),         # Blue in BGR
                "chair": (50, 180, 50),         # Green in BGR
                "door": (50, 50, 220),          # Red in BGR
                "bed": (200, 100, 220),         # Purple/Pink in BGR
                "sofa": (0, 165, 255),          # Orange in BGR
                "cabinet": (180, 180, 50),      # Cyan/Teal in BGR
                "refrigerator": (200, 200, 0),  # Light blue in BGR
            }
            default_color = (0, 200, 220)       # Yellow in BGR

            for node in graph_data.get("nodes", []):
                cat = node.get("category", "object")
                pose = node.get("pose", [0, 0, 0])
                px, py = world_to_pixel(pose[0], pose[1])
                color = color_map.get(cat, default_color)

                if 0 <= px < width and 0 <= py < height:
                    # Draw a distinct square marker for detected objects
                    cv2.rectangle(semantic_img, (px - 5, py - 5), (px + 5, py + 5), color, -1)
                    cv2.rectangle(semantic_img, (px - 5, py - 5), (px + 5, py + 5), (0, 0, 0), 1)
                    label = f"{cat}"
                    cv2.putText(
                        semantic_img, label, (px + 7, py + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1
                    )

            out_semantic_png = os.path.join(run_dir, "map_semantic.png")
            cv2.imwrite(out_semantic_png, semantic_img)
            print(f"[visualize_map] Saved map semantic image: {out_semantic_png}")
        except Exception as e:
            print(f"[visualize_map] Semantic overlay skipped: {e}")


def main():
    parser = argparse.ArgumentParser(description="Generate 2D map, trajectory, and semantic visual images.")
    parser.add_argument("run_dir", help="Path to run data directory (containing map.pgm, map.yaml, etc.)")
    args = parser.parse_args()
    generate_map_visualizations(args.run_dir)


if __name__ == "__main__":
    main()
