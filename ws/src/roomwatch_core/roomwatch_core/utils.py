"""Map, pointcloud, geometric and graph utilities for roomwatch."""

import json
import os
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union

import cv2
import numpy as np
from numpy.linalg import norm
import yaml
import networkx as nx
from networkx.readwrite import json_graph

try:
    from sensor_msgs.msg import PointCloud2, PointField, LaserScan
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header
    HAS_ROS2_MSGS = True
except ImportError:
    HAS_ROS2_MSGS = False

# Default Fetch RGB-D camera intrinsics
intrinsics = [
    [574.0527954101562, 0.0, 319.5],
    [0.0, 574.0527954101562, 239.5],
    [0.0, 0.0, 1.0],
]
fx = intrinsics[0][0]
fy = intrinsics[1][1]
px = intrinsics[0][2]
py = intrinsics[1][2]


def compute_xyz(
    depth_img: np.ndarray,
    fx_val: float,
    fy_val: float,
    px_val: float,
    py_val: float,
    height: int,
    width: int,
) -> np.ndarray:
    """Computes 3D coordinates (X, Y, Z) in the camera frame for each pixel in a depth image.

    :param depth_img: 2D depth image (H, W) in meters
    :param fx_val: Focal length along x
    :param fy_val: Focal length along y
    :param px_val: Principal point x
    :param py_val: Principal point y
    :param height: Image height
    :param width: Image width
    :return: 3D array of shape [H, W, 3]
    """
    clean_depth = np.nan_to_num(depth_img, nan=0.0, posinf=0.0, neginf=0.0)
    indices = np.indices((height, width), dtype=np.float32).transpose(1, 2, 0)
    z_e = clean_depth
    x_e = (indices[..., 1] - px_val) * z_e / fx_val
    y_e = (indices[..., 0] - py_val) * z_e / fy_val
    xyz_img = np.stack([x_e, y_e, z_e], axis=-1)
    return xyz_img


def pose_to_map_pixel(map_metadata: Dict[str, Any], pose: Sequence[float]) -> List[int]:
    """Converts world coordinate (x, y) to map grid pixel coordinates.

    :param map_metadata: Map metadata dictionary (origin, resolution)
    :param pose: [x, y, ...] in map frame
    :return: [map_pixel_x, map_pixel_y]
    """
    pose_x = float(pose[0])
    pose_y = float(pose[1])
    origin_x = float(map_metadata["origin"][0])
    origin_y = float(map_metadata["origin"][1])
    resolution = float(map_metadata["resolution"])

    map_pixel_x = int((pose_x - origin_x) / resolution)
    map_pixel_y = int((pose_y - origin_y) / resolution)

    return [map_pixel_x, map_pixel_y]


def pose_along_line(pose1: np.ndarray, pose2: np.ndarray, distance: float = 2.0) -> np.ndarray:
    """Creates a new pose that is at the specified distance from pose1 along the line towards pose2.

    :param pose1: Starting position (3,) or transformation matrix (4, 4)
    :param pose2: Target position (3,) or transformation matrix (4, 4)
    :param distance: Distance to extend along line
    :return: 3D position vector
    """
    p1 = pose1[:3, 3] if (isinstance(pose1, np.ndarray) and pose1.shape == (4, 4)) else np.asarray(pose1[:3], dtype=np.float64)
    p2 = pose2[:3, 3] if (isinstance(pose2, np.ndarray) and pose2.shape == (4, 4)) else np.asarray(pose2[:3], dtype=np.float64)
    diff = p2 - p1
    d_norm = norm(diff)
    if d_norm < 1e-9:
        return p1.copy()
    unit_vec = diff / d_norm
    return p1 + unit_vec * distance


def read_map_image(map_file_path: str) -> np.ndarray:
    """Reads map image (.pgm or .png)."""
    if not os.path.exists(map_file_path):
        raise FileNotFoundError(f"Map image file does not exist: {map_file_path}")
    map_image = cv2.imread(map_file_path)
    if map_image is None:
        raise ValueError(f"Failed to read image from {map_file_path}")
    return map_image


def read_map_metadata(metadata_file_path: str) -> Dict[str, Any]:
    """Reads map metadata YAML file."""
    if not os.path.exists(metadata_file_path):
        raise FileNotFoundError(f"Map metadata file does not exist: {metadata_file_path}")
    with open(metadata_file_path, "r") as f:
        metadata = yaml.safe_load(f)
    return metadata


def display_map_image(map_image: np.ndarray, write: bool = False, key: int = 0, save_path: str = "map_image.png") -> None:
    """Displays map image or saves to disk safely in headless environments."""
    if write:
        cv2.imwrite(save_path, map_image)
    if "DISPLAY" in os.environ and os.environ["DISPLAY"]:
        try:
            width, height = map_image.shape[1], map_image.shape[0]
            cv2.namedWindow("Map Image", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Map Image", width, height)
            cv2.imshow("Map Image", map_image)
            cv2.waitKey(key)
        except Exception:
            pass


def is_nearby(pose1: Sequence[float], pose2: Sequence[float], threshold: float = 0.5) -> bool:
    """Checks if two 2D poses are within threshold distance."""
    return bool(norm((float(pose1[0]) - float(pose2[0]), float(pose1[1]) - float(pose2[1]))) < threshold)


def normalize_depth_image(depth_array: np.ndarray, max_depth: float) -> np.ndarray:
    """Normalizes floating point depth array to uint8 image (0-255)."""
    clean_depth = np.nan_to_num(depth_array, nan=0.0, posinf=max_depth, neginf=0.0)
    clean_depth = np.clip(clean_depth, 0.0, max_depth)
    depth_image = (max_depth - clean_depth) / max_depth * 255.0
    return depth_image.astype(np.uint8)


def denormalize_depth_image(depth_image: np.ndarray, max_depth: float) -> np.ndarray:
    """Denormalizes uint8 depth image back to floating point depth in meters."""
    depth_array = max_depth * (1.0 - (depth_image.astype(np.float32) / 255.0))
    return depth_array.astype(np.float32)


def get_fov_points_in_baselink(
    depth_array: np.ndarray,
    RT_camera: np.ndarray,
    fx_val: float = fx,
    fy_val: float = fy,
    px_val: float = px,
    py_val: float = py,
) -> List[List[float]]:
    """Calculates field of view footprint points in base_link coordinates."""
    xyz_array = compute_xyz(
        depth_array, fx_val, fy_val, px_val, py_val, depth_array.shape[0], depth_array.shape[1]
    )
    xyz_array = xyz_array.reshape((-1, 3))

    mask = ~(np.all(xyz_array == [0.0, 0.0, 0.0], axis=1))
    xyz_array = xyz_array[mask]
    if len(xyz_array) == 0:
        return [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]

    xyz_base = np.dot(RT_camera[:3, :3], xyz_array.T).T
    xyz_base += RT_camera[:3, 3]

    min_x = float(np.min(xyz_base[:, 0]))
    max_x = float(np.max(xyz_base[:, 0]))
    min_y = float(np.min(xyz_base[:, 1]))
    max_y = float(np.max(xyz_base[:, 1]))

    return [[0.0, 0.0, 0.0], [max_x, min_y, 0.0], [max_x, max_y, 0.0]]


def pose_in_map_frame(
    RT_camera: np.ndarray,
    RT_base: np.ndarray,
    depth_array: np.ndarray,
    segment: Optional[np.ndarray] = None,
    fx_val: float = fx,
    fy_val: float = fy,
    px_val: float = px,
    py_val: float = py,
) -> Optional[List[float]]:
    """Transforms segmented depth points from camera frame into map frame and returns the mean centroid."""
    clean_depth = np.nan_to_num(depth_array, nan=0.0, posinf=0.0, neginf=0.0).copy()
    if segment is not None:
        clean_depth = clean_depth * (segment / 255.0)

    if clean_depth.max() <= 0.0:
        return None

    xyz_array = compute_xyz(
        clean_depth, fx_val, fy_val, px_val, py_val, clean_depth.shape[0], clean_depth.shape[1]
    )
    xyz_array = xyz_array.reshape((-1, 3))

    mask = ~(np.all(xyz_array == [0.0, 0.0, 0.0], axis=1))
    xyz_array = xyz_array[mask]
    if len(xyz_array) == 0:
        return None

    xyz_base = np.dot(RT_camera[:3, :3], xyz_array.T).T
    xyz_base += RT_camera[:3, 3]

    xyz_map = np.dot(RT_base[:3, :3], xyz_base.T).T
    xyz_map += RT_base[:3, 3]

    mean_pose = np.mean(xyz_map, axis=0)
    return mean_pose.tolist()


def is_nearby_in_map(
    pose_list: List[Sequence[float]],
    node_pose: Sequence[float],
    threshold: float = 0.5,
) -> Tuple[List[Sequence[float]], bool]:
    """Checks whether node_pose is within threshold of any pose in pose_list."""
    if len(pose_list) == 0:
        pose_list.append(node_pose)
        return pose_list, False

    pose_array = np.array(pose_list)
    node_pose_array = np.array([node_pose])
    distances = np.linalg.norm(pose_array[:, 0:2] - node_pose_array[:, 0:2], axis=1)
    if np.any(distances < threshold):
        return pose_list, True
    else:
        pose_list.append(node_pose)
        return pose_list, False


def save_graph_json(graph: nx.Graph, file: str = "graph.json") -> None:
    """Saves a NetworkX graph to a JSON file conforming to node-link schema."""
    try:
        data_to_save = json_graph.node_link_data(graph, edges="links")
    except TypeError:
        data_to_save = json_graph.node_link_data(graph)
    with open(file, "w") as f:
        json.dump(data_to_save, f, indent=4)


def read_graph_json(file: str = "graph.json") -> nx.Graph:
    """Reads a NetworkX graph from a node-link formatted JSON file."""
    with open(file, "r") as f:
        data = json.load(f)
    try:
        return json_graph.node_link_graph(data, edges="links")
    except TypeError:
        return json_graph.node_link_graph(data)


def read_and_visualize_graph(
    map_file_path: str,
    map_metadata_filepath: str,
    on_map: bool = False,
    categories: Optional[List[str]] = None,
    graph: Optional[nx.Graph] = None,
) -> None:
    """Visualizes semantic graph nodes either as a graph plot or overlaid on the map image."""
    if graph is None:
        graph = read_graph_json()
    if categories is None:
        categories = []

    color_palette = [[255, 0, 0], [0, 255, 0], [0, 0, 255]]
    if not on_map:
        visualize_graph(graph)
    else:
        map_image = read_map_image(map_file_path)
        map_metadata = read_map_metadata(map_metadata_filepath)
        for node, data in graph.nodes(data=True):
            cat = data.get("category", "")
            if cat in categories:
                x, y = pose_to_map_pixel(map_metadata, data["pose"])
                color_idx = categories.index(cat) % len(color_palette)
                color = color_palette[color_idx]
                cv2.rectangle(
                    map_image,
                    (x - 3, y - 3),
                    (x + 3, y + 3),
                    color,
                    -1,
                )
        display_map_image(map_image, write=True)


def plot_point_on_map(
    map_file_path: str,
    map_metadata_filepath: str,
    position: Sequence[float],
) -> None:
    """Draws a point on the map image at the specified position."""
    map_image = read_map_image(map_file_path)
    map_metadata = read_map_metadata(map_metadata_filepath)
    x, y = pose_to_map_pixel(map_metadata, position)
    cv2.circle(map_image, (x, y), 3, (0, 0, 255), -1)
    display_map_image(map_image, write=False)


def visualize_graph(graph: nx.Graph) -> None:
    """Plots the graph structure using NetworkX and matplotlib (if display available)."""
    if "DISPLAY" not in os.environ or not os.environ["DISPLAY"]:
        return
    try:
        import matplotlib.pyplot as plt
        pos = nx.spring_layout(graph)
        nx.draw(graph, pos, with_labels=True)
        plt.show()
    except Exception:
        pass


# ---------------------------------------------------------
# Point Cloud & Laser Scan Utilities (ROS 2)
# ---------------------------------------------------------

def pointcloud2_to_xyz_array(cloud_msg: Any) -> np.ndarray:
    """Converts a sensor_msgs/PointCloud2 message into an (N, 3) float32 numpy array.

    :param cloud_msg: sensor_msgs.msg.PointCloud2
    :return: (N, 3) numpy array
    """
    if not HAS_ROS2_MSGS:
        raise RuntimeError("ROS 2 sensor_msgs_py is not available.")
    points = point_cloud2.read_points_numpy(cloud_msg, field_names=["x", "y", "z"])
    return points.astype(np.float32)


def xyz_array_to_pointcloud2(
    xyz_array: np.ndarray,
    frame_id: str = "base_link",
    stamp: Any = None,
) -> Any:
    """Converts an (N, 3) numpy array into a sensor_msgs/PointCloud2 message.

    :param xyz_array: (N, 3) array of point coordinates
    :param frame_id: Frame ID header
    :param stamp: Header stamp
    :return: sensor_msgs.msg.PointCloud2
    """
    if not HAS_ROS2_MSGS:
        raise RuntimeError("ROS 2 sensor_msgs_py is not available.")
    header = Header()
    header.frame_id = frame_id
    if stamp is not None:
        header.stamp = stamp
    xyz = np.asarray(xyz_array, dtype=np.float32).reshape((-1, 3))
    return point_cloud2.create_cloud_xyz32(header, xyz)


def laser_scan_to_xyz_array(scan_msg: Any) -> np.ndarray:
    """Converts a 2D sensor_msgs/LaserScan message to 3D Cartesian points (x, y, 0) in scan frame.

    :param scan_msg: sensor_msgs.msg.LaserScan
    :return: (N, 3) float32 numpy array
    """
    ranges = np.array(scan_msg.ranges, dtype=np.float32)
    angles = scan_msg.angle_min + np.arange(len(ranges)) * scan_msg.angle_increment
    valid = (ranges >= scan_msg.range_min) & (ranges <= scan_msg.range_max) & np.isfinite(ranges)
    r = ranges[valid]
    th = angles[valid]
    x = r * np.cos(th)
    y = r * np.sin(th)
    z = np.zeros_like(x)
    return np.column_stack([x, y, z]).astype(np.float32)
