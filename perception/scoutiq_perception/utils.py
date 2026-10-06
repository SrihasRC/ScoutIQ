"""Perception and graph utilities combining RoboKit and spatial mapping functions."""

import json
import logging
import os
import random
import warnings
from typing import List, Optional, Sequence, Tuple, Union

import cv2
import matplotlib.pyplot as plt
import networkx as nx
from networkx.readwrite import json_graph
import numpy as np
from numpy.linalg import norm
from PIL import Image as PILImg, ImageDraw
import supervision as sv
import torch
import yaml

DEFAULT_INTRINSICS = [
    [574.0527954101562, 0.0, 319.5],
    [0.0, 574.0527954101562, 239.5],
    [0.0, 0.0, 1.0],
]
DEFAULT_FX = DEFAULT_INTRINSICS[0][0]
DEFAULT_FY = DEFAULT_INTRINSICS[1][1]
DEFAULT_PX = DEFAULT_INTRINSICS[0][2]
DEFAULT_PY = DEFAULT_INTRINSICS[1][2]

# Backwards compatibility globals
fx = DEFAULT_FX
fy = DEFAULT_FY
px = DEFAULT_PX
py = DEFAULT_PY
intrinsics = DEFAULT_INTRINSICS


# ---------------------------------------------------------
# 3D Geometry and Spatial Utils
# ---------------------------------------------------------

def compute_xyz(
    depth_img: np.ndarray,
    fx: float,
    fy: float,
    px: float,
    py: float,
    height: int,
    width: int
) -> np.ndarray:
    """Computes 3D point coordinates in the camera optical frame from a depth image."""
    indices = np.indices((height, width), dtype=np.float32).transpose(1, 2, 0)
    z_e = np.where(np.isfinite(depth_img), depth_img, 0.0).astype(np.float32)
    z_e = np.clip(z_e, 0.0, 10.0)
    x_e = (indices[..., 1] - px) * z_e / fx
    y_e = (indices[..., 0] - py) * z_e / fy
    xyz_img = np.stack([x_e, y_e, z_e], axis=-1)  # Shape: [H x W x 3]
    return xyz_img


def pose_to_map_pixel(map_metadata: dict, pose: Union[List[float], np.ndarray]) -> List[int]:
    """Converts a 2D map coordinate (x, y) into image pixel coordinates."""
    pose_x = pose[0]
    pose_y = pose[1]
    map_pixel_x = int((pose_x - map_metadata["origin"][0]) / map_metadata["resolution"])
    map_pixel_y = int((pose_y - map_metadata["origin"][1]) / map_metadata["resolution"])
    return [map_pixel_x, map_pixel_y]


def pose_along_line(pose1: np.ndarray, pose2: np.ndarray, distance: float = 2.0) -> np.ndarray:
    """Creates a new pose at the specified distance from pose1 along line from pose1 to pose2."""
    p2 = pose2[0:3, 3] if pose2.ndim == 2 else pose2[:3]
    p1 = pose1[0:3, 3] if pose1.ndim == 2 else pose1[:3]
    diff = p2 - p1
    d = norm(diff)
    if d < 1e-6:
        return p1
    unit_vector = diff / d
    return p1 + unit_vector * distance


def read_map_image(map_file_path: str) -> np.ndarray:
    assert os.path.exists(map_file_path), f"Map file not found: {map_file_path}"
    return cv2.imread(map_file_path)


def read_map_metadata(metadata_file_path: str) -> dict:
    assert os.path.exists(metadata_file_path), f"Map metadata not found: {metadata_file_path}"
    with open(metadata_file_path, "r") as file:
        return yaml.safe_load(file)


def display_map_image(map_image: np.ndarray, write: bool = False, output_file: str = "map_image.png"):
    width, height, _ = map_image.shape
    if write:
        cv2.imwrite(output_file, map_image)


def is_nearby(pose1: Sequence[float], pose2: Sequence[float], threshold: float = 0.5) -> bool:
    return norm((pose1[0] - pose2[0], pose1[1] - pose2[1])) < threshold


def normalize_depth_image(depth_array: np.ndarray, max_depth: float) -> np.ndarray:
    depth_image = (max_depth - depth_array) / max_depth
    depth_image = depth_image * 255.0
    return np.clip(depth_image, 0, 255).astype(np.uint8)


def denormalize_depth_image(depth_image: np.ndarray, max_depth: float) -> np.ndarray:
    depth_array = max_depth * (1.0 - (depth_image / 255.0))
    return depth_array.astype(np.float32)


def get_fov_points_in_baselink(
    depth_array: np.ndarray,
    RT_camera: np.ndarray,
    fx: float = DEFAULT_FX,
    fy: float = DEFAULT_FY,
    px: float = DEFAULT_PX,
    py: float = DEFAULT_PY
) -> List[List[float]]:
    valid = np.isfinite(depth_array) & (depth_array >= 0.35) & (depth_array <= 4.0)
    if np.count_nonzero(valid) < 50:
        return [[0.0, 0.0, 0.0], [1.0, -1.0, 0.0], [1.0, 1.0, 0.0]]

    indices = np.argwhere(valid)
    z_valid = depth_array[valid]
    x_valid = (indices[:, 1] - px) * z_valid / fx
    y_valid = (indices[:, 0] - py) * z_valid / fy
    xyz_array = np.stack([x_valid, y_valid, z_valid], axis=-1)

    xyz_base = np.dot(RT_camera[:3, :3], xyz_array.T).T + RT_camera[:3, 3]
    xyz_base = xyz_base[np.all(np.isfinite(xyz_base), axis=1)]
    if len(xyz_base) == 0:
        return [[0.0, 0.0, 0.0], [1.0, -1.0, 0.0], [1.0, 1.0, 0.0]]

    min_x = float(np.min(xyz_base[:, 0]))
    max_x = float(np.max(xyz_base[:, 0]))
    min_y = float(np.min(xyz_base[:, 1]))
    max_y = float(np.max(xyz_base[:, 1]))

    return [[0.0, 0.0, 0.0], [max_x, min_y, 0.0], [max_x, max_y, 0.0]]


def get_fov_points_in_map(
    depth_array: np.ndarray,
    RT_camera: np.ndarray,
    RT_base: Optional[np.ndarray] = None,
    fx: float = DEFAULT_FX,
    fy: float = DEFAULT_FY,
    px: float = DEFAULT_PX,
    py: float = DEFAULT_PY
) -> List[List[float]]:
    points_baselink = get_fov_points_in_baselink(depth_array, RT_camera, fx, fy, px, py)
    if RT_base is not None:
        points_map = np.dot(RT_base[:3, :3], np.array(points_baselink).T).T + RT_base[:3, 3]
    else:
        points_map = np.array(points_baselink)
    points_clean = []
    for pt in points_map.tolist():
        if all(np.isfinite(v) for v in pt):
            points_clean.append([float(pt[0]), float(pt[1]), float(pt[2])])
        else:
            points_clean.append([0.0, 0.0, 0.0])
    return points_clean


def pose_in_map_frame(
    RT_camera: np.ndarray,
    RT_base: Optional[np.ndarray],
    depth_array: np.ndarray,
    segment: Optional[np.ndarray] = None,
    fx: float = DEFAULT_FX,
    fy: float = DEFAULT_FY,
    px: float = DEFAULT_PX,
    py: float = DEFAULT_PY
) -> Optional[List[float]]:
    """Calculates object centroid in map frame using camera pose and depth image."""
    mask = (segment > 0) if segment is not None else np.ones(depth_array.shape[:2], dtype=bool)

    # Strict valid depth bounds to avoid runtime warnings and polygon crashes
    valid = np.isfinite(depth_array) & (depth_array >= 0.35) & (depth_array <= 4.0) & (mask > 0)
    if np.count_nonzero(valid) < 50:
        return None

    indices = np.argwhere(valid)
    z_valid = depth_array[valid]
    x_valid = (indices[:, 1] - px) * z_valid / fx
    y_valid = (indices[:, 0] - py) * z_valid / fy
    points_3d = np.stack([x_valid, y_valid, z_valid], axis=-1)

    if len(points_3d) == 0:
        return None

    if RT_base is not None:
        # Optical camera frame -> base_link
        xyz_base = np.dot(RT_camera[:3, :3], points_3d.T).T + RT_camera[:3, 3]
        # base_link -> map
        xyz_map = np.dot(RT_base[:3, :3], xyz_base.T).T + RT_base[:3, 3]
    else:
        # Direct optical camera frame -> map
        xyz_map = np.dot(RT_camera[:3, :3], points_3d.T).T + RT_camera[:3, 3]

    mean_pose = np.mean(xyz_map, axis=0)
    if np.any(~np.isfinite(mean_pose)):
        return None
    return mean_pose.tolist()


def is_nearby_in_map(
    pose_list: List[List[float]],
    node_pose: List[float],
    threshold: float = 0.5
) -> Tuple[List[List[float]], bool]:
    """Checks if node_pose is close to any pose in pose_list."""
    if len(pose_list) == 0:
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
    """Saves a NetworkX graph to a JSON file matching original node-link structure."""
    try:
        data_to_save = json_graph.node_link_data(graph, edges="links")
    except TypeError:
        data_to_save = json_graph.node_link_data(graph)

    os.makedirs(os.path.dirname(os.path.abspath(file)), exist_ok=True)
    with open(file, "w") as f:
        json.dump(data_to_save, f, indent=4)


def read_graph_json(file: str = "graph.json") -> nx.Graph:
    """Reads a NetworkX graph from a JSON file."""
    if not os.path.exists(file):
        logging.warning("Graph file %s does not exist, initializing empty Graph", file)
        return nx.Graph()
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
    catgeories: Optional[List[str]] = None,
    graph: Optional[nx.Graph] = None
) -> None:
    if graph is None:
        graph = read_graph_json()
    if catgeories is None:
        catgeories = ["table", "chair", "door"]

    color_palette = [[255, 0, 0], [0, 255, 0], [0, 0, 255]]
    if not on_map:
        pos = nx.spring_layout(graph)
        nx.draw(graph, pos, with_labels=True)
        plt.show()
    else:
        map_image = read_map_image(map_file_path)
        map_metadata = read_map_metadata(map_metadata_filepath)
        for node, data in graph.nodes(data=True):
            cat = data.get("category", "")
            if cat in catgeories:
                x, y = pose_to_map_pixel(map_metadata, data["pose"])
                color = color_palette[catgeories.index(cat) % len(color_palette)]
                map_image[max(0, y - 5) : y + 5, max(0, x - 5) : x + 5, :] = color
        display_map_image(map_image, write=True)


def plot_point_on_map(map_file_path: str, map_metadata_filepath: str, position: List[float]) -> None:
    map_image = read_map_image(map_file_path)
    map_metadata = read_map_metadata(map_metadata_filepath)
    x, y = pose_to_map_pixel(map_metadata, position)
    map_image[max(0, y - 5) : y + 5, max(0, x - 5) : x + 5, :] = [0, 0, 255]
    display_map_image(map_image, write=False)


def visualize_graph(graph: nx.Graph) -> None:
    pos = nx.spring_layout(graph)
    nx.draw(graph, pos, with_labels=True)
    plt.show()


# ---------------------------------------------------------
# Image Annotation and Filtering Utils
# ---------------------------------------------------------

def apply_matplotlib_colormap(depth_pil: PILImg.Image, colormap_name: str = 'inferno') -> PILImg.Image:
    depth_array = np.array(depth_pil, dtype=np.float32)
    denom = depth_array.max() - depth_array.min()
    if denom == 0:
        denom = 1.0
    depth_normalized = (depth_array - depth_array.min()) / denom
    cmap = plt.get_cmap(colormap_name)
    colored_depth = (cmap(depth_normalized) * 255).astype(np.uint8)
    return PILImg.fromarray(colored_depth)


def file_exists(file_path: str) -> bool:
    return os.path.exists(file_path)


def crop_images(original_image: PILImg.Image, bounding_boxes: List[List[int]]) -> List[PILImg.Image]:
    cropped_images = []
    for box in bounding_boxes:
        if len(box) != 4:
            continue
        x_min, y_min, x_max, y_max = box
        if x_min < 0 or y_min < 0 or x_max <= x_min or y_max <= y_min:
            continue
        cropped_image = original_image.crop((x_min, y_min, x_max, y_max))
        cropped_images.append(cropped_image)
    return cropped_images


def annotate(
    image_source: PILImg.Image,
    boxes: Union[torch.Tensor, np.ndarray],
    logits: Union[torch.Tensor, List[float], np.ndarray],
    phrases: List[str]
) -> PILImg.Image:
    """Annotates image with bounding boxes, confidence scores, and phrases."""
    if len(boxes) == 0:
        return image_source.copy()

    if isinstance(boxes, torch.Tensor):
        boxes_np = boxes.detach().cpu().numpy()
    else:
        boxes_np = np.array(boxes)

    if isinstance(logits, torch.Tensor):
        logits_list = logits.detach().cpu().tolist()
    else:
        logits_list = list(logits)

    detections = sv.Detections(
        xyxy=boxes_np,
        class_id=np.zeros(len(boxes_np), dtype=int)
    )

    labels = [
        f"{phrase} {logit:.2f}"
        for phrase, logit in zip(phrases, logits_list)
    ]
    box_annotator = sv.BoxAnnotator()

    scene = np.array(image_source.convert("RGB"))
    annotated_scene = box_annotator.annotate(scene=scene, detections=detections)

    for box, label in zip(detections.xyxy, labels):
        x1, y1, x2, y2 = box.astype(int)
        cv2.putText(
            annotated_scene,
            label,
            (x1, max(12, y1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            2
        )

    return PILImg.fromarray(annotated_scene)


def draw_mask(mask: np.ndarray, overlay_np: np.ndarray, color: Optional[Tuple[int, int, int, int]] = None) -> None:
    if color is None:
        color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255), 153)
    overlay_np[mask.astype(bool)] = color


def overlay_masks(image_pil: PILImg.Image, masks: Union[torch.Tensor, List[torch.Tensor]]) -> PILImg.Image:
    w, h = image_pil.size
    overlay_np = np.zeros((h, w, 4), dtype=np.uint8)
    for mask in masks:
        if isinstance(mask, torch.Tensor):
            m_np = mask[0].detach().cpu().numpy()
        else:
            m_np = mask[0]
        color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255), 153)
        draw_mask(m_np, overlay_np, color=color)

    mask_image = PILImg.fromarray(overlay_np, mode="RGBA")
    img_rgba = image_pil.convert("RGBA")
    img_rgba.alpha_composite(mask_image)
    return img_rgba.convert("RGB")


def combine_masks(gt_masks: torch.Tensor) -> torch.Tensor:
    gt_masks = torch.flip(gt_masks, dims=(0,))
    num, h, w = gt_masks.shape
    bin_mask = torch.zeros((h, w), device=gt_masks.device)
    num_instance = len(gt_masks)
    if num_instance == 0:
        return bin_mask

    for m, object_label in zip(gt_masks, range(1, 1 + num_instance)):
        label_pos = torch.nonzero(m, as_tuple=True)
        bin_mask[label_pos] = object_label
    return bin_mask


def filter_large_boxes(
    boxes: torch.Tensor,
    w: int,
    h: int,
    threshold: float = 0.5
) -> Tuple[torch.Tensor, torch.Tensor]:
    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 2]
    y2 = boxes[:, 3]
    area = (x2 - x1) * (y2 - y1)
    index = area < (w * h) * threshold
    return boxes[index], index.cpu()


def save_mask(
    masks: torch.Tensor,
    output_path: str,
    image_path: str,
    phrases: List[str],
    conf: List[float]
) -> None:
    mask_arrays = masks.cpu().numpy()
    image_name = os.path.splitext(os.path.basename(image_path))[0]
    if image_name.endswith('_color'):
        image_name = image_name[:-6]

    image_output_path = os.path.join(output_path, image_name)
    os.makedirs(image_output_path, exist_ok=True)
    prompt_mask_counts = {}

    for i, mask in enumerate(mask_arrays):
        m = mask[0].astype(np.float32)
        denom = m.max() - m.min()
        if denom > 0:
            m = (m - m.min()) / denom * 255.0
        m = m.astype(np.uint8)

        img = PILImg.fromarray(m)
        phrase = phrases[i]
        conf_ = conf[i]

        prompt_mask_counts.setdefault(phrase, 0)
        prompt_output_path = os.path.join(image_output_path, phrase)
        os.makedirs(prompt_output_path, exist_ok=True)

        mask_index = prompt_mask_counts[phrase]
        mask_filename = f"mask_{phrase}_{mask_index}_{conf_:.2f}.png"
        img.save(os.path.join(prompt_output_path, mask_filename))
        prompt_mask_counts[phrase] += 1


def filter(
    bboxes: torch.Tensor,
    conf_list: torch.Tensor,
    phrases: List[str],
    conf_bound: float = 1.0,
    yVal: float = 0.8,
    precentWidth: float = 0.8,
    precentHeight: float = 0.8,
    precentArea: float = 0.01,
    filterChoice: bool = True
) -> Tuple[torch.Tensor, torch.Tensor, List[str], bool]:
    """Filters out false positives and image artifacts."""
    if not filterChoice:
        if conf_list.size(dim=0) == 0:
            return bboxes, conf_list, phrases, True
        return bboxes, conf_list, phrases, False

    IMAGE_WIDTH = 640
    IMAGE_HEIGHT = 480
    IMAGE_AREA = IMAGE_WIDTH * IMAGE_HEIGHT
    MIN_BOX_AREA = precentArea * IMAGE_AREA

    phrases_np = np.array(phrases)

    if conf_list.size(dim=0) >= 1:
        c1 = bboxes[:, 3] <= precentHeight
        c2 = bboxes[:, 2] <= precentWidth
        c3 = bboxes[:, 1] <= yVal

        box_areas = (bboxes[:, 2] * IMAGE_WIDTH) * (bboxes[:, 3] * IMAGE_HEIGHT)
        c4 = box_areas >= MIN_BOX_AREA

        mask = c1 & c2 & c3 & c4

        door_indices = np.where(phrases_np == 'door')[0]
        for i in door_indices:
            width = bboxes[i, 2] * IMAGE_WIDTH
            height = bboxes[i, 3] * IMAGE_HEIGHT
            if height / max(float(width), 1e-4) < 1.7 or box_areas[i] < 0.04 * IMAGE_AREA:
                mask[i] = False

        bboxes = bboxes[mask]
        conf_list = conf_list[mask]
        phrases_np = phrases_np[mask.cpu().numpy()]

    if conf_list.size(dim=0) == 0:
        return bboxes, conf_list, phrases_np.tolist(), True

    if any(conf >= conf_bound for conf in conf_list):
        return bboxes, conf_list, phrases_np.tolist(), True

    return bboxes, conf_list, phrases_np.tolist(), False
