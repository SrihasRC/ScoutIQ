"""Semantic map update node (CLI: rw-semantic-update)."""

import argparse
import os
import sys
import threading
import time
from typing import Dict, List, Optional

from cv_bridge import CvBridge
import networkx as nx
import numpy as np
from PIL import Image as PILImg
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from shapely.geometry import Point, Polygon
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker, MarkerArray

from .listener import ImageListener
from .perception import (
    FakeObjectPredictor,
    FakeSAMPredictor,
    GroundingDINOObjectPredictor,
    SegmentAnythingPredictor,
)
from .utils import (
    annotate,
    combine_masks,
    filter,
    filter_large_boxes,
    get_fov_points_in_map,
    is_nearby_in_map,
    overlay_masks,
    pose_in_map_frame,
    read_graph_json,
    save_graph_json,
)


class SemanticMapUpdateNode(Node):
    """Updates 3D semantic graph by verifying, removing, and adding objects dynamically."""

    def __init__(
        self,
        input_file: str = "graph.json",
        output_file: str = "graph_updated.json",
        text_prompt: str = "table . chair . sofa . bed . cabinet . refrigerator . door .",
        box_threshold: float = 0.35,
        text_threshold: float = 0.35,
        rate_limit_sec: float = 2.0,
        fake_detector: bool = False,
        model_type: str = "onnx",
        max_iterations: int = -1,
        target_size: int = 800,
    ):
        super().__init__("rw_semantic_update")

        # Declare parameters
        self.declare_parameter("input_file", input_file)
        self.declare_parameter("output_file", output_file)
        self.declare_parameter("text_prompt", text_prompt)
        self.declare_parameter("box_threshold", box_threshold)
        self.declare_parameter("text_threshold", text_threshold)
        self.declare_parameter("rate_limit_sec", rate_limit_sec)
        self.declare_parameter("fake_detector", fake_detector)
        self.declare_parameter("model_type", model_type)
        self.declare_parameter("max_iterations", max_iterations)
        self.declare_parameter("target_size", target_size)

        # Read parameter values
        self.input_file = self.get_parameter("input_file").value
        self.output_file = self.get_parameter("output_file").value
        self.text_prompt = self.get_parameter("text_prompt").value
        self.box_threshold = float(self.get_parameter("box_threshold").value)
        self.text_threshold = float(self.get_parameter("text_threshold").value)
        self.rate_limit_sec = float(self.get_parameter("rate_limit_sec").value)
        self.fake_detector = bool(self.get_parameter("fake_detector").value)
        self.model_type = str(self.get_parameter("model_type").value)
        self.max_iterations = int(self.get_parameter("max_iterations").value)
        self.target_size = int(self.get_parameter("target_size").value)

        self.cv_bridge = CvBridge()
        self.listener = ImageListener(self)

        # Publishers per CONTRACT.md
        self.image_pub = self.create_publisher(Image, "/seg_image", qos_profile_sensor_data)
        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/graph_nodes",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        # Pause subscription on /yes_no
        self.pause = 0
        self.pause_sub = self.create_subscription(
            Int32,
            "/yes_no",
            self.pause_callback,
            10
        )

        # Load graph
        self.graph = read_graph_json(self.input_file)
        self.get_logger().info(
            f"Loaded initial graph from {self.input_file} with {len(self.graph.nodes)} nodes"
        )

        # Initialize perception models
        self.get_logger().info(f"Initializing perception models (fake_detector={self.fake_detector}, model_type={self.model_type})...")
        if self.fake_detector:
            self.gdino = FakeObjectPredictor(device="cpu")
            self.sam = FakeSAMPredictor(device="cpu")
        elif self.model_type == "onnx":
            try:
                from .perception import YOLOWorldONNXPredictor
                self.gdino = YOLOWorldONNXPredictor(device="cpu")
                self.sam = SegmentAnythingPredictor(device="cpu")
                self.get_logger().info("Using YOLO-World ONNX detector + MobileSAM segmenter.")
            except Exception as e:
                self.get_logger().warn(f"Failed to load ONNX detector ({e}), falling back to GroundingDINO.")
                self.gdino = GroundingDINOObjectPredictor(device="cpu")
                self.sam = SegmentAnythingPredictor(device="cpu")
        else:
            self.gdino = GroundingDINOObjectPredictor(device="cpu")
            self.sam = SegmentAnythingPredictor(device="cpu")

        self.pose_list: Dict[str, List[List[float]]] = {}
        self.threshold: Dict[str, float] = {
            "table": 2.2,
            "chair": 1.2,
            "door": 2.0,
            "bed": 2.5,
            "sofa": 2.2,
            "cabinet": 1.5,
            "refrigerator": 1.5,
        }

        # Populate initial pose list from graph
        for _, data in self.graph.nodes(data=True):
            cat = data.get("category", "")
            if cat in self.threshold and "pose" in data:
                self.pose_list.setdefault(cat, []).append(data["pose"])

        self.iter_count = 0
        self.last_process_time = 0.0
        self._running = True

        self.get_logger().info(
            f"rw_semantic_update initialized. Prompt: '{self.text_prompt}', Output: '{self.output_file}'"
        )

        # Worker thread
        self.worker_thread = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker_thread.start()

    def pause_callback(self, msg: Int32) -> None:
        self.pause = msg.data
        if self.pause == 1:
            self.get_logger().info("Semantic map update paused by /yes_no")
        else:
            self.get_logger().info("Semantic map update resumed")

    def _worker_loop(self) -> None:
        while rclpy.ok() and self._running:
            if self.pause == 1:
                time.sleep(0.1)
                continue

            now = time.time()
            if (now - self.last_process_time) < self.rate_limit_sec:
                time.sleep(0.05)
                continue

            if not self.listener.has_data():
                time.sleep(0.05)
                continue

            data = self.listener.get_data()
            if data is None:
                time.sleep(0.05)
                continue

            self.last_process_time = time.time()
            try:
                self.process_frame(data)
                self.iter_count += 1
                if self.max_iterations > 0 and self.iter_count >= self.max_iterations:
                    self.get_logger().info(f"Reached max iterations ({self.max_iterations}). Stopping worker.")
                    self._running = False
                    break
            except Exception as e:
                self.get_logger().error(f"Error updating frame: {e}", throttle_duration_sec=2.0)
                time.sleep(0.1)

    def process_frame(self, data: Dict) -> None:
        im_color = data["im"]
        depth_img = data["depth"]
        rgb_frame_id = data["frame_id"]
        rgb_frame_stamp = data["stamp"]
        RT_camera = data["RT_camera"]
        RT_base = data["RT_base"]
        fx = data["fx"]
        fy = data["fy"]
        px = data["px"]
        py = data["py"]

        RT_camera_to_map = data.get("RT_camera_to_map")

        h, w = im_color.shape[:2]
        img_pil = PILImg.fromarray(im_color)

        # 1. FOV polygon calculation in map frame
        if RT_camera_to_map is not None:
            fov_points = get_fov_points_in_map(depth_img, RT_camera_to_map, None, fx=fx, fy=fy, px=px, py=py)
        else:
            fov_points = get_fov_points_in_map(depth_img, RT_camera, RT_base, fx=fx, fy=fy, px=px, py=py)
        fov_poly = Polygon(fov_points)
        if not fov_poly.is_valid:
            fov_poly = fov_poly.buffer(0)

        # 2. Detection
        bboxes, phrases, gdino_conf = self.gdino.predict(
            img_pil,
            self.text_prompt,
            box_threshold=self.box_threshold,
            text_threshold=self.text_threshold,
            target_size=self.target_size,
        )

        bboxes, gdino_conf, phrases, flag = filter(
            bboxes, gdino_conf, phrases, 1.0, 0.8, 0.8, 0.8, 0.01, True
        )

        # If no detections left after filter, check if expected nodes in FOV need removal
        if flag or len(phrases) == 0:
            if fov_poly.is_valid and not fov_poly.is_empty:
                nodes_to_remove = []
                for node, ndata in self.graph.nodes(data=True):
                    if "new" in node:
                        continue
                    pose_ = ndata.get("pose", [0, 0, 0])
                    point = Point(pose_[0], pose_[1])
                    if fov_poly.contains(point):
                        nodes_to_remove.append(node)
                for n in nodes_to_remove:
                    self.get_logger().info(f"Removing node missing from view: {n}")
                    self.graph.remove_node(n)
            return

        # 3. Scale bounding boxes & SAM
        image_pil_bboxes = self.gdino.bbox_to_scaled_xyxy(bboxes, w, h)
        image_pil_bboxes, masks = self.sam.predict(img_pil, image_pil_bboxes)
        if masks is None or len(masks) == 0:
            return

        # 4. Filter large boxes
        image_pil_bboxes, index = filter_large_boxes(image_pil_bboxes, w, h, threshold=0.5)
        masks = masks[index]
        gdino_conf = gdino_conf[index]
        ind = np.where(index)[0]
        phrases = [phrases[i] for i in ind]
        if len(phrases) == 0:
            return

        # 5. Calculate poses for current detections
        detected_poses: Dict[str, List[List[float]]] = {"door": [], "chair": [], "table": []}
        mask_array = masks.cpu().numpy() if hasattr(masks, "cpu") else np.array(masks)

        valid_detections = []
        for i, mask in enumerate(mask_array):
            cat = phrases[i]
            detected_poses.setdefault(cat, [])
            if RT_camera_to_map is not None:
                pose = pose_in_map_frame(
                    RT_camera_to_map, None, depth_img, segment=mask[0], fx=fx, fy=fy, px=px, py=py
                )
            else:
                pose = pose_in_map_frame(
                    RT_camera, RT_base, depth_img, segment=mask[0], fx=fx, fy=fy, px=px, py=py
                )
            if pose is not None:
                # Outlier rejection: reject points outside the house boundaries or invalid heights
                if (
                    abs(pose[0]) > 10.0
                    or abs(pose[1]) > 10.0
                    or pose[2] < -0.2
                    or pose[2] > 2.5
                ):
                    self.get_logger().warn(
                        f"Outlier detected for {cat} at [{pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f}], skipping."
                    )
                    continue

                detected_poses[cat].append(pose)
                valid_detections.append((i, cat, mask, pose))

        # 6. Verify existing nodes inside FOV
        if fov_poly.is_valid and not fov_poly.is_empty:
            nodes_to_remove = []
            for node, ndata in self.graph.nodes(data=True):
                if "new" in node:
                    continue
                cat = ndata.get("category", "")
                pose_ = ndata.get("pose", [0, 0, 0])
                point = Point(pose_[0], pose_[1])
                thresh = 0.8

                if fov_poly.contains(point):
                    cat_dets = detected_poses.get(cat, [])
                    if len(cat_dets) == 0:
                        nodes_to_remove.append(node)
                    else:
                        dists = np.linalg.norm(np.array(cat_dets)[:, :2] - np.array(pose_)[:2], axis=1)
                        if not np.any(dists < thresh):
                            nodes_to_remove.append(node)

            for n in nodes_to_remove:
                self.get_logger().info(f"Removing node missing from view: {n}")
                self.graph.remove_node(n)

        # 7. Add new detections or refine existing landmarks (merge if dist < 0.8m)
        phrase_iter_ = {"table": 0, "door": 0, "chair": 0}
        for (i, cat, mask, pose) in valid_detections:
            phrase_iter_.setdefault(cat, 0)

            # Check existing nodes of the same category to deduplicate or refine centroid
            best_node = None
            min_dist = float("inf")
            for node_name, ndata in self.graph.nodes(data=True):
                if ndata.get("category") == cat and "pose" in ndata:
                    dist = float(np.linalg.norm(np.array(ndata["pose"][:2]) - np.array(pose[:2])))
                    if dist < min_dist:
                        min_dist = dist
                        best_node = node_name

            if best_node is not None and min_dist < 0.8:
                # Landmark exists within 0.8m: refine its 3D centroid using running average
                old_pose = np.array(self.graph.nodes[best_node]["pose"])
                count = self.graph.nodes[best_node].get("observation_count", 1)
                new_pose = ((old_pose * count + np.array(pose)) / (count + 1)).tolist()
                self.graph.nodes[best_node]["pose"] = new_pose
                self.graph.nodes[best_node]["observation_count"] = count + 1
                self.get_logger().info(
                    f"Refined {best_node} centroid (obs={count+1}, dist={min_dist:.2f}m < 0.8m): "
                    f"[{new_pose[0]:.2f}, {new_pose[1]:.2f}, {new_pose[2]:.2f}]"
                )
            else:
                node_id = f"new_{cat}_{self.iter_count}_{phrase_iter_[cat]}"
                self.get_logger().info(f"Adding updated graph node {node_id} at {pose}")
                self.graph.add_node(
                    node_id,
                    id=node_id,
                    pose=pose,
                    robot_pose=RT_base.tolist(),
                    category=cat,
                    observation_count=1,
                )
                phrase_iter_[cat] += 1

        # 8. Annotate and publish
        bbox_annotated_pil = annotate(
            overlay_masks(img_pil, masks), image_pil_bboxes, gdino_conf, phrases
        )
        im_label = np.array(bbox_annotated_pil)

        rgb_msg = self.cv_bridge.cv2_to_imgmsg(im_label, encoding="rgb8")
        rgb_msg.header.stamp = rgb_frame_stamp
        rgb_msg.header.frame_id = rgb_frame_id
        self.image_pub.publish(rgb_msg)

        # Save updated segmented detection image to disk for inspection
        try:
            out_dir = os.path.dirname(os.path.abspath(self.output_file))
            seg_dir = os.path.join(out_dir, "segmented")
            os.makedirs(seg_dir, exist_ok=True)
            detected_names = "_".join(sorted(list(set(phrases))))
            img_filename = f"update_{self.iter_count:03d}_{detected_names}.png"
            bbox_annotated_pil.save(os.path.join(seg_dir, img_filename))
            self.get_logger().info(f"Saved segmented image to {os.path.join(seg_dir, img_filename)}")
        except Exception as e:
            self.get_logger().warn(f"Failed to save segmented image: {e}")

        # 9. Publish markers and save updated graph
        self.publish_graph_to_rviz()
        save_graph_json(self.graph, file=self.output_file)

    def create_marker(self, pose: List[float], category: str, node_id: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = category
        marker.id = node_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = float(pose[0])
        marker.pose.position.y = float(pose[1])
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a = 1.0

        if category == "table":
            marker.color.r, marker.color.g, marker.color.b = 0.0, 0.0, 1.0
        elif category == "chair":
            marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0
        elif category == "door":
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0
        else:
            marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0, 0.0

        return marker

    def publish_graph_to_rviz(self) -> None:
        marker_array = MarkerArray()
        node_id = 0
        for _, data in self.graph.nodes(data=True):
            pose = data["pose"]
            category = data["category"]
            marker = self.create_marker(pose, category, node_id)
            marker_array.markers.append(marker)
            node_id += 1
        self.marker_pub.publish(marker_array)

    def destroy_node(self) -> bool:
        self._running = False
        if hasattr(self, "worker_thread") and self.worker_thread.is_alive():
            self.worker_thread.join(timeout=1.0)
        if hasattr(self, "listener"):
            self.listener.destroy()
        return super().destroy_node()

    def save_graph(self) -> None:
        self.get_logger().info(f"Saving updated graph with {len(self.graph.nodes)} nodes to {self.output_file}")
        save_graph_json(self.graph, file=self.output_file)


def main(args: Optional[List[str]] = None) -> None:
    from rclpy.executors import ExternalShutdownException
    parser = argparse.ArgumentParser(description="Update 3D semantic graph from robot camera stream.")
    parser.add_argument("--input", "-i", default="graph.json", help="Path to input graph.json")
    parser.add_argument("--output", "-o", default="graph_updated.json", help="Path to output graph_updated.json")
    parser.add_argument("--text-prompt", "--prompt", default="table . chair . sofa . bed . cabinet . refrigerator . door .", help="Text prompt for detector")
    parser.add_argument("--box-threshold", type=float, default=0.35, help="Bounding box confidence threshold")
    parser.add_argument("--text-threshold", type=float, default=0.35, help="Text matching threshold")
    parser.add_argument("--rate-limit", type=float, default=2.0, help="Min seconds between frame processing")
    parser.add_argument("--fake-detector", action="store_true", help="Use fast fake detector for mock tests")
    parser.add_argument("--model-type", default="onnx", choices=["onnx", "gdino", "fake"], help="Detection model backend")
    parser.add_argument("--max-iterations", type=int, default=-1, help="Max frames to process (-1 for continuous)")
    parser.add_argument("--target-size", type=int, default=800, help="Detection image resize target")

    parsed, remaining = parser.parse_known_args(args=args)

    rclpy.init(args=remaining)
    node = SemanticMapUpdateNode(
        input_file=parsed.input,
        output_file=parsed.output,
        text_prompt=parsed.text_prompt,
        box_threshold=parsed.box_threshold,
        text_threshold=parsed.text_threshold,
        rate_limit_sec=parsed.rate_limit,
        fake_detector=parsed.fake_detector,
        model_type=parsed.model_type,
        max_iterations=parsed.max_iterations,
        target_size=parsed.target_size,
    )

    try:
        while rclpy.ok() and node._running:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._running = False
        node.save_graph()
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
