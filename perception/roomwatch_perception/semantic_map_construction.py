"""Semantic map construction node (CLI: rw-semantic-construct)."""

import argparse
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
    is_nearby_in_map,
    overlay_masks,
    pose_in_map_frame,
    save_graph_json,
)


class SemanticMapConstructNode(Node):
    """Constructs 3D semantic graph from camera stream and publishes visualizations."""

    def __init__(
        self,
        text_prompt: str = "table . door . chair .",
        box_threshold: float = 0.35,
        text_threshold: float = 0.35,
        output_file: str = "graph.json",
        rate_limit_sec: float = 2.0,
        fake_detector: bool = False,
        max_iterations: int = -1,
        target_size: int = 800,
    ):
        super().__init__("rw_semantic_construct")

        # Declare ROS parameters with defaults
        self.declare_parameter("text_prompt", text_prompt)
        self.declare_parameter("box_threshold", box_threshold)
        self.declare_parameter("text_threshold", text_threshold)
        self.declare_parameter("output_file", output_file)
        self.declare_parameter("rate_limit_sec", rate_limit_sec)
        self.declare_parameter("fake_detector", fake_detector)
        self.declare_parameter("max_iterations", max_iterations)
        self.declare_parameter("target_size", target_size)

        # Get parameter values
        self.text_prompt = self.get_parameter("text_prompt").value
        self.box_threshold = float(self.get_parameter("box_threshold").value)
        self.text_threshold = float(self.get_parameter("text_threshold").value)
        self.output_file = self.get_parameter("output_file").value
        self.rate_limit_sec = float(self.get_parameter("rate_limit_sec").value)
        self.fake_detector = bool(self.get_parameter("fake_detector").value)
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

        # Initialize perception models
        self.get_logger().info(f"Initializing perception models (fake_detector={self.fake_detector})...")
        if self.fake_detector:
            self.gdino = FakeObjectPredictor(device="cpu")
            self.sam = FakeSAMPredictor(device="cpu")
        else:
            self.gdino = GroundingDINOObjectPredictor(device="cpu")
            self.sam = SegmentAnythingPredictor(device="cpu")

        self.graph = nx.Graph()
        self.pose_list: Dict[str, List[List[float]]] = {"table": [], "chair": [], "door": []}
        self.threshold: Dict[str, float] = {"table": 2.0, "chair": 0.6, "door": 2.0}

        self.iter_count = 0
        self.last_process_time = 0.0
        self._running = True

        self.get_logger().info(
            f"rw_semantic_construct initialized. Prompt: '{self.text_prompt}', Output: '{self.output_file}'"
        )

        # Start background worker thread for inference
        self.worker_thread = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker_thread.start()

    def _worker_loop(self) -> None:
        while rclpy.ok() and self._running:
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
                self.get_logger().error(f"Error processing frame: {e}", throttle_duration_sec=2.0)
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

        h, w = im_color.shape[:2]
        img_pil = PILImg.fromarray(im_color)

        # 1. Detection
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
        if flag or len(phrases) == 0:
            return

        # 2. Scale bounding boxes
        image_pil_bboxes = self.gdino.bbox_to_scaled_xyxy(bboxes, w, h)

        # 3. Segmentation
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

        # 5. Graph node insertion
        mask_array = masks.cpu().numpy() if hasattr(masks, "cpu") else np.array(masks)
        phrase_iter_ = {"table": 0, "door": 0, "chair": 0}

        for i, mask in enumerate(mask_array):
            cat = phrases[i]
            phrase_iter_.setdefault(cat, 0)
            self.pose_list.setdefault(cat, [])
            thresh = self.threshold.get(cat, 1.0)

            pose = pose_in_map_frame(
                RT_camera, RT_base, depth_img, segment=mask[0], fx=fx, fy=fy, px=px, py=py
            )
            if pose is None:
                continue

            self.pose_list[cat], is_nearby = is_nearby_in_map(
                self.pose_list[cat], pose, threshold=thresh
            )
            if not is_nearby:
                node_id = f"{cat}_{self.iter_count}_{phrase_iter_[cat]}"
                self.get_logger().info(f"Adding graph node {node_id} at {pose}")
                self.graph.add_node(
                    node_id,
                    id=node_id,
                    pose=pose,
                    robot_pose=RT_base.tolist(),
                    category=cat,
                )
                phrase_iter_[cat] += 1
                if pose not in self.pose_list[cat]:
                    self.pose_list[cat].append(pose)

        # 6. Annotate and publish image
        bbox_annotated_pil = annotate(
            overlay_masks(img_pil, masks), image_pil_bboxes, gdino_conf, phrases
        )
        im_label = np.array(bbox_annotated_pil)

        rgb_msg = self.cv_bridge.cv2_to_imgmsg(im_label, encoding="rgb8")
        rgb_msg.header.stamp = rgb_frame_stamp
        rgb_msg.header.frame_id = rgb_frame_id
        self.image_pub.publish(rgb_msg)

        # 7. Publish RViz markers and save graph
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

    def save_graph(self) -> None:
        self.get_logger().info(f"Saving graph with {len(self.graph.nodes)} nodes to {self.output_file}")
        save_graph_json(self.graph, file=self.output_file)


def main(args: Optional[List[str]] = None) -> None:
    parser = argparse.ArgumentParser(description="Construct 3D semantic graph from robot camera stream.")
    parser.add_argument("--output", "-o", default="graph.json", help="Path to output graph.json")
    parser.add_argument("--text-prompt", "--prompt", default="table . door . chair .", help="Text prompt for detector")
    parser.add_argument("--box-threshold", type=float, default=0.35, help="Bounding box confidence threshold")
    parser.add_argument("--text-threshold", type=float, default=0.35, help="Text matching threshold")
    parser.add_argument("--rate-limit", type=float, default=2.0, help="Min seconds between frame processing")
    parser.add_argument("--fake-detector", action="store_true", help="Use fast fake detector for mock tests")
    parser.add_argument("--max-iterations", type=int, default=-1, help="Max frames to process (-1 for continuous)")
    parser.add_argument("--target-size", type=int, default=800, help="Detection image resize target")

    parsed, remaining = parser.parse_known_args(args=args)

    rclpy.init(args=remaining)
    node = SemanticMapConstructNode(
        text_prompt=parsed.text_prompt,
        box_threshold=parsed.box_threshold,
        text_threshold=parsed.text_threshold,
        output_file=parsed.output,
        rate_limit_sec=parsed.rate_limit,
        fake_detector=parsed.fake_detector,
        max_iterations=parsed.max_iterations,
        target_size=parsed.target_size,
    )

    try:
        while rclpy.ok() and node._running:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_graph()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
