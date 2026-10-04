"""Publishes surveillance trajectory waypoints as a MarkerArray for RViz visualization."""

import os
import sys
from typing import Optional, Sequence

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


class PosePublisher(Node):
    """Publishes trajectory waypoints as cubes on /visualization_marker_array."""

    def __init__(
        self,
        node_name: str = "pose_publisher",
        traj_file: str = "surveillance_traj.npz",
        publish_rate_hz: float = 1.0,
    ):
        if not rclpy.ok():
            rclpy.init()
        super().__init__(node_name)

        self.publisher = self.create_publisher(
            MarkerArray,
            "/visualization_marker_array",
            10,
        )

        self.traj_file = traj_file
        self.trajectory: Optional[np.ndarray] = None
        self.load_trajectory(self.traj_file)

        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_poses)
        self.get_logger().info(f"PosePublisher initialized (publishing at {publish_rate_hz} Hz).")

    def load_trajectory(self, file_path: str) -> None:
        """Loads trajectory from an NPZ or JSON file."""
        if not os.path.exists(file_path):
            self.get_logger().warn(f"Trajectory file {file_path} not found.")
            return

        try:
            if file_path.endswith(".npz"):
                with np.load(file_path) as data:
                    self.trajectory = np.asarray(data["traj"], dtype=np.float64)
            elif file_path.endswith(".json"):
                from scoutiq_core.utils import read_graph_json
                g = read_graph_json(file_path)
                poses = [data["pose"] for _, data in sorted(g.nodes(data=True))]
                self.trajectory = np.asarray(poses, dtype=np.float64)
            self.get_logger().info(f"Loaded {len(self.trajectory)} poses from {file_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load trajectory from {file_path}: {e}")

    def create_marker_array(self, trajectory: Sequence[Sequence[float]]) -> MarkerArray:
        """Constructs MarkerArray message from waypoints."""
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for sno, data in enumerate(trajectory):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = stamp
            marker.ns = "poses"
            marker.id = sno
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(data[0])
            marker.pose.position.y = float(data[1])
            marker.pose.position.z = float(data[2]) if len(data) > 2 else 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.5
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)

        return marker_array

    def publish_poses(self) -> None:
        """Publishes the markers to /visualization_marker_array."""
        if self.trajectory is None or len(self.trajectory) == 0:
            return

        marker_array = self.create_marker_array(self.trajectory)
        self.publisher.publish(marker_array)


def main(args=None):
    from rclpy.executors import ExternalShutdownException
    node = None
    try:
        if not rclpy.ok():
            rclpy.init(args=args)
        traj_file = "surveillance_traj.npz"
        if len(sys.argv) > 1:
            traj_file = sys.argv[1]

        node = PosePublisher(traj_file=traj_file)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
