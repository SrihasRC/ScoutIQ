"""ROS 2 Image and TF Listener for RGB-D camera and robot poses."""

import threading
import time
from typing import Optional, Tuple
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import SingleThreadedExecutor

import tf2_ros
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge
import message_filters

try:
    from scoutiq_core.ros_utils import ros_qt_to_rt
except ImportError:
    from ros_utils import ros_qt_to_rt


class ImageListener(Node):
    """Listens to RGB-D camera feeds and TF transforms, maintaining synchronized data."""

    def __init__(
        self,
        node_name: str = "image_listener",
        camera: str = "Fetch",
        start_spin_thread: bool = False,
    ):
        # Allow reusing rclpy context if already initialized
        if not rclpy.ok():
            rclpy.init()
        super().__init__(node_name)

        self.camera_type = camera
        self.lock = threading.Lock()
        self.cv_bridge = CvBridge()

        self.im: Optional[np.ndarray] = None
        self.depth: Optional[np.ndarray] = None
        self.rgb_frame_id: Optional[str] = None
        self.rgb_frame_stamp = None
        self.height: int = 0
        self.width: int = 0

        self.base_frame = "base_link"
        self.camera_frame = "head_camera_rgb_optical_frame"
        self.target_frame = self.base_frame

        self.RT_camera: Optional[np.ndarray] = None
        self.RT_laser: Optional[np.ndarray] = None
        self.RT_base: Optional[np.ndarray] = None

        # Camera intrinsics default (Fetch)
        self.intrinsics = np.array([
            [574.0527954101562, 0.0, 319.5],
            [0.0, 574.0527954101562, 239.5],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)
        self.fx = float(self.intrinsics[0, 0])
        self.fy = float(self.intrinsics[1, 1])
        self.px = float(self.intrinsics[0, 2])
        self.py = float(self.intrinsics[1, 2])

        # TF2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # CameraInfo subscriber
        self.info_sub = self.create_subscription(
            CameraInfo,
            "/head_camera/rgb/camera_info",
            self.camera_info_callback,
            qos_profile_sensor_data,
        )

        # Lidar PC publisher
        self.lidar_pub = self.create_publisher(
            PointCloud2,
            "/lidar_pc",
            qos_profile_sensor_data,
        )

        # Message filters for RGB and Depth synchronization
        self.rgb_sub = message_filters.Subscriber(
            self,
            Image,
            "/head_camera/rgb/image_raw",
            qos_profile=qos_profile_sensor_data,
        )
        self.depth_sub = message_filters.Subscriber(
            self,
            Image,
            "/head_camera/depth_registered/image_raw",
            qos_profile=qos_profile_sensor_data,
        )

        queue_size = 10
        slop_seconds = 0.2
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size,
            slop_seconds,
        )
        self.ts.registerCallback(self.callback_rgbd)

        self._executor = None
        self._spin_thread = None
        if start_spin_thread:
            self.start_spinning()

        self.get_logger().info("ImageListener initialized.")

    def start_spinning(self) -> None:
        """Starts an internal background thread to spin this node."""
        if self._spin_thread is not None and self._spin_thread.is_alive():
            return
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

    def stop_spinning(self) -> None:
        """Stops the internal background spinning thread cleanly."""
        if self._executor is not None:
            self._executor.shutdown()
            if self._spin_thread is not None:
                self._spin_thread.join(timeout=2.0)
                self._spin_thread = None
            self._executor = None

    def camera_info_callback(self, msg: CameraInfo) -> None:
        """Updates camera intrinsic parameters from CameraInfo message."""
        try:
            intr = np.array(msg.k, dtype=np.float64).reshape((3, 3))
            if intr[0, 0] > 0.0:
                self.intrinsics = intr
                self.fx = float(intr[0, 0])
                self.fy = float(intr[1, 1])
                self.px = float(intr[0, 2])
                self.py = float(intr[1, 2])
        except Exception as e:
            self.get_logger().warn(f"Failed to parse camera info: {e}")

    def lookup_tf_rt(self, target_frame: str, source_frame: str) -> Optional[np.ndarray]:
        """Looks up transform from target_frame to source_frame and returns 4x4 matrix."""
        try:
            t = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
            posn = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
            quat = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
            return ros_qt_to_rt(quat, posn)
        except Exception:
            return None

    def callback_rgbd(self, rgb: Image, depth: Image) -> None:
        """Synchronized callback for RGB and Depth images."""
        # 1. Lookup TF transforms
        RT_camera = self.lookup_tf_rt(self.base_frame, self.camera_frame)
        RT_laser = self.lookup_tf_rt(self.base_frame, "laser_link")
        RT_base = self.lookup_tf_rt("map", self.base_frame)

        # 2. Convert depth image to numpy array in meters
        depth_cv: Optional[np.ndarray] = None
        try:
            if depth.encoding == "32FC1":
                depth_cv = self.cv_bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough")
                depth_cv = np.nan_to_num(depth_cv, nan=0.0, posinf=0.0, neginf=0.0)
            elif depth.encoding == "16UC1":
                raw_d = self.cv_bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough")
                depth_cv = raw_d.astype(np.float32) / 1000.0
                depth_cv = np.nan_to_num(depth_cv, nan=0.0, posinf=0.0, neginf=0.0)
            else:
                # Direct buffer fallback for 32FC1
                if depth.step >= depth.width * 4:
                    raw = np.frombuffer(depth.data, dtype=np.float32)
                    depth_cv = raw.reshape((depth.height, depth.width))
                    depth_cv = np.nan_to_num(depth_cv, nan=0.0, posinf=0.0, neginf=0.0)
                else:
                    self.get_logger().warn(f"Unsupported depth encoding: {depth.encoding}")
                    return
        except Exception as e:
            self.get_logger().warn(f"Failed to convert depth image: {e}")
            return

        # 3. Convert RGB image to numpy array
        im: Optional[np.ndarray] = None
        try:
            im = self.cv_bridge.imgmsg_to_cv2(rgb, desired_encoding="bgr8")
        except Exception:
            try:
                im = self.cv_bridge.imgmsg_to_cv2(rgb, desired_encoding="passthrough")
            except Exception as e:
                self.get_logger().warn(f"Failed to convert RGB image: {e}")
                return

        # 4. Save thread-safely
        with self.lock:
            self.im = im.copy()
            self.depth = depth_cv.copy()
            self.rgb_frame_id = rgb.header.frame_id
            self.rgb_frame_stamp = rgb.header.stamp
            self.height = depth_cv.shape[0]
            self.width = depth_cv.shape[1]
            if RT_camera is not None:
                self.RT_camera = RT_camera
            if RT_laser is not None:
                self.RT_laser = RT_laser
            if RT_base is not None:
                self.RT_base = RT_base

    def get_data_to_save(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Returns the latest RT_camera and RT_base transformation matrices."""
        with self.lock:
            if self.im is None or self.RT_camera is None or self.RT_base is None:
                return None, None
            return self.RT_camera.copy(), self.RT_base.copy()

    def get_all_data(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
        """Returns (im, depth, RT_camera, RT_base)."""
        with self.lock:
            im = self.im.copy() if self.im is not None else None
            depth = self.depth.copy() if self.depth is not None else None
            rt_cam = self.RT_camera.copy() if self.RT_camera is not None else None
            rt_base = self.RT_base.copy() if self.RT_base is not None else None
            return im, depth, rt_cam, rt_base


def main(args=None):
    from rclpy.executors import ExternalShutdownException
    node = None
    try:
        if not rclpy.ok():
            rclpy.init(args=args)
        node = ImageListener()
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
