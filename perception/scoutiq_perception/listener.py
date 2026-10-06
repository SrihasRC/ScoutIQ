"""ROS 2 image and TF listener using rclpy, tf2_ros, and message_filters."""

import threading
from typing import Dict, Optional, Tuple

from cv_bridge import CvBridge
import message_filters
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener

from .ros_utils import ros_qt_to_rt


class ImageListener:
    """Listens to RGB-D camera stream, CameraInfo, and TF transforms."""

    def __init__(
        self,
        node: Node,
        base_frame: str = "base_link",
        camera_frame: str = "head_camera_rgb_optical_frame",
        map_frame: str = "map",
        laser_frame: str = "laser_link"
    ):
        self.node = node
        self.lock = threading.Lock()
        self.cv_bridge = CvBridge()

        self.base_frame = base_frame
        self.camera_frame = camera_frame
        self.map_frame = map_frame
        self.laser_frame = laser_frame

        self.im: Optional[np.ndarray] = None
        self.depth: Optional[np.ndarray] = None
        self.rgb_frame_id: Optional[str] = None
        self.rgb_frame_stamp = None
        self.RT_camera: Optional[np.ndarray] = None
        self.RT_laser: Optional[np.ndarray] = None
        self.RT_base: Optional[np.ndarray] = None
        self.RT_camera_to_map: Optional[np.ndarray] = None

        # Motion gating: track latest angular velocity
        self.latest_angular_z: float = 0.0

        # Camera intrinsics default (will be updated dynamically by CameraInfo)
        self.fx: float = 574.0527954101562
        self.fy: float = 574.0527954101562
        self.px: float = 319.5
        self.py: float = 239.5
        self.intrinsics_received = False

        # Set up TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        # Odometry subscriber for motion gating
        self.odom_sub = self.node.create_subscription(
            Odometry,
            "/odom",
            self.callback_odom,
            qos_profile_sensor_data
        )

        # CameraInfo subscriber
        self.info_sub = self.node.create_subscription(
            CameraInfo,
            "/head_camera/rgb/camera_info",
            self.callback_camera_info,
            qos_profile_sensor_data
        )

        # Synchronized RGB and Depth subscribers
        self.rgb_sub = message_filters.Subscriber(
            self.node,
            Image,
            "/head_camera/rgb/image_raw",
            qos_profile=qos_profile_sensor_data
        )
        self.depth_sub = message_filters.Subscriber(
            self.node,
            Image,
            "/head_camera/depth_registered/image_raw",
            qos_profile=qos_profile_sensor_data
        )

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.callback_rgbd)

        # Lidar PC publisher (compatibility with original listener)
        self.lidar_pub = self.node.create_publisher(
            PointCloud2,
            "/lidar_pc",
            qos_profile_sensor_data
        )

    def callback_odom(self, msg: Odometry) -> None:
        with self.lock:
            self.latest_angular_z = float(abs(msg.twist.twist.angular.z))

    def callback_camera_info(self, msg: CameraInfo) -> None:
        if not self.intrinsics_received:
            k = msg.k
            if len(k) == 9 and k[0] > 0:
                with self.lock:
                    self.fx = float(k[0])
                    self.fy = float(k[4])
                    self.px = float(k[2])
                    self.py = float(k[5])
                    self.intrinsics_received = True
                self.node.get_logger().info(
                    f"Camera intrinsics received: fx={self.fx:.2f}, fy={self.fy:.2f}, px={self.px:.2f}, py={self.py:.2f}"
                )

    def callback_rgbd(self, rgb: Image, depth: Image) -> None:
        # Motion gating: discard perception frame if robot angular velocity > 0.15 rad/s
        with self.lock:
            ang_z = self.latest_angular_z
        if ang_z > 0.15:
            self.node.get_logger().debug(
                f"Motion gating: skipping frame during rotation ({ang_z:.3f} rad/s > 0.15 rad/s)"
            )
            return

        # 1. Lookup synchronized transforms to global map frame using exact image timestamp
        frame_id = rgb.header.frame_id if rgb.header.frame_id else self.camera_frame
        stamp = rgb.header.stamp

        try:
            # Map -> Camera optical frame lookup with exact timestamp and 0.2s timeout
            t_map_cam = self.tf_buffer.lookup_transform(
                self.map_frame,
                frame_id,
                stamp,
                timeout=Duration(seconds=0.2)
            )
            RT_camera_to_map = ros_qt_to_rt(
                [
                    t_map_cam.transform.rotation.x,
                    t_map_cam.transform.rotation.y,
                    t_map_cam.transform.rotation.z,
                    t_map_cam.transform.rotation.w,
                ],
                [
                    t_map_cam.transform.translation.x,
                    t_map_cam.transform.translation.y,
                    t_map_cam.transform.translation.z,
                ],
            )
        except TransformException as e:
            # If exact timestamp lookup fails, attempt lookup with latest transform before skipping
            try:
                t_map_cam = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    frame_id,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.2)
                )
                RT_camera_to_map = ros_qt_to_rt(
                    [
                        t_map_cam.transform.rotation.x,
                        t_map_cam.transform.rotation.y,
                        t_map_cam.transform.rotation.z,
                        t_map_cam.transform.rotation.w,
                    ],
                    [
                        t_map_cam.transform.translation.x,
                        t_map_cam.transform.translation.y,
                        t_map_cam.transform.translation.z,
                    ],
                )
            except TransformException as e2:
                self.node.get_logger().debug(f"TF lookup skipped for {frame_id} -> map: {e2}")
                return

        # Map -> Base frame lookup
        try:
            t_base = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                stamp,
                timeout=Duration(seconds=0.2)
            )
        except TransformException:
            try:
                t_base = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.base_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.2)
                )
            except TransformException:
                t_base = None

        if t_base is not None:
            RT_base = ros_qt_to_rt(
                [
                    t_base.transform.rotation.x,
                    t_base.transform.rotation.y,
                    t_base.transform.rotation.z,
                    t_base.transform.rotation.w,
                ],
                [
                    t_base.transform.translation.x,
                    t_base.transform.translation.y,
                    t_base.transform.translation.z,
                ],
            )
        else:
            RT_base = RT_camera_to_map.copy()

        # Base -> Camera frame lookup (for backward compatibility)
        try:
            t_cam = self.tf_buffer.lookup_transform(
                self.base_frame,
                frame_id,
                stamp,
                timeout=Duration(seconds=0.2)
            )
            RT_camera = ros_qt_to_rt(
                [
                    t_cam.transform.rotation.x,
                    t_cam.transform.rotation.y,
                    t_cam.transform.rotation.z,
                    t_cam.transform.rotation.w,
                ],
                [
                    t_cam.transform.translation.x,
                    t_cam.transform.translation.y,
                    t_cam.transform.translation.z,
                ],
            )
        except TransformException:
            RT_camera = np.eye(4)

        try:
            t_laser = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.laser_frame,
                stamp,
                timeout=Duration(seconds=0.2)
            )
            RT_laser = ros_qt_to_rt(
                [
                    t_laser.transform.rotation.x,
                    t_laser.transform.rotation.y,
                    t_laser.transform.rotation.z,
                    t_laser.transform.rotation.w,
                ],
                [
                    t_laser.transform.translation.x,
                    t_laser.transform.translation.y,
                    t_laser.transform.translation.z,
                ],
            )
        except TransformException:
            RT_laser = None

        # 2. Decode depth image
        try:
            if depth.encoding == "32FC1":
                depth_cv = self.cv_bridge.imgmsg_to_cv2(depth, desired_encoding="32FC1")
                depth_cv = np.nan_to_num(depth_cv, nan=0.0)
            elif depth.encoding == "16UC1":
                depth_cv = self.cv_bridge.imgmsg_to_cv2(depth, desired_encoding="16UC1").astype(np.float32)
                depth_cv /= 1000.0
            else:
                self.node.get_logger().warn(f"Unsupported depth encoding: {depth.encoding}")
                return
        except Exception as e:
            self.node.get_logger().warn(f"Failed to decode depth image: {e}")
            return

        # 3. Decode RGB image
        try:
            im = self.cv_bridge.imgmsg_to_cv2(rgb, desired_encoding="rgb8")
        except Exception as e:
            self.node.get_logger().warn(f"Failed to decode RGB image: {e}")
            return

        # 4. Store latest synchronized frame
        with self.lock:
            self.im = im.copy()
            self.depth = depth_cv.copy()
            self.rgb_frame_id = frame_id
            self.rgb_frame_stamp = stamp
            self.RT_camera = RT_camera
            self.RT_laser = RT_laser
            self.RT_base = RT_base
            self.RT_camera_to_map = RT_camera_to_map

    def has_data(self) -> bool:
        with self.lock:
            return (
                self.im is not None
                and self.depth is not None
                and self.RT_camera_to_map is not None
            )

    def get_data(self) -> Optional[Dict]:
        with self.lock:
            if not (
                self.im is not None
                and self.depth is not None
                and self.RT_camera_to_map is not None
            ):
                return None
            return {
                "im": self.im.copy(),
                "depth": self.depth.copy(),
                "frame_id": self.rgb_frame_id,
                "stamp": self.rgb_frame_stamp,
                "RT_camera": self.RT_camera.copy() if self.RT_camera is not None else np.eye(4),
                "RT_laser": self.RT_laser.copy() if self.RT_laser is not None else None,
                "RT_base": self.RT_base.copy() if self.RT_base is not None else np.eye(4),
                "RT_camera_to_map": self.RT_camera_to_map.copy(),
                "fx": self.fx,
                "fy": self.fy,
                "px": self.px,
                "py": self.py,
            }

    def get_data_to_save(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        with self.lock:
            if self.RT_camera is None or self.RT_base is None:
                return None, None
            return self.RT_camera.copy(), self.RT_base.copy()

    def destroy(self) -> None:
        """Safely destroy subscriptions and publishers."""
        with self.lock:
            self.im = None
            self.depth = None
        try:
            self.node.destroy_subscription(self.info_sub)
            self.node.destroy_subscription(self.odom_sub)
            self.node.destroy_publisher(self.lidar_pub)
        except Exception:
            pass
