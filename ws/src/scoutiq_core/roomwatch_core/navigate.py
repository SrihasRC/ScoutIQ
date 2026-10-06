"""Nav2 NavigateToPose client, trajectory tracking, and semantic navigation."""

import math
import sys
import threading
import time
from typing import Any, List, Optional, Sequence, Union

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
import tf_transformations
from tf2_ros import Buffer, TransformListener

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Int32

try:
    from scoutiq_core.utils import read_graph_json
except ImportError:
    from utils import read_graph_json


try:
    from action_msgs.msg import GoalStatus
    STATUS_SUCCEEDED = GoalStatus.STATUS_SUCCEEDED
    STATUS_CANCELED = GoalStatus.STATUS_CANCELED
    STATUS_ABORTED = GoalStatus.STATUS_ABORTED
except ImportError:
    STATUS_SUCCEEDED = 4
    STATUS_CANCELED = 5
    STATUS_ABORTED = 6


class Navigate(Node):
    """High-level robot navigation interface wrapping Nav2 NavigateToPose."""

    def __init__(
        self,
        node_name: str = "navigate",
        start_spin_thread: bool = True,
    ):
        if not rclpy.ok():
            rclpy.init()
        super().__init__(node_name)
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", False)

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.goal = NavigateToPose.Goal()
        self.base_position: List[float] = [0.0, 0.0, 0.0]
        self.pause: int = 0

        self.pause_sub = self.create_subscription(
            Int32,
            "/yes_no",
            self.pause_callback,
            10,
        )

        self._executor: Optional[SingleThreadedExecutor] = None
        self._spin_thread: Optional[threading.Thread] = None
        if start_spin_thread:
            self.start_spinning()

        self.get_logger().info("Navigate node initialized.")

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

    def wait_for_server(self, timeout_sec: float = 5.0) -> bool:
        """Waits for NavigateToPose action server."""
        return self.nav_to_pose_client.wait_for_server(timeout_sec=timeout_sec)

    def pause_callback(self, data: Int32) -> None:
        """Callback for /yes_no pause topic (1 = pause, 0 = resume)."""
        self.pause = int(data.data)
        self.get_logger().info(f"Received /yes_no pause state: {self.pause}")

    def clear_goal(self) -> None:
        """Resets the internal goal message."""
        self.goal = NavigateToPose.Goal()

    def set_goal(self, pose: Sequence[float], qt: Optional[Sequence[float]] = None) -> None:
        """Sets target pose and orientation in the goal message (frame: map)."""
        self.goal.pose.header.frame_id = "map"
        self.goal.pose.header.stamp = self.get_clock().now().to_msg()

        self.goal.pose.pose.position.x = float(pose[0])
        self.goal.pose.pose.position.y = float(pose[1])
        self.goal.pose.pose.position.z = float(pose[2]) if len(pose) > 2 else 0.0

        if qt is not None:
            self.goal.pose.pose.orientation.x = float(qt[0])
            self.goal.pose.pose.orientation.y = float(qt[1])
            self.goal.pose.pose.orientation.z = float(qt[2])
            self.goal.pose.pose.orientation.w = float(qt[3])
        else:
            self.goal.pose.pose.orientation.x = 0.0
            self.goal.pose.pose.orientation.y = 0.0
            self.goal.pose.pose.orientation.z = 0.0
            self.goal.pose.pose.orientation.w = 1.0

    def get_base_position(self, timeout_sec: float = 4.0) -> List[float]:
        """Looks up the current robot base position in the map frame via TF."""
        t_end = time.time() + timeout_sec
        while time.time() < t_end and rclpy.ok():
            try:
                t = self.tf_buffer.lookup_transform(
                    "map",
                    "base_link",
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1),
                )
                self.base_position = [
                    float(t.transform.translation.x),
                    float(t.transform.translation.y),
                    float(t.transform.translation.z),
                ]
                return self.base_position
            except Exception:
                time.sleep(0.05)
        self.get_logger().warn("TF lookup for map->base_link timed out.")
        return self.base_position

    def compute_orientation(self, pose1: Sequence[float], pose2: Sequence[float]) -> List[float]:
        """Computes quaternion [x, y, z, w] pointing from pose1 to pose2 in 2D plane."""
        yaw = math.atan2(float(pose2[1]) - float(pose1[1]), float(pose2[0]) - float(pose1[0]))
        qt = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
        return [float(qt[0]), float(qt[1]), float(qt[2]), float(qt[3])]

    def send_goal_and_wait(
        self,
        goal: NavigateToPose.Goal,
        timeout_sec: Optional[float] = None,
        max_retries: int = 2,
    ) -> Optional[Any]:
        """Sends goal to NavigateToPose and waits for result synchronously based on feedback progress."""
        for attempt in range(max_retries + 1):
            if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("NavigateToPose action server not available!")
                return None

            event = threading.Event()
            result_holder = {"goal_handle": None, "result": None, "status": None}
            last_progress_time = [time.time()]
            last_distance_remaining = [float("inf")]

            def goal_response_callback(future):
                goal_handle = future.result()
                result_holder["goal_handle"] = goal_handle
                if not goal_handle.accepted:
                    self.get_logger().warn(
                        f"Goal was rejected by action server (attempt {attempt + 1}/{max_retries + 1})."
                    )
                    event.set()
                    return
                self.get_logger().info("Goal accepted by action server.")
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(result_callback)

            def result_callback(future):
                res = future.result()
                result_holder["result"] = res.result
                result_holder["status"] = res.status
                if res.status == STATUS_SUCCEEDED:
                    self.get_logger().info(f"Navigation completed successfully (status: {res.status})")
                elif res.status == STATUS_ABORTED:
                    self.get_logger().warn(f"Navigation was aborted by action server (status: {res.status})")
                elif res.status == STATUS_CANCELED:
                    self.get_logger().info(f"Navigation was canceled (status: {res.status})")
                else:
                    self.get_logger().warn(f"Navigation finished with non-success status: {res.status}")
                event.set()

            def feedback_callback(feedback_msg):
                fb = feedback_msg.feedback
                dist = getattr(fb, "distance_remaining", None)
                if dist is not None:
                    if dist < last_distance_remaining[0] - 0.05:
                        last_progress_time[0] = time.time()
                        last_distance_remaining[0] = dist

            if self._spin_thread is not None and self._spin_thread.is_alive():
                send_goal_future = self.nav_to_pose_client.send_goal_async(
                    goal, feedback_callback=feedback_callback
                )
                send_goal_future.add_done_callback(goal_response_callback)

                stalled_limit = 45.0
                while not event.is_set() and rclpy.ok():
                    if event.wait(timeout=1.0):
                        break
                    if timeout_sec is not None and (time.time() - last_progress_time[0]) > timeout_sec:
                        self.get_logger().warn(f"Navigation exceeded user timeout ({timeout_sec}s).")
                        if result_holder["goal_handle"] is not None:
                            result_holder["goal_handle"].cancel_goal_async()
                        break
                    if (time.time() - last_progress_time[0]) > stalled_limit:
                        self.get_logger().warn(f"No navigation progress for {stalled_limit}s; canceling goal.")
                        if result_holder["goal_handle"] is not None:
                            result_holder["goal_handle"].cancel_goal_async()
                        break
            else:
                send_goal_future = self.nav_to_pose_client.send_goal_async(
                    goal, feedback_callback=feedback_callback
                )
                rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)
                goal_handle = send_goal_future.result()
                if goal_handle is None or not goal_handle.accepted:
                    return None
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future)
                if result_future.done():
                    res = result_future.result()
                    result_holder["result"] = res.result
                    result_holder["status"] = res.status

            if result_holder["status"] == STATUS_SUCCEEDED:
                return result_holder["result"]
            elif result_holder["status"] == STATUS_ABORTED:
                if attempt < max_retries:
                    self.get_logger().warn(
                        f"Action goal aborted (attempt {attempt + 1}/{max_retries + 1}); "
                        f"executing recovery retry in 1.5s..."
                    )
                    time.sleep(1.5)
                    continue
                else:
                    self.get_logger().error(f"Action goal aborted after {max_retries + 1} attempts.")
                    return None
            else:
                return None

        return None

    def navigate_to(
        self,
        pose: Sequence[float],
        wait_until: bool = False,
        set_orientation: bool = False,
        orientation_qt: Optional[Sequence[float]] = None,
        max_retries: int = 2,
    ) -> Optional[Any]:
        """Navigates robot to target pose in map frame."""
        if set_orientation:
            self.set_goal(pose, orientation_qt)
        else:
            self.get_base_position()
            if self.base_position == [0.0, 0.0, 0.0] and pose[0] == 0.0 and pose[1] == 0.0:
                orientation_qt = [0.0, 0.0, 0.0, 1.0]
            else:
                orientation_qt = self.compute_orientation(self.base_position, pose)
            self.set_goal(pose, orientation_qt)

        result = self.send_goal_and_wait(self.goal, max_retries=max_retries)
        return result

    def track_trajectory(self, waypoints: Sequence[Sequence[float]] = ()) -> None:
        """Navigates sequentially through an array of waypoints, respecting /yes_no pause state."""
        if len(waypoints) == 0:
            return

        self.get_base_position()
        start_idx = 0
        if self.base_position != [0.0, 0.0, 0.0]:
            dist_to_0 = math.hypot(
                self.base_position[0] - waypoints[0][0],
                self.base_position[1] - waypoints[0][1],
            )
            if dist_to_0 < 0.4:
                start_idx = 1

        for i in range(start_idx, len(waypoints)):
            waypoint = waypoints[i]

            while self.pause != 0 and rclpy.ok():
                self.get_logger().info("Navigation paused via /yes_no... Waiting to resume.")
                time.sleep(0.5)

            if not rclpy.ok():
                break

            self.get_logger().info(f"Navigating to waypoint {i}/{len(waypoints) - 1}: {waypoint}")
            res = self.navigate_to(waypoint, max_retries=2)
            if res is None:
                self.get_logger().warn(
                    f"Waypoint {i} was aborted or reached retry limit. Advancing to next waypoint."
                )

    def navigate_to_object_class(
        self,
        graph_file: str,
        _class: str = "door",
        nearest: bool = False,
    ) -> Optional[Any]:
        """Finds object node(s) matching _class from graph JSON and navigates to the target."""
        graph = read_graph_json(file=graph_file)
        matching_nodes = []
        for node, data in graph.nodes(data=True):
            category = data.get("category", "")
            if _class in category:
                matching_nodes.append((data.get("pose", [0.0, 0.0, 0.0]), node))

        if not matching_nodes:
            self.get_logger().warn(f"No objects found in graph with category containing '{_class}'.")
            return None

        if not nearest:
            target_pose = matching_nodes[0][0]
            self.get_logger().info(f"Navigating to first matching '{_class}' at {target_pose}")
            return self.navigate_to(target_pose)
        else:
            self.get_base_position()
            positions = np.array([pose for pose, _ in matching_nodes])
            base_xy = np.array(self.base_position[:2])
            dists = np.linalg.norm(positions[:, :2] - base_xy, axis=1)
            min_idx = int(np.argmin(dists))
            nearest_pose = matching_nodes[min_idx][0]
            self.get_logger().info(f"Navigating to nearest matching '{_class}' at {nearest_pose}")
            return self.navigate_to(nearest_pose)

    def close(self) -> None:
        """Stops spinning and cleans up."""
        self.stop_spinning()


def main(args=None):
    from rclpy.executors import ExternalShutdownException
    if not rclpy.ok():
        rclpy.init(args=args)

    nav = None
    traj_path = "surveillance_traj.npz"
    for arg in sys.argv[1:]:
        if not arg.startswith("-") and ":=" not in arg:
            traj_path = arg
            break

    try:
        nav = Navigate()
        with np.load(traj_path) as traj_file:
            surveillance_traj = traj_file["traj"]
        print(f"Loaded {len(surveillance_traj)} waypoints from {traj_path}")
        nav.track_trajectory(waypoints=surveillance_traj)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as e:
        print(f"Error during navigation: {e}")
    finally:
        if nav is not None:
            nav.close()
            nav.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
