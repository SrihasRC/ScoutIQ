"""Records synchronized robot poses (RT_camera, RT_base) to data/<run>/pose/%06d_pose.npz."""

import datetime
import os
import sys
import time
from typing import Optional

import numpy as np
import rclpy

try:
    from roomwatch_core.listener import ImageListener
except ImportError:
    from listener import ImageListener


class SaveData:
    """Periodically captures and saves robot pose data (RT_camera, RT_base) to npz files."""

    def __init__(
        self,
        time_interval: float = 1.0,
        output_dir: Optional[str] = None,
        root_dir: str = "data",
        warmup_sec: float = 2.0,
    ):
        if not rclpy.ok():
            rclpy.init()

        self.time_delay = float(time_interval)
        self.root_dir = root_dir
        self.output_dir = output_dir

        self.create_directory()

        self.listener = ImageListener(
            node_name="img_listen_n_infer",
            start_spin_thread=True,
        )

        if warmup_sec > 0:
            time.sleep(warmup_sec)

    def create_directory(self) -> None:
        """Creates run directory structure data/<run>/pose/."""
        if self.output_dir is not None:
            self.main_dir_name = self.output_dir
        else:
            current_time = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            self.main_dir_name = os.path.join(self.root_dir, current_time)

        self.pose_dir_name = os.path.join(self.main_dir_name, "pose")
        os.makedirs(self.main_dir_name, exist_ok=True)
        os.makedirs(self.pose_dir_name, exist_ok=True)

    def save_step(self, data_count: int) -> bool:
        """Saves a single pose file if valid data is available. Returns True if saved."""
        RT_camera, RT_base = self.listener.get_data_to_save()
        if RT_camera is None or RT_base is None:
            return False

        file_path = os.path.join(self.pose_dir_name, f"{data_count:06d}_pose.npz")
        np.savez(file_path, RT_camera=RT_camera, RT_base=RT_base)
        return True

    def save_data(self, max_count: Optional[int] = None) -> None:
        """Main recording loop."""
        data_count = 0
        try:
            while rclpy.ok():
                saved = self.save_step(data_count)
                if saved:
                    data_count += 1
                    if max_count is not None and data_count >= max_count:
                        break
                time.sleep(self.time_delay)
        finally:
            self.close()

    def close(self) -> None:
        """Stops the listener background thread and destroys node."""
        if self.listener is not None:
            self.listener.stop_spinning()
            self.listener.destroy_node()


def main(args=None):
    from rclpy.executors import ExternalShutdownException
    if not rclpy.ok():
        rclpy.init(args=args)

    time_interval = 1.0
    output_dir = None

    # Parse arguments
    argv = sys.argv[1:]
    if len(argv) >= 1:
        try:
            time_interval = float(argv[0])
        except ValueError:
            output_dir = argv[0]
    if len(argv) >= 2:
        output_dir = argv[1]

    saver = None
    try:
        saver = SaveData(time_interval=time_interval, output_dir=output_dir)
        saver.save_data()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if saver is not None:
            saver.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
