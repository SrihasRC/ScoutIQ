"""scoutiq_core - Core utilities, trajectory processing, and navigation for scoutiq."""

import sys
import os

# Prioritize virtual environment packages over ~/.local (which contains NumPy 2.x)
_repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
_venv_site = os.path.join(_repo_root, ".venv", "lib", "python3.10", "site-packages")
if os.path.isdir(_venv_site) and _venv_site not in sys.path:
    sys.path.insert(0, _venv_site)

# NumPy compatibility for legacy transforms3d / tf_transformations if NumPy 2 is loaded
try:
    import numpy as np
    if not hasattr(np, "float"):
        np.float = float
    if not hasattr(np, "maximum_sctype"):
        np.maximum_sctype = lambda t: np.float64
except ImportError:
    pass

from scoutiq_core.ros_utils import (
    ros_qt_to_rt,
    ros_pose_to_rt,
    rt_to_ros_pose,
    transform_stamped_to_rt,
    rt_to_transform_stamped,
)

from scoutiq_core.utils import (
    compute_xyz,
    pose_to_map_pixel,
    pose_along_line,
    read_map_image,
    read_map_metadata,
    display_map_image,
    is_nearby,
    normalize_depth_image,
    denormalize_depth_image,
    get_fov_points_in_baselink,
    pose_in_map_frame,
    is_nearby_in_map,
    save_graph_json,
    read_graph_json,
    read_and_visualize_graph,
    plot_point_on_map,
    visualize_graph,
    pointcloud2_to_xyz_array,
    xyz_array_to_pointcloud2,
    laser_scan_to_xyz_array,
)

def __getattr__(name):
    if name == 'ImageListener':
        from scoutiq_core.listener import ImageListener
        return ImageListener
    elif name == 'SaveData':
        from scoutiq_core.save_data import SaveData
        return SaveData
    elif name == 'Navigate':
        from scoutiq_core.navigate import Navigate
        return Navigate
    elif name == 'PosePublisher':
        from scoutiq_core.publish_traj import PosePublisher
        return PosePublisher
    elif name == 'ExtractTrajectory':
        from scoutiq_core.extract_robot_trajectory import ExtractTrajectory
        return ExtractTrajectory
    elif name in ('tsp_greedy_solution', 'preprocess_trajectory_graph', 'compute_surveillance_trajectory'):
        import scoutiq_core.tsp_surveillance_trajectory as mod
        return getattr(mod, name)
    raise AttributeError(f"module '{__name__}' has no attribute '{name}'")


__all__ = [
    'ros_qt_to_rt',
    'ros_pose_to_rt',
    'rt_to_ros_pose',
    'transform_stamped_to_rt',
    'rt_to_transform_stamped',
    'compute_xyz',
    'pose_to_map_pixel',
    'pose_along_line',
    'read_map_image',
    'read_map_metadata',
    'display_map_image',
    'is_nearby',
    'normalize_depth_image',
    'denormalize_depth_image',
    'get_fov_points_in_baselink',
    'pose_in_map_frame',
    'is_nearby_in_map',
    'save_graph_json',
    'read_graph_json',
    'read_and_visualize_graph',
    'plot_point_on_map',
    'visualize_graph',
    'pointcloud2_to_xyz_array',
    'xyz_array_to_pointcloud2',
    'laser_scan_to_xyz_array',
    'ImageListener',
    'SaveData',
    'Navigate',
    'PosePublisher',
    'ExtractTrajectory',
    'tsp_greedy_solution',
    'preprocess_trajectory_graph',
    'compute_surveillance_trajectory',
]
