"""ROS 2 coordinate and transformation utilities using transforms3d."""

from typing import Iterable, Union
import numpy as np
import transforms3d.quaternions as tq
from geometry_msgs.msg import Pose


def ros_qt_to_rt(
    quat: Union[Iterable[float], object],
    posn: Union[Iterable[float], object]
) -> np.ndarray:
    """
    Converts ROS quaternion [x, y, z, w] and position [x, y, z] to a 4x4 transformation matrix.

    :param quat: List, tuple, or quaternion object with (x, y, z, w)
    :param posn: List, tuple, or point object with (x, y, z)
    :return: 4x4 numpy array
    """
    if hasattr(quat, "x"):
        qx, qy, qz, qw = quat.x, quat.y, quat.z, quat.w
    else:
        qx, qy, qz, qw = quat[0], quat[1], quat[2], quat[3]

    if hasattr(posn, "x"):
        px, py, pz = posn.x, posn.y, posn.z
    else:
        px, py, pz = posn[0], posn[1], posn[2]

    mat = np.eye(4, dtype=np.float64)
    # transforms3d expects quaternion in [w, x, y, z] format
    mat[:3, :3] = tq.quat2mat([qw, qx, qy, qz])
    mat[:3, 3] = np.array([px, py, pz], dtype=np.float64)
    return mat


def ros_pose_to_rt(pose: Pose) -> np.ndarray:
    """
    Converts ROS Pose message to a 4x4 transformation matrix.

    :param pose: geometry_msgs.msg.Pose
    :return: 4x4 numpy array
    """
    quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    posn = [pose.position.x, pose.position.y, pose.position.z]
    return ros_qt_to_rt(quat, posn)


def rt_to_ros_pose(mat: np.ndarray) -> Pose:
    """
    Converts a 4x4 transformation matrix to ROS Pose message.

    :param mat: 4x4 numpy array
    :return: geometry_msgs.msg.Pose
    """
    q_wxyz = tq.mat2quat(mat[:3, :3])  # returns [w, x, y, z]
    pose = Pose()
    pose.orientation.x = float(q_wxyz[1])
    pose.orientation.y = float(q_wxyz[2])
    pose.orientation.z = float(q_wxyz[3])
    pose.orientation.w = float(q_wxyz[0])
    pose.position.x = float(mat[0, 3])
    pose.position.y = float(mat[1, 3])
    pose.position.z = float(mat[2, 3])
    return pose
