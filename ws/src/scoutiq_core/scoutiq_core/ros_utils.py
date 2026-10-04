"""ROS 2 geometric and transform utility functions."""

from typing import List, Sequence, Union
import numpy as np
import tf_transformations
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Point, Quaternion


def ros_qt_to_rt(quat: Sequence[float], posn: Sequence[float]) -> np.ndarray:
    """Converts ROS quaternion [x, y, z, w] and position [x, y, z] to a 4x4 transformation matrix.

    :param quat: Sequence of quaternion [x, y, z, w]
    :param posn: Sequence of position [x, y, z]
    :return: 4x4 numpy float64 array
    """
    mat = tf_transformations.quaternion_matrix(quat)
    mat[:3, 3] = np.asarray(posn, dtype=np.float64)
    return mat


def ros_pose_to_rt(pose: Union[Pose, PoseStamped]) -> np.ndarray:
    """Converts ROS Pose or PoseStamped message to a 4x4 transformation matrix.

    :param pose: ROS Pose or PoseStamped message
    :return: 4x4 numpy float64 array
    """
    if hasattr(pose, 'pose'):
        # In case a PoseStamped is passed
        pose_inner = pose.pose
    else:
        pose_inner = pose

    quat = [
        float(pose_inner.orientation.x),
        float(pose_inner.orientation.y),
        float(pose_inner.orientation.z),
        float(pose_inner.orientation.w),
    ]
    posn = [
        float(pose_inner.position.x),
        float(pose_inner.position.y),
        float(pose_inner.position.z),
    ]
    return ros_qt_to_rt(quat, posn)


def rt_to_ros_pose(mat: np.ndarray) -> Pose:
    """Converts a 4x4 transformation matrix to ROS Pose message.

    :param mat: 4x4 numpy array
    :return: geometry_msgs.msg.Pose message
    """
    quat = tf_transformations.quaternion_from_matrix(mat)
    posn = mat[:3, 3]
    pose = Pose()
    pose.orientation.x = float(quat[0])
    pose.orientation.y = float(quat[1])
    pose.orientation.z = float(quat[2])
    pose.orientation.w = float(quat[3])
    pose.position.x = float(posn[0])
    pose.position.y = float(posn[1])
    pose.position.z = float(posn[2])
    return pose


def transform_stamped_to_rt(transform_stamped: TransformStamped) -> np.ndarray:
    """Converts a geometry_msgs/TransformStamped message to a 4x4 matrix.

    :param transform_stamped: TransformStamped message
    :return: 4x4 numpy float64 array
    """
    tr = transform_stamped.transform.translation
    rot = transform_stamped.transform.rotation
    posn = [float(tr.x), float(tr.y), float(tr.z)]
    quat = [float(rot.x), float(rot.y), float(rot.z), float(rot.w)]
    return ros_qt_to_rt(quat, posn)


def rt_to_transform_stamped(
    mat: np.ndarray,
    frame_id: str = "map",
    child_frame_id: str = "base_link",
    stamp=None
) -> TransformStamped:
    """Converts a 4x4 transformation matrix to a geometry_msgs/TransformStamped message.

    :param mat: 4x4 numpy array
    :param frame_id: parent frame ID
    :param child_frame_id: child frame ID
    :param stamp: optional builtin_interfaces.msg.Time
    :return: geometry_msgs.msg.TransformStamped message
    """
    quat = tf_transformations.quaternion_from_matrix(mat)
    posn = mat[:3, 3]
    t = TransformStamped()
    if stamp is not None:
        t.header.stamp = stamp
    t.header.frame_id = frame_id
    t.child_frame_id = child_frame_id
    t.transform.translation.x = float(posn[0])
    t.transform.translation.y = float(posn[1])
    t.transform.translation.z = float(posn[2])
    t.transform.rotation.x = float(quat[0])
    t.transform.rotation.y = float(quat[1])
    t.transform.rotation.z = float(quat[2])
    t.transform.rotation.w = float(quat[3])
    return t
