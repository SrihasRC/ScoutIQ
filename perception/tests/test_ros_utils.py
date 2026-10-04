"""Test ROS 2 transform and geometry utilities."""

import numpy as np
import transforms3d.quaternions as tq
from geometry_msgs.msg import Pose
from roomwatch_perception.ros_utils import ros_pose_to_rt, ros_qt_to_rt, rt_to_ros_pose


def test_ros_qt_to_rt():
    quat = [0.0, 0.0, 0.0, 1.0]  # x, y, z, w
    posn = [1.0, 2.0, 3.0]
    mat = ros_qt_to_rt(quat, posn)
    assert mat.shape == (4, 4)
    assert np.allclose(mat[:3, 3], posn)
    assert np.allclose(mat[:3, :3], np.eye(3))


def test_pose_conversions():
    p = Pose()
    p.position.x = 2.5
    p.position.y = -1.2
    p.position.z = 0.5
    p.orientation.x = 0.0
    p.orientation.y = 0.0
    p.orientation.z = 0.70710678
    p.orientation.w = 0.70710678

    mat = ros_pose_to_rt(p)
    assert np.allclose(mat[:3, 3], [2.5, -1.2, 0.5])

    p_roundtrip = rt_to_ros_pose(mat)
    assert abs(p_roundtrip.position.x - 2.5) < 1e-4
    assert abs(p_roundtrip.position.y - (-1.2)) < 1e-4
    assert abs(p_roundtrip.position.z - 0.5) < 1e-4
    assert abs(p_roundtrip.orientation.z - 0.70710678) < 1e-4
    assert abs(p_roundtrip.orientation.w - 0.70710678) < 1e-4
