"""Unit tests for roomwatch_core.ros_utils."""

import math
import numpy as np
import pytest
from geometry_msgs.msg import Pose, Point, Quaternion

from roomwatch_core.ros_utils import (
    ros_qt_to_rt,
    ros_pose_to_rt,
    rt_to_ros_pose,
    transform_stamped_to_rt,
    rt_to_transform_stamped,
)


def test_ros_qt_to_rt_identity():
    quat = [0.0, 0.0, 0.0, 1.0]
    posn = [1.0, 2.0, 3.0]
    rt = ros_qt_to_rt(quat, posn)

    expected = np.array([
        [1.0, 0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0, 2.0],
        [0.0, 0.0, 1.0, 3.0],
        [0.0, 0.0, 0.0, 1.0],
    ], dtype=np.float64)

    assert rt.shape == (4, 4)
    assert np.allclose(rt, expected)


def test_ros_qt_to_rt_rotation_z():
    # 90 degrees around Z axis
    yaw = math.pi / 2.0
    quat = [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]
    posn = [0.5, -0.5, 1.5]
    rt = ros_qt_to_rt(quat, posn)

    assert np.isclose(rt[0, 0], 0.0, atol=1e-7)
    assert np.isclose(rt[0, 1], -1.0, atol=1e-7)
    assert np.isclose(rt[1, 0], 1.0, atol=1e-7)
    assert np.isclose(rt[1, 1], 0.0, atol=1e-7)
    assert np.allclose(rt[:3, 3], [0.5, -0.5, 1.5])


def test_ros_pose_to_rt_and_roundtrip():
    pose = Pose()
    pose.position.x = 4.0
    pose.position.y = -2.5
    pose.position.z = 1.2
    # 45 deg yaw
    yaw = math.pi / 4.0
    pose.orientation.z = math.sin(yaw / 2.0)
    pose.orientation.w = math.cos(yaw / 2.0)

    rt = ros_pose_to_rt(pose)
    assert rt.shape == (4, 4)
    assert np.allclose(rt[:3, 3], [4.0, -2.5, 1.2])

    # Convert back to Pose
    recovered_pose = rt_to_ros_pose(rt)
    assert np.isclose(recovered_pose.position.x, 4.0)
    assert np.isclose(recovered_pose.position.y, -2.5)
    assert np.isclose(recovered_pose.position.z, 1.2)
    assert np.isclose(recovered_pose.orientation.z, math.sin(yaw / 2.0))
    assert np.isclose(recovered_pose.orientation.w, math.cos(yaw / 2.0))


def test_transform_stamped_conversion():
    mat = np.eye(4, dtype=np.float64)
    mat[:3, 3] = [10.0, 20.0, 30.0]

    ts = rt_to_transform_stamped(mat, frame_id="map", child_frame_id="base_link")
    assert ts.header.frame_id == "map"
    assert ts.child_frame_id == "base_link"
    assert np.isclose(ts.transform.translation.x, 10.0)
    assert np.isclose(ts.transform.translation.y, 20.0)
    assert np.isclose(ts.transform.translation.z, 30.0)

    recovered_mat = transform_stamped_to_rt(ts)
    assert np.allclose(mat, recovered_mat)


def test_reference_pose_file_consistency():
    ref_path = "/home/srihasrc/Music/AutoX-SemMap-main/fetch_ws/src/fetch_gazebo/fetch_gazebo/scripts/2024-10-02_01-37-08/pose/000034_pose.npz"
    import os
    if os.path.exists(ref_path):
        data = np.load(ref_path)
        assert "RT_camera" in data
        assert "RT_base" in data
        rt_cam = data["RT_camera"]
        rt_base = data["RT_base"]

        assert rt_cam.shape == (4, 4)
        assert rt_base.shape == (4, 4)
        # Check bottom row of transformation matrices
        assert np.allclose(rt_cam[3, :], [0.0, 0.0, 0.0, 1.0])
        assert np.allclose(rt_base[3, :], [0.0, 0.0, 0.0, 1.0])

        # Test converting to ROS Pose and back
        pose_cam = rt_to_ros_pose(rt_cam)
        rt_cam_rec = ros_pose_to_rt(pose_cam)
        assert np.allclose(rt_cam, rt_cam_rec, atol=1e-5)
