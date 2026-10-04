#!/usr/bin/env python3
"""Integration and unit tests for scoutiq_nav package."""
import math
import os
import shutil
import subprocess
import tempfile
import time
import pytest
import yaml
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid


PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_DIR = os.path.join(PACKAGE_DIR, 'config')
LAUNCH_DIR = os.path.join(PACKAGE_DIR, 'launch')


def test_nav2_params_configuration():
    """Verify nav2_params.yaml has all required WP4 parameters."""
    nav2_yaml_path = os.path.join(CONFIG_DIR, 'nav2_params.yaml')
    assert os.path.exists(nav2_yaml_path), f"Missing {nav2_yaml_path}"

    with open(nav2_yaml_path, 'r') as f:
        params = yaml.safe_load(f)

    # 1. Footprint ~0.28m radius in costmaps
    local_params = params['local_costmap']['local_costmap']['ros__parameters']
    assert math.isclose(local_params['robot_radius'], 0.28, rel_tol=1e-3), "local_costmap robot_radius != 0.28"
    assert local_params['robot_base_frame'] == 'base_link'
    assert local_params['global_frame'] == 'odom'

    global_params = params['global_costmap']['global_costmap']['ros__parameters']
    assert math.isclose(global_params['robot_radius'], 0.28, rel_tol=1e-3), "global_costmap robot_radius != 0.28"
    assert global_params['robot_base_frame'] == 'base_link'
    assert global_params['global_frame'] == 'map'

    # 2. Costmaps from /scan
    local_scan = local_params['obstacle_layer']['scan']
    assert local_scan['topic'] == '/scan'
    assert local_scan['data_type'] == 'LaserScan'
    assert local_scan['clearing'] is True
    assert local_scan['marking'] is True

    global_scan = global_params['obstacle_layer']['scan']
    assert global_scan['topic'] == '/scan'
    assert global_scan['data_type'] == 'LaserScan'

    # 3. Controller server (DWBLocalPlanner)
    ctrl_params = params['controller_server']['ros__parameters']
    assert 'FollowPath' in ctrl_params['controller_plugins']
    dwb_params = ctrl_params['FollowPath']
    assert dwb_params['plugin'] == 'dwb_core::DWBLocalPlanner'
    assert dwb_params['max_vel_x'] <= 0.4
    assert dwb_params['min_vel_x'] >= 0.0

    # 4. Planner server (NavfnPlanner)
    planner_params = params['planner_server']['ros__parameters']
    assert 'GridBased' in planner_params['planner_plugins']
    assert planner_params['GridBased']['plugin'] == 'nav2_navfn_planner/NavfnPlanner'

    # 5. Behavior server
    behav_params = params['behavior_server']['ros__parameters']
    assert behav_params['robot_base_frame'] == 'base_link'
    assert behav_params['global_frame'] == 'odom'
    assert 'spin' in behav_params['behavior_plugins']
    assert 'backup' in behav_params['behavior_plugins']

    # 6. AMCL
    amcl_params = params['amcl']['ros__parameters']
    assert amcl_params['base_frame_id'] == 'base_link'
    assert amcl_params['odom_frame_id'] == 'odom'
    assert amcl_params['global_frame_id'] == 'map'
    assert amcl_params['scan_topic'] == '/scan'


def test_slam_toolbox_params_configuration():
    """Verify slam_toolbox_params.yaml matches scoutiq contract."""
    slam_yaml_path = os.path.join(CONFIG_DIR, 'slam_toolbox_params.yaml')
    assert os.path.exists(slam_yaml_path), f"Missing {slam_yaml_path}"

    with open(slam_yaml_path, 'r') as f:
        params = yaml.safe_load(f)

    slam_params = params['slam_toolbox']['ros__parameters']
    assert slam_params['base_frame'] == 'base_link'
    assert slam_params['odom_frame'] == 'odom'
    assert slam_params['map_frame'] == 'map'
    assert slam_params['scan_topic'] == '/scan'
    assert slam_params['mode'] == 'mapping'
    assert math.isclose(slam_params['resolution'], 0.05)


def test_launch_files_exist():
    """Verify required launch files are present."""
    mapping_launch = os.path.join(LAUNCH_DIR, 'mapping.launch.py')
    localize_launch = os.path.join(LAUNCH_DIR, 'localize.launch.py')
    assert os.path.exists(mapping_launch), f"Missing {mapping_launch}"
    assert os.path.exists(localize_launch), f"Missing {localize_launch}"


def test_pub_initial_pose_message():
    """Test pub_initial_pose node publishing to /initialpose."""
    os.environ['ROS_DOMAIN_ID'] = '14'
    rclpy.init()
    node = Node('test_pub_initial_pose_subscriber')

    received = []
    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL
    )
    node.create_subscription(
        PoseWithCovarianceStamped,
        '/initialpose',
        lambda msg: received.append(msg),
        qos
    )

    # Launch pub_initial_pose in subprocess
    pub_proc = subprocess.Popen(
        [
            'ros2', 'run', 'scoutiq_nav', 'pub_initial_pose',
            '--ros-args',
            '-p', 'delay:=0.1',
            '-p', 'repeat_count:=2',
            '-p', 'rate:=5.0',
            '-p', 'exit_after_publish:=true'
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )

    start_t = time.time()
    while time.time() - start_t < 5.0 and len(received) == 0:
        rclpy.spin_once(node, timeout_sec=0.1)

    pub_proc.wait(timeout=5)
    node.destroy_node()
    rclpy.shutdown()

    assert len(received) > 0, "No /initialpose message received"
    msg = received[0]
    assert msg.header.frame_id == 'map'
    assert math.isclose(msg.pose.pose.position.x, 0.0)
    assert math.isclose(msg.pose.pose.position.y, 0.0)
    assert math.isclose(msg.pose.pose.position.z, 0.0)
    assert math.isclose(msg.pose.pose.orientation.w, 1.0)
    assert math.isclose(msg.pose.pose.orientation.x, 0.0)
    assert math.isclose(msg.pose.pose.orientation.y, 0.0)
    assert math.isclose(msg.pose.pose.orientation.z, 0.0)

    # Check covariance
    cov = msg.pose.covariance
    assert math.isclose(cov[0], 1.0), "cov_x should be 1.0"
    assert math.isclose(cov[7], 1.0), "cov_y should be 1.0"
    expected_cov_yaw = float(np.deg2rad(10.0) ** 2)
    assert math.isclose(cov[35], expected_cov_yaw, rel_tol=1e-3), "cov_yaw should match"


def test_save_map_utility():
    """Test save_map CLI against mock_robot."""
    os.environ['ROS_DOMAIN_ID'] = '14'
    mock_robot_script = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(PACKAGE_DIR))),
        'tests', 'mock_robot', 'mock_robot.py'
    )
    assert os.path.exists(mock_robot_script), f"Missing {mock_robot_script}"

    mock_proc = subprocess.Popen(
        ['python3', mock_robot_script],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
    time.sleep(2.0)

    with tempfile.TemporaryDirectory() as tmpdir:
        out_prefix = os.path.join(tmpdir, 'saved_map')
        save_script = os.path.join(PACKAGE_DIR, 'scripts', 'save_map')
        res = subprocess.run(
            [save_script, out_prefix],
            capture_output=True,
            text=True,
            timeout=10
        )
        assert res.returncode == 0, f"save_map failed: {res.stderr}\n{res.stdout}"

        pgm_path = f"{out_prefix}.pgm"
        yaml_path = f"{out_prefix}.yaml"
        assert os.path.exists(pgm_path), f"Missing {pgm_path}"
        assert os.path.exists(yaml_path), f"Missing {yaml_path}"
        assert os.path.getsize(pgm_path) > 0
        assert os.path.getsize(yaml_path) > 0

        with open(yaml_path, 'r') as f:
            y = yaml.safe_load(f)
        assert math.isclose(y['resolution'], 0.05), "Map resolution != 0.05"
        assert y['image'] == 'saved_map.pgm' or y['image'].endswith('.pgm')

    mock_proc.terminate()
    try:
        mock_proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        mock_proc.kill()
