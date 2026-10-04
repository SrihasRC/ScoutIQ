#!/usr/bin/env python3
"""
Test script for WP3: Arm tucking and head look-at joint controllers.
Verifies CONTRACT requirements:
- /joint_states contains torso, head, and arm joints
- ros2 run scoutiq_gz set_head orientates head to target
- ros2 run scoutiq_gz tuck_arm tucks arm into target configuration
"""

import os
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

EXPECTED_JOINTS = [
    'torso_lift_joint',
    'head_pan_joint',
    'head_tilt_joint',
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'upperarm_roll_joint',
    'elbow_flex_joint',
    'forearm_roll_joint',
    'wrist_flex_joint',
    'wrist_roll_joint',
]

TUCKED_TARGETS = {
    'torso_lift_joint': 0.05,
    'shoulder_pan_joint': 1.32,
    'shoulder_lift_joint': 1.40,
    'upperarm_roll_joint': -0.2,
    'elbow_flex_joint': 1.72,
    'forearm_roll_joint': 0.0,
    'wrist_flex_joint': 1.66,
    'wrist_roll_joint': 0.0,
}


class WP3Tester(Node):
    def __init__(self):
        super().__init__('wp3_tester')
        self.joint_positions = {}
        self.msg_count = 0
        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._cb,
            10,
        )

    def _cb(self, msg: JointState):
        self.msg_count += 1
        for name, pos in zip(msg.name, msg.position):
            self.joint_positions[name] = pos


def main():
    print("=" * 60)
    print("WP3 Joint Control Verification Test")
    print(f"ROS_DOMAIN_ID: {os.environ.get('ROS_DOMAIN_ID', 'unset')}")
    print(f"IGN_PARTITION: {os.environ.get('IGN_PARTITION', 'unset')}")
    print("=" * 60)

    rclpy.init()
    tester = WP3Tester()

    print("[1/4] Waiting for /joint_states...")
    start = time.time()
    while time.time() - start < 15:
        rclpy.spin_once(tester, timeout_sec=0.1)
        if tester.msg_count >= 5:
            break

    assert tester.msg_count > 0, "FAIL: No /joint_states received"
    print(f"  Received {tester.msg_count} joint_states messages.")
    print(f"  Active joints found: {sorted(tester.joint_positions.keys())}")

    for j in EXPECTED_JOINTS:
        assert j in tester.joint_positions, f"FAIL: Expected joint '{j}' missing from /joint_states"
    print("  [OK] All 10 expected arm, head, and torso joints present.")

    # Phase 2: Test set_head CLI
    print("[2/4] Running set_head CLI (tilt=0.394, pan=0.0)...")
    res = subprocess.run(
        ["ros2", "run", "scoutiq_gz", "set_head", "--tilt", "0.394", "--pan", "0.0", "--timeout", "10.0"],
        capture_output=True,
        text=True,
    )
    print(f"  set_head output: {res.stdout.strip()}")
    assert res.returncode == 0, f"FAIL: set_head failed with code {res.returncode}: {res.stderr}"

    # Verify positions in joint_states
    start = time.time()
    while time.time() - start < 3.0:
        rclpy.spin_once(tester, timeout_sec=0.05)
    
    current_tilt = tester.joint_positions.get('head_tilt_joint', None)
    current_pan = tester.joint_positions.get('head_pan_joint', None)
    print(f"  Current head: pan={current_pan:.4f}, tilt={current_tilt:.4f}")
    assert abs(current_tilt - 0.394) < 0.15, f"FAIL: head_tilt_joint error too high: {current_tilt} vs 0.394"
    assert abs(current_pan - 0.0) < 0.15, f"FAIL: head_pan_joint error too high: {current_pan} vs 0.0"
    print("  [OK] Head orientation verified within tolerance.")

    # Phase 3: Test tuck_arm CLI
    print("[3/4] Running tuck_arm CLI...")
    res = subprocess.run(
        ["ros2", "run", "scoutiq_gz", "tuck_arm", "--timeout", "15.0", "--tolerance", "0.15"],
        capture_output=True,
        text=True,
    )
    print(f"  tuck_arm output: {res.stdout.strip()}")
    assert res.returncode == 0, f"FAIL: tuck_arm failed with code {res.returncode}: {res.stderr}"

    # Verify tucked positions in joint_states
    start = time.time()
    while time.time() - start < 3.0:
        rclpy.spin_once(tester, timeout_sec=0.05)

    print("[4/4] Verifying joint states against CONTRACT tuck targets...")
    for j, target_val in TUCKED_TARGETS.items():
        actual = tester.joint_positions.get(j, None)
        assert actual is not None, f"Joint {j} missing"
        err = abs(actual - target_val)
        print(f"  {j}: actual={actual:+.4f}, target={target_val:+.4f}, err={err:.4f}")
        assert err < 0.20, f"FAIL: Joint {j} failed to reach target (actual={actual}, target={target_val}, err={err})"

    print("=" * 60)
    print("ALL WP3 VERIFICATION TESTS PASSED SUCCESSFULLY!")
    print("=" * 60)
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
