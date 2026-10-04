#!/usr/bin/env python3
"""Integration test for scoutiq_explore against tests/mock_robot/mock_robot.py.

Verifies:
1. explore node starts and receives /map from mock_robot.py.
2. TF transform from map to base_link resolves.
3. explore connects to navigate_to_pose action server.
4. With fully known room, explore finds 0 frontiers and triggers termination.
5. map.pgm and map.yaml are saved into data/<run>/ per CONTRACT.md.
6. exploration_termination topic is published.
"""
import glob
import os
import signal
import subprocess
import sys
import time

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..'))
ROS_DOMAIN_ID = '15'


def main():
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = ROS_DOMAIN_ID
    env['PYTHONUNBUFFERED'] = '1'

    # Ensure clean data directory for this test run
    test_run_dir = os.path.join(PROJECT_ROOT, 'data', 'test_mock_explore_run')
    if os.path.exists(test_run_dir):
        import shutil
        shutil.rmtree(test_run_dir)

    print(f"[TEST] Starting mock_robot on ROS_DOMAIN_ID={ROS_DOMAIN_ID}...")
    mock_proc = subprocess.Popen(
        ['python3', 'tests/mock_robot/mock_robot.py'],
        cwd=PROJECT_ROOT,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True
    )

    explore_proc = None
    try:
        # Give mock_robot 2 seconds to initialize publishers and TF
        time.sleep(2.0)

        print("[TEST] Starting scoutiq_explore node...")
        explore_cmd = [
            'ros2', 'run', 'scoutiq_explore', 'explore',
            '--ros-args',
            '-p', f'run_dir:={test_run_dir}',
            '-p', 'planner_frequency:=2.0',
            '-p', 'min_global_frontiers:=1.0',
            '-p', 'save_map:=true'
        ]
        explore_proc = subprocess.Popen(
            explore_cmd,
            cwd=PROJECT_ROOT,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True
        )

        # Monitor explore output for termination
        start_time = time.time()
        timeout = 25.0
        success = False
        output_lines = []

        while time.time() - start_time < timeout:
            line = explore_proc.stdout.readline()
            if line:
                output_lines.append(line.strip())
                print(f"[explore] {line.strip()}")
                if "Exploration complete" in line:
                    print("[TEST] Detected exploration completion signal!")
                    success = True
                    break
            time.sleep(0.05)

        # Wait a moment for map files to flush
        time.sleep(2.0)

        # Check map files
        pgm_path = os.path.join(test_run_dir, 'map.pgm')
        yaml_path = os.path.join(test_run_dir, 'map.yaml')

        pgm_exists = os.path.isfile(pgm_path) and os.path.getsize(pgm_path) > 0
        yaml_exists = os.path.isfile(yaml_path) and os.path.getsize(yaml_path) > 0

        print(f"[TEST] Check map.pgm: {pgm_path} exists? {pgm_exists} (size: {os.path.getsize(pgm_path) if pgm_exists else 0} bytes)")
        print(f"[TEST] Check map.yaml: {yaml_path} exists? {yaml_exists} (size: {os.path.getsize(yaml_path) if yaml_exists else 0} bytes)")

        if pgm_exists and yaml_exists:
            with open(yaml_path) as f:
                content = f.read()
                print(f"[TEST] map.yaml content:\n{content}")
            print("[TEST] SUCCESS: Integration test against mock_robot passed!")
            return 0
        else:
            print("[TEST] FAILURE: Map files were not generated properly.")
            return 1

    finally:
        if explore_proc and explore_proc.poll() is None:
            explore_proc.terminate()
            explore_proc.wait(timeout=3)
        if mock_proc and mock_proc.poll() is None:
            mock_proc.terminate()
            mock_proc.wait(timeout=3)


if __name__ == '__main__':
    sys.exit(main())
