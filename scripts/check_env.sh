#!/usr/bin/env bash
# Verify the environment. Usage: bash scripts/check_env.sh
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ok=0; bad=0
chk() { if eval "$2" >/dev/null 2>&1; then echo "  [ok]   $1"; ok=$((ok+1)); else echo "  [FAIL] $1"; bad=$((bad+1)); fi; }

set +u
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
[ -f "$ROOT/.venv/bin/activate" ] && source "$ROOT/.venv/bin/activate"

echo "== System =="
chk "ROS 2 humble"            '[ "$ROS_DISTRO" = humble ]'
chk "colcon"                  'command -v colcon'
chk "gz sim 8 (Harmonic)"     'gz sim --version | grep -q "version 8"'
chk "ros_gz_sim"              'ros2 pkg prefix ros_gz_sim'
chk "ros_gz_bridge"           'ros2 pkg prefix ros_gz_bridge'
chk "nav2_bringup"            'ros2 pkg prefix nav2_bringup'
chk "slam_toolbox"            'ros2 pkg prefix slam_toolbox'
chk "robot_state_publisher"   'ros2 pkg prefix robot_state_publisher'
chk "xacro"                   'ros2 pkg prefix xacro'
chk "tf_transformations"      'python3 -c "import tf_transformations"'
chk "uv"                      'command -v uv'
echo "== Python venv ($ROOT/.venv) =="
chk "venv exists"             '[ -x "$ROOT/.venv/bin/python" ]'
chk "rclpy importable"        'python -c "import rclpy"'
chk "cv_bridge importable"    'python -c "from cv_bridge import CvBridge"'
chk "numpy < 2"               'python -c "import numpy,sys; sys.exit(0 if numpy.__version__.startswith(\"1.\") else 1)"'
chk "torch (CPU)"             'python -c "import torch; print(torch.__version__)"'
chk "torchvision"             'python -c "import torchvision"'
chk "groundingdino"           'python -c "import groundingdino"'
chk "mobile_sam"              'python -c "import mobile_sam"'
chk "networkx/shapely/scipy"  'python -c "import networkx, shapely, scipy"'
echo "== Result: $ok ok, $bad failed =="
[ "$bad" -eq 0 ]
