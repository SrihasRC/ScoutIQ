#!/usr/bin/env bash
set -e
export PYTHONNOUSERSITE=1

RUN_DIR="$1"
if [ -z "${RUN_DIR}" ]; then
    echo "Usage: $0 <data_run_dir> [--fake-detector]"
    exit 1
fi

FAKE_ARG="false"
[ "$2" == "--fake-detector" ] && FAKE_ARG="true"

source /opt/ros/humble/setup.bash
[ -f "ws/install/setup.bash" ] && source ws/install/setup.bash
[ -d ".venv" ] && source .venv/bin/activate

echo "Starting Surveillance Traversal & Semantic Construction on: ${RUN_DIR}"
ros2 launch scoutiq_bringup traverse.launch.py run_dir:="${RUN_DIR}" fake_detector:="${FAKE_ARG}"
