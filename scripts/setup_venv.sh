#!/usr/bin/env bash
# Create the Python venv with uv (CPU-only torch). ROS 2 Python packages (rclpy, cv_bridge, ...)
# come from the system via --system-site-packages, so the venv must use /usr/bin/python3 (3.10).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="$ROOT/.venv"

command -v uv >/dev/null || { echo "uv not found: curl -LsSf https://astral.sh/uv/install.sh | sh"; exit 1; }

uv venv "$VENV" --python /usr/bin/python3 --system-site-packages --seed
# shellcheck disable=SC1091
source "$VENV/bin/activate"

# numpy<2 is mandatory: Humble's cv_bridge is built against numpy 1.x.
uv pip install "numpy==1.26.4"

# CPU-only PyTorch (no CUDA download, ~200 MB instead of several GB)
uv pip install --index-url https://download.pytorch.org/whl/cpu \
  --extra-index-url https://pypi.org/simple --index-strategy unsafe-best-match \
  "torch==2.1.2" "torchvision==0.16.2"

uv pip install -r "$ROOT/perception/requirements.txt"

# GroundingDINO (CPU build: skip CUDA op; falls back to pure-PyTorch deformable attention)
BUILD_WITH_CUDA=0 uv pip install --no-build-isolation \
  "git+https://github.com/IDEA-Research/GroundingDINO.git"
# MobileSAM
uv pip install "git+https://github.com/ChaoningZhang/MobileSAM.git"

# Pin numpy again in case a dependency upgraded it
uv pip install "numpy==1.26.4"

echo "[ok] venv ready at $VENV"
bash "$ROOT/scripts/check_env.sh" || true
