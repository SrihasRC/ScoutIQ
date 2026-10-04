# roomwatch

Persistent semantic mapping of indoor spaces on **ROS 2 Humble + Gazebo Harmonic**, running natively
(no Docker). Stages: explore and map → plan a surveillance traversal → localize and build an
open-vocabulary object graph → re-visit and update the graph. Perception runs on **CPU**.

## Setup
```bash
bash scripts/setup_system.sh   # apt packages (needs sudo)
bash scripts/setup_venv.sh     # uv venv + CPU torch + GroundingDINO + MobileSAM
bash scripts/check_env.sh      # verify everything
```

## Layout
- `ws/src/`: colcon packages (ROS 2)
- `perception/`: detector/segmenter code (runs in `.venv`)
- `docs/`: contract + status; `tests/`: unit/integration/e2e; `data/`: generated artifacts
- `AGENTS.md`: rules for coding agents
