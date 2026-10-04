#!/usr/bin/env bash
# Create an isolated git worktree + branch for a work-package agent.
# Usage: bash scripts/new_worktree.sh wp4-nav [base-ref]   (default base: v0-scaffold, else main)
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NAME="${1:?usage: new_worktree.sh <wp-name> [base-ref]}"
BASE="${2:-}"
if [ -z "$BASE" ]; then git -C "$ROOT" rev-parse -q --verify v0-scaffold >/dev/null && BASE=v0-scaffold || BASE=main; fi
DEST="$ROOT/../roomwatch-wt/$NAME"
git -C "$ROOT" worktree add "$DEST" -b "$NAME" "$BASE"
echo "Worktree: $(cd "$DEST" && pwd)  (branch $NAME from $BASE)"
echo "Agent env hints:  export ROS_DOMAIN_ID=<unique 1-100>  IGN_PARTITION=$NAME"
echo "Venv: symlink or reuse $ROOT/.venv  ->  source $ROOT/.venv/bin/activate"
