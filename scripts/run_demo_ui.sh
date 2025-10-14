#!/usr/bin/env bash
set -euo pipefail

# Helper to launch the demo UI with a venv and ROS sourced.

ROOT_DIR=$(cd "$(dirname "$0")/.." && pwd)
ROS_SETUP=${ROS_SETUP_BASH:-}

if [ -z "$ROS_SETUP" ]; then
  if [ -f "$ROOT_DIR/install/setup.bash" ]; then
    ROS_SETUP="$ROOT_DIR/install/setup.bash"
  elif [ -f "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash" ]; then
    ROS_SETUP="/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
  fi
fi

if [ -n "$ROS_SETUP" ]; then
  echo "[run_demo_ui] Sourcing $ROS_SETUP"
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
else
  echo "[run_demo_ui] Warning: ROS setup not found; set ROS_SETUP_BASH."
fi

VENV="$HOME/.venvs/zkml_guard_ui"
if [ ! -d "$VENV" ]; then
  echo "[run_demo_ui] Creating venv at $VENV"
  python3 -m venv "$VENV"
fi
source "$VENV/bin/activate"
pip install -U pip
pip install -r "$ROOT_DIR/src/zkml_guard/requirements.txt"

export PATH="$ROOT_DIR/tools/bin:$PATH"

echo "[run_demo_ui] Starting UI..."
exec python3 "$ROOT_DIR/tools/ui/demo_ui.py"

