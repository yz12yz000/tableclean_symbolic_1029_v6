#!/usr/bin/env bash
# Optional visualization only; never started by run_robot_prepare.sh.
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/tools/robot_startup_common.sh"
if [[ "${1:-}" = --help || "${1:-}" = -h ]]; then
    exec "$ROBOT_PYTHON" -s -B "$ROBOT_WORKSPACE/tools/robot_workspace.py" --help
fi
load_robot_environment
# One display publisher per user/domain. This lock is independent of arm control.
exec 7>"$ROBOT_RUNTIME_DIR/workspace-display.lock"
flock -n 7 || fail '工作空间显示已运行；请先在原终端 Ctrl+C 关闭。'
# Use the matching system NumPy/SciPy, not a user-installed NumPy with a different ABI.
exec "$ROBOT_PYTHON" -s -B "$ROBOT_WORKSPACE/tools/robot_workspace.py" "$@"
