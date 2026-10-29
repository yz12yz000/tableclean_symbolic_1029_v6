#!/usr/bin/env bash
set -euo pipefail
if [[ "${1:-}" == --help ]]; then
    cat <<'EOF'
用法：./run_return_home.sh
使用 place_zones.json 中当前保存的起始位姿和六关节角，按初版 4.5 倍速度上限返回一次后退出（加速度不变）。
先运行 run_robot_prepare.sh，并在示教器启动 External Control。
自动检查状态、准备位置控制器、规划和确认到位；故障时停止，不自动解保护停机。
不识别物体，不开合夹爪，不修改任何示教点。
执行前确认夹爪及物体已脱离接触、返程区域畅通，现场有人看护。
Ctrl+C 请求停止；日志保存在 debug_output/return_home/。
EOF
    exit 0
fi
[[ $# -eq 0 ]] || { echo '用法：./run_return_home.sh [--help]' >&2; exit 2; }
source "$(dirname "${BASH_SOURCE[0]}")/tools/robot_startup_common.sh"
load_robot_environment
source_setup "$ROBOT_WORKSPACE/install/setup.bash"
export PYTHONNOUSERSITE=1
exec "$ROBOT_PYTHON" -B "$ROBOT_WORKSPACE/tools/return_home.py"
