#!/usr/bin/env bash
# Build / preview offline, or subscribe to RGB after the caller sources ROS.
# No driver startup, controller access or motion commands.
set -euo pipefail
table_tool_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
case "${1:-}" in
    build) table_tool_script=build_table_visual_prompt.py ;;
    preview) table_tool_script=preview_table_scene.py ;;
    capture) table_tool_script=capture_table_images.py ;;
    *) echo '用法：./run_table_vision_tool.sh build|preview|capture [参数]；capture 需先加载 ROS 环境' >&2; exit 2 ;;
esac
shift
table_tool_venv="${YOLO_VENV:-/home/rob/yolovision_ws/.venv}"
export PYTHONNOUSERSITE=1
export YOLO_AUTOINSTALL=false
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-4}"
if [[ -d "$table_tool_venv/lib/python3.10/site-packages" ]]; then
    export PYTHONPATH="$table_tool_venv/lib/python3.10/site-packages:${HOME:-/home/rob}/.local/lib/python3.10/site-packages:$table_tool_root/src/yolo_vision${PYTHONPATH:+:$PYTHONPATH}"
else
    export PYTHONPATH="$table_tool_root/src/yolo_vision${PYTHONPATH:+:$PYTHONPATH}"
fi
exec /usr/bin/python3 -B "$table_tool_root/tools/$table_tool_script" "$@"
