#!/usr/bin/env bash
set -euo pipefail
if [[ "${1:-}" = --help ]]; then
    cat <<'EOF'
用法：./run_pipeline_place_execute.sh (--target-zone 左上|左下|中间|右上|右下|blue | --x X --y Y --z Z) [--vision-config 配置.json] [--timeout 900]
一次只抓放一只碗，释放、上退、回到固定起点后退出；下次启动重新识别。
六个位置及各自下降量、固定起点取自 src/grasp_executor/config/place_zones.json。
坐标模式的 X/Y/Z 为 base/tool0 下最终释放位置（米）；姿态采用 blue，起点在释放点上方 0.10 m。
--target-zone 与 --x --y --z 二选一，坐标参数必须三个一起提供。
示教点未填写或未启用时会在连接 ROS 之前拒绝运行；可用 --place-zones-config 指定另一份配置。
默认 ROI 覆盖当前操作桌，要求视野内只有待抓的一只碗；相机或桌子移动后需重新核对 ROI。
仅运动前候选耗尽/几何不足可在总超时内重新观察。
有有效定位目标后才启动位置控制器并真实移动机械臂、开合夹爪。
执行前必须确认所选位置已经通过 plan-only 和现场示教验收。
EOF
    exit 0
fi
for arg in "$@"; do
    if [[ "$arg" = --legacy-multi ]]; then
        echo '错误：此入口只允许单碗单次抓放，请指定 --target-zone 或 --x --y --z。' >&2
        exit 2
    fi
done
source "$(dirname "${BASH_SOURCE[0]}")/tools/robot_startup_common.sh"
load_robot_environment
source_setup "$ROBOT_WORKSPACE/install/setup.bash"
# Do not start a controller until observation has produced graspable targets.
exec "$ROBOT_PYTHON" -B "$ROBOT_WORKSPACE/tools/test_live_place_pipeline.py" --execute "$@"
