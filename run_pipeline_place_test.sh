#!/usr/bin/env bash
set -euo pipefail
if [[ "${1:-}" = --help ]]; then
    cat <<'EOF'
用法：./run_pipeline_place_test.sh (--target-zone 左上|左下|中间|右上|右下|blue | --x X --y Y --z Z) [--vision-config 配置.json] [--timeout 360]
一次只预览一只碗的抓放及返回起点；六个位置取自 src/grasp_executor/config/place_zones.json。
坐标模式的 X/Y/Z 为 base/tool0 下最终释放位置（米）；姿态和下降距离采用 blue 当前配置。
--target-zone 与 --x --y --z 二选一，坐标参数必须三个一起提供。
示教点未填写或未启用时会在连接 ROS 之前拒绝运行；可用 --place-zones-config 指定另一份配置。
默认 ROI 覆盖当前操作桌，每次重新识别唯一一只碗；相机或桌子移动后需重新核对 ROI。
各位置使用各自示教下降量，释放、上退后返回已记录的固定起始关节位置。
空检测、运动前候选耗尽/几何不足可在总超时内重新观察，共用默认两次重试额度。
日志按 round_0001 等轮次保存配置快照、分组候选、失败原因和实际采集时间戳。
只做 plan-only，不切换控制器、不移动机械臂、不驱动夹爪；不会覆盖旧 run_pipeline_test.sh。
前提：run_robot_prepare.sh 已完成，相机、TF、MoveIt 和工作空间已就绪。
EOF
    exit 0
fi
for arg in "$@"; do
    if [[ "$arg" = --legacy-multi ]]; then
        echo '错误：此入口只允许单碗单次抓放，请指定 --target-zone 或 --x --y --z。' >&2
        exit 2
    fi
    if [[ "$arg" = --execute ]]; then
        echo '错误：预览入口不允许 --execute；真实抓放请使用 run_pipeline_place_execute.sh。' >&2
        exit 2
    fi
done
source "$(dirname "${BASH_SOURCE[0]}")/tools/robot_startup_common.sh"
load_robot_environment
source_setup "$ROBOT_WORKSPACE/install/setup.bash"
# Vision validates one actual RGB/depth observation using the effective config.
# A camera timeout is distinct from a successful observation with zero targets.
exec "$ROBOT_PYTHON" -B "$ROBOT_WORKSPACE/tools/test_live_place_pipeline.py" "$@"
