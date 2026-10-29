---
name: gat-arm-deployment
description: 本机单碗位置解析、校验和固定脚本执行规则。
---

# 本机单碗执行

固定工作区`/home/rob/table_clean_ws`。配置`config/robots/arm.yaml`的transport=local，不使用网络部署。

## 放置位置解析

每次先读取 `config/robots/arm_place_zones.yaml`。它是坐标的唯一来源，不在本规则中重复维护数值。坐标系为 `base`，目标类型为 `release_position_base_tool0`，即base/tool0 最终释放点。

1. 使用 `zones` 的名称与 `aliases` 做明确匹配：左上、左下、中间、右上、右下和蓝色区域。
2. `derived_midpoints` 定义左侧、右侧、上侧、下侧，分别按其 `from` 中两个端点逐坐标取平均值 `(A+B)/2`。
3. 用户明确给出两个已知端点、起点终点顺序及比例时，按 `P(t)=(1-t)A+tB` 计算；范围为 `0 <= t <= 1`。数值必须有限，不接受 NaN、Infinity 或超出范围的比例。
4. “靠左一点”等词语不能擅自转成比例。没有唯一匹配、比例或方向不明确时，用 `question` 提问，在参数明确前不生成任务。
5. 当前 YAML 没有 `relative_direction_rule`。对“蓝色区域右侧 25 cm”等相对位移，说明缺少已确认的相对坐标规则，先提问，不沿用历史示例或推断新方向。
6. 命令坐标保留 YAML 指定的六位小数。先以未舍入数值完成计算，最后格式化；不得把命令释放点解释成已经验证的碗落点。

调整参考点、增加名称或别名时，只修改本项目的坐标 YAML；后续任务重新读取并生成计划。未知机械臂接口、其他物体或额外姿态需求先提问。

## 计划与本机执行

arm_task.py保留原有坐标表、UUID、来源hash、计划hash校验。local_execution替代remote_execution；robot.transport=local，命令无SSH。

```bash
python3 generated/arm/arm_task.py --create --target 右侧 --output generated/arm/task_plan.json --mode preview --provider <实际provider> --model <实际模型>
python3 generated/arm/arm_task.py --plan generated/arm/task_plan.json --validate --hash
python3 tools/local_runner.py arm --plan generated/arm/task_plan.json
python3 tools/local_runner.py arm --plan generated/arm/task_plan.json --run
```

preview运行固定run_pipeline_place_test.sh，不运动。execute必须重新生成--mode execute计划，展示目标和命令，在已有明确授权后执行：

```bash
python3 tools/local_runner.py arm --plan generated/arm/task_plan.json --run --confirmed
```

工具用参数数组运行固定脚本，不拼接用户shell；真实执行器由wrapper加载环境。每个UUID只启动一次，已有任务目录拒绝重用。工具等待本机进程真实退出，保留日志和result.json，写reports/arm_last_run.json；source/preflight和结果归属照常检查。不要另开同一命令，不用短超时杀正常运动，不自动重试。

```bash
python3 generated/arm/arm_task.py --plan generated/arm/task_plan.json --validate --result reports/arm_last_run.json
```

本地进程退出不等于机械臂已停，当前能力仍需按真实反馈报告。没有证据不能声称抓牢或精确放好。历史examples仅作记录，不可当作本次授权或执行结果。
