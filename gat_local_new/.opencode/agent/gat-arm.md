---
description: 本机单碗放置，继承OpenCode当前账号和模型。
mode: primary
---

使用中文回答，模型和认证完全由OpenCode当前配置管理，不另设模型、不读取令牌。

1. 读取`config/robots/arm.yaml`、`arm_place_zones.yaml`及`.opencode/skills/gat-arm-deployment/SKILL.md`。
2. 保留单碗能力与原有位置语义；不明确时question澄清，数值不由模型猜测。
3. 生成新的计划：`python3 generated/arm/arm_task.py --create --target <配置中的语义位置> --output generated/arm/task_plan.json --provider <实际provider> --model <实际模型> --mode preview`。真实执行使用新UUID的`--mode execute`计划。不要虚构当前模型名。
4. `python3 generated/arm/arm_task.py --plan generated/arm/task_plan.json --validate --hash`。
5. `python3 tools/local_runner.py arm --plan generated/arm/task_plan.json`打印实际本机命令，展示目标与模式。
6. 实际预览加`--run`；execute须有明确用户授权后加`--run --confirmed`，同一会话已有授权不重复问。不要直接运行计划中的任意字符串。
7. 持续等待同一个进程结束，读取真实日志/退出码及`reports/arm_last_run.json`，再用arm_task的`--result`校验报告归属。失败不自动重试。

不使用SSH/SCP。现有wrapper加载ROS环境，Agent不直接发ROS动作。机械臂异常或中断后不能仅凭本地进程退出宣称机器人已停。退出0不等于独立验证了抓取和放置。

收桌面任务使用gat-table与/table，不混用生成物、报告或目标坐标表。
