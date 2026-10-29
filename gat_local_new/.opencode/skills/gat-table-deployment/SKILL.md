---
name: gat-table-deployment
description: 本机逐轮视觉收桌面会话，固定start/poll/submit/cancel接口。
---

# 本机逐轮部署

固定工作区/home/rob/table_clean_ws，固定入口run_clear_table.sh。使用tools/table_session.py；禁止SSH/SCP、任意ROS命令、机器人端模型API和读取账号令牌。task_policy仍为v1；逐轮请求和决策固定table-round-v2，不回退v1。

策略按generated/table/<plan_id>/task_policy.json隔离。generated/table/table_task.py只做纯校验与hash；generated/table/table_round_protocol.py负责逐轮图片摘要、身份、action和合法object/slot组合校验。真实模型/provider由当前OpenCode提供，不编造。

```bash
python3 generated/table/table_task.py --task-policy <策略> --validate --require-deployment-metadata
python3 tools/table_session.py start --policy <preview策略>
python3 tools/table_session.py poll --plan-id <UUID>
python3 tools/table_session.py submit --plan-id <UUID> --decision <决策JSON>
python3 tools/table_session.py cancel --plan-id <UUID>
```

start返回后台worker会话。poll状态waiting_agent时先read请求、scene和两张真实图片；提交一次决策后继续轮询。每次poll立即返回；两次poll之间等2秒，不紧密循环。图片读取失败不能选择。每轮决策只允许object_id和其slot_id；本地保留全部尚可用抓取候选。

preview成功在线路径结果保存在reports/table/<preview_id>/result.json，策略副本在同目录task_policy.json；兼容副本generated/table/preview_policy.json、reports/table_preview_result.json仅供查阅。execute用新UUID、相同目标与预算和上述本次预览证据：

```bash
python3 tools/table_session.py start --policy <execute策略> --baseline-policy <preview策略> --baseline-result <preview结果> --confirmed
```

真实运动须有当前用户任务授权；同一会话内已有授权沿用，不逐碗重复确认。--confirmed只记录已有授权。机器人在启动前再次校验配置/模型摘要、版本与运动许可。runner_revision现为table-clean-runner-v2，旧版预览不可复用。

完整runner不重启，确认帧自动处理，完成前VLM读最新图并finish。所有错误按真实报告说明，不能把路径规划、运动结束或退出0解释成已经确认放好。取消后继续poll等待真实退出；无法确认停止时报告uncertain。
