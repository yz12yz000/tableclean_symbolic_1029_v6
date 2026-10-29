---
description: 本机单碗抓放，使用原有坐标表解析目标。
agent: gat-arm
---

处理机械臂任务：$ARGUMENTS

读取本项目配置和本机部署规则，通过arm_task.py创建并校验新计划，由tools/local_runner.py本机运行固定入口。真实运动须有明确授权；目标缺失时先用question澄清。
