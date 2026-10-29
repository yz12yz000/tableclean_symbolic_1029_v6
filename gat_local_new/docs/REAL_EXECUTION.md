# 本机启动与运行

机器人设备按照/home/rob/table_clean_ws/docs/startup.md准备。已有硬件驱动运行时不要重复启动。

```bash
cd /home/rob/table_clean_ws/6_GAT_Edu/gat_local
# 如果现场ROS_DOMAIN_ID已设置则沿用；当前shell需与设备会话一致。
./start.sh
```

OpenCode使用当前登录账号和所选模型，无需项目内API密钥。输入：

```text
/table 收拾桌面上的碗，先预览，不要运动
/table 按刚才的任务准备真实执行，核对预览证据
```

Agent生成并校验新策略，在本地运行tools/local_runner.py；preview成功后使用新UUID执行策略及--baseline-policy/--baseline-result。已有明确授权沿用，否则确认后--run --confirmed。无SSH、无SCP。任务一次启动，持续等待真实退出，不重复执行。

结果查看：

```bash
cat reports/table_last_run.json
cat reports/table_last_result.json
```

停止：在任务终端Ctrl+C请求原runner停止，等待真实退出；异常运动使用现场停止/急停，不能仅凭客户端退出断言机器人已安全停止。
