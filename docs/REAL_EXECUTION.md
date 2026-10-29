```bash
# 机器人电脑，终端 A；已有驱动正常运行时跳过启动，不能重复启动
cd /home/rob/table_clean_ws
./run_robot_prepare.sh --check
./run_robot_prepare.sh

# 保持终端 A 运行；在示教器运行含 External Control 节点的程序
```

```bash
# 机器人电脑，终端 B；与终端 A、SSH 登录环境使用相同 ROS_DOMAIN_ID/DDS 配置
cd /home/rob/table_clean_ws
./run_grasp_position_control.sh --check

# 现场检查标定、托盘、释放高度和实际落点；未验收时保留 false
python3 -c 'import json; c=json.load(open("src/yolo_vision/config/clear_table_live.json")); print("calibrated:", c["calibrated"], "motion_approved:", c["motion_approved"])'

# 仅由现场负责人在完成标定验收后修改 motion_approved 为 true；随后重新预览
nano src/yolo_vision/config/clear_table_live.json

# 然后在 Windows 电脑的 OpenCode 输入 /table；网关会自动上传、预览和执行
# 无需在此终端重复运行 run_clear_table.sh
```

```bash
# 任务结束后，在终端 A 按 Ctrl+C 结束本次启动的硬件会话
# 中断 SSH 或结束客户端不等于机械臂已安全停止；异常时使用现场停止流程
```
