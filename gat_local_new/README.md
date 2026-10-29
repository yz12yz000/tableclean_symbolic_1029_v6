# GAT Local：逐轮视觉收桌面

启动本目录的新项目：

```bash
cd /home/rob/table_clean_ws/gat_local_new
./start.sh
```

默认Agent已改为gat-table。直接输入“把桌上的碗收到旁边托盘”或`/table 把桌上的碗和杯子放入托盘`。只预览可输入`/table 清空桌子，先预览，不要运动`。旧单碗指定区域保留为显式`/arm`入口。

Agent每轮读取同一观察的原图、编号图和场景JSON，选择一个物体和托盘空位。本地symbolic model负责抓取候选、完整路径、一次抓放、固定返程与两帧放置确认；之后Agent看新图继续，结束前再做视觉复核。杯/盘按当前机器人配置映射到bowl操作，必须通过原有几何检查；勺子只参与避让，不支持任意物品收纳。

OpenCode使用当前登录账号和选择的模型，必须支持真正的图片输入，不另写模型API、不读取令牌。已打开的旧目录会话不会自动变成新项目，请在本目录启动新会话。

本机通信使用tools/table_session.py的start/poll/submit/cancel，整次任务共用一个后台runner。逐轮协议table-round-v2；任务策略仍为v1。Agent只提交object_id和slot_id，不提交候选编号或运动坐标。真实执行使用新UUID和本版成功在线preview证据，沿用用户已有任务授权；不逐碗确认。

```bash
python3 tools/table_session.py start --policy generated/table/<UUID>/task_policy.json
python3 tools/table_session.py poll --plan-id <UUID>
python3 tools/table_session.py submit --plan-id <UUID> --decision generated/table/<UUID>/decisions/round_0001.json
python3 tools/table_session.py cancel --plan-id <UUID>
```

poll返回waiting_agent后，用OpenCode read读取request、scene和两张图片，再提交合法选择。completion_review只接受finish/reobserve/stop。取消只是请求；需继续poll等待原runner退出和真实结果。失败不自动重启或重抓，不用退出0代替放置确认。

报告保留在reports/table/<UUID>/，机器人原始各轮证据位于/home/rob/table_clean_ws/.agent_tasks/<UUID>/。reports/table_last_run.json仅为最后摘要。tools/local_runner.py保留旧自动策略兼容入口，新的视觉交互使用table_session.py。

模型、ROS_DOMAIN_ID和DDS配置继承启动终端。start.sh仅启动OpenCode；相机、机械臂、TF、夹爪与MoveIt沿用工作区已有环境。不要重复启动设备。软件协议测试不能代替实机抓放验收。
