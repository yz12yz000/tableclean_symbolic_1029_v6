# 本地通信迁移调试记录（2026-10-28）

范围：只修改gat_local，保留原gat-arm-minimal。单碗坐标表与收桌面策略schema保持原逻辑；SSH/SCP替换为本机固定脚本子进程、本机文件复制和结果读取。未实现修改方法.md中的逐轮VLM交互。

当前模型：opencode.json没有model/small_model/provider覆盖，各Agent也无独立model字段。认证和模型选择由OpenCode已登录账号/当前选择提供；未读取或修改账号令牌、未调用模型API、未读取.env。默认Agent仍为gat-arm，/table选择gat-table。

新增start.sh和tools/local_runner.py；默认命令只校验和显示argv，--run才启动wrapper，execute额外要求已有用户明确授权并传--confirmed。arm预览使用run_pipeline_place_test.sh，执行使用run_pipeline_place_execute.sh；table继续使用run_clear_table.sh，并保留成功在线preview基线、UUID与模型/配置摘要校验。

执行的检查：

```bash
/usr/bin/python3 -B -m unittest discover -s tests -v
bash -n start.sh
python3 generated/arm/arm_task.py --plan generated/arm/task_plan.json --validate --hash
python3 generated/table/table_task.py --task-policy generated/table/task_policy.json --validate --hash --require-deployment-metadata
```

26项检查通过，覆盖原坐标解析/篡改/来源hash和结果归属，策略校验，固定本机argv，preview使用无运动入口，未确认execute拒绝启动，execute需要匹配在线preview，重复任务拒绝，模拟进程失败保留真实退出码，结果丢失/损坏不报成功，合法结果归档以及启动脚本在项目目录运行。

包含一个真实子进程传输检查：只运行临时Python进程写合成失败结果，不使用ROS、相机、机械臂或夹爪。OpenCode启动脚本检查使用临时假的opencode可执行文件，只输出目录和--help，未调用账号服务。

限制：当前终端PATH找不到opencode，常见本机位置也未发现可执行文件。实际./start.sh会清楚提示该问题并退出，因此本次没有验证OpenCode实际交互/登录会话；在OpenCode可用的终端中运行，或将安装目录加入PATH。未自动安装OpenCode或修改全局配置。未执行任何真实机器人动作。

reports/table_last_run.json为not_run状态，避免把迁移前的历史远程报告当作本次执行结果。examples/历史样本保留，但当前规则明确不能拿它作为运行依据。
