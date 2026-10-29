# GAT Local 运行规则

这是机器人电脑上的独立OpenCode项目，原来的SSH/SCP通信已改为本机执行。用户本轮已授权此本地迁移，此规则取代从远程副本继承的“禁止本机ROS”要求。项目内负责语义计划、校验和报告；已登记的机器人脚本在`/home/rob/table_clean_ws`运行ROS和动作。仅允许在那里创建本任务的`.agent_tasks/<UUID>`和调用固定入口。

## 模型与启动

- 使用OpenCode当前登录账号和当前选定/默认模型，Agent不固定model/provider、不另写模型HTTP调用、不读取`.env`。
- 已登录时不重复要求登录；需要选择模型时由用户在OpenCode中处理。不得读取、复制或修改账号令牌。
- `./start.sh`只启动OpenCode，不启动相机、机械臂或模型服务器。
- 保留独立`.git`边界，不修改原gat-arm-minimal及用户全局OpenCode配置；父工作区接口的本次开发授权见下文。

## 单碗 /arm

读取`.opencode/agent/gat-arm.md`、`.opencode/skills/gat-arm-deployment/SKILL.md`、`config/robots/arm.yaml`和`arm_place_zones.yaml`。

坐标只能查表或按已有中点规则解析。目标不明确时用question澄清。使用`generated/arm/arm_task.py`生成新UUID计划、校验来源与hash；它只做纯Python解析，不执行动作。

本机固定入口为`run_pipeline_place_test.sh`（preview）和`run_pipeline_place_execute.sh`（execute）。使用`tools/local_runner.py arm --plan <计划>`先打印校验后的命令，再加`--run`实际启动。execute须已获用户明确授权，添加`--confirmed`仅记录这份授权，不能替用户作出确认。授权在同一会话有效，不重复询问已经授权的相同动作。

等待同一本机进程真正退出，不重复启动；结果保存`reports/arm_last_run.json`。不假定动作退出0就是实际抓牢或放好，不臆造停止接口。

## 收桌面 /table（默认入口）

用户说清空桌子/收纳桌上物品时使用gat-table。读取.opencode/agent/gat-table.md及.opencode/skills/gat-table-deployment/SKILL.md。本次更新按修改方法.md实现本机逐轮v2协议。

策略v1不变，Agent每轮实际读取新原图与编号图，从当前choices选object_id和slot_id。具体抓法、全部候选、路径、动作、固定返程和两帧确认由本地symbolic model负责；禁止Agent填写candidate_id、坐标、姿态或任意命令。

tools/table_session.py提供start/poll/submit/cancel；同一任务只启动一次runner。配置、生成物、报告按arm/table及UUID隔离。preview在线完整路径通过才允许新UUID execute，预算与目标不变，使用本次预览原始证据。已有用户授权沿用，不逐碗询问。取消请求不等于停止完成。

禁止更改运动许可或现场阈值绕过失败。动作后不确定、夹持异常、控制错误立即停止，不自动重抓。未实际读入图片不能提交选择。完成前需要completion_review图片复核。只根据原始连续确认报告verified_placed_count，如实保留grasp_verified等字段。

开发本次更新时，用户已授权同时修改父工作区机器人接口以联调；不修改6_GAT_Edu旧副本和全局OpenCode账号配置。
