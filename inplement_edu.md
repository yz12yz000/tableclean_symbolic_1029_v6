# 6_GAT_Edu 实施方案

本文针对“另一台电脑上的 GPT 做任务规划，再把受控命令发给机械臂执行”的目标，说明 6_GAT_Edu 应怎样改。它不把 GPT 变成机械臂驱动器，也不把 `table_clean_ws` 的感知和 MoveIt 搬到 Agent 项目中。当前 `gat-table` 的策略校验、固定 runner 结果校验和本地离线联调已经完成，但真实 SSH/ROS/机械臂验收仍需在目标环境进行。不要把现有 `/arm` 单碗命令当成收桌面入口。

## 先解决现有定位冲突

6_GAT_Edu 目前同时包含三类互不相同的 Agent：

- `gat-edu` 面向 `192.168.0.12` 的移动小车，只发布 `/cmd_vel`。
- `gat-mm` 面向 `192.168.0.10` 的移动操作机器人，使用另一套相机、雷达和吸盘契约。
- `gat-arm` 面向 `192.168.0.13` 的机械臂，但目前只会把“左上/右侧”等语义映射成固定的 `--x --y --z` 单碗放置命令。

收桌面不能塞进 `gat-edu` 或 `gat-mm`。继续把收桌面命令交给现有 `gat-arm` 也不够，因为它不知道托盘、碗的本轮对象、抓取候选、槽位、观察版本和放置确认。

建议新增独立的 `gat-table`（也可命名为 `gat-clear-table`）Agent 和 `/table` 命令，保留原来的 `gat-edu`、`gat-mm` 和单碗 `gat-arm`。这样不会把 `/cmd_vel`、移动机器人地址、静态放置点和机械臂收桌流程混用。`gat-arm-minimal/` 是另一个嵌套 Git 项目，只能作为单碗 Agent 的参考模板；不要让 OpenCode 在两个目录间共享生成文件或规则。

另外，当前根 `6_GAT_Edu` 的 `AGENTS.md`/`gat-edu.md` 强制本地 Qwen，而用户的新目标是另一台电脑的 GPT 规划。两种模式不能同时宣称：

1. **推荐模式**：外部 GPT 在规划电脑上工作，6_GAT_Edu 的 `gat-table` 是声明式计划网关，不在机器人端调用 GPT；机器人端只运行固定的 table runner。
2. **本地 Qwen 模式**：由 6_GAT_Edu 自己规划，外部电脑只发送自然语言；这不等于“另一台电脑的 GPT 规划”。

本文按第一种模式设计。原有本地 Qwen 规则保留给移动机器人 Agent 或标记为 legacy，不要让新 Agent 读取后又偷偷切换模型。

## 推荐拓扑

```text
规划电脑上的 GPT/OpenCode
        │  task_policy.json / round_decision.json
        ▼
6_GAT_Edu gat-table（解析、schema 校验、确认、固定部署）
        │  scp 新文件 + ssh -tt 固定入口
        ▼
192.168.0.13 上的 table_clean_ws
        │
        ├─ yolo_vision：RGB-D/托盘/碗位置
        ├─ rim_locator：碗沿抓取候选
        ├─ ClearTableSession：区域、占用、槽位和确认
        ├─ MoveIt/grasp_executor：无动作验证和真实抓放
        └─ result/scene/verification：结构化证据
```

GPT 不直接发布 ROS 消息、不直接给关节角或三维坐标、不生成任意 Python/SSH shell。所有坐标、姿态、TF 变换、释放补偿和碰撞检查由 `table_clean_ws` 计算。6_GAT_Edu 只调用固定白名单入口。

目前 `arm.yaml` 写的是 `/home/rob/robot_grasp_0929demo`，而本地仓库名是 `/home/rob/table_clean_ws`；这不是可以让 Agent 自己猜的细节。部署前必须在一份机器人配置中确定真实远端路径、用户、主机、入口和版本，并让 Agent 从该配置读取。下面示例沿用现有历史主机 `rob@192.168.0.13`，实际路径必须以机器人上已安装的工作区为准。

## Agent 生成什么

首版 Agent 只生成高层 `task_policy.json`，不生成动作代码：

```json
{
  "schema_version": 1,
  "plan_id": "uuid",
  "task_type": "clear_table",
  "selector": {"kind": "all_bowls"},
  "priority": {"order": "largest_first", "regions": []},
  "limits": {"max_observation_rounds": 30, "max_replans": 12, "deadline_s": 900},
  "mode": "preview",
  "runner_protocol_version": "table-runner-v1",
  "workspace_revision": "sha256-or-release-id",
  "model": {"provider": "openai", "model": "...", "plan_hash": "sha256"}
}
```

这个文件可以包含目标 selector、优先级和预算，但不能包含 `x/y/z`、四元数、候选姿态、MoveIt 轨迹、ROS topic、用户提供的 shell 或远程路径。`model` 只用于审计，不把密钥或提示词写进去。

颜色、保留对象和区域优先级必须有本地视觉证据：当前 `yolo_vision` 只输出 bowl 类别、mask 和几何信息，没有可验证颜色标签，也没有左右区域配置。首版对“收绿色碗、留下粉色碗”应明确返回“不支持/需要颜色感知”，不能让 GPT 从 object ID 或图片印象猜颜色。以后增加颜色分类器或模型类别时，状态中要带 `label、confidence、source、unknown`，低置信度进入重观察而不是抓取。区域优先级也要由 `base` 坐标系中已标定的 region polygon 定义，边界对象标为 unknown。

如果需要 GPT 在每轮选择具体组合，Agent 只能返回如下引用：

```json
{
  "schema_version": 1,
  "plan_id": "uuid",
  "task_id": "uuid",
  "observation_id": "uuid/round_0001",
  "scene_revision": "sha256",
  "selection": {"object_id": 3, "candidate_id": 1, "slot_id": "grid:-0.080000:0.040000"}
}
```

`object_id`、`candidate_id`、`slot_id` 只对这一轮有效。Agent 不得修改候选数组或自行提交几何数据。table runner 收到后会重新验证，并在本地计算实际放置位置。

## 两种 Agent 运行模式

### 首版：策略文件模式（推荐）

GPT 一次生成策略，table runner 在机器人端每轮自行观察、枚举和验证，按策略执行。这样网络中断不影响已授权的本地确定性循环，Agent 不需要接触相机或 ROS。执行结果明确记录 `planner_mode=offline_policy`。

### 后续：在线逐轮重规划

table runner 每轮生成不可变 `scene.json`，通过固定的文件/回传接口交给规划电脑；GPT 返回严格的 round decision；runner 校验后做无动作验证，再重新规划并执行一只。在线模式必须有：

- `task_id、plan_id、observation_id、source_stamp_ns、scene_revision、config_hash` 的完全匹配；
- Agent 响应超时、网络断开、JSON 非法或版本不一致时的停止/重观察策略；
- 验证 token 的 TTL 和一次性使用；
- 真实动作发出后不再等待 GPT 来决定安全停止。

在线模式应在计划和报告中写明“运行时依赖 GPT”。不能把这种系统描述为已经脱离 GPT 独立运行。

## 6_GAT_Edu 的文件修改

建议新增以下文件，且全部使用 `table` 前缀，避免覆盖移动机器人或旧机械臂文件：

```text
.opencode/agent/gat-table.md
.opencode/commands/table.md
.opencode/skills/gat-table-deployment/SKILL.md
config/robots/table.yaml
reference/table_task_plan.schema.json
generated/table/task_policy.json
generated/table/table_task.py       # 仅打印/校验计划，不驱动 ROS
reports/table_last_run.json
```

同时更新根 `AGENTS.md` 和 `opencode.json` 的规则分区：`gat-edu/gat-mm` 继续使用各自既定模型和机器人契约，`gat-table` 单独允许规划电脑选择的 GPT provider；不要用根级默认模型或旧 Qwen 规则覆盖新 Agent，也不要为了新 Agent 改坏移动机器人入口。

`table_task.py` 只负责读取、打印和静态校验计划；不要像移动机器人 `robot_task.py` 那样生成 `/cmd_vel` 节点，也不要在本地运行 ROS2。真正的动作入口是机器人工作区中的固定 `run_clear_table.sh` 包装器。

当前 `table_task.py` 还可以用 `--result result.json` 校验机器人返回的 `plan_id`、协议版本、工作区版本、runner revision、配置摘要和模型摘要；执行阶段必须同时提供 `--baseline-policy <preview_policy.json>` 和 `--baseline-result <preview_result.json>`；新 execute 策略使用新 UUID，分别核对两份结果所属策略，并检查预览到执行之间的 runner/config/model 是否漂移。runner 会在任何 ROS 或控制器启动前核对预览证据。它只校验结果归属和 preflight 完整性，不把预览结果当作运动成功。

`gat-table.md` 应明确：

1. 读取 table 能力清单和 schema，解析用户目标；不确定的对象、颜色、区域或预算先用 question 询问。
2. 生成 `task_policy.json`，计算 hash，使用 `python -m json.tool` 和纯 Python schema 校验；不产生任意 shell。
3. 预览阶段只调用 table runner 的 plan-only 路径，不能把预览当作实际完成。
4. 真实运动前必须展示任务、远端主机、远端目录、固定命令、模式和限制，并通过 question 获取一次明确确认。
5. 确认后只上传这次新生成的计划到唯一的任务目录，等待指定的 SSH 命令结束，记录真实输出和退出码；失败不自动重复启动。
6. 结果只写 `reports/table_last_run.json`，不覆盖 `generated/robot_task.py` 或 `generated/arm/*`。

## 固定部署入口

table 侧应先实现受控参数，例如：

```text
./run_clear_table.sh --task-policy /path/to/policy.json
./run_clear_table.sh --task-policy /path/to/policy.json --execute
```

无 `--execute` 的命令只能预览；带 `--execute` 的计划必须把 `mode` 设置为 `execute`，并且必须是本次新生成、重新确认的 hash。runner 不信任 JSON 中自报的 `confirmed` 字段，确认只能来自 Agent 的实际 question/交互状态。

如果使用在线模式，再增加固定的任务目录/decision 文件接口；不要接受 `--command`、任意用户路径或拼接后的 shell。Agent 的固定远端命令应是类似下面的形式，其中路径和参数由 `config/robots/table.yaml` 读取，不能写死在提示词里：

```text
ssh -tt rob@192.168.0.13 \
  "bash -lic 'cd /home/rob/table_clean_ws && ./run_clear_table.sh \
  --task-policy /home/rob/table_clean_ws/.agent_tasks/<plan_id>/task_policy.json --execute'"
```

实际部署可使用 `scp` 上传到 `<plan_id>.tmp`，远端校验 hash 后原子改名为 `task_policy.json`，再执行固定 wrapper。plan ID 必须由 UUID 校验器生成，禁止把用户文字当路径。确认前可以只上传并运行 preview；执行阶段必须上传同一次新计划，不能运行机器人端旧文件。

远端 preflight 还要返回并核对 `runner_revision`、`config_digest` 和 `model_digest`；`table_clean_ws` 已在 `result.json` 和 `task_config.json` 中写出这些字段，并将 `policy_hash` 一并记录。与 policy 中的 `workspace_revision/runner_protocol_version` 不匹配就拒绝执行。每次报告记录这些版本，避免只上传策略文件却误跑旧的 `table_clean_ws`。

沿用项目已有的 `scp + ssh -tt` 规则即可。不要使用 `scripts/deploy_and_run.py`、Paramiko、密码写入命令行、机器人端直接请求 GPT API、`/cmd_vel` 或本地 ROS 启动。`gat-arm-minimal` 的单碗固定区域流程也不能作为收桌面入口。

## Agent 的失败语义

Agent 只根据 table runner 返回的结构化结果作下一步选择：

- `verification_failed` 且 `command_attempted=false`：可以换候选、槽位、目标顺序或请求重新观察，但必须消除失败组合并受预算限制。
- `stale_scene`、`tray_full`、`unknown_object`、`perception_uncertain`：重新观察或报告无法继续，不猜坐标。
- `command_attempted=true`、夹持异常、控制错误、停止失败或放置确认不确定：立即停止并报告 `partial/uncertain`，不自动重抓。
- 只有源碗消失、槽位出现相符新占用、已有占用保留且达到连续确认帧数，才报告 `verified_placed`。

Agent 不应根据错误字符串自由推断“肯定安全”。table runner 必须把错误分类、是否发过运动、观察引用和剩余目标写入 `result.json`；Agent 只消费这些字段。

## 必须修正的 6_GAT_Edu 问题

### 秘密和模型配置

`opencode.json` 目前包含形状真实的 Qwen API key，`.env.example` 和嵌套项目的 `.env` 也不应放真实密钥。应立即轮换暴露过的密钥，从仓库和交付目录删除，改为环境变量占位符，并检查历史提交和报告。新 Agent 不把 OpenAI/GPT key、图像或完整 prompt 上传到机器人主机。`config/robots/arm.yaml` 等文件中的 `password_default` 也应删除，统一使用已经配置的 SSH key/agent。

根 `opencode.json` 的顶层 cloud fallback、`gat-edu` 的本地 Qwen 约束和 `gat-arm-minimal` 的 ChatGPT 规则互相冲突。应把模型选择限定在 Agent 所属目录：移动机器人继续使用既定本地模型，`gat-table` 按用户选择的 GPT/OpenCode 连接工作；不要让一个 Agent 在模型不可用时静默切换另一种 provider。

### 旧部署脚本

`scripts/deploy_and_run.py` 使用 Paramiko、旧地址和旧 source 流程，与持久化规则冲突。应标为 legacy 或移出默认权限；新 Agent 只使用固定 `scp` 两文件/计划和 `ssh -tt` 命令。README 中也不要再要求为 table agent 安装 Paramiko。

### 坐标来源

`gat-arm` skill、`arm_place_zones.yaml` 和 table 项目的 `place_zones.json` 已出现数值不一致。收桌面不能读取静态 zone 坐标；它的托盘槽和放置补偿来自当前 scene。单碗 Agent 应保留一个明确的坐标 source-of-truth，并在加载时校验 frame；不要把两套数字复制到新 Agent。计划中的 `workspace_revision` 还必须与远端 runner、配置和模型摘要匹配；不匹配就拒绝执行，避免 GPT 按新协议发给旧程序。

### 标定状态

`clear_table_live.json` 的说明仍说释放高度和新碗范围需要现场验证，但 `calibrated=true` 会让 live runner 放行。执行前必须完成托盘模型、尺寸、TF、释放高度和实际落点验证；在此之前把配置设为 `calibrated=false` 或增加独立的 `motion_approved` 现场标志。未标定配置可以用于回放和 plan-only，不能允许运动。

## 验收清单

在 6_GAT_Edu 侧至少用以下场景做离线测试：

1. 合法 `all_bowls` 计划能生成、hash 稳定、上传到唯一目录并调用固定入口。
2. 含 XYZ、四元数、ROS topic、shell、绝对任意路径或旧 observation 的计划被拒绝。
3. 颜色任务在没有视觉颜色字段时明确拒绝，而不是猜测。
4. preview 结果不计完成、不占槽、不触发真实控制器。
5. 远端返回 stale scene、tray full、partial 和 uncertain 时，Agent 报告真实结果且不自动重启。
6. 网络中断或 GPT 超时不会导致机器人继续执行未知命令。
7. 新生成的计划不会覆盖移动机器人 `generated/robot_task.py`、旧 `generated/arm/*` 或其他任务报告。

真实验收必须按 table 文档的顺序进行：回放/模拟 → plan-only → 单碗监督执行 → 多碗连续确认 → 故意制造“无动作规划失败”并观察有限重规划。不要故意制造真实碰撞或夹持异常。

最终分工应保持简单：6_GAT_Edu 负责语义、计划 schema、用户确认和固定部署；`table_clean_ws` 负责所有真实观测、物理候选、验证、动作和成功确认。只有这样，Agent 的选择错误会被本地校验拦截，网络或模型异常也不会直接变成机械臂动作。
