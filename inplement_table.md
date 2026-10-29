# table_clean_ws 实施方案

> 实施状态（2026-10-18）：本方案中的首版 Agent 边界、场景快照、候选/槽位枚举、失败组合账本、逐候选托盘补偿、motion approval 和 runner preflight 已落地到 `table_clean_ws`。本地联调同时覆盖了 `6_GAT_Edu` 的策略校验和结果校验；真实 SSH/ROS/机械臂仍需在目标环境验收。

本文把《修改方案.md》与当前仓库的真实实现对齐，给出可以逐步落地的修改边界。目标是让另一台电脑上的 GPT/Agent 能参与任务规划，同时保持感知、几何、MoveIt 和真实机械臂动作由本机的确定性程序控制。现有收桌模式和单碗模式的默认行为保持不变；新增 Agent 接口只接受严格校验的语义引用。

## 结论

《修改方案.md》的核心分工是合理的：Agent 只理解任务并从真实候选中作有限选择，Symbolic/MoveIt 负责测量、候选生成和无动作验证，执行器只执行已经通过验证的单个抓放组合，动作后重新观察确认。这与本仓库已有的安全边界相符。

当前项目并不是从零开始。下面这些能力已经存在，应当保留并作为新功能的基础：

- `run_clear_table.sh` 调用 `tools/test_live_place_pipeline.py --clear-table`，每轮只处理一只托盘外的碗，返回后重新观察。
- `yolo_vision` 在同一帧同步 RGB-D、深度相机信息和采集时刻 TF；可选的托盘 YOLO 分割、深度平面、尺寸、轮廓和方向检查在 `tray_utils.py` 中已经实现。
- `rim_locator` 产生版本化的多抓取候选，`candidate_protocol.py` 已校验任务、观察、时间戳、坐标系、对象 ID 和候选顺序。
- `ClearTableSession` 已处理托盘内外分类、跨帧位置关联、托盘移动、占用和网格空位、放置后的源位置消失、新占用出现、原占用保留以及连续确认。
- `grasp_executor.candidates.run_candidate_group_task` 的 `plan_only` 路径会规划完整抓放和返程而不发运动命令；规划拒绝会在 `command_attempted=false` 时尝试同组的后续抓取候选。
- 真实动作失败、夹持状态不明或反馈不确定时，现有流程会停止；不能用一条动作成功状态代替下一轮视觉确认。
- Agent 策略进入在线 runner 时要求 UUID、`table-runner-v1`、`release:table-clean-v1` 和自校验 plan hash；结果返回 `runner_revision/config_digest/model_digest`，供规划端核对运行环境。

因此首版不应重搭一个新的任务管理器、场景 Topic、槽位服务或碰撞场景。先把现有流程封装成一个受严格校验的 Agent 边界，再补上“外部选择”和“失败组合去重”两个缺口。

## 哪些修改合理，哪些需要拒绝

| 修改想法 | 判断 | 落地方式 |
|---|---|---|
| Agent 选择目标碗、抓取候选、放置候选和顺序 | 合理，但只能选择现有引用 | 只允许返回本轮 `object_id/candidate_id/slot_id`，不接受 Agent 给出的三维坐标、姿态或关节角 |
| Symbolic 每次观察生成多个候选 | 已有大部分能力 | 复用视觉、`rim_locator`、`tray_utils`；增加可枚举的候选槽位视图，不复制几何算法 |
| 增加无动作验证 | 合理，且当前已有隐式实现 | 把 `plan_only + MoveIt prepare_plans` 封装为结构化 `verify` 结果；验证结果不能直接当作可长期复用的轨迹 |
| 验证失败后换抓法、换槽位、换顺序、必要时重观察 | 合理，但当前缺少失败组合记录 | 增加 `(scene_revision, object_id, candidate_id, slot_id)` 的失败账本和分层预算，每次组合改变都重新算补偿并重新验证 |
| 运动后失败自动换候选继续抓 | 不合理 | `command_attempted=true` 后只能停止、保留 `pending/uncertain` 状态并报告；恢复必须先确认机械臂和物体状态 |
| 用旧观察的验证结果执行新观察中的方案 | 不合理 | 旧 `observation_id/source_stamp_ns/scene_revision` 一律拒绝；执行前必须在同一轮重新检查并重新规划 |
| Agent 直接计算放置 XYZ | 不合理 | 目标由 `slot_id` 表示，放置补偿由本地 `tray_release_config`/执行器根据实际候选计算 |
| 视觉报告有碗就认为还有待抓目标 | 不合理 | 继续使用托盘内外和 `unknown` 分类；位置不清不能当作已收纳或空桌面 |
| 用 YOLO 编号跨轮判断同一个碗 | 不合理 | object ID 只在一轮有效；跨轮继续使用物理位置和托盘占用关联 |
| 增加颜色/类别/区域选择 | 有条件合理 | 当前视觉没有可靠颜色字段，首版只能支持 `all bowls` 和已配置的优先级；接入颜色分类器并带置信度后才开放颜色目标，低置信度必须重观察或停止 |
| 把 `gat-edu` 的 `/cmd_vel` 接口用于机械臂 | 不合理 | 移动机器人、移动操作机器人和本机械臂保持独立契约 |
| 为新功能重写 `rim_detection.py`、候选协议、抓取动作和 MoveIt 适配器 | 不合理且风险高 | 只在已有边界上增加选择、验证和状态字段；修改底层算法必须由单独的测量/回归任务证明必要性 |

## 推荐的运行模型

保留现有两种入口，并新增一个可选 Agent 模式：

1. **原有单碗模式**：`--target-zone` 或 `--x --y --z`，行为不变。
2. **本地收桌模式**：`run_clear_table.sh` 默认采用确定性规则，继续支持预览和 `--execute`。这是没有 Agent 或 Agent 不可用时的安全回退。
3. **Agent 收桌模式**：本机仍拥有观察、候选、验证、执行和确认的控制权；Agent 只提交任务策略或本轮有限选择。Agent 离线时不能越过本机规则直接运动。

状态机应明确写成：

```text
START
  -> OBSERVE
  -> SCENE_READY
  -> WAIT_DECISION
  -> VERIFY_NO_MOTION
       失败且未发运动 -> 记录组合 -> 换候选/槽位/顺序或重新观察
       通过 -> REPLAN_FOR_EXECUTION
  -> EXECUTE_ONE
       command_attempted=true 的任何错误 -> STOP_UNCERTAIN
  -> OBSERVE_CONFIRM
       未确认 -> 有限重观察，不能搬下一只
       已确认 -> 更新占用，回到 OBSERVE
  -> COMPLETE / PARTIAL / STOP
```

`VERIFY_NO_MOTION` 可以继续使用现有 MoveIt 的预规划。它检查完整的接近、下降、抬升、搬运、放置、退爪和返回路径，并调用当前的碰撞、IK、工作空间和轨迹采样检查；但它不是连续碰撞安全的数学证明，也不是执行成功证明。`plan_only` 下夹爪 ready 检查会跳过真实夹爪反馈，因此不能声称已经验证抓持能力。验证通过后，`REPLAN_FOR_EXECUTION` 必须在同一场景引用和当前机械臂状态下重新规划，不能把预览轨迹跨进程直接拿来执行。

## Agent 边界协议

不新增 ROS 场景 Topic。现有 `~/status`、候选 Topic 和每轮目录继续供 runner 内部使用；另外增加一个很小的、版本化的 JSON 控制边界（可放在 `tools/table_agent_protocol.py`，或先放入 `place_rounds.py` 的纯 Python 校验函数）。这个文件是“计划协议”，不是第二套感知协议。

### 任务策略

Agent 生成的初始策略只含语义和预算，例如：

```json
{
  "schema_version": 1,
  "plan_id": "uuid",
  "task_type": "clear_table",
  "selector": {"kind": "all_bowls"},
  "priority": {"order": "largest_first", "regions": []},
  "limits": {"max_observation_rounds": 30, "max_replans": 12, "deadline_s": 900},
  "mode": "preview"
}
```

实际从 `6_GAT_Edu` 部署时还必须带 `runner_protocol_version`、`workspace_revision` 和 `model.plan_hash`；runner 会在严格入口校验 UUID、版本和去掉自身 hash 后的 canonical 策略摘要。

`selector` 暂时只支持 `all_bowls`。颜色、类别、保留集合和区域优先级只有在视觉状态中有同一轮、可校验的属性时才可启用。不能让 GPT 看一张图片后自行猜颜色并发出抓取命令。

### 场景快照

runner 在每轮把已有状态和候选汇总为不可变的 `scene.json`，至少包含：

`schema_version, plan_id, task_id, observation_id, source_stamp_ns, scene_revision, config_hash, frame_id, tray, objects, rejected, grasp_options, placement_options, limits`。

`scene_revision` 应由本轮感知状态、候选批次和托盘配置快照计算，例如 SHA-256；它不是物体永久 ID。每个 `object_id`、`candidate_id`、`slot_id` 都只在本轮有效。Agent 可以读取候选的诊断信息，但不能在返回结果中重新提交坐标。

### 本轮决策

```json
{
  "schema_version": 1,
  "plan_id": "uuid",
  "task_id": "uuid",
  "observation_id": "uuid/round_0001",
  "scene_revision": "sha256",
  "selection": {
    "object_id": 3,
    "candidate_id": 1,
    "slot_id": "grid:-0.080000:0.040000"
  },
  "reason": "先放大碗并为小碗保留空间"
}
```

校验器必须拒绝旧任务、旧观察、旧 revision、越权对象、未枚举候选、不可用槽位、重复决策和额外的坐标/命令字段。验证结果要带 `verification_id`、失败阶段和机器可判断的原因（`stale_scene`、`tray_full`、`workspace`、`collision`、`reachability`、`placement_geometry` 等）。

## table_clean_ws 的具体修改

### `tools/place_rounds.py`

- 保留 `ClearTableSession.observe()` 的默认确定性规则，增加 `list_options()` 和 `apply_selection()`：前者返回当前真实可选的目标、抓法和安全槽位，后者只接受引用并再次调用相同的内外分类、占用检查和 `enumerate_safe_slots/evaluate_slot` 约束，再对 `slot_id` 做 exact-match。
- 增加 `TaskPolicy` 的纯 Python 校验，支持 `all_bowls`、大碗优先、原距离顺序和有限重规划；未知 selector 不得静默降级为全部抓取。
- 增加 `failed_options`，键至少包括 scene revision、对象、抓取候选和槽位。还要保存由对象中心/半径量化、候选几何摘要、托盘局部槽位和失败类别组成的 `failure_fingerprint`，否则每个新帧的 hash 变化会让同一物理组合无限重试。计划拒绝只登记，不占用槽位；下一轮重观察后必须重新计算，并禁止在同一状态重复尝试同一组合。
- 把重规划预算分开：`candidate_retries`、`slot_retries`、`order_replans`、`max_reobservations`、总时间。预算耗尽报告“暂未找到可行方案”，不要声称物理上绝对无解。
- 保持 `record_motion()` 只记录“动作返回成功、等待下一轮确认”的 pending 状态；只有连续确认后才增加 `verified_placed_count`。

### `tools/test_live_place_pipeline.py`

- 把当前 `run_clear_table()` 的内部决策拆成 `observe -> scene bundle -> decision -> verify -> execute -> confirm` 六步；不改变现有默认模式。若决策指定单个候选，发送给 executor 的 batch 只能保留该候选；不能让现有候选循环暗中替 Agent 换成另一个姿态。
- 增加 `--task-policy`、`--agent-decision-dir` 或等价的受限入口。runner 等待的是 JSON 文件/固定适配器，不是任意 shell 或 ROS 命令；超时、文件变更、校验失败都进入有限重观察或停止。
- 每轮只生成一次 immutable scene 快照。Agent 决策通过后，仍在同一 runner 中把引用解析为本地坐标，再以执行配置重新规划；禁止从 Agent 接收 `x/y/z`、四元数、MoveIt 轨迹或脚本。
- 预览模式不得启动真实控制器、不得 claim、不得占用槽位。真实模式每次仍只提交一个对象，并保留现有 TaskLock、namespace、时间 barrier、进程清理和结果目录。
- 把验证和执行结果写入已有 round 目录，新增 `scene.json`、`agent_decision.json`、`verification.json`、`execution.json` 即可，不再创建第二套永久日志系统。

### `src/grasp_executor/grasp_executor/candidates.py` 和 `tray_placement.py`

- 提供一个明确的 `verify_candidate_group()` 包装函数，内部仍使用 `run_candidate_group_task(... plan_only=True)` 的完整规划路径，并返回每个候选的 `stage/reason`。不要复制一套“简化验证”。
- 执行路径必须接收已验证的引用和 scene/config hash；执行前重新读取测量关节状态、重新检查规划场景并重新规划。验证 token 只能证明引用没有过期，不能授权复用旧轨迹。
- 修正测量尺寸模式下的候选绑定问题：当前 runner 在 `compensate_bowl_xy` 时把候选列表截成第一个候选，导致后续抓法无法重新计算放置补偿。应把源碗中心、目标槽位和抓取候选交给统一的 placement geometry 函数，在候选循环中为每个候选单独计算 ready/drop，再分别验证。
- 保留 visual-rim 模式已有的 candidate-specific 补偿；补上测试，确保候选 A 被拒绝后候选 B 的 release XY 与 B 的接触姿态重新计算。
- 任何 `command_attempted=true` 的失败都不得由候选循环吞掉；只允许在完整预规划阶段把 `CandidatePlanRejected` 归为可尝试的几何失败。

### `src/yolo_vision/yolo_vision/tray_utils.py`、视觉状态和候选协议

- 增加只读的 `enumerate_safe_slots()`，返回经过托盘内侧、实测轮廓、已有碗、占用和间距检查的槽位引用；现有 `choose_slot()` 作为默认第一选择继续保留。
- 托盘仍只支持一个已标定的矩形 YOLO 分割目标；尺寸不符、遮挡不足、深度平面不可靠、方向变化超过现有阈值时保持 fail-closed。
- 如果后续支持颜色或区域，字段必须绑定本轮 `object_id`，包含 `label/confidence/source`，并进入 schema 校验；当前不能通过 GPT 自由视觉判断替代它。
- `candidate_protocol` v1 继续兼容现有节点。外部决策协议另设版本，不把任意 Agent 字段塞进候选批次，避免破坏现有严格校验。若 Agent 指定单个 `candidate_id/slot_id`，Symbolic 默认只验证该 exact 组合；允许自动 fallback 时，决策必须显式给出有序的 `fallback_order`，每个替代组合都要重新计算和记录验证结果。

### 配置和入口

增加一份 Agent 策略配置或任务目录模板，但不要把现场坐标、托盘尺寸和放置高度复制到 Agent 项目。真实来源仍是 `clear_table_live.json`、`place_config.json`、`place_zones.json` 和当前已验证的 TF/标定。所有快照在任务输出目录中只读保存，并拒绝覆盖源配置。

当前 `clear_table_live.json` 的说明仍要求新碗释放高度和残差做现场验证，因此配置保留 `calibrated=true` 但设置独立的 `motion_approved=false`。在完成托盘模型、TF、尺寸、释放高度和实际落点验收后，现场负责人必须显式把该标志改为 `true`；未批准配置可以回放和 plan-only，不能放行真实运动。

`run_clear_table.sh` 增加薄包装参数即可：默认仍预览，`--execute` 才允许运动；Agent 模式只能调用固定的 `--task-policy`/`--decision` 白名单。不要把 GPT 生成的文字拼接进 shell。

## 失败处理和不变量

以下不变量必须写成测试，而不是只写在提示词中：

1. 没有匹配的 `task_id + observation_id + source_stamp_ns + scene_revision`，绝不规划或执行。
2. 验证阶段没有调用运动、夹爪或 claim journal；验证失败不会占用槽位。
3. 每次换候选、槽位、目标或观察，都重新计算放置补偿和完整路径。
4. 同一组合在同一场景失败后不得重复尝试；新观察也不能直接复用旧 token。
5. 真实动作一旦发出，任何失败都进入 `partial/uncertain`；不得自动抓下一只。
6. 动作返回成功后仍要观察源位置消失、目标槽出现相符新占用、旧占用保留，并达到配置的连续确认帧数。
7. 托盘内的几何拒绝可以不计为源区抓取失败，但 TF、设备、协议、超时和归属不明错误必须保留。
8. 预览不计完成、不写执行 claim、不表示机械臂已到位；plan-only 需要 MoveIt/TF 时，离线演示只能使用模拟接口。

## 实施顺序和验收

1. 先补纯 Python 的决策协议、场景 revision、失败组合账本和槽位枚举，记录并回归现有策略/运行器测试基线数量。
2. 修正 measured-bowl 的逐候选补偿，增加“候选 A 失败后 B 重新算 XY”的测试。
3. 把当前 plan-only 结果封装成结构化 verify 入口，测试验证阶段没有任何运动和夹爪调用。
4. 增加 Agent policy-file 模式，先不做在线逐轮 GPT；用离线 JSON 回放验证对象/候选/槽位选择、过期拒绝和预算耗尽。
5. 在仿真和保存的 RGB-D 回放中验收多碗、托盘移动、已有占用、托盘满、目标不清、候选失败和最终连续确认。
6. 最后做监督实机：先预览，再一只碗，再两只不同尺寸碗，再测试故意的无动作规划失败。不得故意制造真实碰撞；任何真实动作不确定都立即停止。

建议命令（在正确的 ROS/安装环境中执行）为：

```bash
python3 -m pytest -q tools/test_clear_table_policy.py tools/test_visual_clear_table.py \
  tools/test_clear_table_cli.py tools/test_clear_table_runner.py tools/test_place_rounds.py \
  tools/test_table_agent_protocol.py tools/test_table_edu_integration.py
```

当前验证：相关纯策略、协议、运行器、托盘工具、抓取补偿和 6_GAT_Edu 联调测试为 `226 passed`。涉及 `rclpy`、TF、MoveIt、控制器和夹爪反馈的测试仍需在目标 ROS 环境中执行；离线通过不代表实机可运动。

本文件只描述实现方案；当前仓库已有的收桌代码不应因接入 Agent 而默认改变。首版完成后，若 Agent 不可用，`run_clear_table.sh --execute` 仍应能用本地确定性策略安全完成或给出部分完成结果。
