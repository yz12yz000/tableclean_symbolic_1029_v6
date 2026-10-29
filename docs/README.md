# 抓取工作区说明

当前主流程由三个 ROS 2 Humble Python 包组成，使用 MoveIt 规划。现行完整抓放入口要求桌上只有一只碗，并通过 `--target-zone` 选择左上、左下、中间、右上、右下或 blue 的示教放置准备点；一次抓放后返回固定起始位置并退出，下次启动重新识别。

| 模块 | 当前职责 |
|---|---|
| `yolo_vision` | 同步 RGB、对齐深度与 CameraInfo，一次分割取物区内的碗；保留原始采集时间和观测内物体编号 |
| `rim_locator` | 圆碗径向生成同碗多候选；圆弧覆盖不足时可用局部碗壁备用分支，保留几何诊断和时间戳 |
| `grasp_executor` | 按候选顺序预检含返回起点的完整七段路径，运动前不可行则换候选；执行一只或预览整份快照 |
| `tools/test_live_place_pipeline.py` | 每轮独立话题与节点；现行入口校验单碗、选择目标区，成功后退出；运动前仍支持有限重观测 |

同碗候选与 IK 分支排序沿用原设计。六个准备点保持示教位置，最终释放点在原采集位置下方 2.2 cm，下降距离统一为 12.2 cm；释放后先上退，再返回保存的起始关节位置。当前坐标见 [当前放置点](当前放置点.md)，早期原始记录见 [历史示教点记录](archive/teaching/示教点记录.md)和 [blue 采集记录](records/blue_release_position.json)。

## 使用入口

完整现场顺序见 [New_run 使用说明](../New_run/README.md)。以下命令在工作区根目录运行；`New_run/` 有同名转发入口。

```bash
# 当前无设备时可运行，构建与隔离离线测试
./run_pipeline_place_offline_test.sh

# 以下等机器人与相机接好、准备环境就绪后再运行
./run_pipeline_place_test.sh --target-zone 左上       # 示教后才可预览；不发送动作
./run_pipeline_place_execute.sh --target-zone 左上    # 示教且现场验收后才可真实抓放一次
./run_pipeline_place_test.sh --x -0.237731 --y -0.402783 --z 0.179664  # 当前 blue 最终释放点，只规划
```

六点及返回起点配置在 `src/grasp_executor/config/place_zones.json`，各点原始 `base/tool0` 位姿已按现场固定 TF 换算成 `base_link/grasp_center`。blue 原始采集位置见 [blue 采集记录](records/blue_release_position.json)；当前释放点在该位置下方 2.2 cm，准备点仍保持原值。默认视觉 ROI 覆盖当前操作桌，每次识别恰好一只碗；按现场请求，下次独立启动可继续抓上一放置位置的同一只碗。相机或桌子移动后须重新检查视野。

## 接口与日志

`std_msgs/String` 中的版本化 JSON 是新增分组接口，默认独立调试话题为 `/rim_locator/grasp_candidates`。校验器位于 `src/rim_locator/rim_locator/candidate_protocol.py`，执行器声明对 `rim_locator` 的运行依赖，只共享此纯 Python 协议。

每个物体有候选列表，带 `task_id / observation_id / source_stamp_ns / object_id / candidate_id / geometry`。`object_id` 不作为跨帧身份。旧 `PoseArray` 仍然一只碗对应一个姿态；旧 point/PoseArray 模式和三段抓取入口保留。

完整抓放运行器使用每轮独立 namespace、保留消息和私有预览话题，保存 `task_config.json`、每轮配置/候选/诊断、汇总 `result.json`。结果中有 `target_zone`，本次实际使用的准备位姿、下降量和返回起点保存在 `effective_grasp_config.json`。使用位置及字段解释见 [New_run 使用说明](../New_run/README.md)。

动作命令成功不等于传感器确认抓到或放下，结果明确保留 `grasp_verified=false` 和 `placement_verified=false`。历史失败后的空观测、未解决拒绝与预算耗尽会如实报告，不能当作所有碗均已确认抓完。

## 当前资料与历史记录

以下链接为历史阶段的实施依据和完成记录，不作为当前功能定义：[IMPLEMENT_demo.md](archive/plans/IMPLEMENT_demo.md)，完成记录与测试结果见 [demo_implementation_20260925.md](archive/reports/demo_implementation_20260925.md)。

连接设备后的执行结果、现场 ROI、运行器监督修复和仍未覆盖的情况，见 [2026-09-27 实机验证记录](archive/reports/live_demo_verification_20260927.md)。

本次保留现有机器人 URDF 碰撞和轨迹检查，不增加桌面碰撞模型或执行中恢复；现行定位已包含碗沿圆拟合。夹爪接口按当前控制寄存器及开口位置确认目标状态：已打开不重复发 Open，Close 不要求历史打开状态；未确认目标状态时拒绝继续运动，仍没有抓牢或落座判定。具体接口与验证限制见 `New_run/README.md`。历史硬件文档和早期验证记录集中在 `docs/archive/`；旧文档可能描述单点或旧控制器路径，当前入口和实际代码以上述说明为准。

当前桌面 ROI 为 `[220,300,1000,700]`，分块尺寸 512 像素。依据新采集 RGB/深度调整，验证及边界说明见 [桌面 ROI](桌面ROI_20260928.md)。

## 资料目录

- [当前启动说明](startup.md)、[硬件接口](hardware_adapter.md)、[部署](build/linux_build.md)、[上机检查](build/robot_test_checklist.md)。
- [当前放置点](当前放置点.md)、[当前起始点](当前起始点.md)、[桌面 ROI](桌面ROI_20260928.md)。
- [径向抓取](圆碗径向抓取修改_20260928.md)、[夹爪稳定反馈](夹爪稳定性复查_20260928.md)、[TF 反馈恢复](TF反馈恢复_20260929.md)、[夹爪驱动说明](夹爪说明.md)。
- `records/`：当前配置来源的原始示教及起点数据；`assets/`：手指方向确认图片。
- [历史归档索引与迁移原因](archive/README.md)：旧方案、旧程序、早期记录和原始备份。历史路径可通过归档清单查找。

归档整理未变更运动参数。当前配置以源码 JSON 为准，本轮实际值以运行目录快照为准。
