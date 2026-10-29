# 历史归档与迁移清单

整理日期：2026-10-03。共迁移 95 个文件；没有删除历史内容。当前操作见 [启动说明](../startup.md)，当前代码在工作区 `src/` 和 `tools/`。

| 目录 | 内容 |
| --- | --- |
| `programs/` | 重复入口、旧 FZI 后端及旧硬件启动脚本；不属于运行或测试路径 |
| `plans/` | 各阶段 IMPLEMENT 方案 |
| `reports/` | 初版设计、阶段诊断、旧部署与启动记录 |
| `teaching/` | 早期示教及已替换的采集和执行记录 |
| `backups/` | 原样保存的配置、文档及代码备份快照 |
| `tf/`、`hardware/`、`source/`、`notes/` | TF 快照、硬件文档、源码压缩包、旧提示词及启动说明 |

当前配置来源移到 `../records/`，手指方向确认图片移到 `../assets/`。当前程序和测试的引用已同步修改。归档程序保留原文，仅用于追溯，不应直接作为现行启动入口。`conftest.py` 排除备份中的历史测试，避免 pytest 收集过时快照。

原始 JSON、图片、脚本、备份及压缩包原样保留；历史 Markdown 增加归档标识并修订相对链接。备份目录中的原始内部路径和 manifest 保持不变，描述的是备份当时的目录。文件校验值见 [manifest.json](manifest.json)。

`debug_output/`、`output/`、`log/` 未整理或迁移。根目录的旧三段入口和 `New_run/` 转发入口保留。

## 文件迁移与原因

| 原路径 | 现路径 | 原因 |
| --- | --- |
| `run_robot_prepare_o.sh` | [docs/archive/programs/run_robot_prepare_o.sh](programs/run_robot_prepare_o.sh) | 重复脚本或当前主流程已停用的旧控制后端；仅作历史参考 |
| `bash/run_robot_prepare.sh` | [docs/archive/programs/run_robot_prepare.sh](programs/run_robot_prepare.sh) | 重复脚本或当前主流程已停用的旧控制后端；仅作历史参考 |
| `bash/run_pipeline_place_execute.sh` | [docs/archive/programs/run_pipeline_place_execute.sh](programs/run_pipeline_place_execute.sh) | 重复脚本或当前主流程已停用的旧控制后端；仅作历史参考 |
| `docs/switch_arm_control_mode.sh` | [docs/archive/programs/switch_arm_control_mode.sh](programs/switch_arm_control_mode.sh) | 重复脚本或当前主流程已停用的旧控制后端；仅作历史参考 |
| `docs/arm_base_joy_claw_lidar_camcolor0917.sh` | [docs/archive/programs/arm_base_joy_claw_lidar_camcolor0917.sh](programs/arm_base_joy_claw_lidar_camcolor0917.sh) | 重复脚本或当前主流程已停用的旧控制后端；仅作历史参考 |
| `src/grasp_executor/grasp_executor/cartesian_ros.py` | [docs/archive/programs/cartesian_ros.py](programs/cartesian_ros.py) | 重复脚本或当前主流程已停用的旧控制后端；仅作历史参考 |
| `IMPLEMENT_pengzhuang.md` | [docs/archive/plans/IMPLEMENT_pengzhuang.md](plans/IMPLEMENT_pengzhuang.md) | 历史实施方案；当前入口、参数和功能以源码及现行说明为准 |
| `IMPLEMENT_zhuawanfangzhi_r.md` | [docs/archive/plans/IMPLEMENT_zhuawanfangzhi_r.md](plans/IMPLEMENT_zhuawanfangzhi_r.md) | 历史实施方案；当前入口、参数和功能以源码及现行说明为准 |
| `IMPLEMENT_debug.md` | [docs/archive/plans/IMPLEMENT_debug.md](plans/IMPLEMENT_debug.md) | 历史实施方案；当前入口、参数和功能以源码及现行说明为准 |
| `IMPLEMENT_demo.md` | [docs/archive/plans/IMPLEMENT_demo.md](plans/IMPLEMENT_demo.md) | 历史实施方案；当前入口、参数和功能以源码及现行说明为准 |
| `IMPLEMENT.md` | [docs/archive/plans/IMPLEMENT.md](plans/IMPLEMENT.md) | 历史实施方案；当前入口、参数和功能以源码及现行说明为准 |
| `IMPLEMENT_zhuawanfangzhi.md` | [docs/archive/plans/IMPLEMENT_zhuawanfangzhi.md](plans/IMPLEMENT_zhuawanfangzhi.md) | 历史实施方案；当前入口、参数和功能以源码及现行说明为准 |
| `gpt_command.txt` | [docs/archive/notes/gpt_command.txt](notes/gpt_command.txt) | 历史提示词或旧启动说明；当前使用 docs/startup.md |
| `run.md` | [docs/archive/notes/run.md](notes/run.md) | 历史提示词或旧启动说明；当前使用 docs/startup.md |
| `frames_2026-09-22_15.16.08.gv` | [docs/archive/tf/frames_2026-09-22_15.16.08.gv](tf/frames_2026-09-22_15.16.08.gv) | 历史 TF 快照，不代表当前运行状态 |
| `frames_2026-09-22_15.16.08.pdf` | [docs/archive/tf/frames_2026-09-22_15.16.08.pdf](tf/frames_2026-09-22_15.16.08.pdf) | 历史 TF 快照，不代表当前运行状态 |
| `frames_2026-09-14_15.25.33.pdf` | [docs/archive/tf/frames_2026-09-14_15.25.33.pdf](tf/frames_2026-09-14_15.25.33.pdf) | 历史 TF 快照，不代表当前运行状态 |
| `src.zip` | [docs/archive/source/src.zip](source/src.zip) | 历史源码压缩包，当前源码位于 src/ |
| `docs/ROS2+UR_v6.docx` | [docs/archive/hardware/ROS2+UR_v6.docx](hardware/ROS2+UR_v6.docx) | 历史硬件参考资料 |
| `docs/automatic_count_20260923.md` | [docs/archive/reports/automatic_count_20260923.md](reports/automatic_count_20260923.md) | 旧版设计、诊断、采集或阶段验证记录，不作为当前操作步骤 |
| `docs/bowl_pipeline_review_20260923.md` | [docs/archive/reports/bowl_pipeline_review_20260923.md](reports/bowl_pipeline_review_20260923.md) | 旧版设计、诊断、采集或阶段验证记录，不作为当前操作步骤 |
| `docs/collision_model_removal_20260923.md` | [docs/archive/reports/collision_model_removal_20260923.md](reports/collision_model_removal_20260923.md) | 旧版设计、诊断、采集或阶段验证记录，不作为当前操作步骤 |
| `docs/debug_implementation_20260923.md` | [docs/archive/reports/debug_implementation_20260923.md](reports/debug_implementation_20260923.md) | 旧版设计、诊断、采集或阶段验证记录，不作为当前操作步骤 |
| `docs/demo_implementation_20260925.md` | [docs/archive/reports/demo_implementation_20260925.md](reports/demo_implementation_20260925.md) | 旧版设计、诊断、采集或阶段验证记录，不作为当前操作步骤 |
| `docs/grasp_executor_第一版设计说明.md` | [docs/archive/reports/grasp_executor_第一版设计说明.md](reports/grasp_executor_第一版设计说明.md) | 旧版设计、诊断、采集或阶段验证记录，不作为当前操作步骤 |
| `docs/live_demo_verification_20260927.md` | [docs/archive/reports/live_demo_verification_20260927.md](reports/live_demo_verification_20260927.md) | 旧版设计、诊断、采集或阶段验证记录，不作为当前操作步骤 |
| `docs/pipeline_validation_20260921.md` | [docs/archive/reports/pipeline_validation_20260921.md](reports/pipeline_validation_20260921.md) | 旧版设计、诊断、采集或阶段验证记录，不作为当前操作步骤 |
| `docs/place_offline_debug_20260923.md` | [docs/archive/reports/place_offline_debug_20260923.md](reports/place_offline_debug_20260923.md) | 旧版设计、诊断、采集或阶段验证记录，不作为当前操作步骤 |
| `docs/planning_diagnosis_20260923.md` | [docs/archive/reports/planning_diagnosis_20260923.md](reports/planning_diagnosis_20260923.md) | 旧版设计、诊断、采集或阶段验证记录，不作为当前操作步骤 |
| `docs/rim_locator_第一版设计说明.md` | [docs/archive/reports/rim_locator_第一版设计说明.md](reports/rim_locator_第一版设计说明.md) | 旧版设计、诊断、采集或阶段验证记录，不作为当前操作步骤 |
| `docs/supported_rim_contacts_20260923.md` | [docs/archive/reports/supported_rim_contacts_20260923.md](reports/supported_rim_contacts_20260923.md) | 旧版设计、诊断、采集或阶段验证记录，不作为当前操作步骤 |
| `docs/yolo_vision_第一版设计说明.md` | [docs/archive/reports/yolo_vision_第一版设计说明.md](reports/yolo_vision_第一版设计说明.md) | 旧版设计、诊断、采集或阶段验证记录，不作为当前操作步骤 |
| `docs/五点统一姿态_20260928_163826.md` | [docs/archive/reports/五点统一姿态_20260928_163826.md](reports/五点统一姿态_20260928_163826.md) | 旧版设计、诊断、采集或阶段验证记录，不作为当前操作步骤 |
| `docs/起始点姿态更新_20260928_164623.md` | [docs/archive/reports/起始点姿态更新_20260928_164623.md](reports/起始点姿态更新_20260928_164623.md) | 旧版设计、诊断、采集或阶段验证记录，不作为当前操作步骤 |
| `docs/抓取成功率改进分析_20260925.md` | [docs/archive/reports/抓取成功率改进分析_20260925.md](reports/抓取成功率改进分析_20260925.md) | 旧版设计、诊断、采集或阶段验证记录，不作为当前操作步骤 |
| `docs/0921/startup_scripts.md` | [docs/archive/reports/0921/startup_scripts.md](reports/0921/startup_scripts.md) | 旧版启动和验证记录，操作说明已由 docs/startup.md 替代 |
| `docs/0921/moveit_grasp_test_report.md` | [docs/archive/reports/0921/moveit_grasp_test_report.md](reports/0921/moveit_grasp_test_report.md) | 旧版启动和验证记录，操作说明已由 docs/startup.md 替代 |
| `docs/build/linux_build.md` | [docs/archive/reports/build/linux_build.md](reports/build/linux_build.md) | 旧版部署及上机清单，保留原始版本并重写现行说明 |
| `docs/build/robot_test_checklist.md` | [docs/archive/reports/build/robot_test_checklist.md](reports/build/robot_test_checklist.md) | 旧版部署及上机清单，保留原始版本并重写现行说明 |
| `docs/backups/home_orientation_20260928_164623/place_zones.json` | [docs/archive/backups/home_orientation_20260928_164623/place_zones.json](backups/home_orientation_20260928_164623/place_zones.json) | 原始备份快照，保持内容及内部 manifest 不变 |
| `docs/backups/left_lower_release_20260928_203434/place_zones.json` | [docs/archive/backups/left_lower_release_20260928_203434/place_zones.json](backups/left_lower_release_20260928_203434/place_zones.json) | 原始备份快照，保持内容及内部 manifest 不变 |
| `docs/backups/left_lower_release_20260928_203434/test_place_rounds.py` | [docs/archive/backups/left_lower_release_20260928_203434/test_place_rounds.py](backups/left_lower_release_20260928_203434/test_place_rounds.py) | 原始备份快照，保持内容及内部 manifest 不变 |
| `docs/backups/left_lower_release_20260928_203434/当前放置点.md` | [docs/archive/backups/left_lower_release_20260928_203434/当前放置点.md](backups/left_lower_release_20260928_203434/当前放置点.md) | 原始备份快照，保持内容及内部 manifest 不变 |
| `docs/backups/left_lower_release_20260929_143829/place_zones.json` | [docs/archive/backups/left_lower_release_20260929_143829/place_zones.json](backups/left_lower_release_20260929_143829/place_zones.json) | 原始备份快照，保持内容及内部 manifest 不变 |
| `docs/backups/left_lower_release_20260929_143829/test_place_rounds.py` | [docs/archive/backups/left_lower_release_20260929_143829/test_place_rounds.py](backups/left_lower_release_20260929_143829/test_place_rounds.py) | 原始备份快照，保持内容及内部 manifest 不变 |
| `docs/backups/left_lower_release_20260929_143829/当前放置点.md` | [docs/archive/backups/left_lower_release_20260929_143829/当前放置点.md](backups/left_lower_release_20260929_143829/当前放置点.md) | 原始备份快照，保持内容及内部 manifest 不变 |
| `docs/backups/left_upper_release_20260928_200015/place_zones.json` | [docs/archive/backups/left_upper_release_20260928_200015/place_zones.json](backups/left_upper_release_20260928_200015/place_zones.json) | 原始备份快照，保持内容及内部 manifest 不变 |
| `docs/backups/left_upper_release_20260928_200015/当前放置点.md` | [docs/archive/backups/left_upper_release_20260928_200015/当前放置点.md](backups/left_upper_release_20260928_200015/当前放置点.md) | 原始备份快照，保持内容及内部 manifest 不变 |
| `docs/backups/place_orientation_20260928_163826/place_zones.json` | [docs/archive/backups/place_orientation_20260928_163826/place_zones.json](backups/place_orientation_20260928_163826/place_zones.json) | 原始备份快照，保持内容及内部 manifest 不变 |
| `docs/backups/place_orientation_20260928_163826/示教点记录.md` | [docs/archive/backups/place_orientation_20260928_163826/示教点记录.md](backups/place_orientation_20260928_163826/示教点记录.md) | 原始备份快照，保持内容及内部 manifest 不变 |
| `docs/backups/release_points_before_down_22mm_20260929_143947/New_run/README.md` | [docs/archive/backups/release_points_before_down_22mm_20260929_143947/New_run/README.md](backups/release_points_before_down_22mm_20260929_143947/New_run/README.md) | 原始备份快照，保持内容及内部 manifest 不变 |
| `docs/backups/release_points_before_down_22mm_20260929_143947/docs/README.md` | [docs/archive/backups/release_points_before_down_22mm_20260929_143947/docs/README.md](backups/release_points_before_down_22mm_20260929_143947/docs/README.md) | 原始备份快照，保持内容及内部 manifest 不变 |
| `docs/backups/release_points_before_down_22mm_20260929_143947/docs/当前放置点.md` | [docs/archive/backups/release_points_before_down_22mm_20260929_143947/docs/当前放置点.md](backups/release_points_before_down_22mm_20260929_143947/docs/当前放置点.md) | 原始备份快照，保持内容及内部 manifest 不变 |
| `docs/backups/release_points_before_down_22mm_20260929_143947/run_pipeline_place_execute.sh` | [docs/archive/backups/release_points_before_down_22mm_20260929_143947/run_pipeline_place_execute.sh](backups/release_points_before_down_22mm_20260929_143947/run_pipeline_place_execute.sh) | 原始备份快照，保持内容及内部 manifest 不变 |
| `docs/backups/release_points_before_down_22mm_20260929_143947/run_pipeline_place_test.sh` | [docs/archive/backups/release_points_before_down_22mm_20260929_143947/run_pipeline_place_test.sh](backups/release_points_before_down_22mm_20260929_143947/run_pipeline_place_test.sh) | 原始备份快照，保持内容及内部 manifest 不变 |
| `docs/backups/release_points_before_down_22mm_20260929_143947/src/grasp_executor/config/place_zones.json` | [docs/archive/backups/release_points_before_down_22mm_20260929_143947/src/grasp_executor/config/place_zones.json](backups/release_points_before_down_22mm_20260929_143947/src/grasp_executor/config/place_zones.json) | 原始备份快照，保持内容及内部 manifest 不变 |
| `docs/backups/release_points_before_down_22mm_20260929_143947/tools/place_rounds.py` | [docs/archive/backups/release_points_before_down_22mm_20260929_143947/tools/place_rounds.py](backups/release_points_before_down_22mm_20260929_143947/tools/place_rounds.py) | 原始备份快照，保持内容及内部 manifest 不变 |
| `docs/backups/release_points_before_down_22mm_20260929_143947/tools/test_place_rounds.py` | [docs/archive/backups/release_points_before_down_22mm_20260929_143947/tools/test_place_rounds.py](backups/release_points_before_down_22mm_20260929_143947/tools/test_place_rounds.py) | 原始备份快照，保持内容及内部 manifest 不变 |
| `docs/backups/release_points_current_20260929_143457/README.md` | [docs/archive/backups/release_points_current_20260929_143457/README.md](backups/release_points_current_20260929_143457/README.md) | 原始备份快照，保持内容及内部 manifest 不变 |
| `docs/backups/release_points_current_20260929_143457/docs/blue_release_position.json` | [docs/archive/backups/release_points_current_20260929_143457/docs/blue_release_position.json](backups/release_points_current_20260929_143457/docs/blue_release_position.json) | 原始备份快照，保持内容及内部 manifest 不变 |
| `docs/backups/release_points_current_20260929_143457/docs/current_start_position.json` | [docs/archive/backups/release_points_current_20260929_143457/docs/current_start_position.json](backups/release_points_current_20260929_143457/docs/current_start_position.json) | 原始备份快照，保持内容及内部 manifest 不变 |
| `docs/backups/release_points_current_20260929_143457/docs/teach_capture_center_20260928_153251.json` | [docs/archive/backups/release_points_current_20260929_143457/docs/teach_capture_center_20260928_153251.json](backups/release_points_current_20260929_143457/docs/teach_capture_center_20260928_153251.json) | 原始备份快照，保持内容及内部 manifest 不变 |
| `docs/backups/release_points_current_20260929_143457/docs/teach_capture_left_lower_release_20260928_current.json` | [docs/archive/backups/release_points_current_20260929_143457/docs/teach_capture_left_lower_release_20260928_current.json](backups/release_points_current_20260929_143457/docs/teach_capture_left_lower_release_20260928_current.json) | 原始备份快照，保持内容及内部 manifest 不变 |
| `docs/backups/release_points_current_20260929_143457/docs/teach_capture_left_upper_release_20260928_current.json` | [docs/archive/backups/release_points_current_20260929_143457/docs/teach_capture_left_upper_release_20260928_current.json](backups/release_points_current_20260929_143457/docs/teach_capture_left_upper_release_20260928_current.json) | 原始备份快照，保持内容及内部 manifest 不变 |
| `docs/backups/release_points_current_20260929_143457/docs/teach_capture_right_lower_20260928_191303.json` | [docs/archive/backups/release_points_current_20260929_143457/docs/teach_capture_right_lower_20260928_191303.json](backups/release_points_current_20260929_143457/docs/teach_capture_right_lower_20260928_191303.json) | 原始备份快照，保持内容及内部 manifest 不变 |
| `docs/backups/release_points_current_20260929_143457/docs/teach_capture_right_upper_20260928_171849.json` | [docs/archive/backups/release_points_current_20260929_143457/docs/teach_capture_right_upper_20260928_171849.json](backups/release_points_current_20260929_143457/docs/teach_capture_right_upper_20260928_171849.json) | 原始备份快照，保持内容及内部 manifest 不变 |
| `docs/backups/release_points_current_20260929_143457/docs/当前放置点.md` | [docs/archive/backups/release_points_current_20260929_143457/docs/当前放置点.md](backups/release_points_current_20260929_143457/docs/当前放置点.md) | 原始备份快照，保持内容及内部 manifest 不变 |
| `docs/backups/release_points_current_20260929_143457/manifest.json` | [docs/archive/backups/release_points_current_20260929_143457/manifest.json](backups/release_points_current_20260929_143457/manifest.json) | 原始备份快照，保持内容及内部 manifest 不变 |
| `docs/backups/release_points_current_20260929_143457/src/grasp_executor/config/place_zones.json` | [docs/archive/backups/release_points_current_20260929_143457/src/grasp_executor/config/place_zones.json](backups/release_points_current_20260929_143457/src/grasp_executor/config/place_zones.json) | 原始备份快照，保持内容及内部 manifest 不变 |
| `docs/teach_capture_right_lower_release_20260929_143541.json` | [docs/records/teach_capture_right_lower_release_20260929_143541.json](../records/teach_capture_right_lower_release_20260929_143541.json) | 当前示教及起点来源记录，保持原始采集数据 |
| `docs/start_position_capture_20260928_154558.json` | [docs/archive/teaching/start_position_capture_20260928_154558.json](teaching/start_position_capture_20260928_154558.json) | 已被后续示教替代的采集或执行记录，保持原始数据 |
| `docs/teach_capture_20260928_152557.json` | [docs/archive/teaching/teach_capture_20260928_152557.json](teaching/teach_capture_20260928_152557.json) | 已被后续示教替代的采集或执行记录，保持原始数据 |
| `docs/teach_capture_left_upper_release_20260928_current.json` | [docs/records/teach_capture_left_upper_release_20260928_current.json](../records/teach_capture_left_upper_release_20260928_current.json) | 当前示教及起点来源记录，保持原始采集数据 |
| `docs/home_orientation_execute_20260928_164707.json` | [docs/archive/teaching/home_orientation_execute_20260928_164707.json](teaching/home_orientation_execute_20260928_164707.json) | 已被后续示教替代的采集或执行记录，保持原始数据 |
| `docs/teach_capture_right_upper_20260928_171849.json` | [docs/records/teach_capture_right_upper_20260928_171849.json](../records/teach_capture_right_upper_20260928_171849.json) | 当前示教及起点来源记录，保持原始采集数据 |
| `docs/home_orientation_execute_20260928_164941.json` | [docs/archive/teaching/home_orientation_execute_20260928_164941.json](teaching/home_orientation_execute_20260928_164941.json) | 已被后续示教替代的采集或执行记录，保持原始数据 |
| `docs/home_orientation_execute_20260928_172621.json` | [docs/archive/teaching/home_orientation_execute_20260928_172621.json](teaching/home_orientation_execute_20260928_172621.json) | 已被后续示教替代的采集或执行记录，保持原始数据 |
| `docs/teach_capture_left_lower_release_20260929_143829.json` | [docs/records/teach_capture_left_lower_release_20260929_143829.json](../records/teach_capture_left_lower_release_20260929_143829.json) | 当前示教及起点来源记录，保持原始采集数据 |
| `docs/teach_capture_right_lower_20260928_191303.json` | [docs/archive/teaching/teach_capture_right_lower_20260928_191303.json](teaching/teach_capture_right_lower_20260928_191303.json) | 已被后续示教替代的采集或执行记录，保持原始数据 |
| `docs/teach_capture_left_lower_20260928_153517.json` | [docs/archive/teaching/teach_capture_left_lower_20260928_153517.json](teaching/teach_capture_left_lower_20260928_153517.json) | 已被后续示教替代的采集或执行记录，保持原始数据 |
| `docs/teach_capture_20260928_152456.json` | [docs/archive/teaching/teach_capture_20260928_152456.json](teaching/teach_capture_20260928_152456.json) | 已被后续示教替代的采集或执行记录，保持原始数据 |
| `docs/left_lower_release_update.json` | [docs/archive/teaching/left_lower_release_update.json](teaching/left_lower_release_update.json) | 已被后续示教替代的采集或执行记录，保持原始数据 |
| `docs/teach_capture_left_upper_20260928_153732.json` | [docs/archive/teaching/teach_capture_left_upper_20260928_153732.json](teaching/teach_capture_left_upper_20260928_153732.json) | 已被后续示教替代的采集或执行记录，保持原始数据 |
| `docs/teach_capture_left_lower_release_20260928_current.json` | [docs/archive/teaching/teach_capture_left_lower_release_20260928_current.json](teaching/teach_capture_left_lower_release_20260928_current.json) | 已被后续示教替代的采集或执行记录，保持原始数据 |
| `docs/home_orientation_execute_20260928_172604.json` | [docs/archive/teaching/home_orientation_execute_20260928_172604.json](teaching/home_orientation_execute_20260928_172604.json) | 已被后续示教替代的采集或执行记录，保持原始数据 |
| `docs/teach_capture_center_20260928_153251.json` | [docs/records/teach_capture_center_20260928_153251.json](../records/teach_capture_center_20260928_153251.json) | 当前示教及起点来源记录，保持原始采集数据 |
| `docs/teach_capture_right_upper_20260928_153056.json` | [docs/archive/teaching/teach_capture_right_upper_20260928_153056.json](teaching/teach_capture_right_upper_20260928_153056.json) | 已被后续示教替代的采集或执行记录，保持原始数据 |
| `docs/place_orientation_capture_20260928_163826.json` | [docs/archive/teaching/place_orientation_capture_20260928_163826.json](teaching/place_orientation_capture_20260928_163826.json) | 已被后续示教替代的采集或执行记录，保持原始数据 |
| `docs/home_orientation_update_20260928_164623.json` | [docs/archive/teaching/home_orientation_update_20260928_164623.json](teaching/home_orientation_update_20260928_164623.json) | 已被后续示教替代的采集或执行记录，保持原始数据 |
| `docs/current_start_position.json` | [docs/records/current_start_position.json](../records/current_start_position.json) | 当前示教及起点来源记录，保持原始采集数据 |
| `docs/blue_release_position.json` | [docs/records/blue_release_position.json](../records/blue_release_position.json) | 当前示教及起点来源记录，保持原始采集数据 |
| `docs/teach_capture_left_lower_20260928_172409.json` | [docs/archive/teaching/teach_capture_left_lower_20260928_172409.json](teaching/teach_capture_left_lower_20260928_172409.json) | 已被后续示教替代的采集或执行记录，保持原始数据 |
| `docs/示教点记录.md` | [docs/archive/teaching/示教点记录.md](teaching/示教点记录.md) | 早期示教值与当前六点配置不同，保留为原始记录 |
| `docs/left_finger_positive_y_rviz_20260928.png` | [docs/assets/left_finger_positive_y_rviz_20260928.png](../assets/left_finger_positive_y_rviz_20260928.png) | 当前手指方向与坐标轴确认依据 |
| `docs/left_finger_reference_20260928.jpeg` | [docs/assets/left_finger_reference_20260928.jpeg](../assets/left_finger_reference_20260928.jpeg) | 当前手指方向与坐标轴确认依据 |
