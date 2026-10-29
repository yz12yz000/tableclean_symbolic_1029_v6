# 逐轮收桌面联调记录

日期：2026-10-28。

最新现场结果：用户已要求停止继续尝试。绿色碗完成一次抓放并回初始位，照片可见其在托盘上；白碗未执行。由于放置后场景不确定，正式确认仍为 verified_placed_count=0，整体任务未完成。详见[现场联调图文汇总](../reports/现场联调汇总_20261028/现场联调汇总.md)及[离线图片报告](../reports/现场联调汇总_20261028/现场联调汇总.html)。

## 已实现

- gat_local_new默认进入gat-table；自然语言清桌面指令使用托盘收纳，/arm保留旧单碗指定区域功能。
- 本机table_session.py提供start/poll/submit/cancel，一个后台worker守候一个完整runner。每次调用立即返回状态，不使用SSH或独立模型API。
- 原图来自分割使用的同一冻结RGB-D，保留各自时间戳与frame。空目标和深度无效场景也保存新图；编号图含物体ID、托盘边界和每物体最多3个合法空位。
- table-round-v2请求绑定图片、scene摘要、身份、阶段和选择。任务策略保持schema v1，runner_revision升级为table-clean-runner-v2；旧版预览不能当作新版本执行基线。
- Agent选择object_id和slot_id，本地symbolic model保留该物体全部尚可用的抓取候选，负责几何和完整路径。第一次候选失败不锁死其他候选。
- 同一个任务维护占用、预算、失败选项及两帧确认。确认帧自动处理，完成前发布completion_review，由Agent看新图提交finish/reobserve/stop。
- 每轮检查固定示教初始位、静止和反馈时效；等待期间移动、过期选择、图片篡改、配置变化或身份不符均拒绝。选择文件原子发布，重复提交不会重复执行。
- 跨帧关联使用observation_match_tolerance，默认25 mm；放置预留仍保留现场5 mm，不修改运动参数或运动许可。

## 验证结果

机器人侧相关pytest：282项通过。gat_local_new unittest：26项通过。Python语法检查及git diff --check通过。

新增用例包括：同帧图片（含RGB/深度不同时间戳）、空场景、深度无效、摘要改变、错误物体/空位、旧版本/旧轮次、重复和冲突提交、保留多个抓取候选、固定初始位反馈异常、配置变化、等待超时、取消、完成阶段拒绝假完成，以及真实runner配模拟ROS/执行器的三物体完整闭环。三物体用例每只均两帧确认，确认帧不请求Agent，最后由Agent finish。

最初的真实 OpenCode 在线 preview 记录（后续进展见文首现场报告）：

- plan_id：8d3ba515-a337-4d23-aa3c-028b32e9829c。
- 结果：[原始结果](../reports/table/8d3ba515-a337-4d23-aa3c-028b32e9829c/result.json)。
- 实际状态：failed，robot_not_stationary_at_home；没有发送机器人动作，也尚未完成在线完整路径预览。
- 只读关节检查确认机器人静止，但明显不在place_zones.json所记录的固定初始位，见[检查记录](../reports/home_readonly_check_20261028.json)。没有放宽阈值或自动修改home。

另外独立采集了当前相机图片，通过OpenCode read输入模型，模型正确描述两只碗、左侧托盘及遮挡托盘的机械臂。见[图片输入记录](../reports/camera_read_test_20261028_system/vlm_read_result.json)和[相机原图](../reports/camera_read_test_20261028_system/frame_001.png)。这证明此次图片输入能够工作，不代表运动路径或抓放已经成功。两次headless OpenCode调用均产生了相应文件，但CLI退出码为1且未输出终端文本，因此保留文件证据，不将CLI本身记作完整成功。

## 现场剩余工作

后续已通过完整路径预览，并完成绿色碗一次真实抓放。已修复耗时计算后的初始位反馈刷新，以及夹爪运行地址配置；v2 针对性测试 22 项通过。白碗后续预览因 object_position_uncertain、source_outside_detection_view，以及一次 UR 驱动断连导致的 home_feedback_missing 而停止，没有执行白碗抓放。

目前用户要求停止并汇总，未继续运动。只读检查确认机械臂在初始位静止，没有抓取任务运行。待处理：近边缘物体的放置确认、托盘及碗口几何稳定性、放置误差预算、UR 断连根因和关键节点健康监控。两只碗连续实机验收尚未完成，不宣称桌面已清空。

用户原先打开的OpenCode终端cwd为/home/rob/table_clean_ws/6_GAT_Edu/gat_local。该旧会话未修改；请在/home/rob/table_clean_ws/gat_local_new运行./start.sh使用本次更新。旧6_GAT_Edu部署仍保留原版本，遇到新版runner证据会拒绝版本不匹配。

杯/盘仍须通过当前bowl操作的几何与路径检查；勺子仅避让。此更新不增加夹持传感器，也不保证消除漏检、IK不可达、托盘遮挡或实际抓持失败。结果始终保留真实grasp_verified和verified_placed_count。
