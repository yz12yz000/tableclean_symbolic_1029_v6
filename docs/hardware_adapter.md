# 当前 MoveIt 与硬件接口

主流程使用 MoveIt 和 UR 轨迹控制器。启动步骤见 [当前启动说明](startup.md)，参数及日志见 [完整抓放说明](../New_run/README.md)。

## 数据与动作

完整 place 输入为 `/rim_locator/grasp_candidates` 上的 `std_msgs/String` 版本化 JSON，包含观测身份、采集时间及每个物体的候选列表。单碗运行器校验唯一目标，按原顺序尝试候选；只有完整路径预检通过才执行。

七段轨迹为：接近、抓取下降、抓后抬升、移到放置准备点、放置下降、释放后上退、返回固定起点。夹爪在对应阶段确认开合状态。预览只规划，真实入口重新采集、规划和检查。

MoveIt 使用 `/compute_ik`、`/plan_kinematic_path`、`/compute_cartesian_path` 等接口；轨迹通过 `/execute_trajectory` 交给现有控制器映射，最终使用 `scaled_joint_trajectory_controller`。工具坐标为 `grasp_center`，规划组为 `ur_manipulator`，规划末端为 `tool0`。

工具换算为 `T_base_tool0 = T_base_grasp_center × inverse(T_tool0_grasp_center)`。`grasp_offset` 沿基座轴作用于目标夹持中心，不重复补偿工具长度。放置点使用保存的参考变换，并由现场 TF 核对。

旧 PointStamped/PoseArray 输入和三段抓取入口仍保留兼容用途。旧 FZI 通信模块及模式切换脚本已移至 [历史程序目录](archive/README.md)，当前主流程不调用。

## 当前配置

完整流程基线在 `src/grasp_executor/config/place_config.json`。目标准备点、各点下降距离和返回起点由 `place_zones.json` 覆盖；运行目录中的 `effective_grasp_config.json` 是本轮实际使用值。通用抓取配置 `config.json` 有独立参数，不能混用其数值。

| 参数 | 当前 place 基线 |
| --- | --- |
| 抓取接近高度 | 0.10 m |
| 搬运线速度上限 | 0.15 m/s |
| 下降、抬升线速度上限 | 0.05 m/s |
| 工具角速度上限 | 0.45 rad/s |
| MoveIt 速度 / 加速度比例 | 0.30 / 0.10 |
| 关节速度 / 加速度上限 | 0.90 rad/s / 0.30 rad/s² |
| 夹爪操作超时 / 闭合稳定时间 | 18 s / 3 s |
| 放置下降距离 | 六个示教目标均为 0.122 m |
| 固定起点 | `place_zones.json` 的 `return_home` |

基线 JSON 内旧固定放置的 0.20 m 是兼容多碗路径的值，现行目标模式会覆盖为目标配置；不能据此判断单碗实际下降距离。实际速度还受轨迹、加减速和机器人限速约束。

## 反馈与停止

控制器、External Control、关节新鲜度、持续静止、轨迹、限位和已有碰撞检查继续生效。程序不建立桌面模型或额外夹爪保护盒。

正常位姿反馈使用新鲜 TF。TF 暂时失效时，仅在启动时通过模型一致性核验后，才允许用新鲜测量关节和当前 URDF 的本地 FK 反馈；细节见 [TF 反馈恢复](TF反馈恢复_20260929.md)。

夹爪反馈通过驱动 `/rosout` 中的原始串口包读取，校验 CRC、设备和寄存器信息；打开或闭合未确认则停止。当前张开位置校准为 680±10 原始计数，不确认物体夹持或落座。

`grasp_verified=false`、`placement_verified=false` 表示仍无物体结果传感确认。历史测试通过数和实机记录见 [归档索引](archive/README.md)，不能作为当前全部目标的实机验收证明。
