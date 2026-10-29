# 当前启动顺序与控制模式

在 `/home/rob/table_clean_ws` 根目录运行命令。完整抓放使用 MoveIt OMPL、Cartesian 路径和 `scaled_joint_trajectory_controller`，每次只处理一只碗。参数与结果说明见 [完整抓放说明](../New_run/README.md)。

## 构建

源码或安装文件修改后按 [构建与部署](build/linux_build.md) 构建三个包。根目录启动脚本按脚本自身所在位置确定工作区，`New_run/` 的同名脚本仅转发。

## 硬件准备

终端 A：

```bash
cd /home/rob/table_clean_ws
./run_robot_prepare.sh --check
./run_robot_prepare.sh
```

已有硬件驱动运行时先核对进程，不重复启动。准备脚本启动 UR5e、固定 TF、MoveIt/RViz、夹爪、Gemini2L 和 Mid-360，不启动 Servo 或手柄。底盘驱动仍由外部提供 `odom -> base_footprint`。

默认机器人 IP 为 `192.168.10.102`，夹爪波特率为 115200；串口设备不在或已占用时拒绝启动。可使用：

```bash
GRIPPER_PORT=/dev/serial/by-id/实际夹爪设备 ./run_robot_prepare.sh
START_LIVOX=false LAUNCH_RVIZ=false ./run_robot_prepare.sh
```

`START_GRIPPER`、`START_ORBBEC`、`START_LIVOX`、`LAUNCH_RVIZ` 默认开启。环境工作区可通过 `MOVEIT_WS`、`UR_DRIVER_WS`、`UR_MODEL_WS`、`GRIPPER_WS`、`ORBBEC_WS`、`LIVOX_WS`、`JOYSTICK_WS` 指定。各终端保持一致的 ROS_DOMAIN_ID 和 DDS 设置。

## 临时查看机械臂 / 夹爪工作空间

准备脚本运行后，在另一终端按需启动（不会自动随准备脚本开启）：

```bash
./show_robot_workspace.sh --rviz
```

这会另外打开一个临时 RViz，默认显示两组 **Workspace Boundary**、地面和地面交集：蓝色为法兰 `tool0`，橙色为夹爪抓取点 `grasp_center`，模型来自正在运行的 `/robot_description`。左侧 Displays 有六个独立开关，可随时切换，不需要重启：

| 显示项 | 内容 | 默认 |
| --- | --- | --- |
| Arm - Workspace Boundary | 机械臂法兰外包络线框 | 开 |
| Gripper - Workspace Boundary | 夹爪抓取点外包络线框 | 开 |
| Arm - Original Points | 原来的机械臂采样点云 | 关 |
| Gripper - Original Points | 原来的夹爪采样点云 | 关 |
| Ground Plane (wheel contact) | 按底盘模型估算的接地平面及网格 | 开 |
| Ground - Gripper Section | 地面落在夹爪外包络内的部分，黄色填充、轮廓及面积 | 开 |

可设置初始显示；打开窗口后仍能勾选任意组合：

```bash
./show_robot_workspace.sh --rviz --only arm                 # 仅机械臂边界
./show_robot_workspace.sh --rviz --only gripper             # 仅夹爪边界
./show_robot_workspace.sh --rviz --view points              # 原来的两组点云
./show_robot_workspace.sh --rviz --view both                # 边界和点云一起显示
./show_robot_workspace.sh --rviz --only gripper --view points
./show_robot_workspace.sh --rviz --no-ground --no-ground-section # 暂时隐藏地面相关显示
```

关闭临时窗口或在终端 Ctrl+C 即停止；图形最多 2 秒自动消失。原有硬件和 MoveIt/RViz 继续运行。`--check` 只读取模型并打印采样范围，无需重新构建。

若要叠加到现有 RViz，运行 `./show_robot_workspace.sh`，在 RViz 选择 **Add → By topic**，添加 `/ur5e_workspace/arm_boundary`、`gripper_boundary`、`arm_points`、`gripper_points`、`ground_plane`、`ground_section` 对应的 MarkerArray，可各自开关。兼容话题 `/ur5e_workspace/markers` 显示命令行选中的组合。`--only` / `--view` 控制前四项，地面两项独立控制。

地面按当前底盘 STL 的最低点估算为 `mobile_base_link` 的 `Z=-0.030957 m`，换到机械臂 `base_link` 为 `Z=-0.749957 m`（约低于机械臂基座 0.750 m）。模型将轮子合并在底盘网格里，此高度不是实测四轮接地标定。`--ground-z` 可覆盖地面在 `--ground-frame`（默认 `mobile_base_link`）中的 Z；`--ground-size` 设置显示平面的边长，默认 3 m，不裁剪交集计算。原来位于机械臂基座高度的 Grid 默认关闭，可单独开启。

交集由**当前显示的夹爪凸外包络与地面切平面**直接求交，不用圆或球代替。当前模型、默认 10 万组采样和 seed=5 时，面积约 **1.151 m²**，等面积圆半径约 **0.605 m**；在 `base_link` 中的 XY 包围盒约为 `X=[-0.605, 0.602] m`、`Y=[-0.621, 0.636] m`。实际轮廓不是正圆，终端与黄色标注会按当前模型重新计算面积。填充图形为避免闪烁抬高 3 mm，计算仍使用原地面高度。

该交集包含底盘正下方的部分，未扣除底盘、地面碰撞、夹爪姿态或路径限制，不能解释为地面实际可抓取范围。

**Boundary 是全部采样点的凸外包络线框**：不填充内部，保留最外侧采样点，但会跨过凹陷和内部空洞，线段上的点也不保证可达。因此它用于观察近似外轮廓，不是精确可达域的完整内外边界，也不是碰撞安全边界。

这里的 workspace 是**固定底座、允许末端改变姿态、按 URDF 关节硬/软限位采样得到的位置集合**，不是整个机械臂扫过的体积，也不是避障规划结果。点云和打印的包围盒是有限采样近似，未扣除自碰撞、底盘、桌面、环境障碍，也未读取示教器另外设置的安全平面/关节范围；点云内不保证某个指定姿态可达或存在可执行路径。

当前模型的 `tool0 → grasp_center` 平移为 `(0.033, 0.001089906, 0.166452628)` m，长度约 169.7 mm。每个关节配置下有 `p_grasp = p_tool0 + R_tool0 × offset`：夹爪偏移会随法兰姿态旋转，因此不能把整个 workspace 固定平移，也不能简单把机械臂半径加上夹爪长度当作准确边界。固定姿态时，这个位置偏移才是固定的；加入碰撞和抓取姿态要求后，可用范围会进一步受限。

UR5e 的[厂家标称 reach 为 850 mm](https://www.universal-robots.com/manuals/EN/HTML/SW10_7/Content/prod-usr-man/complianceUR5e/H_g5_sections/appendix_g5/tech_spec_sheet.htm)。此工具按实际模型计算，不用一个 850 mm 实心球代替工作空间。

## External Control 和只读检查

在示教器打开含 External Control 节点的程序并按运行。Enable Robot 和准备脚本不能代替此步骤。

终端 B：

```bash
cd /home/rob/table_clean_ws
./run_grasp_position_control.sh --check
```

检查控制器、MoveIt、机器人运行状态、实时六关节反馈及静止状态。`--check` 不切换控制器、不发送轨迹。真实 place 入口会在获得合法目标后准备位置控制器；也可单独执行 `./run_grasp_position_control.sh` 进行模式切换，此命令不发送抓取轨迹。

## 单碗预览和执行

桌上只放一只碗，关闭其他 YOLO、定位和抓取节点，再独立运行：

```bash
./run_pipeline_place_test.sh --target-zone 左上
```

预览检查完整七段路径，不发送动作。确认结果后再启动一轮真实执行：

```bash
./run_pipeline_place_execute.sh --target-zone 左上
```

目标可选左上、左下、中间、右上、右下、blue；也可用 `--x --y --z` 一起提供最终释放点的 `base/tool0` 坐标（米），两种方式不能混用。

0 只或多只碗不运动；空检测与运动前几何/规划失败支持有限重新观察。默认 ROI 为 `[220,300,1000,700]`，输入为同步 RGB、对齐深度图和 CameraInfo。七段全部规划通过后才允许运动，释放后上退并返回保存的起始关节角。成功后退出，下次命令重新采集和规划。

夹爪需通过当前寄存器及开口稳定反馈确认开合状态；程序仍不判断碗是否抓牢或落座。具体反馈参数见 [完整抓放说明](../New_run/README.md)，当前目标见 [当前放置点](当前放置点.md)。

单独返程使用 `./run_return_home.sh`，详细条件见完整抓放说明。

## 手柄控制与停止

```bash
./run_joystick_control.sh --check
./run_joystick_control.sh
```

手柄模式启动 Servo 并切换到 `forward_position_controller`，与抓取模式互斥；进入抓取模式前在手柄终端 Ctrl+C 退出。三个准备/模式脚本均支持 `--help` 和 `--check`。

停止时先结束抓取或手柄任务，最后结束硬件准备会话。清理只针对本次创建的进程，不使用全局进程清理。

旧 `run_pipeline_test.sh`、`run_pipeline_execute.sh` 和 `run_grasp_executor.sh` 保留三段抓取兼容用途，不包含完整放置和返回起点。旧启动说明及 FZI 控制脚本见 [历史归档](archive/README.md)。
