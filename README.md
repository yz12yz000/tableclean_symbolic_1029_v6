# 机器人抓放工作区

原单碗入口每次处理一只碗，完成抓取、放置、释放、上退和返回固定起点后退出。目标支持左上、左下、中间、右上、右下、blue，或完整的 XYZ 释放坐标。

新增 `run_clear_table.sh` 按 [implement_new2.md](implement_new2.md) 接入收桌面模式：每轮只抓托盘外一只碗，沿用原抓放动作，返回后重新观察；托盘内已有碗用于占用判断，选择其他空位继续放置。托盘位置可在两轮之间改变。

收桌面默认使用当前灰托盘的 `live_five_v001` YOLOE 固定视觉提示模型。裁剪桌面 ROI `[330,375,930,595]` 后一次识别碗/盘、托盘、杯子和勺子，输入960、阈值0.25、不分块，掩膜还原到原图后与同帧深度对齐。`plate/cup`按当前配置统一输出为`bowl`，通过原有几何与路径检查后才可抓取；勺子只参与占用避让。当前托盘内侧40×28 cm、外侧42×30 cm，尺寸由用户现场确认。碗的中心和占用范围来自点云拟合；大碗优先，空位搜索，每个原始抓取候选单独计算放置XY，连续两帧确认放好后继续。提示库与实际预览限制见 [当前入口状态](docs/current_clear_table_status.md)。

```bash
cd /home/rob/table_clean_ws
./run_clear_table.sh                   # 相机观察和路径预览，不运动
./run_clear_table.sh --execute         # 逐只收纳；满盘或确认失败时停止
./run_clear_table.sh --offline-demo --scenario visual  # 无设备软件演示
```

在线命令沿用已启动的相机、机械臂、TF、MoveIt 和 External Control。工作区来自本机固定相机标定，`clear_table_tray.json` 保留为示例。释放高度和姿态继续使用blue示教值，运动许可已按用户授权开启。放置误差余量于2026-10-27按用户要求改为5 mm，深度XY拟合阈值同步为5 mm；为后续物体留位后，优先靠边并在同等条件下靠角落。边缘间距与物体间隙仍各5 mm。历史落点偏差可达15～25 mm，减小预留不表示实机精度已经提高，需要重新预览和核验落点。

软件演示使用合成观察和模拟执行接口，不启动硬件。新流程的自动化测试和保存的实拍点云回放不能代替新版本实机验收。使用方式见 [收桌面模式](New_run/README.md#收桌面模式)。

- [启动顺序与控制模式](docs/startup.md)
- [完整抓放参数与日志](New_run/README.md)
- [模块、接口与当前资料索引](docs/README.md)
- [构建与部署](docs/build/linux_build.md)
- [上机检查](docs/build/robot_test_checklist.md)
- [历史资料与归档清单](docs/archive/README.md)

在项目根目录运行：

```bash
./run_pipeline_place_test.sh --target-zone 左上
# 完成现场检查后，独立运行一次真实抓放：
./run_pipeline_place_execute.sh --target-zone 左上
```

根目录脚本是实际入口，`New_run/` 中的同名脚本转发到根目录。现行代码在 `src/` 和 `tools/`；重复脚本、旧控制后端和旧方案已集中归档。历史记录不代表当前参数或已完成实机验收。

逐轮视觉Agent入口见 [gat_local_new](gat_local_new/README.md)。默认使用gat-table：Agent每轮读图片，选物体和托盘空位；本地symbolic model规划抓取候选和完整路径。新协议table-round-v2保留单任务runner、固定初始位检查、两帧确认及结束前视觉复核；旧的单碗区域入口保留为显式`/arm`。
