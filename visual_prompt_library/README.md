# 最简现场视觉提示

> 最新更新：cup与plate均映射为bowl，杯子现在采用与碗相同的碗沿定位、候选规划、抓取及blue放置流程。勺子仍只识别/避让。杯子须通过原有深度几何与路径检查；分类合并不等于已通过实机抓取验证。


当前识别预览按用户指定口径，把盘子也归入 `bowl`。输出四类：

- `bowl`：碗和盘子。
- `tray`：托盘。
- `cup`：杯子。
- `spoon`：勺子。

`live_vision.json` 和 `live_five_vision.json` 中设置 `class_aliases: {"plate": "bowl"}`。继续使用已测试的 `live_five_v001` 视觉提示权重；模型原始类别仍有 plate，适配器在输出时统一归为 bowl，然后去掉近乎相同掩膜的重复标签。不改视觉特征、置信度或坐标，不会把同一盘子重复计数。勺子等较小的重叠物体仍独立输出。

当前摆放对应 4 个 bowl（3 碗＋1 盘）、1 托盘、1 杯子和1勺子。最新验证见 `debug_output/live_containers_20261022/`。这表示按新的任务类别合并，不表示模型学会精确区分碗和盘子。

## 最简使用

先加载 ROS 环境，只采集图片：

```bash
source /opt/ros/humble/setup.bash
./run_table_vision_tool.sh capture --output-dir debug_output/新采集目录 --count 3
```

用视觉提示识别保存的图片：

```bash
./run_table_vision_tool.sh preview debug_output/新采集目录/frame_001.png \
  --vision-config visual_prompt_library/live_vision.json \
  --output-dir debug_output/新的识别结果目录
```

只取当前桌面 `[330,375,930,595]`，避开桌下示教器；输入960，置信度0.25，无分块，每帧一次YOLOE预测。结果包含原图坐标掩膜、叠加图、类别、置信度和耗时。采集不启动驱动，预览不初始化 ROS，不发机械臂/夹爪命令。

## 参考与扩展

- `classes.json` 保留稳定的原始编号，旧 plate/knife/fork 编号不删除，以免历史标注错位。
- `images/` 保存参考原图。
- `annotations/live_containers_20261022.json` 已将参考盘子标为 bowl；包含四个 bowl、一个 tray、一个 cup 和一个 spoon。
- `profiles/live_containers_20261022.json` 用于以后直接构建四类视觉提示模型。它是下一版构建输入，当前运行仍使用上述已测试权重＋类别别名。

以后重新生成四类模型可使用：

```bash
./run_table_vision_tool.sh build \
  --profile visual_prompt_library/profiles/live_containers_20261022.json \
  --base-model /本地路径/yoloe-26s-seg.pt \
  --output-dir src/yolo_vision/models/visual_prompt_releases/live_containers_v001
```

已有版本不覆盖。新物体添加原图和标注，新类别追加编号。工具支持框坐标以及可选 `polygon_xy`；容器可用 `exclude_class_ids` 排除重叠餐具作为参考像素。

## 历史记录与范围

旧五类测试在 `debug_output/live_five_visual_prompts_20261022/`，当时仍要求单独区分 plate，因此报告中记录了“盘子误判为碗”。旧七类数据和 `live_seven_vision.json` 保留；刀叉已不在当前预览输出中。

这里修改的是识别类别。正式收桌面配置仍使用此前的两类模型，运动许可保持 false；杯子、勺子和盘子的实际抓取方式没有因类别合并而自动验证。
