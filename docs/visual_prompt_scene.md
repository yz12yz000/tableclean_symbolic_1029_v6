# 收桌面：单次 YOLOE 视觉提示分割

> 当前默认入口已更新为灰托盘 live_five_v001、碗盘合并及其他物体占用避让；参见 [当前入口状态](current_clear_table_status.md)。以下原 pattern_v001 参数仅是此前版本记录，不是当前默认。


默认流程为：裁剪完整桌面 → 一次 YOLOE 同时分割 bowl/tray → 两类掩膜还原到原图 → 同帧深度和标定定位 → 托盘空位选择 → 原有抓放规划与执行 → 返回后重新观察，连续两帧确认放置和更新占用。

## 当前配置

`src/yolo_vision/config/clear_table_vision.json`：

- `scene_classes: ["bowl", "tray"]` 启用单模型模式。
- `model_path: models/visual_prompt_releases/pattern_v001/model.pt`。
- `inference_roi: [220,300,1000,700]`，保留完整操作区域，裁剪后为 780×400。
- `imgsz: 960`、`confidence: 0.25`、`device: cpu`，保持托盘轮廓细节；按比例缩放并补边。
- `inference_tile_size: 0`，单模型模式拒绝开启分块。

ROI 同时包含待抓物体和整个托盘，需留边。它沿用现有相机画面中的操作范围，不自动检测桌子边缘。换相机位置后需核验 ROI 和标定。

`clear_table_live.json` 中的 `yolo` 保留为托盘几何模块接口，但其模型路径、阈值、输入尺寸和设备必须与视觉配置一致。单模型模式不会再加载第二份托盘模型。两处不一致时，初始化拒绝继续，避免修改了不生效的配置。

运行时用普通 `YOLO` 的分割预测接口读取已固定提示的 YOLOE checkpoint；模型架构仍为 YOLOE。此方式不调用 YOLOE 的文本提示编码，也避免其预测包装器强制启用跨类别 NMS，保留可能重叠的 bowl/tray 实例。

## 提示库

`visual_prompt_library/` 保存 `classes.json`、原始 `images/`、原图坐标的 `annotations/` 和选择样例及 ROI 的 `profiles/`。

当前两个 profile：

- `pattern_table.json`：旧现场花纹托盘和白/绿碗，生成默认 `pattern_v001`。
- `gray_table.json`：用户提供的第二张灰托盘图片，生成候选 `gray_v001`。该图片是参考样例，不能算独立验证集。

第一版每个 profile 使用一张完整参考照片，允许同类多个框，由 YOLOE 原生方式提取该类视觉提示。不自行平均多张照片的特征。稳定类别为 `0=bowl, 1=tray`。这是视觉提示提取，不执行监督训练。

生成工具检查标注尺寸、框是否完整位于 ROI、类别是否齐全、路径是否合法，并重新加载保存的模型检查类别。已有版本目录不覆盖；失败构建不发布半成品。模型、来源校验值、参数和参考图冒烟结果一起保存。

在项目根目录构建新版本（基础权重必须已在本地，不自动下载）：

```bash
./run_table_vision_tool.sh build \
  --profile visual_prompt_library/profiles/gray_table.json \
  --base-model /本地路径/yoloe-26s-seg.pt \
  --output-dir src/yolo_vision/models/visual_prompt_releases/gray_v002
```

构建后检查 `reference_overlay.jpg`、`manifest.json`、`validation.json`，再使用另外的照片验证。新增参考图不会自动更改运行中的模型。更换现场版本时同步修改视觉配置和托盘配置中的模型路径，然后重新构建部署。

## 只用照片检查

```bash
./run_table_vision_tool.sh preview image/你的照片.jpg \
  --output-dir debug_output/新的预览目录

# 对候选模型独立预览，无需修改现场配置：
./run_table_vision_tool.sh preview image/你的照片.jpg \
  --model src/yolo_vision/models/visual_prompt_releases/gray_v001/model.pt \
  --output-dir debug_output/新的灰托盘预览目录
```

可提供多张图片。输出包含原图掩膜 NPZ、完整叠加图、桌面两倍展示图和 `scene.json`。每张图只调用一次网络；耗时包含裁剪、模型预测和掩膜还原，不含绘图保存，第一张包含模型预测器初始化，非严格性能基准。此工具不启动 ROS、相机或机器人，也不把二维图片检测宣称为成功抓放。

## 构建和运行

按 `docs/build/linux_build.md` 加载 ROS 及 MoveIt 环境后：

```bash
colcon build --symlink-install --packages-select yolo_vision rim_locator grasp_executor
./run_clear_table.sh --offline-demo --scenario visual
```

实际设备准备好后，`./run_clear_table.sh` 使用新单模型链路，仅进行在线规划。真实执行仍受 `motion_approved` 和现有规划/反馈约束；Agent 执行还需要匹配本版模型与配置的成功在线预览。旧版预览不能替代新模型预览。

原单碗入口继续使用其独立 `place_config.json`。旧配置未设置 `scene_classes` 时保留原识别接口，便于已有工具和回归测试使用。

## 当前验证边界

结构改造不保证视觉提示能识别所有碗。默认模型在保存的 `initial` 图片检出 2 碗/1 托盘，在 `moved` 和参考 `after_reset` 图片各检出 1 碗/1 托盘，仍存在漏检。灰托盘候选模型在参考图检出 4 碗/1 托盘，必须另外验证遮挡、套叠和不同视角。

新灰托盘的真实长宽、释放高度及相机标定不会从视觉提示自动获得。默认现场几何仍对应旧花纹托盘，不能拿灰色候选权重直接配旧尺寸执行。`motion_approved` 保持 false。本轮仅做软件和记录数据验证，未进行实机抓放。

本次图片预览位于 `debug_output/yoloe_joint_validation_20261022/`。灰托盘 profile 用第二张图作为参考，在三张图都输出 4 个 bowl 和 1 个 tray；这只是模型输出数量，第三张套叠场景仍有碗盘混合、不完整轮廓及漏检，不能当作 4 个正确可抓取实例。

`tools/test_single_scene_ros.py` 将保存的 RGB-D 以新时间戳输入真实视觉节点，在空闲的 localhost ROS domain 内检查两类输出、原图点云和时间戳，不启动驱动或运动节点。它依赖本机保存的 `after_reset/observation.npz` 和当前安装的模型，迁移时若未携带历史数据可跳过。`ros_single_scene_result.json` 保存本次通过结果。
