> 当前默认入口已更新为灰托盘 live_five_v001、碗盘合并及其他物体占用避让；参见 [当前入口状态](../../../docs/current_clear_table_status.md)。以下原 pattern_v001 参数仅是此前版本记录，不是当前默认。

收桌面默认使用 `visual_prompt_releases/pattern_v001/model.pt`：一份包含
`bowl` 和 `tray` 的固定视觉提示 YOLOE-26s 权重。构建来源、类别和校验值
见同目录 `manifest.json`，参考图冒烟结果见 `validation.json`。参考图片和
原图坐标标注位于工作区根目录 `visual_prompt_library/`。另附灰托盘候选
`visual_prompt_releases/gray_v001/model.pt`，其参考检测不代表实机验收。
操作说明见 [单模型视觉提示流程](../../../docs/visual_prompt_scene.md)。

原单碗入口继续使用上一级目录中已提交的模型：

```text
../yolo26n-seg_openvino_model/
  *.xml
  *.bin
  metadata.yaml
```

构建时该模型目录会安装到 package share。也可以在 JSON 中将
`model_path` 指向其他外部模型目录；相对路径以已安装的 package share 为基准。
确保模型导出输入尺寸与配置 `imgsz` 匹配，并保留类别元数据。

收桌面模式另外使用一个托盘 YOLO 分割模型，在
`config/clear_table_tray.json` 的 `yolo` 中设置。默认示例为
`model_path: "models/tray-seg.pt"`、`target_class: "tray"`、
`confidence: 0.5`、`imgsz: 640`、`device: "cpu"`。

旧双模型流程的花纹托盘专用权重 `models/yoloe-tray-live.pt` 保留，来源与 SHA256 记录在同名 `.provenance.json`，当前单模型默认配置已不加载它。它是花纹托盘的 YOLOE 视觉参考权重，并未训练成通用托盘检测器。通用示例 `clear_table_tray.json` 的权重仍需另行提供。现有 COCO 碗模型没有 `tray` 类别，
不能靠修改类别名称获得托盘识别能力。需要准备包含目标托盘类别的本地
YOLO 实例分割 `.pt` 权重，或包含 `.xml`、`.bin` 和 `metadata.yaml`
的 OpenVINO 导出目录。模型类别名称应与 `target_class` 一致，模型必须有
分割掩膜输出；仅有检测框的模型不满足托盘几何定位要求。

建议把现场权重放在固定位置并使用绝对路径。相对路径以安装后的
`yolo_vision` package share 为基准，和原碗模型一致；不是相对托盘配置
文件。在线初始化会检查本地模型文件、类别和分割能力，不自动下载缺失模型。

训练或验收图像应包含实际相机视角、不同托盘位置和转角、已有碗的遮挡及
现场光照。标注整个矩形托盘轮廓，并核对分割后的外侧尺寸、有效深度和
可用内侧范围；示例尺寸与纯软件演示不能代替这些检查。
