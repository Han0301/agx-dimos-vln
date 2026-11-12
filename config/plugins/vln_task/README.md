# VLN Task 上层应用

源码为 `dimos/vln_task/`，独立于基础插件。入口为 `python -m dimos.vln_task`，配置为 `vln_task.yaml`，通过显式配置路径组合 LLM 和 VLM。相对配置路径以工作空间根目录解析，支持从外部目录使用绝对入口/配置启动。

```bash
python -m dimos.vln_task --check
python -m dimos.vln_task --goal "描述红色物体的位置" --image path/to/image.jpg
```

每次 run 独立持有 goal、图像与消息历史。LLM 可调用独立模式的工具 inspect_image，工具结果回到同一会话，再由 LLM 回复。纯文字任务直接经 LLM；图像任务由 LLM 选择视觉工具。未知工具、非法参数、空输出或超过 max_steps 均失败。独立模式用于问答、任务理解和图像分析。整合模式通过下文的 profile 加载可见目标定位和路径预览技能。

任务期限覆盖整个工具循环。同步 HTTP 在剩余期限内调用；在调用前后检查取消和过期，取消后丢弃迟到结果。远端请求可能继续执行至响应或 HTTP 超时。结果含 answer、elapsed_ms、tool_calls 及逐步骤响应，便于分别统计 LLM/VLM 耗时。CLI 退出关闭本任务客户端。

真实服务基准使用合成测试图，串行运行每组 1 次预热和 10 次正式采样，保留每次响应、错误及配置。百分位使用成功样本的 nearest rank；预热单独保存。

```bash
python -m dimos.vln_task.benchmark --image path/to/shapes.jpg --output path/to/benchmark.json --samples 10 --warmup 1
```

基准中的颜色和左右位置检查针对红色左图形、蓝色右图形测试图。输入其他图像时，这项预置检查不适用。基准会临时启动随机回环端口的 Web，结束时清理自身进程资源。

## 调用链与兼容

`agent.py` 管理 LLM 工具调用；`find_object.py` 分派首帧结果；`search/` 组织扫视、定向靠近和复检；`perception.py` 负责检测与定位；`preview.py` 组合路径和速度预览；`approach.py` 持续更新规划，`target_refresh.py` 管理接近中的目标刷新。`assembly.py` 装配客户端与跟踪器，`web.py` 提供模型预热及任务状态展示策略。

LLM/VLM 只调用模型，XNavDP 只请求和校验路径，tracker 保留控制及停止保护。任务策略由 VLN Task 组织。`web_input/transport.py` 提供可注入 runner 的通用 HTTP、队列及取消接口。

旧搜索路径 `dimos.plugins.active_search` 转调 `dimos.vln_task.search`。

## 整合路径预览

`config/profiles/vln_preview.yaml` 引用任务、Web、XNavDP、tracker 和本体安装配置。相机或 bag 独立供数。Web 与 CLI 使用同一装配入口。

```bash
python -m dimos.vln_task --profile config/profiles/vln_preview.yaml --check
source sh/env.sh
python -m dimos.vln_task --profile config/profiles/vln_preview.yaml --goal "找当前画面里的白色垃圾桶，预览接近路径"
bash sh/start_web_input.sh --profile config/profiles/vln_preview.yaml
bash sh/run_rviz.sh
```

D455 预览读取现有压缩 JPEG、对齐 PNG 深度、CameraInfo 和 PoseStamped odom。预览、探索建图和记忆写入都使用与 RGB 时间最接近的深度和位姿。配对时间差超过 150 ms 时记录告警并继续。冻结观测后请求归一化二维框，以实测深度和 `body_config` 的相机安装位置及俯仰角计算世界坐标接近点。坐标转换采用 D455 光学轴到本体轴的映射，回放使用与录制安装一致的外参。时间戳来自原消息，等待和缓存使用单调时钟。

装配时先订阅相机输入并建立 XNavDP 目标和结果话题端点，DDS 在等待任务期间完成发现。每次规划仍检查发布者独占、服务状态及原始 RGB 时间戳关联，取消或过期的响应丢弃。结果的 `timing_ms` 分别记录观测等待、DDS 发现与检查、服务就绪、规划往返和校验。

Web worker 启动时按 profile 中的 `llm_preload_timeout_s` 预加载 Ollama 模型。网页可立即打开和提交，队列在预加载结束后开始执行。通过 `/health` 的 `llm_warmup` 查看状态和耗时。预加载失败会记录日志，后续任务继续按正常流程调用模型。省略该设置或置为 `null` 可跳过预加载。

tracker 用通过校验的路径和计算时最新的深度与里程计做一次速度预览。任一数据缺失或接收超过 0.5 秒，返回零速及 `tracking_inputs_stale`。RViz 使用原生路径、候选和目标。该 profile 只计算一次速度预览。实机持续规划入口见 [VLN 启动与排查 skill](../../../doc/skills/vln-operations/SKILL.md#实机-go2)。`locate_visible_object` 输出可见目标的位置，`preview_approach` 继续规划。加 `--locate-only` 只注册定位技能，适合尚未准备好 XNavDP 的回放测试。

找物时，Agent 把完整目标描述交给 `find_object`。检测提示词要求可见时返回单个 `{"name": "...", "bbox": [x1, y1, x2, y2]}`，不可见时返回 `None`。解析只使用 `bbox`，按 Simple 的 Qwen3 路径把 0–1000 坐标换算到图像像素后取深度。找物流程直接用候选框定位。没有框或回复无法解析时进入 [主动找物](../active_search/README.md)。有框但深度无效时，实机优先沿框方向定向靠近。用 `--goal "搜索多个视角里的白色脚踏垃圾桶，找到后预览接近路径"` 调用。网页显示检测、搜索和结束状态。profile 中的 `active_search_config` 引用搜索配置。

启用了 `find_object` 的 profile 下，网页提交文字和参考图后，任务层先让 VLM 从参考图提取外观，再将外观补进目标描述，用单张 D455 帧进行检测。指令中明确指定的属性、位置和排除条件优先。检测返回现场帧上的归一化框，任务层用该帧的深度和位姿定位。图片问答调用 `inspect_image`。

找物指令可指定停距，例如「找到灭火器箱，停在距目标约 0.8 米处」。停距应用于初次定位和接近中的目标刷新，默认值来自 profile 的 `approach_stop_m`。实机 tracker 以里程计原点到估计目标点的水平距离检查停距，到达后发零速并结束该次接近。接近中用定位有效的候选框刷新目标。新物体位置与开始接近时的物体位置在世界坐标系下的 XY 欧氏距离超过 1 米时，丢弃该次刷新。无框、定位失败或刷新被拒绝时沿用上一有效坐标。核查停车时，在终端 3 的 tracker 日志和终端 1 的基座日志分别确认零速发布与转发。
