---
name: vln-operations
description: 按 DimOS Mininum 的入口装配和验收 VLN，适用于 Web Agent 的 Go2 实机运行、RGB-D bag 回灌、RViz 路径显示和独立运行日志排查。
---

# VLN 启动与排查

在项目根目录执行。依赖准备见 [安装 skill](../dimos-install/SKILL.md)，独立调用和接口见 [插件索引](../../../config/plugins/README.md)。按用户要求选择实机、回放或只做检查。启动实机运动、真实推理与共享模型 reset 分别遵循当前任务授权。

VLN Task 的唯一上层实现位于 `dimos/vln_task/`；搜索工作流属于其 `search/`，基础插件提供输入输出。现有启动命令、配置字段与运动保护沿用原契约。

## 实机 Go2

按 [README 的实机终端表](../../../README.md#实机-go2) 启动。采集时先开日志终端。Go2、D455、Web 与 tracker、RViz 各用其终端。Web 进程按 [VLM 配置](../../../config/plugins/vlm/README.md) 直接连接 DashScope。

`vln_motion` 在 Web 进程内运行 tracker。找物先让 VLM 在当前 D455 画面返回候选框和匹配判定。命中后定位，有框但判断不确定时做一次视觉复核。首帧明确未见目标时以首帧为 0°，按 60°、120°、180°、240°、300° 原地扫视，匹配但深度无效时优先沿候选框方向换视角。实机探索要求新鲜 D455 RGB-D 和里程计。默认实机 tracker 不订阅 `/lidar`，`base_motion.yaml` 关闭 Go2 本体避障。到探索视角后先清路径发零速，再用新帧检测和复核。明确排除候选或无法确认时继续剩余朝向，扫完后报告结果，确认目标后进入接近。探索和接近都按新观测请求 XNavDP，校验通过的路径进入跟踪。接近期间，后台每隔至少 0.5 秒用 VLM 插件重检，VLM 明确命中且定位与当前目标一致时刷新世界坐标接近点。未命中或候选有歧义时沿用最近的有效目标。默认按 0.5 秒间隔尝试更新，每次最多等待 3 秒，任务总期限以任务配置为准。RGB-D 或里程计断流时清除计划并输出零速，恢复后等待新路径。到达或网页 Stop 时清除计划并输出零速。`tracker_motion.yaml` 开启 `/cmd_vel` 发布，`base_motion.yaml` 开启 Go2 运动接收。开始前确认相机、里程计和控制链就绪，避免同时启动会提交目标的独立 XNavDP 客户端。

## Bag 回灌与路径预览

按 [README 的回灌终端表](../../../README.md#bag-回灌与路径预览) 启动。回放前退出实机 VLN 入口。短 bag 可先等 `llm_warmup.status` 到终态，再开始回放。提交任务时确认已收到目标场景的观测。

使用与录制一致的 D455 安装外参。推理需要压缩 RGB、压缩对齐深度、`/realsense_camera_info` 与 PoseStamped `/odom`。仅有压缩图的 bag 仍可显示原生路径与候选，RViz 的图像和深度点云面板依赖原始图。Preview 依靠录制数据提供新视角，返回一次速度计算结果，不发布 `/cmd_vel`。详见 [数据源与时钟](../../architecture/data-sources.md) 和 [RViz](../../architecture/visualization.md)。

## 网页与验收

- 首次部署用 VLM 的 `--health` 检查本机凭据，再用独立单图请求确认云端推理。 [VLM 检查与请求](../../../config/plugins/vlm/README.md)
- 在运行 Web 服务的 Orin 本机打开 `http://127.0.0.1:5556/`，默认只监听回环地址。提交前可用 `curl -fsS http://127.0.0.1:5556/health` 查看 `llm_warmup.status`。预热期间任务可排队，失败后 worker 继续。VLM 随任务调用，XNavDP 的 ROS 端点在接单前建立。
- 查找请求写清目标外观、相对位置和易混物。分别验收当前视角命中、探索后发现并复核、无目标或不可探索。实机明确未见时按预定朝向原地扫视，命中但深度无效时可发布定向探索点。候选确认命中且深度有效后才提交物体接近点。缺少新鲜 LiDAR 或无可达探索点时检查网页终态和零速。回放从 bag 等待新视角。 [主动找物](../../../config/plugins/active_search/README.md)
- 规划成功时，在 RViz 核对 `/xnavdp/trajectory` 与 `/xnavdp/result` 的路径、目标和候选。在 preview 查一次速度结果。实机核对同一目标的请求时间戳持续递增，tracker 是否依次接收校验通过的路径，以及到达或 Stop 后是否归零。配置检查、模型输出、跟踪和实际运动分别验收。

## 独立日志终端

实机终端 3 和 Web 预览终端会显示任务排队、Agent 工具调用、模型调用、规划与跟踪结果。按 `task_id` 串起任务日志，按 `request_stamp_ns` 核对 tracker 的路径与速度状态。成功的 `GET /status` 轮询省略，失败请求仍显示。

在所需模块启动前打开记录器。它按实际运行模块建目录，`live` 和 `bag` 只标注本次记录，Ctrl+C 结束采集后其他终端可继续运行。

默认会话目录是 `data/log/session_*/`。从 `timing.jsonl` 确认模块是否注册。按 `task_id` 对照原始任务、观测配对、DET 和导航终态。用 `request_stamp_ns` 和计划代次找 XNavDP、tracker 与发零事件。同机底盘桥的转发和里程计速度估计按单调时钟对齐。回放检查 `bag_play` 和 RViz 的时钟元数据。

需要失败时的原图或模型回复，可在日志终端使用 `bash sh/start_log.sh --mode live --evidence failures`，回放将 `live` 换成 `bag`。文件布局、限额及历史查询见 [运行日志](../../../config/plugins/runtime_log/README.md)。

## 只读入口检查

```bash
.venv/bin/python -m dimos.plugins.vlm --check
bash sh/start.sh --config config/base_motion.yaml --check
bash sh/start_vln_motion.sh --config config/profiles/vln_motion.yaml --check
bash sh/start_web_input.sh --profile config/profiles/vln_preview.yaml --check
bash sh/run_rviz.sh --time-source wall --dry-run
bash sh/run_rviz.sh --time-source sim --dry-run
bash sh/play_bag.sh --bag "data/bags/your_recording" --dry-run
bash sh/start_log.sh --help
```
