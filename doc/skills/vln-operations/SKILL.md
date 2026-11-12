---
name: vln-operations
description: 按 DimOS Mininum 的入口装配和验收 VLN，适用于 Web Agent 的 Go2 实机运行、RGB-D bag 回灌、RViz 路径显示和独立运行日志排查。
---

# VLN 启动与排查

在项目根目录执行。依赖准备见 [安装 skill](../dimos-install/SKILL.md)，独立调用和接口见 [插件索引](../../../config/plugins/README.md)。按用户要求选择实机、回放或只做检查。启动实机运动、真实推理与共享模型 reset 分别遵循当前任务授权。

VLN Task 的唯一上层实现位于 `dimos/vln_task/`；搜索工作流属于其 `search/`，基础插件提供输入输出。现有启动命令、配置字段与运动保护沿用原契约。

## 实机 Go2

按 [README 的实机终端表](../../../README.md#实机-go2) 启动。采集时先开日志终端。Go2、D455、Web 与 tracker、RViz 各用其终端。Web 进程按 [VLM 配置](../../../config/plugins/vlm/README.md) 直接连接 DashScope。

`vln_motion` 在 Web 进程内运行 tracker。找物先让 VLM 在当前 D455 画面按 Simple 格式返回候选框或 `None`。有框时用深度定位。首帧无框时以首帧为 0°，按 60°、120°、180°、240°、300° 原地扫视，有框但深度无效时优先沿候选框方向换视角。实机探索要求新鲜 D455 RGB-D 和里程计。Go2 的 `/lidar` 体素点云用于生成 `/global_costmap`，实机 tracker 用代价地图和 D455 深度检查碰撞。`base_motion.yaml` 关闭 Go2 本体避障。到探索视角后先清路径发零速，再用新帧检测。无框时继续剩余朝向，扫完后报告结果。候选框定位成功后进入接近。探索和接近都按新观测请求 XNavDP，校验通过的路径进入跟踪。接近期间，后台每隔至少 0.5 秒用 VLM 插件重检，候选框在同一世界坐标系中定位成功时刷新接近点。无框或定位失败时沿用最近的有效目标。默认按 0.5 秒间隔尝试更新，每次最多等待 3 秒，任务总期限以任务配置为准。控制用原始深度、里程计或代价地图过期时清除计划并输出零速，恢复后等待新路径。到达或网页 Stop 时清除计划并输出零速。`tracker_motion.yaml` 开启 `/cmd_vel` 发布，`base_motion.yaml` 开启 Go2 运动接收。开始前确认相机、里程计和控制链就绪，避免同时启动会提交目标的独立 XNavDP 客户端。

## 自由探索测试

实机 Web/tracker 默认启用记忆探索。使用以下命令启动 Web/tracker 和带探索显示的 RViz。先以 `--check` 检查运动配置，RViz 用 `--dry-run` 检查启动参数。

```bash
bash sh/start_vln_motion.sh
bash sh/run_rviz_mem.sh --time-source wall
```

在 Web 提交找物任务。检测帧及位姿交给相邻 `dimos_simple` 的原版 `SpatialMemory` 按其规则采样。首帧未发现时先查历史视角，候选须通过当前地图的净空检查，再经 XNavDP 前往并停稳复检。无可用候选或记忆不可用时继续前沿探索，跳过已完成或冷却中的前沿。可用 `DIMOS_SIMPLE_ROOT` 指定 Simple 检出位置。记忆进程首次使用时加载本地 CLIP 模型。短程航点由 XNavDP 规划，路径经地图净空检查后交给现有 tracker。选点帧新鲜且未提交过时可直接用于首个规划请求。每段前沿的规划与执行最多使用 15 秒，受整项任务的剩余时间约束。到达后由里程计更新唤醒上层，取符合停稳时间边界的观测，检测后更新记忆。确认并定位目标后，连续靠近使用整项任务的剩余时间。没有可达前沿时回查记忆，跳过查询间隔限制。同一轮已经查过记忆时不重复查询。记忆也没有可用视角时结束搜索。

Web 显示查询记忆、前往历史视角、选前沿、规划、跟踪、停稳复检、冷却换点及终止原因。Web 轮询展示最新状态。RViz 核对活动前沿和短程航点。

日志入口仍为 `bash sh/start_log.sh --mode live`，回放用 `--mode bag`。在归档的 `web_input/<本次运行>/modules/active_search.jsonl` 用 `task_id` 和 `leg_id` 找探索阶段、冷却和段终态，用 `observation_id` 对照检测与记忆更新。规划与控制分别查 `vln_task.jsonl`、`path_tracker.jsonl`，通过 `request_stamp_ns` 关联。按 Stop 后检查任务取消、清路径、发零及 RViz 叠加层清理。

## Bag 回灌与路径预览

按 [README 的回灌终端表](../../../README.md#bag-回灌与路径预览) 启动。回放前退出实机 VLN 入口。短 bag 可先等 `llm_warmup.status` 到终态，再开始回放。提交任务时确认已收到目标场景的观测。

使用与录制一致的 D455 安装外参。推理需要压缩 RGB、压缩对齐深度、`/realsense_camera_info` 与 PoseStamped `/odom`。仅有压缩图的 bag 仍可显示原生路径与候选，RViz 的图像和深度点云面板依赖原始图。Preview 依靠录制数据提供新视角，返回一次速度计算结果，不发布 `/cmd_vel`。详见 [数据源与时钟](../../architecture/data-sources.md) 和 [RViz](../../architecture/visualization.md)。

## 网页与验收

- 首次部署用 VLM 的 `--health` 检查本机凭据，再用独立单图请求确认云端推理。 [VLM 检查与请求](../../../config/plugins/vlm/README.md)
- 在运行 Web 服务的 Orin 本机打开 `http://127.0.0.1:5556/`，默认只监听回环地址。提交前可用 `curl -fsS http://127.0.0.1:5556/health` 查看 `llm_warmup.status`。预热期间任务可排队，失败后 worker 继续。VLM 随任务调用，XNavDP 的 ROS 端点在接单前建立。
- 查找请求写清目标外观、相对位置和易混物。分别验收当前视角命中、探索后定位、无目标或不可探索。实机明确未见时按预定朝向原地扫视，命中但深度无效时可发布定向探索点。候选框定位成功后才提交物体接近点。缺少新鲜 LiDAR 或无可达探索点时检查网页终态和零速。回放从 bag 等待新视角。 [主动找物](../../../config/plugins/active_search/README.md)
- 规划成功时，在 RViz 核对 `/xnavdp/trajectory` 与 `/xnavdp/result` 的路径、目标和候选。在 preview 查一次速度结果。实机核对同一目标的请求时间戳持续递增，tracker 是否依次接收校验通过的路径，以及到达或 Stop 后是否归零。配置检查、模型输出、跟踪和实际运动分别验收。

## 独立日志终端

实机终端 3 和 Web 预览终端会显示任务排队、Agent 工具调用、模型调用、规划与跟踪结果。按 `task_id` 串起任务日志，按 `request_stamp_ns` 核对 tracker 的路径与速度状态。成功的 `GET /status` 轮询省略，失败请求仍显示。

在所需模块启动前打开记录器。它按实际运行模块建目录，`live` 和 `bag` 只标注本次记录，Ctrl+C 结束采集后其他终端可继续运行。

默认会话目录是 `data/log/session_*/`。从 `timing.jsonl` 确认模块是否注册。按 `task_id` 对照原始任务、观测配对、DET 和导航终态。用 `request_stamp_ns` 和计划代次找 XNavDP、tracker 与发零事件。同机底盘桥的转发和里程计速度估计按单调时钟对齐。回放检查 `bag_play` 和 RViz 的时钟元数据。

实机和回放的日志会话默认保留失败或不确定检测的原图和模型回复。需要关闭故障证据采集时，在启动记录器时加 `--evidence off`。文件布局、限额及历史查询见 [运行日志](../../../config/plugins/runtime_log/README.md)。

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
