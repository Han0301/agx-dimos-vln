# 运行日志

## 日志解耦准则

- 未启动记录器且未显式配置日志目录时，模块走原有业务路径。日志专用的图像编码与拷贝、轨迹和地图序列化、元数据探测、后台队列及写盘全部跳过。
- 新增诊断在构造日志字段前检查录制开关。启用录制时，诊断通过有界后台队列写入，队列满或写入失败时丢弃诊断并记录健康状态，推理和控制线程继续执行。
- 验收同时覆盖开关两态。关闭时用测试确认日志专用工作未执行，启用时核对事件和附件可关联。涉及高频路径时对照测量调用延迟、CPU、内存及队列丢弃。

在一个独立终端启动记录器，再在各自的终端按原命令启动模块。

在项目根目录执行：

```bash
bash sh/start_log.sh --mode live
```

实机用 `--mode live`，回放用 `--mode bag`。记录器等待随后启动并注册的模块，终端显示生命周期、任务、规划与速度状态。按 Ctrl+C 归档截至本次会话结束时的记录，其他模块可继续运行。远端模型服务的日志由服务端提供，可用请求时间戳对照本机归档。

每次采集写入 `data/log/session_<时间>_<进程号>_<后缀>/`。目录按实际启动的模块划分，常用内容如下。

| 路径 | 内容 |
| --- | --- |
| `timing.jsonl` | 会话模式、已采集模块、各模块的生命周期及耗时，采用北京时间 |
| `<模块>/<本次运行>/run.json` | 进程、主机、ROS domain、RMW 及入口提供的 profile、bag 或 RViz 元数据 |
| `<模块>/<本次运行>/stdout.log`、`stderr.log` | 模块终端输出和错误输出，保持原输出渠道 |
| `<模块>/<本次运行>/ros/` | ROS 节点及 launch 原生日志 |
| `vln_motion/<本次运行>/{stdout,stderr}.log` | 实机终端 3 的 Web 与 tracker 合并输出 |
| `web_input/<本次运行>/modules/*.jsonl` | Agent、LLM、VLM、搜索、XNavDP 和跟踪日志。规划请求按 `task_id` 查找，实机 tracker 在同目录的 `path_tracker.jsonl` |
| `path_tracker/<本次运行>/modules/path_tracker.jsonl` | 独立 tracker 接收路径、MPC 耗时和速度输出状态，按 `request_stamp_ns` 与 XNavDP 关联 |
| `cameras/<本次运行>/d455.log`、`x3.log` | 已选相机组的输出，D455 组也包括随相机启动的压缩器 |
| `body/<本次运行>/modules/robot.jsonl` | 速度转发、watchdog 和运动期间及发零后的低频里程计速度估计 |
| `<模块>/<本次运行>/writer_health.json`、`*.capture.json` | 有丢弃或写入失败时记录计数 |

生命周期探针每 250 ms 检查一次，首次输出和退出采用记录器观察时间。组件的 JSONL 同时保留北京时间、事件发生时的本机 `monotonic_ns`、源消息时间戳和各阶段耗时。跨主机对照用请求时间戳，同机耗时用单调时钟。采集期间先写有界的本机暂存，再由独立进程归档。

诊断事件和终端副本经有界后台队列写入。队列满时丢弃新诊断，存储写入失败时继续运行模块。现有 ROS 日志由原节点管理。归档记录 `writer_health.json` 和控制台副本的丢弃计数，前提是相应文件可写。

## 空间记忆复查

先用原命令启动记录器，再启动带 `--exploration-memory` 的 VLN。归档的 `spatial_memory/<本次运行>/images/` 保存实际入库的 RGB JPEG，子进程诊断写入该运行目录的 `stderr.log`。记忆图片有独立的数量和容量上限，仍可通过 JSONL 看到超过限额后的采样结果。

- 先在 `web_input/<本次运行>/modules/` 按 `task_id` 查 `simple_memory_sample`，确认位姿、观测时间戳、采样结果、跳过原因和 JPEG 路径。采样和查询由 `spatial_memory.jsonl` 记录，上层提前跳过的情况在 `vln_task.jsonl`。
- 用 `memory_lookup_id` 关联 `simple_memory_lookup` 和 `active_search.jsonl` 中的 `memory_candidate_checked`。候选日志记录排名、相似度和未选原因。用 `memory_id` 回查入库时的 JPEG，用 `leg_id` 接着查导航与停稳复检。

## 回放诊断

普通录制会把选中观测的 RGB、压缩深度、视觉检测回包，以及较大的地图和轨迹数据放在模块运行目录的 `diagnostics/`。每个模块运行最多 256 个附件、合计 128 MiB、单件 4 MiB。用 `diagnostic_health.json` 查配额拒绝，用 `writer_health.json` 查后台队列丢弃和写入失败。结构化事件持续记录，附件是否写入以事件路径和归档文件为准。空间记忆 JPEG 与失败证据各用独立额度。

- 感知复现查 `observation_id` 和 `det_id`。观测附带配对时间戳、相机标定、位姿和深度缩放，`detection_result` 记录候选框 `bbox_2d`、相机外参和原始回包附件路径 `vlm_response_artifact`。
- VLM 实际输入查 `modules/vlm.jsonl` 的 `vlm_call_finished`，按 `call_id` 打开 `input_image_artifact`。附件保存 HTTP 转码或 DashScope 最终重压缩后的 JPEG。若 `input_image_archive_status` 为 `not_queued`，查看附件配额和写入状态。
- 探索和避障查 `frontier_selection`、`lidar_costmap_update`、`tracker_costmap_accepted`、`tracker_costmap_rejected` 和 `tracker_collision_veto`。阻挡事件附带来源、命中位置和判定阈值。
- 运动复现按 `request_stamp_ns` 查 XNavDP 轨迹和 `tracker_mpc_ready`。逐条速度查 `tracker_command_sample` 和 `body_command_sample`，用同主机单调时钟对齐；底盘收到的 `Twist` 无任务或请求 ID。
- 运行复查查 `run.json` 的脱敏生效配置、代码提交与工作区状态。Simple 本地 CLIP 文件记录大小、修改时间和首尾采样摘要。Web 任务失败记录脱敏错误内容与调用栈。

## 按任务排查

- 用网页的 `task_id` 找导航任务的原始指令、Agent 目标、阶段和导航终态。
- 用 `observation_id`、`det_id` 关联实际选中的 RGB、深度、里程计、相机标定与检测结果。对照 VLM 原始回包、解析出的候选框、框内深度统计和定位坐标。
- 用探索段 ID、`request_stamp_ns` 和计划代次关联短目标、XNavDP 分段耗时、MPC 和 tracker 速度。Stop 或到达时，对照最后活动路径、清路径、发零、底盘转发和里程计估计。底盘的 `Twist` 按同机时间对照，车身静止是基于里程计的估计。
- Bag 的 `run.json` 提供文件、速率、循环、`/clock` 设置和源 bag 的话题消息数。RViz 的 `rviz_clock_selected` 提供实际选定的时钟。

记录器默认启用 `--evidence failures`，为检测或定位失败保留证据附件。故障证据每个模块运行最多 32 个文件、总计 16 MiB。使用 `--evidence off` 可关闭故障证据采集。普通回放附件仍按 `diagnostics/` 的独立配额归档。

```bash
python3 -m dimos.plugins.runtime_log web_input --latest
```

`--latest` 查询已有日志。模块的帮助、配置检查和 dry-run 不注册运行。终端输出可能含任务结果，分享前检查内容。
