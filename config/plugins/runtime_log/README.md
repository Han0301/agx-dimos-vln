# 运行日志

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

## 按任务排查

- 用网页的 `task_id` 找导航任务的原始指令、Agent 目标、阶段和导航终态。
- 用 `observation_id`、`det_id` 关联实际选中的 RGB、深度、里程计、相机标定与检测结果。对照原始判定、复核、框内深度统计和定位坐标。
- 用探索段 ID、`request_stamp_ns` 和计划代次关联短目标、XNavDP 分段耗时、MPC 和 tracker 速度。Stop 或到达时，对照最后活动路径、清路径、发零、底盘转发和里程计估计。底盘的 `Twist` 按同机时间对照，车身静止是基于里程计的估计。
- Bag 的 `run.json` 提供文件、速率、循环、`/clock` 设置和源 bag 的话题消息数。RViz 的 `rviz_clock_selected` 提供实际选定的时钟。

需要故障帧时，在启动记录器时追加 `--evidence failures`。该模式为失败或不确定的检测保存有上限的 JPEG 和模型回复，每个运行最多 32 个文件、总计 16 MiB。系统判为成功的误检不会自动保存原图。两种场景的完整命令见 [VLN 启动与排查 skill](../../../doc/skills/vln-operations/SKILL.md)。

```bash
python3 -m dimos.plugins.runtime_log web_input --latest
```

`--latest` 查询已有日志。模块的帮助、配置检查和 dry-run 不注册运行。终端输出可能含任务结果，分享前检查内容。
