# 上层插件契约

上层插件通过配置组装，保持相机与本体接口稳定。VLN preview 接收 D455 或 bag 观测并计算路径和速度预览，实机 profile 由 Web 任务通过 VLM 插件刷新世界坐标目标，并根据新观测持续更新路径，并把校验通过的结果交给进程内 tracker。

| 能力 | 输入 → 输出 | 边界 |
| --- | --- | --- |
| XNavDP | 原生点目标 → trajectory(Path)、result(JSON) | 服务端直读 RGB-D/odom/标定；客户端不转发传感器、不发 cmd_vel |
| VLM | 指定图像、提示词、推理参数 → 模型文本和请求耗时 | 结构化输出由调用方校验，保留观测时间和来源 |
| LLM | 文本、显式历史、推理参数 → Ollama 助手消息与推理指标 | 工具由任务层执行；Web worker 可在启动时预热 |
| Web 输入 | 文本、可选图像 → 任务 ID、状态与结果 | 单 worker、有界队列和取消 |
| VLN 主 Agent | 用户任务、观测、工具结果 → 回复、目标或路径预览 | 管理每任务历史、工具调用和期限；设备 SDK 由基座持有 |
| Active search | 未见目标、RGB-D/odom、实机 LiDAR → 探索进度与经复核的候选 | 首帧由 VLN task 的 VLM 分支检查。实机探索点走现有 XNavDP，回放使用录制视角 |
| Path tracker | result、odom、原始深度、可选障碍输入 → 速度 | 默认仅终端预览；显式配置后只发布原 /cmd_vel；不取得 Go2 连接所有权 |
| 运行日志 | 已登记进程的终端、ROS 日志及结构化事件 → 按会话归档 | 独立启停，通过 `task_id` 和 `request_stamp_ns` 查调用链 |

## 每个插件必须说明

1. 真实可用的入口、配置、依赖和服务所有者；不把占位配置写成能力。
2. 原生输入输出、frame、单位、时间戳与请求关联，不先造统一 TaskGoal 总线。
3. 超时、取消、错误和迟到结果处理；限制并发和排队。
4. 会话历史与重试副作用；不得自动 reset、重启共享服务。
5. start/stop 只管理自己的资源；插件失败不应清理基础通信。
6. health、模拟协议、真实推理和运动执行分别验收。

当前 XNavDP 保留整数纳秒请求关联，路径是 base_link 局部空间路径，不是可直接执行的时序轨迹。VLM 支持 DashScope 直连和 image-inference-base64 HTTP，LLM 使用 Ollama chat 协议。vln_task 持有每任务历史。独立模式注册 inspect_image；VLN profile 额外提供可见物体定位、多视角搜索与路径预览。预览使用已校验路径做一次 MPC 计算，实机跟踪使用连续校验的任务路径。Web 提交返回任务 ID，以实际执行结果更新状态。取消后丢弃迟到回复，已经提交的远端推理仍可能执行到返回或超时。单 worker 和有界队列限制并发，终态记录在容量达到上限时淘汰。目录规则见 [plugins.md](plugins.md)，已实现参数见 [插件索引](../../config/plugins/README.md)。
