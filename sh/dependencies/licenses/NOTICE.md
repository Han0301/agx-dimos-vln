# 来源与重构边界

Go2 Ethernet 检查、wired ICE 策略以及设备通信逻辑来源于本机 dimos_simple。保留原代码 Apache-2.0 声明；wired_link.py、wired_webrtc.py 原样复制。连接类仅保留本体数据订阅和显式控制，去除相机、回放、reactivex、LCM 及框架依赖。

camera_ws 复制现有工作空间，第三方源码、SDK 和各自许可证保持原样。仅对复制后的生成环境文件中绝对工作空间路径做迁移；没有修改原工作空间。

未复制原完整 blueprint、模型权重、旧 docker 和 Web。独立 path_tracker 插件从 simple 的 navigation/xnavdp/tracking.py 提取 MPC、MotionPlan 和碰撞检查，保留 Apache-2.0 来源；输入与生命周期适配为 minimum 原生接口。可选 RViz 辅助实现沿用原项目显示逻辑。此版本不是原完整 VLN 功能的替代品。

独立 vlm/llm/vln_task/web_input 插件沿用 dimos_simple 的 Qwen 图像服务、Ollama chat 和 Web 任务输入语义，重新实现轻量客户端、工具循环及任务生命周期。Qwen 服务协议来源为 dimos/models/vl/qwen3_local.py，原项目许可为 Apache-2.0。
