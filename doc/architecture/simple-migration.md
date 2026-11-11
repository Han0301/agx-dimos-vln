# 从 simple 到最小版

## 已保留

- Go2 SDK、有线 ICE 与原生 ROS 本体数据接口。
- 独立 camera_ws、C++ RGB-D 压缩和相机原生标定。
- 独立 path_tracker 提取 simple MPC、减速和碰撞算法，默认仅输出速度预览。
- XNavDP 的 DDS 7411 配置、原生目标及 trajectory/result 协议与独立请求客户端。

## 已拆除或不进入默认启动

完整 Agent/MCP、空间记忆、建图、Frontier 和人物跟随暂未进入最小版。Web、MPC 与 RViz 按独立入口启动。已移除重复相机驱动、Python RGB-D 压缩器和重复 CameraInfo 发布节点。

## 后续顺序

1. 稳定真实/录制数据来源、时间与协议验收。
2. VLM、LLM 原生请求及文件输入主 agent 已独立迁移；继续验收真实服务协议与任务输出。
3. VLN preview 已装配实时或 bag 观测、目标定位、多视角搜索、XNavDP 与 MPC 预览。主动换视角、持续探索、房间搜索和目标重捕获后续迁移。
4. 实机 profile 已装配独立跟踪器，后续完成控制权管理与实机端到端验收。

不要重新导入 simple 的完整 blueprint 来获得单项能力。算法插件使用自己的入口与配置；不将模型子进程或图像转发重新放入基座。

当前实现和职责以 [workspace.md](workspace.md) 为准，完成状态见 [当前进度](../progress.md)。
