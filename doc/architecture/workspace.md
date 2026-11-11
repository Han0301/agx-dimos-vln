# 工作空间架构

## 目标和默认环境

无参数执行 `bash sh/setup/install.sh`，配置 D455、C++ 压缩器、Go2、本地 XNavDP 客户端、path_tracker 及独立 agent 客户端。补齐系统依赖、增量构建相机，并将 Python 依赖安装到本工作空间 `.venv`；不兼容旧环境自动备份重建。X3 需要另外选择及提供官方 SDK；远端模型/GPU 环境不安装到本机。ROS/librealsense 运行时沿用 camera_ws/vendor。

安装与启动分开：依赖统一准备，模块独立启停。模型或跟踪插件退出不影响设备遥测，不增加消息总线或数据中转话题。

```text
D455 或 bag → 原始 RGB-D → C++ 压缩 → 原压缩话题 ─┐
Go2 或 bag ───────────────────────→ /odom ──────┤→ 远端 XNavDP
驱动或 bag ───────────────→ /realsense_camera_info ┘       ↑
                                                 /xnavdp/point_goal
                                                        ↑
                                     独立 XNavDP 客户端或 VLN 主 Agent

远端 XNavDP ──→ /xnavdp/trajectory ─────────────→ 可选 RViz
            └→ /xnavdp/result ──┬─────────────→ 可选 RViz
                                ↓
/odom + 原始深度 + 可选 lidar → 独立 path_tracker
                                ↓
                    默认：终端速度预览，不发 /cmd_vel
                    显式启用输出：/cmd_vel → Go2 本体通信
```

真实数据与 bag 输入互斥。采集、回放、压缩、推理、跟踪和显示分别启动；实机与回放的组合入口见 [VLN 启动与排查 skill](../skills/vln-operations/SKILL.md)。

## 职责和参数

| 模块 | 职责 | 参数/入口 |
| --- | --- | --- |
| 本体通信 | Go2 连接、odom/点云/状态/TF，运动开关、限速与指令超时 | config/base.yaml；sh/start.sh |
| 数据源 | D455/X3、原始 bag、C++ 压缩；不启动算法 | camera_ws/config/、config/bag.yaml；相机/录制/播放/压缩各自入口 |
| XNavDP 请求 | 健康查询、构造点目标、检查原生结果；不发速度 | config/plugins/xnavdp/；sh/start_xnavdp.sh |
| 跟踪 | result 接收、参考位姿补偿、simple MPC 与碰撞检查；默认只预览速度 | config/plugins/path_tracker/；sh/start_tracker.sh |
| RViz | 显示路径、候选、点云，辅助节点只随显示启动 | sh/run_rviz.sh |
| Web 与 VLN Agent | 文件问答、实时观测、工具调用与任务状态；装配 VLM、LLM、active_search 与 XNavDP | config/plugins/；sh/start_web_input.sh |
| 实机装配 | Web 内运行 tracker，持续跟踪已校验的任务路径 | config/profiles/vln_motion.yaml；sh/start_vln_motion.sh |
| 运行日志 | 在独立终端收集实际启动的模块、任务链耗时和跟踪输出 | sh/start_log.sh；data/log/ |

共享接口见 [interfaces.yaml](../../config/interfaces.yaml)。/odom 为 PoseStamped；标定只由驱动或 bag 发布。本体负责安装 TF，驱动负责内部 TF。基座不托管算法，不解码转发相机图像。仅启动相应插件时导入算法依赖。

## XNavDP 输出如何使用

DDS 共用 `sh/xnavdp_ros2/`，远端 TCP 端口固定 7411。当前服务端为 `10.10.10.56`。启用 TUN 时，为该 IP 保留优先于代理的 `lookup main` 路由，`ip route get 10.10.10.56` 应显示实际网卡和源地址。`sh/env.sh` 会将该 IP 加入大小写 `no_proxy`，DDS TCP 仍依赖系统路由。已建立的代理连接由所属模块在安全时机重连。服务端读取原传感器话题，发布 v3 trajectory/result。请求客户端配对检查这两类结果。独立 tracker 订阅 result。实机 VLN 入口在进程内将校验通过的任务路径交给 tracker，RViz 继续直连原生输出。

跟踪器将 result 中参考时刻的 base_link 路径恢复到世界坐标，再结合当前 odom 变换到当前本体坐标，交给 MPC 生成速度。深度检查读取最新本地原始对齐深度，不绕经 PNG 压缩/解压。候选与评分保留校验，不重新选路。

时序、异常和 simple 保留边界见 [跟踪器说明](../../config/plugins/path_tracker/README.md)。通信测试的前方 3 米目标会随请求变化，不等同固定目标导航。

## 生命周期和维护

启动只替换同项目、同 ROS domain 的同模块旧实例，检查和健康查询保持只读。每个进程仅清理自己的资源，不全局 pkill，不重启设备或远端模型。XNavDP 推理推进共享模型历史，初始化/reset 必须显式选择。

安装不进行相机采集、机器人连接、远端推理或运动。软件验收与实机链路分别判断；收到模型路径不代表已经形成可执行导航闭环。
