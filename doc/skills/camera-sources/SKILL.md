---
name: camera-sources
description: 维护最小 DimOS 相机、bag 录制回放、独立压缩和时钟适配；适用于 record_bag.sh、play_bag.sh、start_compressor.sh 及输入时序故障，不用于 SDK 安装。
---

# 数据源与 bag 维护

先读 [数据源规范](../../architecture/data-sources.md) 和 [脚本目录](../../architecture/scripts.md)。项目根从本目录向上三级定位。

- 相机采集在 camera_ws；独立压缩在 camera_ws/start_compressor.sh。sh/record_bag.sh 只录制用户给定话题，sh/play_bag.sh 只回放原话题及 /clock，不启动压缩、设备或模型。
- 输出统一 data/bags/record_<北京时间>_<唯一后缀>/，MCAP，无覆盖。空/非法参数和 dry-run 不创建目录。Ctrl+C 由 rosbag2 正常完成索引；零消息不是有效录制。
- config/bag.yaml 是 bag 工具环境和回放参数的唯一来源；sh/internal/rosbag_env.bash 仅加载环境，不创建 ROS 节点。rosbag2 的独立 ROS Jazzy 环境不能与 camera_ws/vendor 的 Python/动态库混入同一进程。
- 录制/回放不强制话题类别，不自动添加 TF、标定或转换层。XNavDP 链路另行要求有消息的原压缩 RGB-D、PoseStamped odom 和 /realsense_camera_info。
- 独立压缩入口等待输入，并检测 /clock 或 rosbag2 发布者，选择 use_sim_time 和兼容的原始图像 QoS；已有 C++ 压缩器时先检查并纠正时钟，再验证压缩图与 /clock 一致；复用 bag 的压缩输入时不修改原消息。不启动重复压缩器。自动模式无输入时等待，不静默退回墙钟；可先启动压缩入口再启动回放。需要强制指定仿真时间时用 --time-source sim。切换数据源后再次运行入口核对并纠正已有压缩器时钟。仿真时钟初始化前不输出零时间戳。
- XNavDP 目标 stamp 来自 RGB header，等待与接收新鲜度使用 monotonic 时间；历史日期不是数据过期。输入就绪等待受 sensor_timeout_s 限制，不重置共享模型。循环/时间倒退的历史需由模型所有者处理。
- 不混用同名真实与回放源；每个入口只管理自己的进程，不全局 pkill。退出播放不停止压缩或插件。

修改时检查 shell 语法、无副作用 dry-run、任意话题/空 bag、参数错误、原生输出和退出传播。实际录制/回放与时钟适配仅在隔离 ROS domain 验证；不向生产 domain 注入模拟数据。同步源码、参数、使用说明和测试。知识库不保存当天测试数量、临时环境过程或排障日志。

隔离 C++ 测试必须同时设置独立 ROS domain 和 COMPRESSED_RGBD_DDS_PROFILE，测试 XML 不得含远端地址或固定生产 discovery 端口；原压缩配置包含固定 initialPeers，单改 domain 不足以保证隔离。正常部署不设置该测试覆盖项。
