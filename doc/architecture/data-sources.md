# 数据源、录制与回放

采集、本体、录制、回放和推理各自启停。录制不会启动设备；回放只运行 rosbag2 播放器，发布 bag 内原话题及标准 /clock，不启动压缩器、相机或模型。

## 录制

```bash
bash sh/record_bag.sh /odom /camera/camera/color/image_raw /camera/camera/aligned_depth_to_color/image_raw /realsense_camera_info
```

只记录传入的话题，不自动追加 TF 或图像。话题必须是绝对 ROS 名称；重复项去重。可用 --dry-run 查看计划。原始 RGB-D/odom 用于 XNavDP 时推荐上面的四个输入；通用录制和回放不强制限定话题种类。

输出固定在项目 `data/bags/record_<北京时间>_<唯一后缀>/`，包含 MCAP 和 metadata.yaml，不进入版本库。每次创建新目录，不覆盖旧数据。Ctrl+C 正常结束并完成索引；rosbag2 等待发布者期间可能录到零消息，录完需确认数量。

## 独立回放

```bash
bash sh/play_bag.sh --bag data/bags/你的bag目录
# 可选减速、循环或仅检查
bash sh/play_bag.sh --bag data/bags/你的bag目录 --rate 0.5 --loop --dry-run
```

默认正常速度、播放一次并生成 /clock；原始消息内容、header、frame 不转换。允许任意话题 bag，拒绝空 bag 或缺失存储文件。相对路径按项目根解析；--bag-config 可指定另一份完整参数文件。

录制和回放共用 `config/bag.yaml` 中的 rosbag_setup；bag_path、rate、loop、clock_hz 只用于回放。当前 vendor 不含 rosbag2，配置指定本机兼容的 ROS Jazzy 工具环境，只加载到录制/播放进程，不混入相机或基座进程。

## 压缩和 XNavDP

```bash
# 终端 1：先启动回放并保持有数据
bash sh/play_bag.sh --bag data/bags/你的bag目录
# 终端 2：独立压缩；默认检测 /clock
bash camera_ws/start_compressor.sh
# 终端 3：独立检查和推理
source sh/env.sh
python -m dimos.plugins.xnavdp
python -m dimos.plugins.xnavdp --mode inference --requests 1
```

压缩入口默认等待输入并检测 /clock 或 rosbag2 发布者：回放使用 use_sim_time=true，真实采集使用实时钟；同时检测原始 RGB-D 的发布 QoS，兼容 Reliable 和 Best Effort；已有 C++ 压缩器时先核对并纠正 use_sim_time，确认压缩输出与 /clock 对齐后复用；bag 自带压缩图时只验证时间戳，不修改录制数据。不再启动重复压缩器。可在回放之前启动等待来源，也可显式使用 --time-source sim；切换实时/回放来源后再次运行入口重新核对已有压缩器时钟。可用 --time-source wall 强制实时，--dry-run 仅查看计划。真实 D455 采集入口仍按现有配置自带 C++ 压缩，不必再启动一份。

压缩器只处理原始 RGB-D，输出仍是原两个 /compressed 话题；不发布 odom 或标定。仿真时钟初始化前跳过图像，避免零时间戳。bag 已录制压缩图时，XNavDP 可直接读取，无需解压再压缩。

XNavDP 目标 stamp 始终来自 RGB header，等待和数据接收新鲜度使用单调时钟，历史日期不会被当作过期；暂停回放而没有新输入会超时。按 sensor_timeout_s 等待服务端 RGB/depth/odom 缓存和标定就绪，不自动启动数据源或 reset 模型。

XNavDP 服务端需要有消息的压缩 RGB、压缩深度、/odom (PoseStamped) 和 /realsense_camera_info。原始深度为 uint16 设备尺度，保持原编码和单位。只回放原始三类图像/位姿而不提供标定不能完成该推理链路。

同名真实与回放数据源不能混用，由操作方先停止原来源；纯回放脚本不停止任何外部模块。循环/时间倒退可能影响服务端模型历史，不自动 reset 共享模型；主动推理前确认实例空闲。基础录制回放、消息通信、模型输出和运动执行分别验收。
