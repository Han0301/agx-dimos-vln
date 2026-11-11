# XNavDP 独立通信插件

DDS 物理 TCP 端口保持 7411；客户端初始 peer 的逻辑发现端口显式为 7410，与当前服务端 domain 0/participant 0 对应。物理与逻辑端口含义见 [Fast DDS TCP 配置](https://fast-dds.docs.eprosima.com/en/2.x/fastdds/transport/tcp/tcp.html)。服务端若改变 domain/participant 或发现端口，需要同步检查这项契约；不能把逻辑端口当成模型服务的新端口。不使用参考项目的 HTTP 8889。基础通信、传感器、回放和可视化分别管理，客户端不启动或停止它们。

本地依赖统一由 `bash sh/setup/install.sh` 配置，不需要为 XNavDP 或 tracker 单独安装；远端服务与 GPU 环境仍由服务端维护。

## 启动与参数

```bash
bash sh/start_xnavdp.sh --check
# 默认 health_only，只读状态
bash sh/start_xnavdp.sh
# 主动请求一次；实例空闲且模型已初始化时执行
bash sh/start_xnavdp.sh --mode inference --requests 1
```

入口自动加载环境并透传原参数。推理启动前先停止同项目、同 ROS domain 的旧 XNavDP 客户端并等待退出；停止失败则不叠加新实例。直接使用 Python 模块入口也遵循此规则。health_only、--check 和 --help 是只读操作，不替换推理实例；远端模型与其他模块保持独立。

参数在 xnavdp_test.yaml，--config 可另选；默认不 reset、不运动。初始化必须由实例所有者显式传入 --reset-if-needed 或配置 initialization.reset_if_needed=true，且已有标定。重启服务后不能把 health 可达当作模型已初始化。该选项只在 inference 模式生效；已有初始化时不重复 reset，不改写参数文件。

```bash
bash sh/start_xnavdp.sh --mode inference --requests 10000 --reset-if-needed
```

## 输入和目标

服务端直读原压缩 RGB、压缩深度、/odom (PoseStamped)、/realsense_camera_info。客户端只使用原 RGB header 和 odom 构造前方 3 米局部目标，将其转换为 odom/world XY 后发布 /xnavdp/point_goal。目标 stamp 精确保留 RGB 的整数纳秒；不转发图像或位姿，不发布 cmd_vel。

按 sensor_timeout_s 等待服务端 RGB/depth/odom 缓存与标定就绪；模型初始化、busy 和独占检查保持显式。可见发布者检查不等于服务端独占锁；主动推理会推进模型历史。

## 唯一结果接口

协议为 xnavdp-ros2-topics-v3，模型 release 仍为 unified-20261104。服务端只发布：

- `/xnavdp/trajectory`：nav_msgs/Path，本体 base_link 坐标；路径及点的 header 使用请求观测时间。
- `/xnavdp/result`：std_msgs/String，JSON。成功为 status=success，包含 request_stamp_ns、trajectory[1,H,3]、all_trajectory[1,N,H,3]、all_values[1,N]、trajectory_frame、goal_frame、goal[2]、robot_pos[3]、robot_quat[4]、world_frame、pose_mode、传感器时间和匹配间隔、roundtrip_seconds。

trajectory_frame 和 goal_frame 为 base_link；robot_pos/robot_quat 是服务端实际采用的世界参考位姿，goal 是该参考下的局部目标。不是把 world 路径简单改名。JSON 与 Path 的选中轨迹数值一致。

错误仅通过 result 返回 status=error、request_stamp_ns、error，不发布伪路径。客户端按整数 request_stamp_ns 关联，等待两类输出，检查坐标、长度、评分、有限数值和世界目标可重建性；迟到结果不匹配当前请求。超时失败不 reset 共享模型。

路径是局部空间路径，不是可直接执行的时序轨迹。独立 [path_tracker](../path_tracker/README.md) 直接读取 result 内的选中路径和参考位姿，结合当前 odom 恢复轨迹，执行 MPC 和碰撞检查；默认只在终端预览速度，不创建 /cmd_vel 发布者。XNavDP 请求入口不自动启动跟踪器。

服务端源码位于 guest@10.10.10.56:/home/guest/xnavdp_ros2_topics/server.py，启动脚本为同目录 run_server.sh；模型 worker 及 GPU 环境归服务端维护，不复制进基座依赖。服务端与客户端协议变更必须同步，通信层仅使用两个结果输出，不保留平行旧结果接口。

## bag 与时间

```bash
# 终端 1：只回放原话题与 /clock
bash sh/play_bag.sh --bag data/bags/你的bag目录
# 终端 2：独立压缩；已录压缩图时复用现有输入
bash camera_ws/start_compressor.sh
# 终端 3
bash sh/start_xnavdp.sh --mode inference --requests 1
```

压缩入口默认等待来源，检测 /clock 或 rosbag2 发布者使用仿真时间；复用已有压缩器前检查并纠正时钟。也可显式指定 --time-source sim。客户端使用观测 header，接收新鲜度/等待用 monotonic 时间，历史日期不会被误判为过期；暂停无新输入会超时。循环或时间倒退不自动重置共享历史。详见 [数据源规范](../../../doc/architecture/data-sources.md)。

## 独立速度预览

```bash
# 与推理入口分终端运行；不启动模型，不驱动 Go2
bash sh/start_tracker.sh --output-only
```

订阅 result、/odom、原始对齐深度和可选 /lidar；不需要订阅 trajectory，因为选中路径已包含在 result。配置和各子模块职责见跟踪插件 README。

## 可选 RViz

`bash sh/run_rviz.sh` 使用与 XNavDP 相同的 xnavdp.rviz 配置。仅该入口启动深度点云、缺失 TF 补全与 result→显示 Marker 的辅助节点，推理链本身不转发结果。退出只停止显示资源。见 [可视化规范](../../../doc/architecture/visualization.md)。

## 验收

配置检查不创建 ROS 资源。普通测试不发布模拟传感器；ROS 集成测试显式启用且仅在隔离 domain 173 使用本地测试 DDS。协议模拟、真实模型输出和运动分别验收。关闭客户端只清理自身资源，不取消已发送的服务器推理，不用 reset 模拟取消。
