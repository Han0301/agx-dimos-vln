# XNavDP：独立 ROS2 话题

服务器直接订阅原相机的两条压缩话题，不重新打包 RGB-D/odom/goal，也不添加相机转发话题。

| 话题 | 消息类型 | 发布 → 订阅 |
| --- | --- | --- |
| `/camera/camera/color/image_raw/compressed` | sensor_msgs/CompressedImage | 原相机压缩节点 → 服务器 |
| `/camera/camera/aligned_depth_to_color/image_raw/compressed` | sensor_msgs/CompressedImage | 原相机压缩节点 → 服务器 |
| `/odom` | geometry_msgs/PoseStamped | GO2Connection 原生 ROSTransport → 本机导航/VLM/服务器 |
| `/realsense_camera_info` | sensor_msgs/CameraInfo | DimOS 标定 → 服务器 |
| `/xnavdp/point_goal` | geometry_msgs/PoseStamped | 本机导航 → 服务器，world 中的目标点 |
| `/xnavdp/model_path` | nav_msgs/Path | 服务器 → 本机 MPC；world 坐标，可直接在 RViz 显示 |
| `/xnavdp/model_candidates` | visualization_msgs/MarkerArray | 服务器 → 本机；每条候选曲线的 text 是 critic 分数 |
| `/xnavdp/inference_pose` | geometry_msgs/PoseStamped | 服务器 → 本机；本次实际匹配的 odom 位姿 |
| `/xnavdp/status` | diagnostic_msgs/DiagnosticArray | 服务器发布匹配时间戳、局部目标和错误 |

`/xnavdp/health`、`/xnavdp/reset` 为标准 std_srvs/Trigger 服务。旧的 `/xnavdp/inference/request` 和 `/response` 已从新版代码移除。

目标点 header.stamp 指定 RGB 观测时间，header.frame_id 必须与 odom 的 world frame 相同。服务器缓存最近 2 秒（按接收时间淘汰），先取最接近目标时间的 RGB，再按 RGB 时间取最近的 depth 和 odom，每个 gap 最大 150 ms。模型使用匹配的 odom 把 world 目标转换为局部目标。返回的 path、候选、inference_pose 都带同一请求时间戳；MPC 使用该匹配位姿补偿推理期间的机器人运动。

模型 Python 3.10 与 ROS Python 3.12 之间仅在服务器内部通过 stdio 管道连接，local_model_protocol.py 只用于此本机 ABI 隔离，不用于 ROS/网络传输。

## 启动

服务器：`guest@10.10.10.56`，登录后进入 `~/xnavdp_ros2_topics`，执行 `./run_server.sh`，日志 `server.log`。

Orin/当前机器：先重启原相机启动程序（在仓库根目录执行 `bash camera_ws/start_cameras.sh`，使用原有相机选择参数），再启动 DimOS：

```bash
source ./sh/activate_vln.sh
dimos --debug_mode true --robot-ip 192.168.123.161 --navigation-policy xnavdp run unitree-go2-vln-local
```

相机启动脚本读取 camera_ws/config/xnavdp_dds.bash，与 DimOS 共用本目录 dds_env.sh。ROS_DOMAIN_ID 默认 0，Fast DDS TCP 服务端口 7411，保留本机 UDP DDS。这个设置必须在相机发布进程创建 ROS 节点之前加载；正在运行且 ROS_LOCALHOST_ONLY=1 的旧相机进程不会自动改变。

XNavDP 模式在 blueprint 合并完成后，将显式共享内存传输改为原生 ROSTransport，包括 odom、RGB/depth、CameraInfo、costmap 和 cmd_vel；Go2 odom 直接发布 ROS，不使用共享内存中转。Qwen 模型接口与机器人 WebRTC 接收/驱动不变。此改动没有启动机器人或发送速度指令。

## 检查

```bash
source ./sh/activate_vln.sh
python -m dimos.navigation.xnavdp.diagnostics --health-only
ros2 topic hz /odom
ros2 topic hz /camera/camera/color/image_raw/compressed
ros2 topic echo /xnavdp/point_goal
ros2 topic echo /xnavdp/status
```

health 返回服务器最近 2 秒各传感器缓存数量和最后时间戳；odom=0 表示尚未启动新版 DimOS 或 /odom 未连通。RGB/depth=0 时检查原相机进程是否已加载 DDS 环境并重启。

推理检查会重置模型历史；仅在停止导航之后执行 `python -m dimos.navigation.xnavdp.diagnostics --requests 3`。它只读取真实 ROS 图像/odom 并发送点目标用于模型测试，不发送 cmd_vel，不向真实话题注入模拟数据。

旧版 DimOS 正在运行时，不要用另一客户端重置其模型；旧版进程和旧版服务器需在任务结束后再停止。隔离测试使用 ROS_DOMAIN_ID=71，仅用于合成传感器输入和真实 GPU 推理，不影响正式域。
