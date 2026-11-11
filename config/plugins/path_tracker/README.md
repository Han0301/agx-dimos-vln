# 独立路径跟踪插件

使用 minimum 原生输入提取 simple 的 XNavDP 跟踪执行部分；不请求模型、不启动或停止设备，不增加中转话题。默认只在终端输出速度，不创建 `/cmd_vel` 发布者。

## 安装与启动

```bash
cd /home/deepvln/dimos_mininum
# 一键配置整体环境：D455、本体、XNavDP 客户端与跟踪器
bash sh/setup/install.sh

bash sh/start_tracker.sh --check
bash sh/start_tracker.sh --output-only
```

参数默认读取 `config/plugins/path_tracker/tracker.yaml`，可用 `--config` 指定另一份完整配置。启动脚本固定使用本工作空间 `.venv/bin/python`；运行前检查 NumPy、SciPy、PyYAML 均来自本 `.venv`。ROS 来自本项目 camera_ws/vendor，不复制进 Python 包依赖。默认安装会补齐 D455 系统依赖并增量构建相机；不兼容的旧 venv 自动备份重建。SciPy 版本与 simple 相同，由默认安装入口一并安装；插件仍独立启动。

`output_only: true` 是默认值；命令行 `--output-only` 强制开启，即使配置为 false 也不会发布速度。终端每个控制周期输出 `linear_x`（m/s）、`angular_z`（rad/s）、停止原因和 `published: false`。不发布预览话题。实机装配引用 [tracker_motion.yaml](tracker_motion.yaml)，其 `output_only: false` 会发布原 `/cmd_vel`。Go2 的 `allow_motion` 由基座配置管理。回放验证使用 preview profile 或独立 tracker 的 `--output-only`。完整命令见 [VLN 启动与排查 skill](../../../doc/skills/vln-operations/SKILL.md)。

启动前只替换本项目、同 ROS domain 的旧 tracker，等待它停止控制并退出后再启动；`--check`、`--help` 不替换实例。退出只清理自己的线程和 ROS 节点。它不替换 XNavDP、bag、压缩器、RViz 或本体。

## 输入输出

| 输入 | 类型 | 行为 |
| --- | --- | --- |
| `/xnavdp/result` | std_msgs/String，v3 JSON | 读取选中路径、候选、评分、参考位姿、目标、世界 frame 和整数请求时间 |
| `/odom` | geometry_msgs/PoseStamped | 最新平面位姿，记录 monotonic 接收时间 |
| `/camera/camera/aligned_depth_to_color/image_raw` | sensor_msgs/Image | 最新原始对齐深度；16UC1/mono16 除以 depth_scale 得到米，32FC1 按米使用 |
| `/lidar` | sensor_msgs/PointCloud2，可选 | 原 XYZ 点云；world 与 base_link 输入适配为碰撞器需要的本体 XY |
| costmap 配置项 | nav_msgs/OccupancyGrid，可选 | 默认 null，不订阅也不生成；启用时须与 odom 同世界 frame |

独立 tracker 以 Reliable/Volatile/KeepLast(10) 订阅 result。实机 VLN 装配由 Web 进程直接把已配对校验的任务路径提交给 tracker。odom、深度和激光使用 BestEffort/Volatile/KeepLast(1)。回调更新缓存，MPC 规划与 10 Hz 控制分别运行。RViz 继续使用原生 `/xnavdp/trajectory` 和 `/xnavdp/result`。

## 沿用 simple 的算法与边界

- `mpc.py`、`plan.py`、`collision.py` 的求解、减速和碰撞算法从 simple 提取，函数体保持一致；仅将框架消息类型换为最小数据适配器。
- 结果参考位姿与当前 odom 使用原平面 yaw 变换；世界 frame 必须一致。选中路径进入 MPC，候选评分不重新选路。
- 最新规划任务替换等待任务；计算完成后检查是否被更新取代。MPC 失败保留旧计划至其原有效期。
- 默认 10 Hz、horizon 15、期望/最大线速度 0.4 m/s、最大角速度 0.2 rad/s；约 1 秒开始减速、3 秒失效，odom 超过 0.5 秒未收到即停止。到目标 0.2 米停止。
- 障碍检查沿用 costmap 优先、否则激光，另加前向深度否决；仅允许碰撞检查通过的旋转。
- 独立入口解码失败时保留旧缓存，计划按原到期规则停止。实机 VLN 装配额外检查传感器新鲜度。循环 bag 时间倒退时，独立 tracker 会忽略较旧结果，需重启 tracker。
- 所有控制和过期计时沿用 simple 的 monotonic；不依赖 ROS 定时器，不把 bag header 与墙钟比较。没有新增 /clock 处理。
- VLN 任务层负责固定世界目标、持续规划、到达和取消。独立入口从 result.goal 与参考位姿恢复目标。前方 3 米通信测试会随请求改变目标。

## 文件职责

`messages.py` 负责原生消息/JSON 适配；`node.py` 负责订阅；`tracker.py` 负责缓存、规划与控制；`mpc.py`、`plan.py`、`collision.py` 为算法；`output.py` 为速度出口；`session.py` 只处理本插件重启；`__main__.py` 为参数和生命周期。插件不运行任何 simple 源码或导入其虚拟环境。
