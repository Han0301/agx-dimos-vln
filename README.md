# DimOS Mininum

面向 Go2 的最小视觉语言导航工作空间。Go2 本体通信、D455/X3 采集、bag 回放和上层任务独立启停。VLN 支持文件问答、目标定位、多视角找物、路径与速度预览，以及显式开启的 Go2 探索和跟踪。默认基座关闭运动，实机端到端验收进度见 [当前进度](doc/progress.md)。

## 架构

```text
D455 + Go2 或 bag ── RGB-D、标定、/odom ──┬──→ VLN 观测缓存
                                          └──→ XNavDP 服务

Web 文字 ──→ vln_task ──→ find_object ──→ D455 当前视角检测
                               └── 当前视角未见 ──→ active_search
探索点或接近点 + 新观测 ──→ /xnavdp/point_goal ──→ XNavDP

XNavDP ──→ /xnavdp/trajectory + /xnavdp/result ──→ RViz、MPC 速度预览
XNavDP ──→ 已校验的任务路径 ──→ 内置 tracker ──→ /cmd_vel ──→ Go2（实机）
```

| 模块 | 职责 |
| --- | --- |
| `web_input`、`vln_task` | 网页接收文字和可选图片。主 Agent 管理任务历史、工具调用、期限及状态 |
| `llm`、`vlm` | 分别调用 Ollama 与 DashScope。VLN Task 的 Web 适配启动时预热 LLM，VLM 由当前进程按需调用 |
| `vln_task/search` | 默认实机入口使用记忆与前沿探索；未启用记忆的 motion 装配使用原地扫视。深度无效时定向换视角，回放用录制视角 |
| `xnavdp` | 用原始 RGB 时间戳提交点目标，关联并校验服务端的原生轨迹与结果 |
| `path_tracker` | 用路径、最新里程计、原始深度和可选障碍输入计算速度。实机配置发布 `/cmd_vel` |
| `runtime_log` | 独立前台终端，按实际启动的模块归档输出、ROS 日志与阶段耗时 |

上层任务位于 `dimos/vln_task/`，基础能力位于 `dimos/plugins/`；旧搜索命名空间仅作兼容转调，执行实现只维护一份。

上层调用基础插件的关系如下。图示对应当前代码，实机验收进度仍见[当前进度](doc/progress.md)。

![VLN Task 上层与基础插件调用关系](doc/architecture/diagrams/vln_modules.png)

<details>
<summary>展开非记忆 motion 模式的找物、扫视与深度恢复流程</summary>

此图对应未启用记忆探索的 motion 分支；默认实机 Shell 入口的记忆与前沿流程见 [VLN 操作说明](doc/skills/vln-operations/SKILL.md#自由探索测试)。

图中 LiDAR 是显式配置 `topics.lidar` 时的附加探索输入。默认探索局部图使用 RGB-D，`/lidar` 另用于 tracker 的控制代价地图。

![未启用记忆时的找物、扫视与接近](doc/architecture/diagrams/object_search.png)

</details>

两张图的矢量版、可编辑源文件及维护方式见[架构图维护](doc/architecture/workspace.md#架构图维护)。模块接口、时序与参数见 [工作空间架构](doc/architecture/workspace.md)、[插件契约](doc/architecture/plugin-contracts.md) 和 [插件索引](config/plugins/README.md)。

## 安装与检查

目标设备为 NVIDIA Jetson Orin AGX，系统为 Ubuntu 24.04 ARM64、Python 3.12、ROS Jazzy。以下安装命令在目标设备执行。首次检出项目：

```bash
git clone https://github.com/Han0301/agx-dimos-vln.git dimos_mininum
cd dimos_mininum
```

后续文档命令均以工作空间根目录为执行位置，项目文件路径相对该根目录，不要求固定用户名或检出位置。运行时归档及其 SHA256 由现有部署单独提供。首次安装：

```bash
bash sh/setup/install.sh --runtime-archive /path/to/dimos-runtime.tar.gz \
  --runtime-sha256 SHA256_OF_ARCHIVE
```

`sh/setup/install.sh` 是整个工作空间的一键安装、编译和环境检查入口。`sh/start.sh` 仅启动本体通信。实际安装会先检查 ARM64 与 Ubuntu 24.04，非目标平台在安装前停止。

更新已有部署的目录布局后，在 Orin 重新运行该安装入口，刷新 editable 安装来源和命令入口；`--check-only` 只检查，不修复旧安装。依赖与构建资料统一位于 [sh/dependencies/](sh/dependencies/)，文件用途及安装来源见 [目录规范](doc/architecture/workspace.md#python-源码与安装目录)。

安装器校验归档后恢复 `camera_ws/vendor/`。默认补齐 Ubuntu 系统包，在本项目构建 D455/C++ 压缩器和独立 `.venv`，并检查 Go2、XNavDP、tracker 和 Agent 客户端。运行时已就位后可直接执行。

```bash
bash sh/setup/install.sh --dry-run
bash sh/setup/install.sh
bash sh/setup/install.sh --check-only
```

在已有可用运行时的机器上，从项目根目录导出归档和校验值。解引用库链接，以便在另一台机器恢复。

```bash
tar --dereference --exclude='vendor/compat/usr/share/doc/*/changelog.Debian.gz' \
  -czf /tmp/dimos-runtime.tar.gz -C camera_ws vendor
sha256sum /tmp/dimos-runtime.tar.gz
```

归档、构建产物、虚拟环境、bag 和运行日志均由 Git 忽略。离线安装可用 `DIMOS_WHEELHOUSE=/path/to/wheels bash sh/setup/install.sh --offline`，目标机需预装 Ubuntu 系统包。X3 另需官方 SDK。Go2 和 DashScope 凭据在目标机本地配置，LLM 与 XNavDP 服务由服务器侧维护。详见 [安装 skill](doc/skills/dimos-install/SKILL.md)。

## 实机 Go2

从项目根目录分终端启动。需要归档时先开日志终端。

| 终端 | 命令 |
| --- | --- |
| 日志 | `bash sh/start_log.sh --mode live` |
| 1 · Go2 | `bash sh/start.sh --config config/base_motion.yaml` |
| 2 · D455 | `bash camera_ws/start_cameras.sh --camera d455` |
| 3 · Web、VLN Agent、tracker | `bash sh/start_vln_motion.sh --config config/profiles/vln_motion.yaml` |
| 4 · RViz | `bash sh/run_rviz.sh --time-source wall` |

这组配置开启速度发布和 Go2 运动接收，并关闭 Go2 本体避障。VLN 将 Go2 世界坐标 LiDAR 体素点云投影为 `/global_costmap`。实机 tracker 用代价地图检查车身扫掠，并保留 D455 前向深度停车。详情见 [主动找物](config/plugins/active_search/README.md)。只检查本体通信时使用 `config/base.yaml`，其 `allow_motion` 默认为 false。Go2 默认有线地址为 `192.168.123.161`，网卡为 `end0`，凭据从 `~/.config/dimos/unitree.env` 读取。XNavDP 的 DDS TCP 7411 需要到服务器的直连路由，代理环境配置见 [工作空间架构](doc/architecture/workspace.md#xnavdp-输出如何使用)。

## Bag 回灌与路径预览

| 终端 | 命令 |
| --- | --- |
| 日志 | `bash sh/start_log.sh --mode bag` |
| Web 与 VLN Agent | `bash sh/start_web_input.sh --profile config/profiles/vln_preview.yaml` |
| RViz | `bash sh/run_rviz.sh --time-source sim` |
| bag 播放 | `bash sh/play_bag.sh --bag "data/bags/your_recording"` |

回放需要 RGB-D、标定和 PoseStamped `/odom`。已有压缩 RGB-D 可直接使用。仅有原始图时，在另一终端运行 `bash camera_ws/start_compressor.sh --time-source sim`。回放发布原话题和 `/clock`，与同名实机数据源互斥。回灌前退出实机 VLN 入口。Preview 从 bag 读取视角变化并计算一次速度结果，路径仍通过原生 XNavDP 话题发布。

```bash
# 在有数据的环境中录制指定话题，Ctrl+C 完成写入
bash sh/record_bag.sh /odom /camera/camera/color/image_raw /camera/camera/aligned_depth_to_color/image_raw /realsense_camera_info
```

录制输出位于 `data/bags/`。播放参数、QoS 和时间戳见 [数据源规范](doc/architecture/data-sources.md)，显示输入见 [RViz 说明](doc/architecture/visualization.md)。

## 网页与日志

在运行 Web 服务的 Orin 本机打开 [VLN 网页](http://127.0.0.1:5556/)，默认仅监听回环地址；用 `curl -fsS http://127.0.0.1:5556/health` 查看 LLM 预热状态。日志终端等待后续模块启动，Ctrl+C 结束采集，记录保存在 `data/log/session_*/`。按 `task_id` 查 Agent 调用和耗时，用 XNavDP 日志的 `stamp_ns` 对照 tracker 日志的 `request_stamp_ns`。场景检查与排查步骤见 [vln-operations skill](doc/skills/vln-operations/SKILL.md)，日志文件与探针见 [运行日志说明](config/plugins/runtime_log/README.md)。

## 独立模块测试

按需执行，长期运行的服务各用一个终端。

```bash
source sh/env.sh
python -m dimos.plugins.llm --prompt "把找杯子拆成几个步骤"
python -m dimos.plugins.vlm --image path/to/image.jpg --prompt "描述图片"
python -m dimos.vln_task --goal "红色物体在哪里" --image path/to/image.jpg
bash sh/start_web_input.sh                 # 文字与图片上传模式
bash sh/start_xnavdp.sh                   # 只读健康查询
bash sh/start_tracker.sh --output-only    # 订阅现有结果，终端显示速度
```

VLM 默认直接连接 DashScope，复用 `~/.config/dimos/vlm_orin.yaml` 中的私有凭据。原有 HTTP 配置仍可选用。见 [VLM 配置](config/plugins/vlm/README.md)。离线配置可用 `--check` 检查。独立 XNavDP 推理与共享模型初始化见 [XNavDP 说明](config/plugins/xnavdp/README.md)，可复用的启动检查命令见 [vln-operations skill](doc/skills/vln-operations/SKILL.md#只读入口检查)。

## 目录

| 路径 | 内容 |
| --- | --- |
| `dimos/` | Python 包入口；`core/` 放基座启动与配置，`robot/` 放本体通信，`plugins/` 放基础能力，`vln_task/` 放上层任务 |
| `sh/dependencies/` | Python 构建声明、依赖版本约束、源码归档清单与构建辅助、许可资料；通过现有安装入口使用 |
| `camera_ws/` | 相机驱动、C++ 压缩、launch、vendor 及数据源入口 |
| `config/` | 基座和原生接口参数，`plugins/` 放插件参数，`profiles/` 放组合配置 |
| `sh/` | 公共启动和环境入口，`setup/` 管安装，`xnavdp_ros2/` 管 DDS |
| `data/bags/`、`data/log/` | 本地录制数据与日志 |
| `tests/` | 模块测试与隔离 ROS 集成测试 |
| `doc/` | [知识库索引](doc/README.md)、架构、进度、规范和项目 skills |
