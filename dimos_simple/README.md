# DimOS Go2 视觉语言导航

本目录运行 Go2 视觉语言导航，配合同一仓库的 `camera_ws` 和远端 XNavDP、视觉语言模型及 Ollama 服务使用。

## 整体框架

```text
X3 / D455 → camera_ws → ROS 2 图像与深度话题 ─┐
Go2 → 位姿与机器人状态 → DimOS ────────────────┤
用户指令 → DimOS Agent → Qwen3-VL / Ollama     │
                  ↓ 目标点                     ↓
               XNavDP 推理服务器 → 路径 → DimOS 本地控制 → Go2
```

`camera_ws` 负责相机采集和压缩。DimOS 负责指令理解、环境感知、目标选择和本地控制。XNavDP 服务器订阅相机及位姿话题，返回导航路径；远端 Qwen3-VL 和 Ollama 提供视觉及语言推理。实际启动入口为 `unitree-go2-vln-local`，通信话题和诊断见 [XNavDP 说明](sh/xnavdp_ros2/README.md)。

## 首次准备

当前使用 Ubuntu 24.04 aarch64、Python 3.12 和 ROS 2 Jazzy。以下依赖需要在目标机器准备好：

1. 安装 `uv`、Git 和 Git LFS，并确保能访问 Python 包源、系统软件源及 CLIP 模型来源。安装脚本会按 `uv.lock` 安装依赖，并可能通过 sudo 安装缺失的系统包。
2. 准备与本目录同级的 `NavDP/ros2_ws/src/navdp_bridge`，其中须有 `setup.py`。该包是当前 `navdp` extra 的本地路径依赖，并未包含在本仓库。
3. 准备 ROS 2 Jazzy 和 DimOS ROS bridge。默认路径分别为 `~/ros2_jazzy` 和 `~/dimos-ros2-bridge`，其他路径可在启动前设置 `ROS2_JAZZY` 和 `ROS2_BRIDGE_ROOT`。bridge 的 `env.sh` 必须与目标机的 ROS 环境兼容。
4. 在目标机器配置相机及远端服务。相机环境恢复和启动见 [camera_ws README](../camera_ws/README.md)。

在本目录运行：

```bash
bash sh/start_venv.sh --dry-run  # 检查前置条件并查看安装步骤
bash sh/start_venv.sh            # 创建 .venv，安装依赖和 CLIP 模型
bash sh/check_venv.sh --skip-remote
```

安装器会准备 CLIP 模型；离线安装可提供与上游 LFS 指针校验值一致的归档，通过 `DIMOS_CLIP_ARCHIVE` 指定，并使用 `--offline`。重新创建已有环境可使用 `bash sh/start_venv.sh --recreate`，脚本会先备份旧环境。

## 配置与启动

检查 `dimos/agents/skills/config/vln_config.yaml` 中 Qwen3-VL、Ollama 和相机参数。默认 Qwen3-VL 地址为 `10.10.10.56:8001`，Ollama 为 `10.10.10.56:11434`。XNavDP 的 DDS 对端配置位于 `sh/xnavdp_ros2/client_dds.xml`，默认连接 `10.10.10.56:7411`。两个工作空间的 ROS domain 和 DDS 配置必须一致。修改后需要重启相机及 DimOS 进程。

远端 XNavDP 服务需要单独启动，现有服务器启动方法见 [XNavDP 说明](sh/xnavdp_ros2/README.md)。在目标机器先按相机 README 启动相机，再在另一个终端执行：

```bash
cd /path/to/orin-vln-deployment/dimos_simple
source ./sh/activate_vln.sh
bash sh/check_dimos.sh
dimos --debug_mode true --robot-ip 192.168.123.161 --navigation-policy xnavdp run unitree-go2-vln-local
```

将命令中的 IP 改成实际机器人地址。激活脚本默认使用 `192.168.123.161`、Fast DDS 和 ROS domain 0，并加载 `~/.config/dimos/unitree.env`（如果存在）；设备凭据应在目标机单独配置。该命令连接真实机器人，启动前检查设备连接及运行环境。

常用检查：

```bash
bash sh/check_venv.sh
source ./sh/activate_vln.sh
python -m dimos.navigation.xnavdp.diagnostics --health-only
```

需要单独启动 Go2 body ROS 桥时，可执行 `bash sh/start_go2_body.sh <机器人IP>`。网页交互入口在现有运行指南中记为 `http://127.0.0.1:5556`，以实际服务输出为准。

## XNavDP 通信测试

参考 [11 月 6 日测试记录](https://aez5tpi4f5.feishu.cn/docx/JHf1dXGNRoiG65xy0pTcHnZUneg)，在独立终端依次启动 D455 和 Go2 本体桥：

```bash
# 终端 1：先设置 DIMOS_XNAVDP_ROOT，见 camera_ws README
cd /path/to/camera_ws
bash start_cameras.sh --camera d455

# 终端 2：启动本体桥和 /odom
cd /path/to/orin-vln-deployment/dimos_simple
bash sh/start_go2_body.sh 192.168.123.161
```

在推理服务器上启动现有服务：

```bash
ssh -t guest@10.10.10.56
cd ~/xnavdp_ros2_topics
nohup ./run_server.sh >> server.log 2>&1 &
```

在 Orin 的另一终端加载环境并检查缓存，确认 `rgb`、`depth`、`odom` 均大于 0：

```bash
cd /path/to/orin-vln-deployment/dimos_simple
source ./sh/activate_vln.sh
python -m dimos.navigation.xnavdp.diagnostics --health-only
```

测试记录使用 `python -m dimos.navigation.xnavdp.diagnostics --requests 1000` 进行推理测试。该测试会重置模型历史，应在导航停止后执行。完整导航使用前一节的 `unitree-go2-vln-local` 启动方式，避免同时运行两个本体连接。

## 目录

- `dimos/`：机器人、导航、感知及 Agent 源码。
- `sh/`：环境安装、激活、检查及 XNavDP DDS 配置。
- `docs/`：现有运行指南和工作空间说明，其中部分路径来自原机器，迁移时以本 README 和实际目录为准。
- `data/`、`run/` 和 `.venv/`：模型、运行数据与本机虚拟环境，需在目标机重建或另行提供。
