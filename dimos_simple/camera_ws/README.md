# camera_ws 相机工作空间

本目录提供 Insta360 X3 与 Intel RealSense D455 的 ROS 2 Jazzy 相机采集。D455 发布彩色、对齐深度和压缩 RGB-D，X3 发布双鱼眼、全景、四方向图像及 IMU。相机数据经 ROS 2 供同仓库的 [DimOS](../README.md) 和远端 XNavDP 服务使用。

```text
X3 → Insta360 驱动 → 全景 / 四方向图像 / IMU ─┐
D455 → RealSense 驱动 → 彩色 / 对齐深度 ──────┼→ ROS 2 → DimOS / XNavDP
                       → 压缩图像节点 ─────────┘
```

## 恢复运行环境

当前部署环境为 Ubuntu 24.04 aarch64。Git 仓库保留源码和配置，`vendor/` 中的 ROS 2 Jazzy 与兼容库以及 `install/` 中的构建结果均需另行恢复。`start.sh` 和 `sh/build.sh` 都依赖 `vendor/`，从 Git 克隆后不能直接运行。

已有配置好的源机器可使用其 `camera_ws/sh/package_upload.sh` 生成部署包，该包包含 `vendor/` 和 `install/`，不包含专有 CameraSDK。将包传到目标机，解压到 DimOS 仓库根目录，恢复其中的 `camera_ws/`，确认与目标机架构和系统版本兼容。部署包自带源码和配置，更新本仓库后应同步对应版本并按需重新构建。CameraSDK 需从 [Insta360 官方渠道](https://www.insta360.com/sdk/home) 单独取得。

在包含运行时的源机器上执行：

```bash
cd dimos_simple/camera_ws
bash sh/package_upload.sh
# 生成 upload/camera_ws-*.tar.gz 和对应的 sha256 文件
```

将压缩包和校验文件传到目标机，在存放它们的目录核对校验，再解压到 DimOS 仓库根目录（确保目标目录没有需要保留的文件）：

```bash
sha256sum -c camera_ws-<时间戳>.tar.gz.sha256
tar -xzf camera_ws-<时间戳>.tar.gz -C dimos_simple
cd dimos_simple/camera_ws
```

将示例中的文件名替换为实际名称。首次准备 X3 所需的 SDK、系统依赖和设备权限：

```bash
bash start.sh --install-system-deps --setup-udev \
  --sdk-archive CameraSDK.zip --prepare-only
```

也可用 `--sdk-dir CameraSDK` 指定已解压的 SDK，或用 `--sdk-url URL` 指定下载地址。`--install-system-deps` 和 `--setup-udev` 需要 sudo。脚本的 `--prepare-only` 仍会检查 SDK，即使只计划运行 D455，也应先按当前脚本要求准备 SDK。

## 配置和启动

相机和 DimOS 必须在启动 ROS 节点之前加载一致的 DDS 配置。相机脚本默认从上一级 DimOS 工作空间加载配置：

```bash
bash sh/check_standalone.sh
bash start_cameras.sh --camera x3 d455 --dry-run
bash start_cameras.sh --camera x3 d455
```

按照 [11 月 6 日通信测试记录](https://aez5tpi4f5.feishu.cn/docx/JHf1dXGNRoiG65xy0pTcHnZUneg)，单独测试 XNavDP 时使用 `bash start_cameras.sh --camera d455`。按需要也可改为 `--camera x3`。`config/xnavdp_dds.bash` 默认查找父目录，需要使用其他 DimOS 工作空间时可设置 `DIMOS_XNAVDP_ROOT`。DimOS 的 DDS 配置默认使用 ROS domain 0 和远端 `10.10.10.56:7411`；地址、domain 变更后需重启相机与 DimOS 进程。

查看状态可执行 `bash start_cameras.sh --status`。X3 使用 Android USB 模式，需有 `/dev/insta`；D455 需要可用的 RealSense 设备权限。其他启动选项可用 `bash start.sh --help` 查看。

## 重新构建与目录

`third_party/` 包含相机驱动源码，`src/` 包含压缩等 ROS 包，`config/` 和 `launch/` 保存配置与启动文件。SDK 安装完成且 `vendor/` 可用后，源码变更可以通过下列命令重新构建：

```bash
bash sh/build.sh
```

在终端手动使用 ROS 命令时，可运行 `source sh/setup_local.bash` 和 `source install/local_setup.bash` 加载本工作空间环境。
