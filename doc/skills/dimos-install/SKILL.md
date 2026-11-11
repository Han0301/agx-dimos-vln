---
name: dimos-install
description: 维护、规范化或评审 DimOS 最小通信项目的 sh/setup/install.sh，覆盖工作空间初始化、相机 SDK 与 ROS 构建、独立 venv 和重复环境检查；适用于新增依赖或排查安装验收失败。
---

# DimOS 工作空间安装规范

让用户通过 `bash sh/setup/install.sh` 默认完成 D455、Go2、XNavDP 客户端、path_tracker、独立 agent 客户端的整体软件初始化，并在安装后通过同一入口再次检查环境。项目根目录从本 skill 所在目录向上三级定位，文中源码路径均相对项目根目录。

## 先确认当前实现

读取 `sh/setup/install.sh` 和根 `README.md`。涉及算法或模型依赖时，先核对 [工作空间架构](../../architecture/workspace.md) 的基座/插件边界。按本次改动选择补读：

- Python 依赖：`dimos/pyproject.toml`、`dimos/requirements.lock`、`sh/env.sh`。
- 相机和 SDK：`camera_ws/README.md`、`camera_ws/sh/setup_local.bash`、驱动的 `CMakeLists.txt` 和所选相机 launch 文件。
- DDS：`sh/xnavdp_ros2/dds_env.sh`、`client_dds.xml`。
- 验收：[references/acceptance.md](references/acceptance.md) 和 `tests/test_install.py`。

安装、构建和检查使用当前工作空间的源码、运行时和独立虚拟环境。

## 保留的项目约定

1. **统一入口。** 安装、SDK 准备、相机编译和环境检查由 `sh/setup/install.sh` 组织。保留 `--dry-run` 与 `--check-only`；不要要求用户额外运行另一套验证脚本。驱动 CMake、配置和测试保留在各自目录，不把源码副本塞入安装器。
2. **可以重复运行。** 默认模式补齐系统及 Python 依赖、执行 D455/C++ 压缩增量构建并检查；系统包安装可请求 sudo 凭据。`--check-only` 不安装或重建；`--dry-run` 不执行安装操作。两者允许查询状态，临时检查文件及时清理，避免持久修改被检查环境。
3. **环境独立。** 安装目标是本项目 `.venv`，校验 Python 版本、prefix 和 editable 来源。新 venv 不继承系统 site-packages；调用者已激活的环境不能改变安装目标。默认自动识别不兼容环境并备份重建；`--recreate` 可强制重建，保留备份忽略规则。只检查模式不得迁移环境。
4. **明确运行时来源。** 当前目标为 Ubuntu 24.04 ARM64、Python 3.12、ROS Jazzy。ROS、librealsense 与兼容库来自匹配的 `camera_ws/vendor`。首次安装可用 `--runtime-archive` 和 `--runtime-sha256` 从单独分发的本地归档恢复，校验失败不发布 vendor。若任务要求改变此部署方案，同步修改实现、依赖和验收要求。
5. **按相机准备依赖。** 默认 D455；X3/all 才需要 CameraSDK。D455 launch 依赖 `insta360_ros_driver` 的公共 Python 工具，因此不能跳过整个包；使用 `BUILD_INSTA360_CAMERA=OFF` 保留公共工具，跳过 X3 专用程序。切回 X3/all 时显式启用开关。
6. **SDK 来源可追溯。** 支持用户提供的目录、压缩包或下载地址；不臆造专有 SDK 地址。检查 SDK 完整性、目标架构与所请求的 SHA256。下载采用完成后才发布的缓存文件；提取前校验归档路径。不能依赖尚未生成的 `install/` 来完成首次 SDK 准备。
7. **维持最小依赖。** 修改依赖时同步项目声明与版本约束，核实 wired ICE 使用的 aiortc/aioice 兼容关系。ROS 构建使用预期的系统 Python 和本地 vendor 环境，避免调用者 venv 污染构建。NavDP 模型与可视化栈独立部署。已实现的 Go2、path_tracker 与 agent 客户端依赖由默认无参数入口统一安装，检查本 .venv 来源和 MPC 求解；XNavDP 远端模型不安装在本机。插件独立启停不等于要求用户分别安装依赖。高级裁剪可使用 --camera/--plugin，日常文档统一提供无参数命令。
8. **报告真实结果。** 软件环境、设备就绪、实机数据分别说明。不能只凭 pip 成功、XML 可解析或进程退出码为零就宣布可用；ROS 空生命周期还要检查 DDS/RTPS 错误日志。`PENDING` 表示尚未满足或未验证的设备条件，不能当作实机通过。
9. **保持操作范围。** 普通安装和软件检查不连接 Go2、不启动相机、不发送运动命令，也不静默更改网口地址或 DDS 服务端。凭据只校验存在和格式，不打印值。需要实机验证或通信参数变更时，按用户任务范围执行；记录本 skill 不构成执行这些操作的授权。

## 修改与交付

先界定此次改变影响的模式和依赖；在现有函数中实现，并保持错误指向可执行的下一步。涉及参数、模式或验收范围时，同步 README 和验收参考。避免为一次临时错误增加适用于所有环境的固定配置。

按照 [验收规范](references/acceptance.md) 选择有意义的检查。安装器测试重点覆盖副作用边界、隔离、模式选择和错误传播；软件生命周期与实机数据分别验证。新建 skill 或只修改文档时，检查文档和链接，不因此触发重装或机器人操作。

交付说明写清：改了什么、实际验证了哪些模式、哪些依赖只是复用、还有哪些条件待处理。没有执行全新安装、SDK 下载/编译或实机通信时，明确说明，不用历史记录代替本次验收。
