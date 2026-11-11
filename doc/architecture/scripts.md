# 脚本目录约定

工作空间脚本统一放在 `sh/`，安装和构建维护放在 `sh/setup/`；相机独立入口保留在 `camera_ws/`。沿用已有入口，不在项目根目录增加转调脚本。

## 路径约定

文档命令默认在工作空间根目录执行，项目文件使用相对路径。脚本依据自身 `BASH_SOURCE` 推导根目录，Python 模块依据 `__file__` 定位工作区资源，不写死用户名、机器上的检出位置，也不依赖启动终端恰好位于某个目录。

显式输入路径按原入口契约解析；外部 ROS、SDK 和凭据位置由配置或环境变量指定。构建配置按工具规则相对配置文件定位源码，不能机械改成相对终端目录。迁移后从不同目录执行检查或预演，验证默认资源仍来自该工作空间。

| 目录 | 内容 |
| --- | --- |
| `sh/start_web_input.sh` | 启动 Web 和主 Agent，支持文件输入或 VLN preview profile；默认回环监听 |
| `sh/start_log.sh` | 先于被采集模块启动；`--mode live` 或 `--mode bag` 标记会话，Ctrl+C 结束采集 |
| `sh/start_vln_motion.sh` | 实机 Web、持续规划和进程内 tracker 入口 |
| `sh/start_tracker.sh` | 独立轨迹跟踪；默认只输出速度，不创建命令发布者 |
| `sh/start_xnavdp.sh` | 独立 XNavDP 健康查询与推理；透传插件参数 |
| `sh/start.sh` | 启动最小本体通信 |
| `sh/play_bag.sh` | 仅回放 bag 话题及 /clock |
| `sh/record_bag.sh` | 仅录制指定话题到 data/bags/ |
| `sh/internal/rosbag_env.bash` | 录制/回放共用环境加载，不创建 ROS 节点 |
| `sh/internal/runtime_log.bash` | 接入已开启的日志会话，保留原启动进程与终端输出 |
| `sh/run_rviz.sh` | 显式启动可选 XNavDP 显示及其辅助节点 |
| `sh/env.sh` | 激活项目运行环境 |
| `sh/setup/install.sh` | Orin AGX / Ubuntu 24.04 ARM64 工作空间统一安装、相机编译和环境检查入口 |
| `sh/dependencies/` | Python 构建声明、依赖版本约束、源码归档辅助与许可资料 |
| `sh/xnavdp_ros2/` | 共用 DDS 环境与 XML |
| `camera_ws/start_compressor.sh` | 独立 C++ 压缩与启动时时钟检测 |
| `camera_ws/start_cameras.sh` | 唯一真实相机公共入口及环境隔离 |
| `camera_ws/sh/start_cameras.sh` | 相机选择和会话管理实现 |
| `camera_ws/sh/setup_local.bash` | 相机 vendor 环境 |
| `camera_ws/sh/setup/` | 相机增量构建、SDK 准备 |
| `camera_ws/sh/maintenance/` | 相机软件检查、部署包制作 |
| `camera_ws/sh/internal/` | 重启检查、压缩输入检测等内部辅助程序 |

`camera_ws/start_cameras.sh` 是真实相机的唯一公共入口，负责清理继承环境并进入内部实现；不在项目 sh/ 再做转调。bag 回放使用独立的 `sh/play_bag.sh`，不加入相机选择器。

迁移脚本时同步所有调用者、基于 BASH_SOURCE 的根路径计算、测试夹具和现行文档。新增维护任务优先归入已有子目录。

实机、bag 和日志组合命令见 [VLN 启动与排查 skill](../skills/vln-operations/SKILL.md)。

启动入口统一遵循：先停止同项目、同 ROS domain 的同模块旧实例，确认退出后再按当前参数启动；不清理其他模块，不重启远端服务。检查、帮助、dry-run 和健康查询不替换运行实例。
