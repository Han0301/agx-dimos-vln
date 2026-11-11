# 安装入口验收要求

维护 `sh/setup/install.sh` 时按改动选择相关项；不要求每次编辑都联网下载、重装依赖或连接设备。当前基线来自项目脚本，新增能力需同时补充实现、命令说明与行为验证。

## 命令和模式

以下命令在项目根目录执行。修改安装器通常先运行无安装副作用的预演：

```bash
bash -n sh/setup/install.sh
bash sh/setup/install.sh --dry-run
bash sh/setup/install.sh --dry-run --camera all
```

| 模式 | 应有行为 | 不应被误报的结果 |
| --- | --- | --- |
| 默认运行 | 默认补齐系统包，构建 D455/C++ 压缩，安装 Go2/path_tracker/agent 客户端并检查插件配置 | 不能只安装 Python 就报告整个工作空间可用 |
| `--check-only` | 检查当前软件；可使用并清理临时目录 | 不执行 apt、pip 安装、SDK 下载、构建或 venv 重建 |
| `--dry-run` | 展示计划，可读取现状 | 不创建 venv、下载缓存或构建产物；不表示前置条件已满足 |
| `--recreate` | 将原 venv 改名备份后创建独立环境 | 不直接删除已有 venv，不声称备份可免验证地跨位置运行 |
| `--skip-build` | 复用现有产物，继续验证来源和动态库 | 不能因为跳过构建就跳过相机验收 |
| `--offline` | 禁止 APT 和 SDK URL 下载；uv 使用缓存，pip 使用显式 wheelhouse | 缓存或构建依赖不足时不能联网补齐后仍称离线成功 |
| 非法或冲突参数 | 安装副作用前报错 | 不忽略参数后继续安装 |

默认重复执行仍可能增量构建和调用包安装器；只检查应使用 `--check-only`。非默认相机必须传入对应参数，当前实现不持久化选择。

## 软件检查覆盖

- 前置环境：系统、架构、Python ABI、源码和匹配的 vendor。当前 `--check-only` 也检查安装器声明的系统构建依赖，缺失会失败。
- venv：目录与 prefix 对应、Python 3.12、禁止系统 site-packages、editable 指向本项目。检查导入时移除会掩盖错误安装来源的 `PYTHONPATH`、`PYTHONHOME`。
- Python：必需包及所选 extra 可导入，已安装的受约束依赖版本正确，`uv pip check` 或 `pip check` 通过。当前 `requirements.lock` 用作版本约束，不把它表述为带哈希的完整锁定安装。
- ROS：`rclpy` 来源在本工作空间内；消息类及原生类型支持可加载；DDS 配置可解析。
- 生命周期：使用 `config/empty.yaml`，禁用设备模块；设置超时、收集退出状态和日志。超时、非零退出、DDS/RTPS 原生错误均导致失败。发现错误后不得通过删除错误检测或吞日志制造通过。
- 相机：用实际相机进程的系统 Python 检查 OpenCV、NumPy、ROS launch 和公共工具；包 prefix 在当前 `camera_ws/install`；动态库可解析；launch 描述可加载。X3 还需检查源码和安装区 SDK/JSON 一致。

XML 可解析不代表远端 DDS 服务已连通；`--show-args` 可加载不代表驱动已取得图像。用户请求实机验收时，另行检查真实话题的数据、时间戳和相机标定，不用发现话题名代替数据到达。

## 默认跟踪依赖

无参数入口安装 `dimos[go2,path_tracker,agent]`，验证 NumPy、SciPy、PyYAML 位于本工作空间 .venv，执行无 ROS/设备的 MPC 求解，并检查 XNavDP/tracker 参数。默认 D455 系统依赖和相机增量构建一并完成。`--check-only` 只检查；`--dry-run` 只展示计划。不兼容或继承系统包的旧 venv 默认自动备份重建，不放宽隔离校验；兼容环境重复使用。

## 独立 agent 依赖

`--plugin all` 是默认值，同时准备跟踪和 agent。`--plugin agent` 仅附加 HTTP、Pillow、FastAPI、uvicorn 客户端依赖；保留 Go2 基础依赖。安装检查验证这些包来自本项目 .venv，执行四个插件的 `--check`，不请求远端模型。`--plugin path_tracker` 和 `none` 保留裁剪语义。模型请求、Web 服务与 ROS/设备数据按用户任务独立验收。

## 相机与 SDK 组合

| 选择 | 构建和检查重点 |
| --- | --- |
| `--camera d455` | RealSense、C++ 压缩器与公共工具；没有 CameraSDK 时也能配置构建；X3 专用程序关闭 |
| `--camera x3` | X3 SDK、专用程序与公共工具；不强制要求 D455 真机接入 |
| `--camera all` | 两套相机软件依赖；显式启用 X3 CMake 开关 |
| `--camera none` | 省略相机构建和相机产物检查，仍需 venv 和本体通信所用 ROS 软件环境 |

SDK 测试关注正常归档、目录穿越、链接/特殊文件、错误架构、缺失文件、错误校验值和多份嵌套 SDK 的明确选择。新增或修改归档处理时验证相关边界；检查失败不得留下被当作完整下载复用的半成品。

## 设备状态与停止条件

软件检查通过后，Go2 凭据格式和有线接口/路由、X3 USB 访问等可以标为 `READY` 或 `PENDING`。D455 当前始终提示需要真实 USB 权限与数据测试。整体零退出只说明软件检查通过，不表示所有 `PENDING` 已解决。

安装、构建或软件检查失败就停止并报告原因；不自动循环重装，也不通过放宽隔离、修改服务器地址或连接机器人来掩盖失败。离线缓存或 SDK 来源不足时，提供缺项与修复命令。

## 验证命令和证据

```bash
python -m pytest tests/test_install.py -q
python -m pytest tests -q
# 现有环境检查；会运行无设备的 ROS 节点生命周期：
bash sh/setup/install.sh --check-only
```

使用项目匹配的测试 Python 和源码路径。若宿主机自动加载无关的 pytest 插件而干扰收集，可显式禁用该次运行的插件自动加载，并说明使用的环境；不要将测试夹具或其他工作空间路径作为正式运行依赖。

需要证明首次构建时，在临时构建目录验证，避免现有 CameraSDK 或 `install/` 掩盖缺项。需要证明全新安装时，使用干净环境和完整输入；复用已有包文件的导入检查只能证明该软件检查路径可运行。

交付保留实际命令、退出结果和未覆盖项。知识库只保留维护要求，不保存单次测试数量或缺包清单。
