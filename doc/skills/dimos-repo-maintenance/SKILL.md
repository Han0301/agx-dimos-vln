---
name: dimos-repo-maintenance
description: 核对和维护 DimOS Mininum / agx-dimos-vln 的仓库规范与插件化设计；用于目录整理、插件拆分、接口评审、保持行为的重构及知识库同步。仅适用于该系统；安装、采集和实机运行按对应专项 skill 处理。
---

# DimOS 仓库与插件维护

先定位用户指定的检出目录，读取该目录的 `AGENTS.md` 和 [知识库索引](../../README.md)，核对当前源码。此 skill 托管于 `doc/skills/dimos-repo-maintenance/`；经软链接加载时按真实位置定位仓库，不假设所有检出都在同一路径。

## 按改动选择资料

- 分层与依赖方向：[工作空间架构](../../architecture/workspace.md)。
- 插件设计、输入输出、独立验收：[插件契约](../../architecture/plugin-contracts.md)；目录与兼容入口见 [插件组织](../../architecture/plugins.md)。
- 脚本和配置迁移：[脚本约定](../../architecture/scripts.md)、[配置索引](../../../config/plugins/README.md) 和实际调用者。
- 安装/SDK/编译转 [dimos-install](../dimos-install/SKILL.md)，相机/bag/时钟转 [camera-sources](../camera-sources/SKILL.md)，实机装配/回放/运行日志转 [vln-operations](../vln-operations/SKILL.md)。只补读相关部分，不复制专项流程。

## 插件化审查

按「调用方 → 输入 → 最小能力 → 输出/错误 → 资源归属」追踪真实代码，再判断逻辑放置。

- `dimos/vln_task/` 是上层，组织任务提示词、历史、工具调用、扫视、深度恢复、目标刷新及接近策略。LLM/VLM/XNavDP 客户端和 Web transport 不组织找物流程。
- 基础插件保留必要的协议解析、输入输出校验、超时及资源清理。tracker 的 MPC、碰撞检查、数据过期停车属于完整控制能力，不因“只保留输入输出”而剥离。
- 基础能力可以注入假服务、假观测或假执行器单独验证，不以构造完整 VLN Agent、连接 Go2 或启动相机为前提。简单函数不强制增加进程、HTTP 或空生命周期。
- 基础实现不反向依赖上层。既有 `_agent_cli.py`、旧搜索命名空间和 `web_input/app.py` 属兼容入口，可转调同一上层实现；不得在其中再维护一套策略。
- 基座持有 Go2 连接；相机由数据源入口持有。插件消费既有接口，不接管设备，不增加无必要的转发话题或任务总线。配置、DDS、模型和凭据各保留单一来源。

## 目录与兼容约束

- 文档命令以工作空间根目录为执行位置，项目路径写为相对路径；脚本/模块通过 `BASH_SOURCE`、`__file__` 推导根目录，不固化用户名或检出位置。显式输入保持原解析语义，外部 ROS/SDK/凭据由配置或环境变量指定，构建文件遵循工具自身的相对路径规则。迁移后补做外部目录及不同检出位置的只读检查。

- 安装编译沿用 `sh/setup/install.sh`，本体通信沿用 `sh/start.sh`；工作空间脚本放 `sh/`，维护放其子目录，相机独立入口保留 `camera_ws/`。不在项目根目录增加包装入口。
- 包入口保留在 `dimos/__init__.py`、`dimos/__main__.py`；本体 CLI、配置与运行生命周期归 `dimos/core/`。构建声明与依赖约束集中在 `sh/dependencies/pyproject.toml`、`sh/dependencies/requirements.lock`，不散放在工作空间或包根目录。`requirements.lock` 不是许可证。
- `sh/dependencies/` 通过 `../../dimos` 直接引用唯一源码，不放源码链接或副本；许可资料归其 `licenses/`。源码归档辅助 `_build.py` 仅在发布树内暂存源码，过滤越界路径，解除元数据硬链接后再改写归档映射。迁移安装根时分别核对 editable 与实际包来源，验证源码归档可独立重建 wheel，且工作区文件及目录不被构建污染。
- 移动模块前找调用者、默认/显式路径、根路径计算、shell 入口及测试夹具；同步 `sh/dependencies/pyproject.toml` 的显式 `packages` 和必要的 `package-data`。兼容转调只指向一份实现。
- 纯整理保留原话题、坐标系、时间戳关联、模型提示词/工具定义、HTTP 路由、配置字段、阈值、调用顺序、超时、取消与停止语义。业务或协议变更按用户明确的任务范围单独实现。

## 核验与交付

按影响选择验证：目录与链接检查、相关单元/假服务测试、CLI `--check`、shell 语法及 `--dry-run`；涉及打包时检查 wheel 内容和脱离源码目录的导入。不能用删除断言、吞错或生成假产物解决验收失败。

目标设备是 Orin AGX / Ubuntu 24.04 ARM64。宿主机的静态与模拟检查不等同目标设备编译或实机效果；涉及设备故障应在实际故障设备验证。整理请求不自动启动设备、reset 模型、发布运动或修改系统路由。用户要求保留进程时不重启；规范中的生命周期要求也不能当作所有现有入口已经实现的事实。

架构图的 DOT 源与 SVG/PNG 导出集中在 `doc/architecture/diagrams/`；流程变化时同步图源、导出、README 与飞书的职责说明，不把临时截图或验收日志放入该目录。

同步 README、知识库、配置说明、入口和相关 skill；文档仅保留职责、接口、可复用操作与真实进度，不写当天测试数量、临时环境过程或机器在线状态。交付说明实际改动、验证结果和未覆盖项；核对发现的问题应区分规范冲突、代码缺陷和后续建议。
