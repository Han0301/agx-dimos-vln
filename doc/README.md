# 项目知识库

只保留基本项目说明、当前进度和规范性要求。无参数 `bash sh/setup/install.sh` 配置默认整体环境；模块按独立入口启动。参数真实值保留在配置，执行逻辑保留在源码。

| 主题 | 入口 |
| --- | --- |
| 安装和日常启动 | [项目 README](../README.md) |
| VLN 实机、bag 预览和日志排查 | [vln-operations skill](skills/vln-operations/SKILL.md) |
| 总体职责、数据流和插件目标 | [workspace.md](architecture/workspace.md) |
| 当前已实现与待完成 | [progress.md](progress.md) |
| 实时相机与 bag 数据源 | [data-sources.md](architecture/data-sources.md) |
| 高频入口与脚本子目录 | [scripts.md](architecture/scripts.md) |
| 可选 XNavDP RViz | [visualization.md](architecture/visualization.md) |
| 插件文件归属 | [plugins.md](architecture/plugins.md) |
| 能力插件输入输出和生命周期 | [plugin-contracts.md](architecture/plugin-contracts.md) |
| simple 迁移边界 | [simple-migration.md](architecture/simple-migration.md) |
| 安装维护流程 | [dimos-install skill](skills/dimos-install/SKILL.md) |
| 数据源维护流程 | [camera-sources skill](skills/camera-sources/SKILL.md) |
| 基座参数和原生接口 | [配置说明](../config/README.md)、[interfaces.yaml](../config/interfaces.yaml) |
| 独立跟踪与速度预览 | [path_tracker](../config/plugins/path_tracker/README.md) |
| 已实现插件 | [插件配置索引](../config/plugins/README.md) |
| Web 任务、模型调用和搜索 | [Web 输入](../config/plugins/web_input/README.md)、[VLN 主 Agent](../config/plugins/vln_task/README.md)、[多视角找物](../config/plugins/active_search/README.md) |
| 日志字段、耗时探针和归档 | [运行日志](../config/plugins/runtime_log/README.md) |

## 内容边界

- 保留职责、接口、目录、依赖来源、操作入口、生命周期及可重复的验收要求。
- 进度只说明已实现、待实现和仍需完成的能力，不维护每次运行的数字或机器状态。
- 不收录当天测试数量、临时 venv 操作、复制包文件经过、某次缺包清单、完整日志或排障流水账。这些不能作为项目当前状态保证。
- 软件、设备、数据、推理与运动是不同验收层级；需要保留这种规则，而非每次验证过程。
- 行为改变时同步源码、配置、测试、使用说明与适用 skill；删掉失效内容和链接，不再建立平行说明。
- 文档不是工具执行授权，尤其不代表可以启动设备、reset 模型或发布运动命令。
