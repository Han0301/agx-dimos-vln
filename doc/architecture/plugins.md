# 插件组织约定

能力插件有自己的参数、入口和生命周期，通过既有 ROS/服务接口使用数据，不取得相机或 Go2 连接的所有权。每个插件使用独立入口。

| 内容 | 位置 |
| --- | --- |
| 基座参数 | `config/base.yaml`、`config/empty.yaml` |
| 基座启动与配置实现 | `dimos/core/` |
| Python 构建与依赖约束 | `sh/dependencies/` |
| 插件参数与 README | `config/plugins/<name>/` |
| 基础插件实现 | `dimos/plugins/<name>/` |
| VLN Task 上层与搜索 | `dimos/vln_task/`、`dimos/vln_task/search/` |
| 插件测试 | `tests/test_<name>*.py` |
| 共享接口与 DDS | `config/interfaces.yaml`、`sh/xnavdp_ros2/` |

配置是单一来源；不向插件目录复制 vendor、密钥、模型或另一套共享配置。代码中关闭的插件不导入重依赖；`--check` 不创建 ROS 或模型连接。简单无状态适配器不必为了插件化强行新增进程或 HTTP 服务。

新增或迁移插件时，同步默认/显式配置路径、包清单、外部目录启动、测试与链接。README 写清已实现能力、I/O、错误、历史状态和退出语义，不记录当天测试数字。

XNavDP 当前用独立 `bash sh/start_xnavdp.sh` 入口。基座不再提供 algorithms 进程托管；插件配置由独立入口或上层 profile 显式加载。具体能力见 [配置索引](../../config/plugins/README.md)，约束见 [plugin-contracts.md](plugin-contracts.md)。

## VLN Task 上层归属

`dimos/vln_task/` 是独立上层入口，组织 LLM 工具、当前帧感知、主动搜索、目标刷新和持续接近。基础客户端、Web transport 和 tracker 算法不得反向依赖该目录；既有 `plugins/_agent_cli.py`、`plugins/active_search/` 与 `web_input/app.py` 是兼容入口，可转调同一上层实现。

`web_input/transport.py` 的 HTTP、任务队列、状态传输和取消可以通过注入 runner 独立使用，初始化及展示策略由回调提供。现有工厂仍默认提供插件目录中的 VLN 页面，以及导航提交和相机 feed 路由；这部分应用绑定保持现状。模型预热和导航状态解释归 `vln_task/web.py`，上层通过现有工厂与回调复用 Web，不复制传输和队列实现。

`plugins/_agent_cli.py` 仍持有既有命令的参数解析、配置关联及默认 VLN/Web 启动分支；`vln_task/__main__.py` 沿用该入口，profile 装配转入 `vln_task/cli.py`。`plugins/_agent_common.py` 的配置加载器仍识别 `vln_task`，Web 配置中的 `task_config` 仍绑定默认任务配置。`web_input/app.py` 只转调 `vln_task/web.py`，旧 `plugins/active_search/` 只转调上层搜索实现。这些兼容关系不表示命令和配置已完全脱离 VLN 应用。

LLM/VLM 的提示词、历史和调用顺序由任务层提供。控制、碰撞检查、过期停车及通用停止条件留在 tracker。既有插件遵守 [插件保护准则](workspace.md#上层任务与基础插件)；兼容入口的存在不放宽该准则，新增上层策略通过现有接口放在应用目录中。

配置仍沿用原路径和字段；兼容入口与新入口调用同一实现，不保留平行业务代码。
