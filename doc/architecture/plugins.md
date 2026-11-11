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

XNavDP 当前用独立 `bash sh/start_xnavdp.sh` 入口。基座不再提供 algorithms 进程托管；插件配置只由插件入口加载。具体能力见 [配置索引](../../config/plugins/README.md)，约束见 [plugin-contracts.md](plugin-contracts.md)。

## VLN Task 上层归属

`dimos/vln_task/` 是独立上层入口，组织 LLM 工具、当前帧感知、主动搜索、目标刷新和持续接近。基础客户端、Web transport 和 tracker 算法不得反向依赖该目录；既有 `plugins/_agent_cli.py`、旧 `plugins/vln_task/`、`plugins/active_search/` 与 `web_input/app.py` 是兼容入口，可转调同一上层实现。

`web_input/transport.py` 只管理 HTTP、任务队列、状态传输和取消，通过回调注入初始化及展示策略。模型预热和导航状态解释归 `vln_task/web.py`。LLM/VLM 的提示词、历史和调用顺序由任务层提供。控制、碰撞检查、过期停车及通用停止条件留在 tracker。

配置仍沿用原路径和字段；兼容入口与新入口调用同一实现，不保留平行业务代码。
