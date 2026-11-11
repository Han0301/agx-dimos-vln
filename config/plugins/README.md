# 基础插件与上层任务配置

每个插件使用 `config/plugins/<插件名>/` 保存专属参数和使用说明。基础通信配置仍在 `config/` 根目录，插件使用自己的加载器和独立入口。

| 插件 | 参数 | 使用说明 | Python 入口 |
| --- | --- | --- | --- |
| Path tracker | [tracker.yaml](path_tracker/tracker.yaml) | [README](path_tracker/README.md) | `python -m dimos.plugins.path_tracker` |
| VLM 客户端 | [vlm.yaml](vlm/vlm.yaml) | [README](vlm/README.md) | `python -m dimos.plugins.vlm` |
| LLM | [llm.yaml](llm/llm.yaml) | [README](llm/README.md) | `python -m dimos.plugins.llm` |
| VLN Task 上层入口 | [vln_task.yaml](vln_task/vln_task.yaml) | [README](vln_task/README.md) | `python -m dimos.vln_task` |
| VLN Task 搜索工作流 | [active_search.yaml](active_search/active_search.yaml) | [README](active_search/README.md) | `python -m dimos.vln_task.search --check` |
| Web input | [web_input.yaml](web_input/web_input.yaml) | [README](web_input/README.md) | `python -m dimos.plugins.web_input` |
| 运行日志 | 默认 `data/log` | [README](runtime_log/README.md) | `bash sh/start_log.sh` |
| XNavDP | [xnavdp_test.yaml](xnavdp/xnavdp_test.yaml) | [README](xnavdp/README.md) | `python -m dimos.plugins.xnavdp` |

基础插件实现位于 `dimos/plugins/<插件名>/`，VLN Task 及搜索工作流位于 `dimos/vln_task/`；配置保留现有 `config/plugins/` 路径，测试位于 `tests/`。共用的 DDS 配置和接口契约不复制到插件目录。新增或迁移插件时，按 [知识库组织约定](../../doc/architecture/plugins.md) 同步默认路径、测试和文档索引。

非执行性检查示例，在项目根目录运行：

```bash
python -m dimos.plugins.xnavdp --check
python -m dimos.plugins.xnavdp --config config/plugins/xnavdp/xnavdp_test.yaml --check
```

真实 health 查询、推理和 reset 的作用范围见插件自己的使用说明。VLM、LLM、主 Agent 和 Web 支持独立文件输入。[VLN preview profile](../profiles/vln_preview.yaml) 装配 D455 或 bag 的观测、目标定位、多视角搜索、XNavDP 和一次 MPC 预览。[VLN motion profile](../profiles/vln_motion.yaml) 复用 Web preview 的配置，在 Web 进程内启动持续规划与实机 tracker。组合启动见 [VLN 启动与排查 skill](../../doc/skills/vln-operations/SKILL.md)。
