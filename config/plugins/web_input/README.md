# 独立 Web 任务输入

```bash
python -m dimos.plugins.web_input --check
bash sh/start_web_input.sh
bash sh/start_web_input.sh --profile config/profiles/vln_preview.yaml
```

配置为 `web_input.yaml`，默认监听 `127.0.0.1:5556`，网页接受文字及可选 JPEG/PNG。每个任务独立绑定上传图。单 worker 调用主 Agent，队列和任务记录有容量上限；达到记录上限时按登记顺序淘汰最早的终态记录，没有可淘汰记录时拒绝新任务。

首页提供 VLN Navigation 界面。使用 `vln_preview` 配置时，RealSense RGB 和深度面板读取现有 ROS 相机或 bag 数据。Go2 和 Frontier 面板保留等待提示。Navigate 与 Stop 分别提交和取消任务。多视角找物显示检测、搜索、路径就绪或搜索结束未找到。

## 实现与配置边界

`dimos/plugins/web_input/transport.py` 通过 `runner_factory` 注入执行器，HTTP、队列和任务状态不要求构造 VLN Agent。现有工厂仍默认注册 VLN 首页、`/navigate` 和相机 feed 路由，页面资源仍位于插件目录。模型预热、导航状态文案和默认任务装配位于 `dimos/vln_task/web.py`；旧 `web_input/app.py` 仅作兼容转调。

官方 CLI 仍经 `dimos/plugins/_agent_cli.py` 关联任务和模型配置，配置加载器保留 `task_config` 字段；默认任务是 VLN Task。profile 使用上层装配入口，独立 runner 则可通过 Python 工厂注入。现有配置路径、字段、页面和路由保持原契约。

## HTTP 接口

| 接口 | 输入与结果 |
| --- | --- |
| `GET /` | VLN Navigation 页面 |
| `GET /health` | 当前进程 worker、队列和运行状态。profile 启用 LLM 预加载时包含 `llm_warmup` |
| `POST /tasks`、`POST /navigate` | JSON goal、可选 image_base64，返回 202 和 task_id |
| `POST /navigate` | 网页 multipart 表单，字段为 goal 和可选 image，返回 200、success 和 task_id |
| `GET /status` | 查询网页任务状态，可传 `task_id`；未指定时查询最近的网页任务。搜索状态返回 `stage` 和 `task_id` |
| `POST /stop` | 取消范围见下文；成功返回 `{"success": true}` |
| `GET /tasks/{task_id}` | 查询任务状态和结果。可用 `wait_s=20` 等待任务完成、失败或取消，超时返回当前状态 |
| `POST /tasks/{task_id}/cancel` | 取消指定任务，后续迟到结果丢弃 |
| `GET /realsense_feed`、`GET /depth_feed` | 有相机源时提供 MJPEG；没有相机源时返回 404 |
| `GET /camera_feed`、`GET /frontier_feed` | 当前没有数据源，返回 404，网页显示等待提示 |

`image_base64` 是裸 base64，请求体默认最多 8 MiB，解码图像最多 5 MiB，JPEG/PNG 每边不超过 4096 像素。goal 必须为非空文字且不超过 4000 字符。输入错误返回 422，请求体超限返回 413，队列或记录容量用尽返回 429，按 task_id 查询未知或淘汰的任务返回 404。`wait_s` 的范围为 0–20 秒，超出范围返回 422。任务结果含主 Agent 总耗时和逐步骤指标；`/health` 报告当前进程的队列、worker 和可选预热状态。

## 取消与退出

普通模式下，runner 未提供 `cancel_motion`，`POST /stop` 只取消通过 multipart `/navigate` 表单创建的未完成网页任务；通过 JSON `/tasks` 或 `/navigate` 提交的任务使用对应 task_id 单独取消。运动模式提供 `cancel_motion`，`POST /stop` 会取消当前已登记的所有未完成任务，包括排队中的 API 任务，避免它们在停止后接着执行；正在运行的任务同时通知运动取消钩子。

取消会立即标记本地任务、唤醒等待方，并丢弃后续迟到结果和进度；它不保证正在进行的远端推理立即结束。取消已完成的任务不改变其终态。Web 正常退出时停止取新任务、取消全部未完成任务，等待 worker 和当前请求清理，再关闭自身 runner 资源。远端调用可能持续至返回或超时。队列与任务记录仅保存在当前进程内存。

启动入口按项目、UID 和 ROS_DOMAIN_ID 记录并替换自身旧实例。`--check` 不替换实例。工作进程和 HTTP 资源随本服务退出。
