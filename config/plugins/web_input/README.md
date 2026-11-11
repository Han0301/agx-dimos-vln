# 独立 Web 任务输入

```bash
python -m dimos.plugins.web_input --check
bash sh/start_web_input.sh
bash sh/start_web_input.sh --profile config/profiles/vln_preview.yaml
```

配置为 `web_input.yaml`，默认监听 `127.0.0.1:5556`，网页接受文字及可选 JPEG/PNG。每个任务独立绑定上传图。单 worker 调用主 agent，队列和终态记录有容量上限；达到记录上限时优先淘汰最早的终态记录。

首页提供 VLN Navigation 界面。使用 `vln_preview` 配置时，RealSense RGB 和深度面板读取现有 ROS 相机或 bag 数据。Go2 和 Frontier 面板保留等待提示。Navigate 与 Stop 分别提交和取消任务。多视角找物显示搜索、复核、路径就绪或搜索结束未找到。

| 接口 | 输入与结果 |
| --- | --- |
| `GET /` | VLN Navigation 页面 |
| `GET /health` | 当前进程 worker、队列和运行状态。profile 启用 LLM 预加载时包含 `llm_warmup` |
| `POST /tasks`、`POST /navigate` | JSON goal、可选 image_base64，返回 202 和 task_id |
| `POST /navigate` | 网页 multipart 表单，字段为 goal 和可选 image，返回 success 和 task_id |
| `GET /status`、`POST /stop` | 查询网页任务状态、取消未完成的网页任务。查询可传 `task_id`，搜索状态返回 `stage` 和 `task_id` |
| `GET /tasks/{task_id}` | 查询任务状态和结果。可用 `wait_s=20` 等待任务完成、失败或取消，超时返回当前状态 |
| `POST /tasks/{task_id}/cancel` | 取消指定任务，后续迟到结果丢弃 |

image_base64 是裸 base64，JSON 请求体默认最多 8 MiB，解码图像最多 5 MiB。输入错误 422、请求体超限 413、队列/记录满 429、未知或淘汰任务 404。任务结果含主 agent 总耗时和逐步骤指标。取消在本地立即可见，正在进行的远端推理可能执行至返回或超时；进程退出等待当前请求清理。队列仅保存在本地内存。

启动入口按项目、UID 和 ROS_DOMAIN_ID 记录并替换自身旧实例。`--check` 不替换实例。工作进程和 HTTP 资源随本服务退出。
