# VLM 独立请求

在项目根目录运行。默认 `backend: dashscope` 由调用进程直接向 DashScope 发起 WebSocket 请求。项目安装入口 `bash sh/setup/install.sh` 默认准备 agent 依赖。VLN Agent 通过同一 VLM 插件发起推理。

```bash
.venv/bin/python -m dimos.plugins.vlm --check
.venv/bin/python -m dimos.plugins.vlm --health
.venv/bin/python -m dimos.plugins.vlm --image path/to/image.jpg --prompt "描述图像"
```

配置入口是 [vlm.yaml](vlm.yaml)。`credentials_file` 默认复用 Orin 的 `~/.config/dimos/vlm_orin.yaml`。凭据文件可设置 `api_key`、`model` 和 `workspace_id`，也可用 `websocket_url` 指定 WSS 端点。`DASHSCOPE_API_KEY` 可覆盖文件里的密钥。密钥保存在项目外并限制为当前用户可读。`--check` 校验公开配置，`--health` 检查本地凭据与模型字段。带图请求可验证云端推理。

输入支持 JPEG 和 PNG，最大 5 MiB、最长边 4096 像素。返回 `text`、`elapsed_ms`、`source`、`stamp_ns` 和 `image_size`。文件请求的观测时间戳为空。DashScope 模式将图像重新编码成 JPEG 后以 Base64 发送，超过传输尺寸或大小限制时按比例缩小。编码会逐级降低 JPEG 质量，直到 JPEG 不超过 190,000 字节。单次请求受 `timeout_s` 限制。

DashScope 还支持 `VLMClient.query_images((first, second), prompt)`。两张图按顺序分别编码和发送，在同一轮提交并生成一条回复。每张图独立受图像大小限制，两帧发送与模型回复共享请求期限。图片的角色及结果格式由调用方在提示词中指定。

## 兼容 HTTP 端点

既有配置可继续使用。通过 `--config path/to/vlm-http.yaml` 独立调用，或在 VLN task 的 `vlm_config` 中指定该文件。

```yaml
plugin: vlm
backend: http
base_url: http://127.0.0.1:8001
timeout_s: 60.0
max_new_tokens: 256
temperature: 0.1
top_p: 0.9
```

HTTP 模式通过 `/v1/image-inference-base64` 发送一张 JPEG，健康检查读取服务的 `/health` 和 `/info`。省略 `backend` 时按既有 HTTP 配置处理。
