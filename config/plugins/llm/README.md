# LLM 独立请求

入口为 `python -m dimos.plugins.llm`，配置为 `llm.yaml`，依赖由默认安装入口准备。

```bash
python -m dimos.plugins.llm --check
python -m dimos.plugins.llm --health
python -m dimos.plugins.llm --prompt "将去办公室找杯子拆成步骤"
```

复用 Ollama `/api/chat`，调用方显式传 messages，可选 tools 和 JSON schema。默认关闭 thinking，使用配置中的输出 token 上限。返回 assistant message、客户端完整响应耗时及服务端 token/load 指标。每次调用不保留隐式对话历史，传入消息保持原值。

`keep_alive_s` 指定模型在每次请求结束后的空闲驻留秒数，省略时使用 Ollama 的默认值。`LLMClient.preload()` 通过空消息请求加载配置中的模型，返回客户端耗时和服务端可用的加载耗时。

服务、HTTP、协议错误向上返回；不自动重试、不拉取模型。退出只关闭自己的连接。`--health` 读取 `/api/tags`，`--check` 只检查配置。模型及其服务由部署方管理。
