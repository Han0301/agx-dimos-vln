"""Load local DashScope credentials without including them in diagnostics."""

import os
from urllib.parse import urlencode, urlsplit

import yaml


def _mapping(path, label):
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError):
        raise ValueError(f"Could not read {label}: {path}") from None
    if not isinstance(value, dict):
        raise ValueError(f"Expected a mapping in {label}")
    return value


def load_upstream(path):
    config = _mapping(path, "private DashScope config")
    if "qwen_omni_realtime_node" in config:
        config = config["qwen_omni_realtime_node"]
        config = config.get("ros__parameters") if isinstance(config, dict) else None
        if not isinstance(config, dict):
            raise ValueError("Invalid private DashScope config structure")

    api_key = os.environ.get("DASHSCOPE_API_KEY") or config.get("api_key")
    model = config.get("model")
    if not isinstance(api_key, str) or not api_key.strip():
        raise ValueError("Set DASHSCOPE_API_KEY or api_key in the private config")
    if not isinstance(model, str) or not model.strip():
        raise ValueError("Set model in the private DashScope config")

    websocket_url = config.get("websocket_url")
    if not websocket_url:
        workspace = config.get("workspace_id")
        if not isinstance(workspace, str) or not workspace or not all(
            c.isascii() and (c.isalnum() or c == "-") for c in workspace
        ):
            raise ValueError("Set workspace_id or websocket_url in the private config")
        query = urlencode({"model": model})
        websocket_url = (
            f"wss://{workspace}.cn-beijing.maas.aliyuncs.com/api-ws/v1/realtime?{query}"
        )
    if not isinstance(websocket_url, str):
        raise ValueError("websocket_url must be a WSS endpoint")
    endpoint = urlsplit(websocket_url)
    if endpoint.scheme != "wss" or not endpoint.hostname or endpoint.username is not None or endpoint.password is not None:
        raise ValueError("websocket_url must be a WSS endpoint")
    return {"model": model, "websocket_url": websocket_url, "api_key": api_key}
