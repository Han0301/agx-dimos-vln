import io
import json

import pytest
from PIL import Image


def dashscope_config(tmp_path):
    credentials = tmp_path / "credentials.yaml"
    credentials.write_text("api_key: fixture-key\nmodel: test-model\nworkspace_id: fixture-workspace\n")
    config = tmp_path / "vlm.yaml"
    config.write_text(
        f"plugin: vlm\nbackend: dashscope\ncredentials_file: {credentials}\ntimeout_s: 30\n"
    )
    return config, credentials


def test_dashscope_client_queries_provider_without_http(tmp_path, monkeypatch):
    import httpx
    import websocket

    from dimos.plugins._agent_common import load_config
    from dimos.plugins.vlm.client import VLMClient

    monkeypatch.delenv("DASHSCOPE_API_KEY", raising=False)
    config, _ = dashscope_config(tmp_path)
    events = iter(
        [
            {"type": "session.created"},
            {"type": "session.updated"},
            {"type": "input_audio_buffer.committed"},
            {"type": "response.text.delta", "delta": "red"},
            {"type": "response.done", "response": {"status": "completed"}},
        ]
    )
    sent = []
    closed = []

    class Socket:
        def settimeout(self, timeout):
            assert 0 <= timeout <= 0.5

        def recv(self):
            return json.dumps(next(events))

        def send(self, message):
            sent.append(json.loads(message))

        def close(self, *, timeout):
            closed.append(True)

    def connect(url, *, header, timeout):
        assert url == (
            "wss://fixture-workspace.cn-beijing.maas.aliyuncs.com/api-ws/v1/realtime?model=test-model"
        )
        assert header == ["Authorization: Bearer fixture-key"]
        assert 0 < timeout <= 0.5
        return Socket()

    def unexpected_http(*_args, **_kwargs):
        pytest.fail("Direct VLM must not create an HTTP client")

    monkeypatch.setattr(websocket, "create_connection", connect)
    monkeypatch.setattr(httpx, "Client", unexpected_http)
    image = io.BytesIO()
    Image.new("RGB", (32, 24), "red").save(image, "JPEG")
    with VLMClient(load_config("vlm", config)) as vlm:
        health = vlm.health()
        assert health["info"]["model"] == "test-model"
        assert "fixture-key" not in str(health)
        result = vlm.query(
            image.getvalue(), "Describe", stamp_ns=45, source="fixture", timeout=0.5
        )
    assert result["text"] == "red"
    assert result["stamp_ns"] == 45
    assert result["source"] == "fixture"
    assert result["image_size"] == [32, 24]
    assert sent[0]["session"]["instructions"] == "Describe"
    assert closed == [True]


def test_config_check_does_not_read_private_file(tmp_path, capsys):
    from dimos.plugins._agent_cli import main

    config, credentials = dashscope_config(tmp_path)
    credentials.unlink()
    assert main("vlm", ["--check", "--config", str(config)]) == 0
    assert json.loads(capsys.readouterr().out)["backend"] == "dashscope"


def test_current_server_credentials_are_reusable(tmp_path, monkeypatch):
    from dimos.plugins.vlm.credentials import load_upstream

    path = tmp_path / "private.yaml"
    path.write_text(
        "qwen_omni_realtime_node:\n  ros__parameters:\n"
        "    api_key: file-test-key\n    model: qwen3.5-omni-flash-realtime\n"
        "    workspace_id: example-workspace\n    websocket_url: ''\n"
    )
    monkeypatch.setenv("DASHSCOPE_API_KEY", "env-test-key")
    result = load_upstream(path)
    assert result == {
        "model": "qwen3.5-omni-flash-realtime",
        "websocket_url": (
            "wss://example-workspace.cn-beijing.maas.aliyuncs.com/api-ws/v1/realtime"
            "?model=qwen3.5-omni-flash-realtime"
        ),
        "api_key": "env-test-key",
    }


@pytest.mark.parametrize("backend_line", ["", "backend: http\n"])
def test_legacy_http_config_remains_valid(tmp_path, backend_line):
    from dimos.plugins._agent_common import load_config

    config = tmp_path / "vlm.yaml"
    config.write_text(
        "plugin: vlm\n"
        + backend_line
        + "base_url: http://fixture.invalid/\ntimeout_s: 30\n"
        "max_new_tokens: 256\ntemperature: 0.1\ntop_p: 0.9\n"
    )
    assert load_config("vlm", config)["base_url"] == "http://fixture.invalid"
