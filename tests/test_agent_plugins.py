"""Standalone protocol and task tests. No ROS, robot, camera or remote model."""

import base64
import io
import json
import threading
import time

import httpx
import pytest
from PIL import Image


def components():
    try:
        from dimos.plugins.llm.client import LLMClient
        from dimos.plugins.vlm.client import VLMClient
        from dimos.vln_task.agent import TaskAgent, TaskCancelled
        from dimos.plugins.web_input.app import create_app

        return VLMClient, LLMClient, TaskAgent, TaskCancelled, create_app
    except ModuleNotFoundError as exc:
        pytest.fail(f"Standalone agent capability is missing: {exc}")


@pytest.fixture
def jpeg():
    buffer = io.BytesIO()
    Image.new("RGB", (32, 24), "red").save(buffer, "JPEG")
    return buffer.getvalue()


def settings(name):
    from dimos.plugins._agent_common import load_config

    if name == "vlm":
        return {
            "plugin": "vlm",
            "base_url": "http://fixture.invalid",
            "timeout_s": 30,
            "max_new_tokens": 256,
            "temperature": 0.1,
            "top_p": 0.9,
        }
    return load_config(name)


def test_vlm_preserves_jpeg_and_native_protocol(jpeg):
    VLMClient, *_ = components()

    def remote(request):
        body = json.loads(request.content)
        assert request.url.path == "/v1/image-inference-base64"
        assert base64.b64decode(body["image_base64"]) == jpeg
        assert body["prompt"] == "Find the red shape"
        assert body["max_new_tokens"] == 256
        return httpx.Response(
            200, json={"status": "success", "response": '{"color":"red"}'}
        )

    with httpx.Client(transport=httpx.MockTransport(remote)) as http:
        with VLMClient(settings("vlm"), http) as client:
            result = client.query(
                jpeg, "Find the red shape", source="fixture", stamp_ns=123
            )
    assert result["text"] == '{"color":"red"}'
    assert result["source"] == "fixture" and result["stamp_ns"] == 123
    assert result["image_size"] == [32, 24]
    assert result["elapsed_ms"] >= 0


@pytest.mark.parametrize(
    "response",
    [
        {"status": "error", "error": "provider unavailable"},
        {"status": "success", "response": None},
        {"status": "success", "response": ""},
    ],
)
def test_vlm_rejects_service_failure_and_invalid_output(jpeg, response):
    VLMClient, *_ = components()
    http = httpx.Client(
        transport=httpx.MockTransport(lambda _: httpx.Response(200, json=response))
    )
    with http, VLMClient(settings("vlm"), http) as client:
        with pytest.raises(ValueError):
            client.query(jpeg, "describe")


def test_vlm_rejects_invalid_image_before_request():
    VLMClient, *_ = components()

    def unexpected(_):
        pytest.fail("Invalid image must fail before model inference")

    with httpx.Client(transport=httpx.MockTransport(unexpected)) as http:
        with VLMClient(settings("vlm"), http) as client:
            with pytest.raises(ValueError):
                client.query(b"not an image", "describe")


def test_llm_passes_explicit_history_tools_and_options():
    _, LLMClient, *_ = components()
    messages = [{"role": "user", "content": "hello"}]

    def remote(request):
        payload = json.loads(request.content)
        assert request.url.path == "/api/chat"
        assert payload["messages"] == messages
        assert payload["stream"] is False and payload["think"] is False
        assert payload["options"]["num_predict"] == 256
        return httpx.Response(
            200,
            json={
                "model": "qwen3:14b",
                "done": True,
                "message": {"role": "assistant", "content": "Hello"},
                "eval_count": 3,
                "eval_duration": 100000000,
                "load_duration": 5000000,
            },
        )

    with httpx.Client(transport=httpx.MockTransport(remote)) as http:
        with LLMClient(settings("llm"), http) as client:
            result = client.chat(messages)
    assert result["message"]["content"] == "Hello"
    assert result["tokens_per_s"] == 30
    assert result["load_ms"] == 5
    assert messages == [{"role": "user", "content": "hello"}]


def test_llm_preloads_without_generating_and_keeps_the_model_after_chat():
    _, LLMClient, *_ = components()
    config = {**settings("llm"), "keep_alive_s": 900}
    messages = [{"role": "user", "content": "hello"}]
    payloads = []

    def remote(request):
        assert request.url.path == "/api/chat"
        payload = json.loads(request.content)
        payloads.append(payload)
        if not payload.get("messages"):
            return httpx.Response(
                200,
                json={"model": config["model"], "done": True, "done_reason": "load"},
            )
        return httpx.Response(
            200,
            json={
                "done": True,
                "message": {"role": "assistant", "content": "Hello"},
            },
        )

    with httpx.Client(transport=httpx.MockTransport(remote)) as http:
        client = LLMClient(config, http)
        preloaded = client.preload(timeout=1)
        answer = client.chat(messages)
    assert payloads[0] == {
        "model": config["model"],
        "messages": [],
        "stream": False,
        "keep_alive": 900,
    }
    assert payloads[1]["messages"] == messages
    assert payloads[1]["keep_alive"] == 900
    assert preloaded["model"] == config["model"]
    assert preloaded["load_ms"] is None
    assert preloaded["elapsed_ms"] >= 0
    assert answer["message"]["content"] == "Hello"


def test_agent_executes_visual_tool_then_returns_answer(jpeg):
    VLMClient, LLMClient, TaskAgent, *_ = components()
    calls = []

    def remote(request):
        body = json.loads(request.content)
        calls.append(request.url.path)
        if request.url.path.endswith("image-inference-base64"):
            return httpx.Response(
                200, json={"status": "success", "response": "A red rectangle"}
            )
        if len(body["messages"]) == 2:
            return httpx.Response(
                200,
                json={
                    "done": True,
                    "message": {
                        "role": "assistant",
                        "content": "",
                        "tool_calls": [
                            {
                                "function": {
                                    "name": "inspect_image",
                                    "arguments": {"prompt": "Describe colors"},
                                }
                            }
                        ],
                    },
                },
            )
        assert body["messages"][-1]["role"] == "tool"
        assert "A red rectangle" in body["messages"][-1]["content"]
        return httpx.Response(
            200,
            json={
                "done": True,
                "message": {"role": "assistant", "content": "红色矩形"},
            },
        )

    with httpx.Client(transport=httpx.MockTransport(remote)) as http:
        with (
            VLMClient(settings("vlm"), http) as vlm,
            LLMClient(settings("llm"), http) as llm,
        ):
            result = TaskAgent(llm, vlm, settings("vln_task")).run("描述图像", jpeg)
    assert result["answer"] == "红色矩形"
    assert result["tool_calls"] == 1
    assert calls == ["/api/chat", "/v1/image-inference-base64", "/api/chat"]
    assert len(result["steps"]) == 3


@pytest.mark.parametrize("tool", ["move", "reset", "unknown"])
def test_agent_rejects_unregistered_tools(tool):
    VLMClient, LLMClient, TaskAgent, *_ = components()
    response = {
        "done": True,
        "message": {
            "role": "assistant",
            "content": "",
            "tool_calls": [{"function": {"name": tool, "arguments": {}}}],
        },
    }
    with httpx.Client(
        transport=httpx.MockTransport(lambda _: httpx.Response(200, json=response))
    ) as http:
        with (
            VLMClient(settings("vlm"), http) as vlm,
            LLMClient(settings("llm"), http) as llm,
        ):
            with pytest.raises(ValueError, match="tool"):
                TaskAgent(llm, vlm, settings("vln_task")).run("test")


def test_agent_cancellation_discards_late_reply():
    VLMClient, LLMClient, TaskAgent, TaskCancelled, _ = components()
    cancel = threading.Event()

    def remote(_):
        cancel.set()
        return httpx.Response(
            200,
            json={"done": True, "message": {"role": "assistant", "content": "late"}},
        )

    with httpx.Client(transport=httpx.MockTransport(remote)) as http:
        with (
            VLMClient(settings("vlm"), http) as vlm,
            LLMClient(settings("llm"), http) as llm,
        ):
            with pytest.raises(TaskCancelled):
                TaskAgent(llm, vlm, settings("vln_task")).run("test", cancel=cancel)


def test_agent_rejects_invalid_image_before_model_request():
    VLMClient, LLMClient, TaskAgent, *_ = components()

    def remote(_):
        pytest.fail("Invalid task image must fail before requesting the LLM")

    with httpx.Client(transport=httpx.MockTransport(remote)) as http:
        with (
            VLMClient(settings("vlm"), http) as vlm,
            LLMClient(settings("llm"), http) as llm,
        ):
            with pytest.raises(ValueError, match="image"):
                TaskAgent(llm, vlm, settings("vln_task")).run("describe", b"invalid")


def test_web_tasks_isolate_images_and_report_actual_result(jpeg):
    *_, create_app = components()
    from fastapi.testclient import TestClient

    received = []

    class Runner:
        def run(self, goal, image=None, cancel=None):
            received.append((goal, image))
            return {"answer": goal, "elapsed_ms": 1}

    with TestClient(create_app(settings("web_input"), Runner)) as web:
        response = web.post(
            "/tasks",
            json={"goal": "red", "image_base64": base64.b64encode(jpeg).decode()},
        )
        assert response.status_code == 202
        task_id = response.json()["task_id"]
        for _ in range(100):
            task = web.get(f"/tasks/{task_id}").json()
            if task["status"] in ("succeeded", "failed"):
                break
            time.sleep(0.01)
        assert task["status"] == "succeeded" and task["result"]["answer"] == "red"
        assert received == [("red", jpeg)]
        assert web.get("/tasks/unknown").status_code == 404
        assert web.post("/tasks", json={"goal": " "}).status_code == 422
        assert (
            web.post("/tasks", json={"goal": "bad", "image_base64": "???"}).status_code
            == 422
        )


def test_web_cancellation_and_queue_limit(jpeg):
    *_, create_app = components()
    from fastapi.testclient import TestClient

    entered = threading.Event()
    release = threading.Event()

    class Runner:
        def run(self, goal, image=None, cancel=None):
            entered.set()
            release.wait(2)
            return {"answer": "late"}

    config = settings("web_input")
    config["queue_size"] = 1
    with TestClient(create_app(config, Runner)) as web:
        active = web.post("/tasks", json={"goal": "first"}).json()["task_id"]
        assert entered.wait(1)
        queued = web.post("/tasks", json={"goal": "second"}).json()["task_id"]
        assert web.post("/tasks", json={"goal": "third"}).status_code == 429
        assert web.post(f"/tasks/{active}/cancel").json()["status"] == "cancelled"
        assert web.post(f"/tasks/{queued}/cancel").status_code == 200
        release.set()
        time.sleep(0.05)
        task = web.get(f"/tasks/{active}").json()
        assert task["status"] == "cancelled" and task["result"] is None


def test_agent_bounds_repeated_tools_and_preserves_task_history_isolation(jpeg):
    VLMClient, LLMClient, TaskAgent, *_ = components()
    initial_messages = []

    def remote(request):
        if request.url.path.endswith("image-inference-base64"):
            return httpx.Response(200, json={"status": "success", "response": "red"})
        body = json.loads(request.content)
        initial_messages.append(len(body["messages"]))
        return httpx.Response(
            200,
            json={
                "done": True,
                "message": {
                    "role": "assistant",
                    "content": "",
                    "tool_calls": [
                        {
                            "function": {
                                "name": "inspect_image",
                                "arguments": {"prompt": "color"},
                            }
                        }
                    ],
                },
            },
        )

    config = settings("vln_task")
    config["max_steps"] = 2
    with httpx.Client(transport=httpx.MockTransport(remote)) as http:
        with (
            VLMClient(settings("vlm"), http) as vlm,
            LLMClient(settings("llm"), http) as llm,
        ):
            agent = TaskAgent(llm, vlm, config)
            for _ in range(2):
                with pytest.raises(ValueError, match="max_steps"):
                    agent.run("color?", jpeg)
    assert initial_messages == [2, 4, 2, 4]


@pytest.mark.parametrize(
    "payload", [[], None, {"role": "assistant", "content": "", "tool_calls": [None]}]
)
def test_llm_or_agent_rejects_malformed_provider_response(payload):
    VLMClient, LLMClient, TaskAgent, *_ = components()
    with httpx.Client(
        transport=httpx.MockTransport(
            lambda _: httpx.Response(200, json={"done": True, "message": payload})
        )
    ) as http:
        with (
            VLMClient(settings("vlm"), http) as vlm,
            LLMClient(settings("llm"), http) as llm,
        ):
            with pytest.raises(ValueError):
                TaskAgent(llm, vlm, settings("vln_task")).run("test")


def test_web_startup_reports_invalid_runner():
    *_, create_app = components()
    from fastapi.testclient import TestClient

    def broken():
        raise ValueError("invalid model config")

    with pytest.raises(ValueError, match="invalid model config"):
        with TestClient(create_app(settings("web_input"), broken)):
            pass


def test_web_rejects_oversize_body():
    *_, create_app = components()
    from fastapi.testclient import TestClient

    config = settings("web_input")
    config["max_body_bytes"] = 32

    class Runner:
        def run(self, *args, **kwargs):
            pytest.fail("Oversize request must not execute")

    with TestClient(create_app(config, Runner)) as web:
        assert web.post("/tasks", json={"goal": "x" * 64}).status_code == 413


def test_all_check_entrypoints_work_without_ros_or_http_imports():
    components()
    import subprocess
    import sys
    from pathlib import Path

    root = Path(__file__).resolve().parents[1]
    code = """
import sys
from dimos.plugins._agent_cli import main
for name in ('vlm', 'llm', 'vln_task', 'web_input'):
    assert main(name, ['--check']) == 0
assert not {'rclpy', 'httpx', 'fastapi', 'numpy'} & sys.modules.keys()
"""
    result = subprocess.run(
        [sys.executable, "-c", code], cwd=root, capture_output=True, text=True
    )
    assert result.returncode == 0, result.stderr


def test_simple_web_form_preserves_upload_and_cancels_preview(jpeg):
    from fastapi.testclient import TestClient

    *_, create_app = components()
    received = []

    class Runner:
        def run(self, goal, image, cancel):
            received.append((goal, image))
            cancel.wait(2)
            return {}

    with TestClient(create_app(settings("web_input"), Runner)) as web:
        response = web.post(
            "/navigate",
            files={"goal": (None, "找红色目标"), "image": ("ref.jpg", jpeg, "image/jpeg")},
        )
        assert response.status_code == 200
        payload = response.json()
        assert payload["success"] is True
        task_url = f"/tasks/{payload['task_id']}"
        for _ in range(100):
            if web.get(task_url).json()["status"] == "running":
                break
            time.sleep(0.01)
        assert web.get("/status").json()["status"] == "navigating: 找红色目标"
        assert web.post("/stop").json()["success"] is True
        assert web.get("/status").json()["status"] == "stopped"
        assert web.get(task_url).json()["status"] == "cancelled"
        assert received == [("找红色目标", jpeg)]
        assert web.post("/navigate", json={"goal": "API client"}).status_code == 202


def test_simple_web_camera_routes_show_placeholders_without_sources():
    from fastapi.testclient import TestClient

    *_, create_app = components()
    with TestClient(create_app(settings("web_input"), object)) as web:
        page = web.get("/")
        assert "<title>VLN Navigation</title>" in page.text
        for route in ("/camera_feed", "/realsense_feed", "/depth_feed", "/frontier_feed"):
            assert f'src="{route}"' in page.text
            assert web.get(route).status_code == 404


def test_vlm_logs_exact_http_input_after_jpeg_conversion_even_on_failure(tmp_path):
    from dimos.plugins.runtime_log import capture, task_context

    VLMClient, *_ = components()
    source = io.BytesIO()
    Image.new("RGB", (32, 24), "green").save(source, "PNG")
    sent = []

    def remote(request):
        sent.append(base64.b64decode(json.loads(request.content)["image_base64"]))
        return httpx.Response(503, text="provider unavailable")

    with capture("web_input", base=tmp_path) as run, task_context("test-task"):
        with httpx.Client(transport=httpx.MockTransport(remote)) as http:
            with VLMClient(settings("vlm"), http) as client:
                with pytest.raises(httpx.HTTPStatusError):
                    client.query(source.getvalue(), "describe", source="explicit_image")

    records = [json.loads(line) for line in (run / "modules/vlm.jsonl").read_text().splitlines()]
    call = next(row for row in records if row.get("event") == "vlm_call_finished")
    assert call["status"] == "failed"
    assert call["task_id"] == "test-task"
    assert call["input_image_archive_status"] == "queued"
    assert call["call_id"] in call["input_image_artifact"]
    assert sent[0].startswith(b"\xff\xd8")
    assert (run / call["input_image_artifact"]).read_bytes() == sent[0]


@pytest.mark.parametrize("first_arguments", ['{"target":"black water dispenser"}', {"goal": "black water dispenser"}])
def test_reference_search_recovers_navigation_tool_arguments(jpeg, first_arguments):
    from types import SimpleNamespace

    _, _, TaskAgent, _, _ = components()
    searches = []
    uploads = []

    def chat(messages, *, tools, timeout):
        if messages[-1]["role"] == "tool":
            return {"message": {"role": "assistant", "content": "Found"}}
        arguments = first_arguments if len(messages) == 2 else {"target": "black water dispenser"}
        return {"message": {"role": "assistant", "content": "", "tool_calls": [
            {"function": {"name": "find_object", "arguments": arguments}},
        ]}}

    def query(image, prompt, **kwargs):
        uploads.append(image)
        return {"text": "Glossy black cabinet"}

    def find(target, *args, **kwargs):
        assert kwargs["stop_m"] == 1.0
        searches.append(target)
        return {"status": "localized"}

    agent = TaskAgent(
        SimpleNamespace(chat=chat, config={"timeout_s": 5}),
        SimpleNamespace(query=query, config={"timeout_s": 5}),
        {"timeout_s": 10, "max_steps": 4},
        find_object=find,
    )
    result = agent.run_with_reference(
        "find a black water dispenser and walk up to it until you are about 1.0 meters away", jpeg,
    )
    assert result["navigation"]["status"] == "localized"
    assert uploads == [jpeg]
    assert len(searches) == 1 and searches[0].startswith("black water dispenser")
