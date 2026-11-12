import json
import threading

import httpx
from fastapi.testclient import TestClient

from dimos.plugins._agent_common import load_config
from dimos.plugins.llm.client import LLMClient
from dimos.plugins.runtime_log import capture
from dimos.plugins.vlm.client import VLMClient
from dimos.vln_task.agent import TaskAgent
from dimos.plugins.web_input.app import create_app


def test_web_text_task_correlates_used_components_without_payloads(tmp_path):
    private_goal = "private test prompt"
    private_answer = "private test answer"

    def response(request):
        assert request.url.path == "/api/chat"
        return httpx.Response(
            200,
            json={
                "done": True,
                "message": {"role": "assistant", "content": private_answer},
            },
        )

    with (
        capture("web_input", base=tmp_path) as run,
        httpx.Client(transport=httpx.MockTransport(response)) as http,
    ):

        def runner():
            return TaskAgent(
                LLMClient(load_config("llm"), http),
                VLMClient(
                    {"base_url": "http://fixture.invalid", "timeout_s": 30}, http
                ),
                load_config("vln_task"),
            )

        with TestClient(create_app(load_config("web_input"), runner)) as web:
            task = web.post("/tasks", json={"goal": private_goal}).json()
            task_id = task["task_id"]
            result = web.get(f"/tasks/{task_id}", params={"wait_s": 2}).json()
            assert result["status"] == "succeeded"
            assert result["result"]["answer"] == private_answer

    modules = run / "modules"
    assert {path.stem for path in modules.glob("*.jsonl")} == {
        "web_input",
        "vln_task",
        "llm",
    }
    for name in ("web_input", "vln_task", "llm"):
        contents = (modules / f"{name}.jsonl").read_text()
        assert private_goal not in contents and private_answer not in contents
        events = [json.loads(line) for line in contents.splitlines()]
        assert any(event.get("task_id") == task_id for event in events)


def test_web_queues_tasks_while_llm_preloads_once():
    started, release = threading.Event(), threading.Event()
    calls = []

    class Runner:
        def preload_llm(self, timeout):
            calls.append(("preload", timeout))
            started.set()
            release.wait(5)
            return {"model": "test", "elapsed_ms": 125.0, "load_ms": 120.0}

        def run(self, goal, image=None, cancel=None):
            calls.append(("run", goal))
            return {"answer": goal}

    app = create_app(load_config("web_input"), Runner, llm_preload_timeout_s=20)
    with TestClient(app) as web:
        try:
            assert started.wait(2)
            assert web.get("/health").json()["llm_warmup"] == {"status": "warming"}
            task_urls = []
            for goal in ("first", "second"):
                task = web.post("/tasks", json={"goal": goal}).json()
                url = f"/tasks/{task['task_id']}"
                task_urls.append(url)
                assert web.get(url).json()["status"] == "queued"
            assert calls == [("preload", 20)]

            release.set()
            for url in task_urls:
                assert (
                    web.get(url, params={"wait_s": 2}).json()["status"] == "succeeded"
                )
            assert calls == [("preload", 20), ("run", "first"), ("run", "second")]
            assert web.get("/health").json()["llm_warmup"] == {
                "status": "ready",
                "model": "test",
                "elapsed_ms": 125.0,
                "load_ms": 120.0,
            }
        finally:
            release.set()


def test_web_worker_handles_tasks_after_preload_failure(caplog):
    class Runner:
        def preload_llm(self, timeout):
            raise httpx.ConnectError("provider unavailable")

        def run(self, goal, image=None, cancel=None):
            return {"answer": goal}

    app = create_app(load_config("web_input"), Runner, llm_preload_timeout_s=20)
    with TestClient(app) as web:
        task = web.post("/tasks", json={"goal": "test"}).json()
        url = f"/tasks/{task['task_id']}"
        assert web.get(url, params={"wait_s": 2}).json()["status"] == "succeeded"
        health = web.get("/health").json()
        assert health["llm_warmup"] == {"status": "failed"}
        assert health["worker_alive"]
        assert "Ollama preload failed error_type=ConnectError" in caplog.text
        assert "provider unavailable" not in caplog.text


def test_wait_returns_terminal_result_when_worker_finishes():
    release = threading.Event()

    class Runner:
        def run(self, goal, image=None, cancel=None):
            release.wait(2)
            return {"answer": goal}

    with TestClient(create_app(load_config("web_input"), Runner)) as web:
        assert "llm_warmup" not in web.get("/health").json()
        task_id = web.post("/tasks", json={"goal": "test"}).json()["task_id"]
        url = f"/tasks/{task_id}"
        assert web.get(url).json()["status"] in {"queued", "running"}
        timer = threading.Timer(0.1, release.set)
        timer.start()
        try:
            response = web.get(url, params={"wait_s": 1})
            assert response.json()["status"] == "succeeded"
            assert response.json()["result"] == {"answer": "test"}
        finally:
            release.set()
            timer.join()


def test_wait_returns_snapshot_at_timeout_and_after_cancel():
    release = threading.Event()

    class Runner:
        def run(self, goal, image=None, cancel=None):
            release.wait(2)
            return {"answer": goal}

    with TestClient(create_app(load_config("web_input"), Runner)) as web:
        try:
            task_id = web.post("/tasks", json={"goal": "test"}).json()["task_id"]
            url = f"/tasks/{task_id}"
            assert web.get(url, params={"wait_s": 0.01}).json()["status"] in {
                "queued",
                "running",
            }
            assert web.get(url, params={"wait_s": 21}).status_code == 422
            web.post(f"{url}/cancel")
            assert web.get(url, params={"wait_s": 1}).json()["status"] == "cancelled"
        finally:
            release.set()


def test_browser_search_status_tracks_the_submitted_task():
    started, release = threading.Event(), threading.Event()

    class Runner:
        def run(self, goal, image=None, cancel=None):
            return self.run_with_progress(goal, image, cancel)

        def run_with_progress(self, goal, image=None, cancel=None, on_progress=None):
            if goal == "后续任务":
                return {"answer": goal}
            if on_progress is not None:
                on_progress({"status": "searching", "attempts": 1, "max_attempts": 4})
            started.set()
            release.wait(2)
            return {"navigation": {"status": "search_exhausted"}}

    with TestClient(create_app(load_config("web_input"), Runner)) as web:
        first = web.post("/navigate", files={"goal": (None, "寻找目标")}).json()
        task_id = first["task_id"]
        try:
            assert started.wait(1)
            assert web.get("/status").json().get("stage") == "searching"
            second = web.post("/navigate", files={"goal": (None, "后续任务")}).json()
            assert web.get("/status").json() == {"status": "queued: 后续任务"}
            progress = web.get("/status", params={"task_id": task_id}).json()
            assert progress["task_id"] == task_id
            assert progress["stage"] == "searching"
            assert "1/4" in progress["status"]
            assert web.get(f"/tasks/{task_id}").json()["progress"]["attempts"] == 1
            release.set()
            assert (
                web.get(f"/tasks/{task_id}", params={"wait_s": 1}).json()["status"]
                == "succeeded"
            )
            final = web.get("/status", params={"task_id": task_id}).json()
            assert final["stage"] == "search_exhausted"
            assert "未找到目标" in final["status"]
            web.get(f"/tasks/{second['task_id']}", params={"wait_s": 1})
            assert web.get("/status").json() == {"status": "succeeded: 后续任务"}
        finally:
            release.set()


def test_cancelled_search_ignores_late_progress():
    started, release, late_reported = (threading.Event() for _ in range(3))

    class Runner:
        def run(self, goal, image=None, cancel=None):
            return self.run_with_progress(goal, image, cancel)

        def run_with_progress(self, goal, image=None, cancel=None, on_progress=None):
            if on_progress is not None:
                on_progress({"status": "searching"})
            started.set()
            release.wait(2)
            if on_progress is not None:
                on_progress({"status": "confirmed"})
            late_reported.set()
            return {"navigation": {"status": "preview_ready"}}

    with TestClient(create_app(load_config("web_input"), Runner)) as web:
        response = web.post("/navigate", files={"goal": (None, "寻找目标")}).json()
        url = f"/tasks/{response['task_id']}"
        try:
            assert started.wait(1)
            assert web.get(url).json().get("progress") == {"status": "searching"}
            assert web.post(f"{url}/cancel").json()["status"] == "cancelled"
            release.set()
            assert late_reported.wait(1)
            assert web.get("/status").json() == {"status": "stopped"}
            task = web.get(url).json()
            assert task["progress"] == {"status": "searching"}
            assert task["result"] is None
        finally:
            release.set()


def test_web_task_failure_logs_redacted_error_and_stack(tmp_path):
    class Runner:
        def run(self, goal, image=None, cancel=None):
            raise ValueError("plan input is invalid token=log-test-secret")

    with capture("web_input", base=tmp_path) as run:
        with TestClient(create_app(load_config("web_input"), Runner)) as web:
            task_id = web.post("/tasks", json={"goal": "diagnostic"}).json()["task_id"]
            assert web.get(f"/tasks/{task_id}", params={"wait_s": 2}).json()["status"] == "failed"

    contents = (run / "modules/web_input.jsonl").read_text()
    records = [json.loads(line) for line in contents.splitlines()]
    failure = next(record for record in records if record.get("event") == "task_failed")
    assert failure["task_id"] == task_id
    assert "plan input is invalid" in failure["error_message"]
    assert "log-test-secret" not in contents
    assert failure["traceback"][-1]["function"] == "run"
