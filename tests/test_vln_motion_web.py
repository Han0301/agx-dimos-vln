import asyncio
import threading

import httpx
import pytest

from dimos.plugins._agent_common import load_config
from dimos.plugins.web_input.app import create_app


@pytest.mark.parametrize("endpoint", ["stop", "task", "shutdown"])
def test_motion_cancellation_reaches_tracker(endpoint, monkeypatch):
    from dimos.plugins.runtime_log import current_task_id
    from dimos.plugins.web_input import app as module

    diagnostics = []
    monkeypatch.setattr(module, "event", lambda _, event, **fields:
                        diagnostics.append((event, current_task_id(), fields)))
    started = threading.Event()
    release = threading.Event()
    events = []

    class Runner:
        def run(self, goal, image, cancel):
            events.append(("started", cancel))
            started.set()
            release.wait(1)
            return {"navigation": {"status": "arrived"}}

        def cancel_motion(self, cancel):
            assert cancel.is_set()
            events.append(("cleared", cancel))
            release.set()

    async def exercise():
        app = create_app(load_config("web_input"), Runner)
        async with app.router.lifespan_context(app):
            async with httpx.AsyncClient(
                transport=httpx.ASGITransport(app=app), base_url="http://test"
            ) as web:
                task = (await web.post("/tasks", json={"goal": "approach"})).json()
                assert started.wait(1)
                if endpoint == "shutdown":
                    return
                url = f"/tasks/{task['task_id']}"
                cancel_url = "/stop" if endpoint == "stop" else f"{url}/cancel"
                assert (await web.post(cancel_url)).status_code == 200
                assert len(events) == 2
                assert (await web.get(url)).json()["status"] == "cancelled"

    asyncio.run(exercise())
    assert events == [("started", events[0][1]), ("cleared", events[0][1])]
    _, submitted_task, submitted = next(row for row in diagnostics if row[0] == "task_submitted")
    assert submitted["goal_chars"] == len("approach")
    assert submitted["source"] == "api"
    _, stopped_task, stop = next(row for row in diagnostics if row[0] == "stop_requested")
    assert stopped_task == submitted_task
    assert stop == {
        "source": {"stop": "web_stop", "task": "api_cancel", "shutdown": "web_shutdown"}[endpoint],
        "was_running": True,
    }


def test_arrival_has_its_own_browser_status():
    class Runner:
        def run(self, goal, image, cancel):
            return {"navigation": {"status": "arrived"}}

    async def exercise():
        app = create_app(load_config("web_input"), Runner)
        async with app.router.lifespan_context(app):
            async with httpx.AsyncClient(
                transport=httpx.ASGITransport(app=app), base_url="http://test"
            ) as web:
                task = (await web.post("/navigate", files={"goal": (None, "找杯子")})).json()
                url = f"/tasks/{task['task_id']}"
                result = (await web.get(url, params={"wait_s": 1})).json()
                assert result["status"] == "succeeded"
                status = (await web.get("/status")).json()
                assert status["stage"] == "arrived"
                assert "已到达" in status["status"]

    asyncio.run(exercise())
