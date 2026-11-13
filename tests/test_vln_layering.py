"""Exercise the new upper-layer entry and isolated, reusable capabilities."""

import importlib.util
import json
import subprocess
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest


def test_upper_layer_search_preserves_recovery_and_rotation_order():
    assert importlib.util.find_spec("dimos.vln_task"), "VLN Task must live outside plugins"
    from dimos.vln_task.search.search import ActiveSearch

    trace = []
    frames = iter([NS(stamp_ns=2), NS(stamp_ns=3), NS(stamp_ns=4)])
    statuses = iter(["invalid_depth", "target_mismatch", "localized"])

    def next_view(*_):
        trace.append("turn")
        return next(frames)

    def toward(*_):
        trace.append("recover_depth")
        return next(frames)

    def detect(frame, target, remaining, *, stop_m):
        assert target == "white bin" and stop_m == 0.8
        trace.append(f"detect:{frame.stamp_ns}")
        return {"status": next(statuses)}

    def plan(frame, result, remaining):
        trace.append(f"plan:{frame.stamp_ns}")
        return {"status": "preview_ready"}

    result = ActiveSearch(None, detect, plan)(
        "white bin", lambda: 10, next_view=next_view, toward_target=toward,
        initial_frame=NS(stamp_ns=1),
        initial_detection={"status": "not_visible", "standoff_m": 0.8},
    )
    assert result["status"] == "preview_ready"
    assert trace == ["turn", "detect:2", "recover_depth", "detect:3", "turn", "detect:4", "plan:4"]
    assert [row["stamp_ns"] for row in result["search"]["attempts"]] == [1, 2, 3, 4]


def test_perception_localizes_without_constructing_a_planner_or_tracker():
    assert importlib.util.find_spec("dimos.vln_task"), "VLN Task must live outside plugins"
    from dimos.vln_task.perception import ObjectPerception

    vlm = NS(config={"timeout_s": 2}, query=lambda *a, **kw: {
        "text": json.dumps({"name": "bin", "bbox": [400, 400, 600, 600]}),
        "elapsed_ms": 5,
    })
    frame = NS(
        jpeg=b"frame", stamp_ns=100, wait_ms=0, depth_gap_ms=0, odom_gap_ms=0,
        depth=np.full((100, 100), 2.), k=[100, 0, 50, 0, 100, 50, 0, 0, 1],
        distortion=(), distortion_model="",
        odom=NS(header=NS(frame_id="world"), pose=NS(
            position=NS(x=1., y=2., z=0.), orientation=NS(x=0., y=0., z=0., w=1.),
        )),
    )
    perception = ObjectPerception(vlm, {"translation": [0, 0, 0], "pitch": 0}, 0.8)
    result = perception.detect(frame, "white bin", lambda: 2)
    assert result["status"] == "localized"
    assert result["target_xyz"] == pytest.approx([3., 2., 0.])
    assert result["goal_xy"] == pytest.approx([2.2, 2.])
    frame.depth.fill(0)
    assert perception.detect(frame, "white bin", lambda: 2, allow_bad_depth=True)["status"] == "invalid_depth"


def test_web_transport_runs_generic_tasks_without_importing_navigation():
    assert importlib.util.find_spec("dimos.plugins.web_input.transport"), "Web transport must be independent"
    code = '''
import sys
import asyncio
import httpx
from dimos.plugins._agent_common import load_config
from dimos.plugins.web_input.transport import create_app
class Runner:
    def run(self, goal, image=None, cancel=None):
        return {"answer": goal.upper()}
async def exercise():
    app = create_app(load_config("web_input"), Runner)
    assert not {"/", "/navigate", "/stop", "/status", "/realsense_feed"} & {
        route.path for route in app.routes
    }
    async with app.router.lifespan_context(app):
        async with httpx.AsyncClient(
            transport=httpx.ASGITransport(app=app), base_url="http://test",
        ) as web:
            submitted = (await web.post("/tasks", json={"goal": "hello"})).json()
            result = (await web.get("/tasks/" + submitted["task_id"], params={"wait_s": 2})).json()
            assert result["status"] == "succeeded", result
            assert result["result"] == {"answer": "HELLO"}, result
asyncio.run(exercise())
assert not any(name.startswith(("dimos.vln_task", "dimos.plugins.llm", "dimos.plugins.vlm", "dimos.plugins.path_tracker", "rclpy")) for name in sys.modules)
'''
    completed = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, timeout=20)
    assert completed.returncode == 0, completed.stderr


@pytest.mark.parametrize("stage,label", [("arrived", "已到达接近点"), ("invalid_depth", "目标深度无效")])
def test_upper_web_entry_preserves_browser_status(stage, label):
    assert importlib.util.find_spec("dimos.vln_task"), "VLN Task must live outside plugins"
    from fastapi.testclient import TestClient
    from dimos.plugins._agent_common import load_config
    from dimos.vln_task.web import create_app

    class Runner:
        def run(self, goal, image=None, cancel=None):
            return {"navigation": {"status": stage}}

    with TestClient(create_app(load_config("web_input"), Runner)) as web:
        submitted = web.post("/navigate", files={"goal": (None, "找杯子")}).json()
        task = web.get(f"/tasks/{submitted['task_id']}", params={"wait_s": 2}).json()
        assert task["status"] == "succeeded"
        assert web.get("/status").json() == {
            "status": f"{label} · 找杯子", "stage": stage, "task_id": submitted["task_id"],
        }
