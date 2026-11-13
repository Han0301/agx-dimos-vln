import json
from collections import deque
from io import BytesIO
from threading import Condition
from types import SimpleNamespace as NS

import numpy as np
import pytest
from PIL import Image


@pytest.mark.parametrize("depth_m", [2.0, 6.0, 6.01])
def test_geometry_uses_frozen_capture_and_explicit_coordinates(depth_m):
    from dimos.vln_task.perception import TargetDepthError
    from dimos.vln_task.preview import parse_detection, target_and_goal

    frame = NS(
        k=[100.0, 0.0, 50.0, 0.0, 100.0, 50.0, 0.0, 0.0, 1.0],
        depth=np.full((100, 100), depth_m),
        distortion_model="",
        distortion=(),
        odom=NS(
            pose=NS(
                position=NS(x=1.0, y=2.0, z=0.0),
                orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        ),
    )
    box = parse_detection('```json\n{"name":"bin","bbox":[400,400,600,600]}\n```')
    assert box == (400., 400., 600., 600.)
    assert all(type(value) is float for value in box)
    camera = {"translation": [0.0, 0.0, 0.0], "pitch": 0.0}
    if depth_m == 6.01:
        with pytest.raises(TargetDepthError, match="outside the 6 m localization range"):
            target_and_goal(frame, box, camera, 0.6)
        return
    target, goal, depth = target_and_goal(frame, box, camera, 0.6)
    assert target == pytest.approx([1.0 + depth_m, 2.0, 0.0])
    assert goal == pytest.approx([1.0 + depth_m - 0.6, 2.0])
    assert depth == depth_m
    assert parse_detection("None") is None
    assert parse_detection('{"name":"bin","bbox_3d":[1,2,3]}') is None


def test_profile_keeps_tracking_preview_only():
    from dimos.vln_task.profile import load_profile

    profile = load_profile("config/profiles/vln_preview.yaml")
    assert profile["tracker"]["output_only"] is True
    assert profile["xnavdp"]["sensor_timeout_s"] > 0
    assert profile["observation"]["gap_warning_ms"] == 150


def test_tracking_uses_new_depth_and_odometry_after_capture(monkeypatch):
    from dimos.vln_task import observation as module

    now = 10.0
    monkeypatch.setattr(module.time, "monotonic", lambda: now)
    observations = module.Observations.__new__(module.Observations)
    observations.cv = Condition()
    observations.frames = {
        topic: deque() for topic in (module.DEPTH_TOPIC, module.ODOM_TOPIC)
    }
    observations.depth_scale = 1000
    observations.info = NS(width=2, height=2)

    def depth(stamp, millimeters):
        buf = BytesIO()
        Image.fromarray(np.full((2, 2), millimeters, dtype=np.uint16)).save(buf, "PNG")
        return NS(header=NS(stamp=NS(sec=stamp, nanosec=0)), data=buf.getvalue())

    captured_odom = NS()
    latest_odom = NS()
    observations.frames[module.ODOM_TOPIC].extend(
        [(9.7, captured_odom), (9.9, latest_odom)]
    )
    observations.frames[module.DEPTH_TOPIC].extend(
        [(9.7, depth(1, 2000)), (9.9, depth(2, 750))]
    )

    odom, depth_m, stamp = observations.tracking_inputs()
    assert odom is latest_odom
    assert depth_m.tolist() == [[0.75, 0.75], [0.75, 0.75]]
    assert stamp == 2_000_000_000
    now += 0.5
    assert observations.tracking_inputs() is None


def test_preview_passes_correlated_result_and_blocks_cancelled_result():
    from dimos.vln_task.agent import TaskCancelled
    from dimos.vln_task.preview import NavigationPreview

    frame = NS(
        k=[100.0, 0.0, 50.0, 0.0, 100.0, 50.0, 0.0, 0.0, 1.0],
        depth=np.full((100, 100), 2.0),
        distortion_model="",
        distortion=(),
        odom=NS(
            pose=NS(
                position=NS(x=1.0, y=2.0, z=0.0),
                orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
            header=NS(frame_id="world"),
        ),
        jpeg=b"frame",
        stamp_ns=1000,
        depth_gap_ms=200,
        odom_gap_ms=300,
        wait_ms=0,
    )
    vlm = NS(
        config={"timeout_s": 1},
        query=lambda *a, **kw: {
            "text": '{"name":"bin","bbox":[400,400,600,600]}',
            "elapsed_ms": 3,
        },
    )
    previewed = []
    cancel = False

    def remaining():
        if cancel:
            raise TaskCancelled()
        return 10

    def plan(snapshot, goal, check, timing_ms):
        nonlocal cancel
        assert snapshot is frame
        assert goal == pytest.approx([2.4, 2.0])
        cancel = plan.cancel
        return '{"request_stamp_ns":1000}', 0.01, 3

    plan.cancel = False
    skill = NavigationPreview(
        vlm,
        lambda _: frame,
        plan,
        lambda data: previewed.append(json.loads(data)) or {"linear_x": 0.2},
        {"translation": [0.0, 0.0, 0.0], "pitch": 0.0},
        0.6,
    )
    result = skill("bin", remaining)
    assert result["status"] == "preview_ready"
    assert result["observation"]["depth_gap_ms"] == 200
    assert result["path_points"] == 3
    assert len(previewed) == 1
    plan.cancel = True
    with pytest.raises(TaskCancelled):
        skill("bin", remaining)
    assert len(previewed) == 1


def test_agent_routes_camera_request_without_upload(monkeypatch):
    from dimos.vln_task import agent as module

    calls = []
    diagnostics = {}
    monkeypatch.setattr(module, "event", lambda _, name, **fields: diagnostics.update({name: fields}))
    goal = "预览接近白色垃圾桶的路径"

    class Skill:
        plan = True

        def __call__(self, target, remaining, *, plan_path):
            assert remaining() > 0
            calls.append((target, plan_path))
            return {"status": "preview_ready"}

    class LLM:
        config = {"timeout_s": 1}

        def chat(self, messages, *, tools, timeout):
            assert messages[1] == {"role": "user", "content": goal}
            assert [tool["function"]["name"] for tool in tools] == [
                "preview_approach",
                "locate_visible_object",
            ]
            if messages[-1]["role"] == "tool":
                assert json.loads(messages[-1]["content"])["status"] == "preview_ready"
                return {"message": {"role": "assistant", "content": "预览就绪"}}
            return {
                "message": {
                    "role": "assistant",
                    "content": "",
                    "tool_calls": [
                        {
                            "function": {
                                "name": "preview_approach",
                                "arguments": {"target": "白色垃圾桶"},
                            }
                        }
                    ],
                }
            }

    agent = module.TaskAgent(
        LLM(), None, {"timeout_s": 2, "max_steps": 3}, navigation_preview=Skill()
    )
    result = agent.run(goal)
    assert calls == [("白色垃圾桶", True)]
    assert result["navigation"]["status"] == "preview_ready"
    assert diagnostics["navigation_tool_started"]["original_goal"] == goal
    assert diagnostics["navigation_tool_started"]["target"] == "白色垃圾桶"
    assert diagnostics["navigation_tool_finished"]["navigation_status"] == "preview_ready"


def test_malformed_reply_is_a_miss_before_navigation():
    from dimos.vln_task.preview import NavigationPreview

    response = '{"name":"bin","bbox":[301,344,400,]}'

    target = "white rectangular pedal bin with a lid on the floor left of the plants"
    frame = NS(jpeg=b"frame", stamp_ns=1000, depth_gap_ms=3, odom_gap_ms=4, wait_ms=0)

    def query(image, prompt, **kwargs):
        assert image == frame.jpeg
        assert target in prompt
        return {"text": response, "elapsed_ms": 31}

    def must_not_run(*args):
        pytest.fail("A malformed detection must not reach planning or tracking")

    skill = NavigationPreview(
        NS(query=query, config={"timeout_s": 1}),
        lambda _: frame,
        must_not_run,
        must_not_run,
        {},
        0.6,
    )
    result = skill(target, lambda: 1)
    assert result["status"] == "not_visible"
    assert result["observation"]["stamp_ns"] == 1000
    assert result["timing_ms"]["vlm"] == 31
    assert "bbox_2d" not in result
