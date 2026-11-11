import math
import threading
from types import SimpleNamespace as NS

import pytest

from dimos.plugins.active_search.search import ActiveSearch


def frame(stamp, yaw):
    return NS(
        stamp_ns=stamp,
        odom=NS(
            header=NS(frame_id="world"),
            pose=NS(
                position=NS(x=0., y=0., z=0.),
                orientation=NS(x=0., y=0., z=math.sin(yaw / 2), w=math.cos(yaw / 2)),
            ),
        ),
    )


@pytest.mark.parametrize("hit_at", [None, 2])
def test_sweep_checks_five_new_headings_or_stops_at_hit(hit_at):
    from dimos.plugins.active_search.exploration import RotationSweep

    initial = frame(100, math.radians(160))
    turns = []
    detections = []
    stopped = False

    def turn(yaw, world_frame, remaining):
        nonlocal stopped
        assert world_frame == "world"
        remaining()
        turns.append(yaw)
        stopped = True

    def observe(remaining, *, after_stamp_ns, accept_odom, max_age_s):
        assert stopped
        assert after_stamp_ns == 100 + len(turns) - 1
        observation = frame(after_stamp_ns + 1, turns[-1])
        assert accept_odom(observation.odom)
        return observation

    def detect(observation, target, remaining):
        nonlocal stopped
        assert stopped
        stopped = False
        detections.append(observation.stamp_ns)
        return {"status": "localized" if len(detections) == hit_at else "not_visible"}

    sweep = RotationSweep(observe, lambda: 100, turn, initial, threading.Event(), settle_s=0)
    search = ActiveSearch(
        observe, detect,
        lambda observation, result, remaining: {"status": "localized"},
    )
    result = search(
        "box", lambda: 90, next_view=sweep,
        initial_frame=initial, initial_detection={"status": "not_visible"},
    )
    expected = [220, 280] if hit_at else [220, 280, 340, 400, 460]
    assert [math.degrees(yaw) % 360 for yaw in turns] == pytest.approx([x % 360 for x in expected])
    assert detections == list(range(101, 101 + len(expected)))
    assert result["status"] == ("localized" if hit_at else "search_exhausted")
    assert result["search"]["reason"] == ("confirmed" if hit_at else "sweep_complete")


def test_sweep_hands_a_matching_box_with_bad_depth_to_directed_approach():
    from dimos.plugins.active_search.exploration import RotationSweep

    initial = frame(100, 0.)
    actions = []

    def turn(yaw, world_frame, remaining):
        actions.append("turn")

    def observe(remaining, **options):
        return frame(101, math.pi / 3)

    def toward_target(previous, detection, remaining):
        assert previous.stamp_ns == 101
        assert detection["bbox_2d"] == [400, 400, 600, 600]
        actions.append("approach")
        return frame(102, math.pi / 3)

    def detect(observation, target, remaining):
        if observation.stamp_ns == 101:
            return {"status": "invalid_depth", "bbox_2d": [400, 400, 600, 600]}
        return {"status": "localized"}

    search = ActiveSearch(observe, detect, lambda *a: {"status": "localized"})
    result = search(
        "box", lambda: 90, initial_frame=initial, initial_detection={"status": "not_visible"},
        next_view=RotationSweep(observe, lambda: 100, turn, initial, threading.Event(), settle_s=0),
        toward_target=toward_target,
    )
    assert actions == ["turn", "approach"]
    assert result["status"] == "localized"


@pytest.mark.parametrize("status, expected", [
    ("not_visible", "没有找到目标。"),
    ("invalid_depth", "发现匹配目标，但未能取得有效深度。"),
])
def test_agent_reports_exhausted_scan_without_another_llm_call(status, expected):
    from dimos.plugins.vln_task.agent import TaskAgent

    messages = iter([{"role": "assistant", "tool_calls": [
        {"function": {"name": "find_object", "arguments": {"target": "杯子"}}},
    ]}])
    llm = NS(config={"timeout_s": 1}, chat=lambda *a, **k: {"message": next(messages)})
    agent = TaskAgent(
        llm, None, {"timeout_s": 5, "max_steps": 2}, NS(plan=None),
        find_object=lambda *a, **k: {
            "status": "search_exhausted",
            "search": {"reason": "sweep_complete", "attempts": [{"status": status}]},
        },
    )
    assert agent.run("找杯子")["answer"] == expected


def test_directed_checks_leave_remaining_sweep_headings_available():
    from dimos.plugins.active_search.exploration import RotationSweep

    initial = frame(100, 0.)
    turns = []
    directed_views = []

    def turn(yaw, world_frame, remaining):
        turns.append(yaw)

    def observe(remaining, *, after_stamp_ns, **kwargs):
        return frame(after_stamp_ns + 1, turns[-1])

    def toward_target(previous, detection, remaining):
        observation = frame(previous.stamp_ns + 1, turns[-1])
        directed_views.append(observation.stamp_ns)
        return observation

    def detect(observation, target, remaining):
        return {"status": "invalid_depth" if observation.stamp_ns < 108 else "not_visible"}

    search = ActiveSearch(observe, detect, lambda *a: pytest.fail("No localized target"))
    result = search(
        "box", lambda: 90, initial_frame=initial, initial_detection={"status": "not_visible"},
        next_view=RotationSweep(observe, lambda: 100, turn, initial, threading.Event(), settle_s=0),
        toward_target=toward_target,
    )
    assert directed_views == list(range(102, 109))
    assert [math.degrees(yaw) % 360 for yaw in turns] == pytest.approx([60, 120, 180, 240, 300])
    assert result["search"]["reason"] == "sweep_complete"
