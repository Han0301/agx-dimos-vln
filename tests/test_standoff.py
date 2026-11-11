from types import SimpleNamespace as NS

import numpy as np
import pytest


@pytest.mark.parametrize("text,expected", [
    ("find a fire extinguisher box and walk up to it until you are about 0.8 meters away", 0.8),
    ("approach the box and stop 0.8 metres from it", 0.8),
    ("找到灭火器箱，停在距目标约 0.8 米处", 0.8),
    ("靠近灭火器箱，保持 80 厘米距离", 0.8),
    ("find a 0.8 meter tall red cabinet", None),
    ("find a fire extinguisher box 2 meters away from the door", None),
    ("找门旁 0.8 米高的灭火器箱", None),
])
def test_explicit_standoff_distinguishes_object_size_and_location(text, expected):
    from dimos.plugins.vln_task.standoff import requested_standoff

    assert requested_standoff(text) == expected


@pytest.mark.parametrize("visible_first", [True, False])
def test_original_request_distance_reaches_real_localization_even_after_a_miss(visible_first):
    from dimos.plugins.active_search.search import ActiveSearch
    from dimos.plugins.vln_task.agent import TaskAgent
    from dimos.plugins.vln_task.find_object import FindObject
    from dimos.plugins.vln_task.preview import NavigationPreview

    def frame(stamp):
        return NS(
            k=[100., 0., 50., 0., 100., 50., 0., 0., 1.],
            depth=np.full((100, 100), 2.0),
            distortion_model="",
            distortion=(),
            odom=NS(
                header=NS(frame_id="world"),
                pose=NS(
                    position=NS(x=1., y=2., z=0.),
                    orientation=NS(x=0., y=0., z=0., w=1.),
                ),
            ),
            jpeg=b"frame", stamp_ns=stamp, depth_gap_ms=0, odom_gap_ms=0, wait_ms=0,
        )

    def query(*args, stamp_ns, **kwargs):
        found = visible_first or stamp_ns > 100
        return {
            "text": '[{"bbox_2d":[400,400,600,600],"label":"box","appearance_match":"match"}]' if found else "[]",
            "elapsed_ms": 1,
        }

    skill = NavigationPreview(
        NS(config={"timeout_s": 1}, query=query), lambda _: frame(100),
        None, None, {"translation": [0., 0., 0.], "pitch": 0.}, 0.6,
    )
    search = ActiveSearch(
        lambda previous, remaining: frame(previous.stamp_ns + 1),
        skill.detect,
        lambda frame, result, remaining: result,
    )

    def explore(target, remaining, progress, cancel, initial_frame, initial_detection):
        return search(
            target, remaining, progress, initial_frame=initial_frame,
            initial_detection=initial_detection,
        )

    find = FindObject(skill.snapshot, skill.detect, explore, lambda frame, result, remaining: result)
    llm = NS(
        config={"timeout_s": 1},
        chat=lambda *args, **kwargs: {"message": {"tool_calls": [{"function": {
            "name": "find_object", "arguments": {"target": "fire extinguisher box"},
        }}]}},
    )

    def approach(result, remaining, cancel, progress, *, target):
        assert result["standoff_m"] == 0.8
        assert result["goal_xy"] == pytest.approx([2.2, 2.])
        return {**result, "status": "arrived"}

    agent = TaskAgent(
        llm, None, {"timeout_s": 5, "max_steps": 2}, skill,
        find_object=find, approach=approach,
    )
    result = agent.run("find a fire extinguisher box and walk up to it until you are about 0.8 meters away")
    assert result["navigation"]["status"] == "arrived"


def test_background_detection_reuses_task_distance():
    from collections import deque
    from threading import Event

    from dimos.plugins.vln_task.approach import ContinuousApproach

    stop = Event()
    updates = deque(maxlen=1)

    def detect(frame, target, remaining, *, stop_m):
        assert stop_m == 0.8
        stop.set()
        return {"status": "not_visible"}

    controller = ContinuousApproach(
        lambda *args, **kwargs: NS(stamp_ns=2, odom=NS(header=NS(frame_id="world"))),
        None, None, detect=detect, refresh_interval_s=0,
    )
    try:
        controller._refresh_targets(
            "box",
            {"observation": {"stamp_ns": 1}, "world_frame": "world", "target_xyz": [1., 0., 0.],
             "standoff_m": 0.8},
            lambda: 5, Event(), stop, updates,
        )
    finally:
        controller.close()
    assert stop.is_set()
